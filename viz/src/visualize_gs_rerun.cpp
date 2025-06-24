#include "viz/visualize_gs_rerun.hpp"

#include <glog/logging.h>
#include <algorithm>
#include <chrono>

namespace viz {

GaussianSplatRerunVisualizer::GaussianSplatRerunVisualizer(
    std::shared_ptr<core::storage::MapStore> map_store, const std::string& recording_id,
    const std::string& app_name, const std::string& host, uint16_t port)
    : map_store_(map_store),
      recording_id_(recording_id),
      app_name_(app_name),
      host_(host),
      port_(port),
      running_(false),
      enabled_(true),
      current_timestamp_(0.0) {}

GaussianSplatRerunVisualizer::~GaussianSplatRerunVisualizer() {
    disconnect();
}

bool GaussianSplatRerunVisualizer::initialize() {
    if (running_) {
        LOG(WARNING) << "GaussianSplatRerunVisualizer already initialized";
        return true;
    }

    if (!initializeRerunClient()) {
        LOG(ERROR) << "Failed to initialize Rerun client for Gaussian splat visualization";
        return false;
    }

    // Start the visualization worker thread
    running_ = true;
    visualization_thread_ = std::thread(&GaussianSplatRerunVisualizer::visualizationWorker, this);

    LOG(INFO) << "GaussianSplatRerunVisualizer initialized successfully";
    return true;
}

bool GaussianSplatRerunVisualizer::isConnected() const {
    return rerun_visualizer_ && rerun_visualizer_->isConnected();
}

void GaussianSplatRerunVisualizer::disconnect() {
    if (running_) {
        running_ = false;
        notification_cv_.notify_all();

        if (visualization_thread_.joinable()) {
            visualization_thread_.join();
        }
    }

    if (rerun_visualizer_) {
        rerun_visualizer_->disconnect();
    }
}

void GaussianSplatRerunVisualizer::clear() {
    if (rerun_visualizer_) {
        rerun_visualizer_->clear();
    }
}

void GaussianSplatRerunVisualizer::setTimestamp(double timestamp) {
    current_timestamp_ = timestamp;
}

void GaussianSplatRerunVisualizer::notifyNewSplatBatch() {
    if (!enabled_ || !running_) {
        return;
    }

    std::lock_guard<std::mutex> lock(notification_mutex_);
    notification_cv_.notify_one();
}

void GaussianSplatRerunVisualizer::notifyNewSplatBatch(uint32_t batch_id) {
    if (!enabled_ || !running_) {
        return;
    }

    // Add batch ID to pending queue for event-driven processing
    {
        std::lock_guard<std::mutex> lock(pending_batches_mutex_);
        pending_batch_ids_.push(batch_id);
    }

    // Notify worker thread
    {
        std::lock_guard<std::mutex> lock(notification_mutex_);
        notification_cv_.notify_one();
    }

    LOG(INFO) << "Queued splat batch " << batch_id << " for visualization";
}

void GaussianSplatRerunVisualizer::setEnabled(bool enabled) {
    enabled_ = enabled;
    if (enabled) {
        notifyNewSplatBatch();  // Trigger update when re-enabled
    }
}

bool GaussianSplatRerunVisualizer::isEnabled() const {
    return enabled_;
}

bool GaussianSplatRerunVisualizer::initializeRerunClient() {
    try {
        // Create RerunVisualizer with shared recording ID for multi-process coordination
        // This allows both rosbag and GS processes to publish to the same Rerun viewer
        rerun_visualizer_ =
            std::make_shared<RerunVisualizer>(app_name_, recording_id_, host_, port_);

        // Initialize the Rerun client (using same pattern as existing code)
        if (!rerun_visualizer_->initialize()) {
            LOG(ERROR) << "Failed to initialize RerunVisualizer for Gaussian splats";
            return false;
        }

        LOG(INFO) << "Successfully initialized Rerun client for Gaussian splat visualization with "
                     "recording_id: "
                  << recording_id_;
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception while initializing Rerun client: " << e.what();
        return false;
    }
}

void GaussianSplatRerunVisualizer::visualizationWorker() {
    LOG(INFO) << "Gaussian splat visualization worker thread started";

    while (running_) {
        std::unique_lock<std::mutex> lock(notification_mutex_);

        // Wait for notification or timeout
        notification_cv_.wait_for(lock, std::chrono::seconds(1),
                                  [this] { return !running_ || enabled_; });

        if (!running_) {
            break;
        }

        if (enabled_ && isConnected()) {
            // Process pending batches first (event-driven)
            while (true) {
                uint32_t batch_id;
                {
                    std::lock_guard<std::mutex> batch_lock(pending_batches_mutex_);
                    if (pending_batch_ids_.empty()) {
                        break;
                    }
                    batch_id = pending_batch_ids_.front();
                    pending_batch_ids_.pop();
                }

                visualizeSpecificSplatBatch(batch_id);
            }

            // Fallback: still do full scan occasionally (for robustness)
            // This handles any missed notifications or initialization
            // visualizeLatestSplatBatches();
        }
    }

    LOG(INFO) << "Gaussian splat visualization worker thread stopped";
}

void GaussianSplatRerunVisualizer::visualizeLatestSplatBatches() {
    if (!map_store_ || !rerun_visualizer_) {
        return;
    }
    LOG(INFO) << "This should not actually be called visualizeLatestSplatBatches()";

    try {
        // Get all splat batches from storage
        auto splat_batches = map_store_->getAllGaussianSplatBatches();

        if (splat_batches.empty()) {
            return;
        }

        // Filter out already processed batches
        std::vector<core::types::GaussianSplatBatch> new_batches;
        {
            std::lock_guard<std::mutex> lock(processed_batches_mutex_);
            for (const auto& batch : splat_batches) {
                if (std::find(processed_batch_ids_.begin(), processed_batch_ids_.end(),
                              batch.batch_id) == processed_batch_ids_.end()) {
                    new_batches.push_back(batch);
                    processed_batch_ids_.push_back(batch.batch_id);
                }
            }
        }

        if (new_batches.empty()) {
            return;
        }

        LOG(INFO) << "Visualizing " << new_batches.size() << " new Gaussian splat batches";

        // Visualize each new batch
        for (const auto& batch : new_batches) {
            rerun_visualizer_->addGaussianSplatBatch(batch, "gaussian_splats", batch.timestamp);
        }

        // Also create a combined view of all recent splats
        std::vector<core::types::GaussianSplat> all_splats;
        for (const auto& batch : splat_batches) {
            all_splats.insert(all_splats.end(), batch.splats.begin(), batch.splats.end());
        }

        if (!all_splats.empty()) {
            double timestamp = current_timestamp_ > 0
                                   ? current_timestamp_
                                   : std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::steady_clock::now().time_since_epoch())
                                             .count() /
                                         1e9;

            rerun_visualizer_->addGaussianSplats(all_splats, "odom/gaussian_splats/all_splats",
                                                 timestamp);
            LOG(INFO) << "Combined visualization: " << all_splats.size() << " total splats";
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in visualizeLatestSplatBatches: " << e.what();
    }
}

void GaussianSplatRerunVisualizer::visualizeSpecificSplatBatch(uint32_t batch_id) {
    if (!map_store_ || !rerun_visualizer_) {
        return;
    }

    LOG(INFO) << "visualizeSpecificSplatBatch: Visualizing splat " << batch_id;
    try {
        // Check if this batch was already processed
        {
            std::lock_guard<std::mutex> lock(processed_batches_mutex_);
            if (std::find(processed_batch_ids_.begin(), processed_batch_ids_.end(), batch_id) !=
                processed_batch_ids_.end()) {
                LOG(INFO) << "Splat batch " << batch_id << " already processed, skipping";
                return;
            }
        }

        // Load specific batch from storage
        auto batch_opt = map_store_->getGaussianSplatBatch(batch_id);
        if (!batch_opt) {
            LOG(WARNING) << "Failed to load splat batch " << batch_id << " for visualization";
            return;
        }

        const auto& batch = batch_opt.value();

        // Mark as processed
        {
            std::lock_guard<std::mutex> lock(processed_batches_mutex_);
            processed_batch_ids_.push_back(batch_id);
        }

        LOG(INFO) << "Event-driven visualization of splat batch " << batch_id << " with "
                  << batch.splats.size() << " splats";

        // Visualize the specific batch
        rerun_visualizer_->addGaussianSplatBatch(batch, "odom/gaussian_splats", batch.timestamp);

        // Also update combined view with this new batch
        auto all_batches = map_store_->getAllGaussianSplatBatches();
        std::vector<core::types::GaussianSplat> all_splats;
        for (const auto& b : all_batches) {
            all_splats.insert(all_splats.end(), b.splats.begin(), b.splats.end());
        }

        if (!all_splats.empty()) {
            double timestamp = current_timestamp_ > 0 ? current_timestamp_ : batch.timestamp;
            rerun_visualizer_->addGaussianSplats(all_splats, "odom/gaussian_splats/all_splats",
                                                 timestamp);
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in visualizeSpecificSplatBatch(" << batch_id << "): " << e.what();
    }
}

}  // namespace viz
