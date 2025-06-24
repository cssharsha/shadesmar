#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "viz/rerun_viz.hpp"
#include "core/storage/map_store.hpp"
#include "core/types/gaussian_splat.hpp"

namespace viz {

class GaussianSplatRerunVisualizer {
public:
    GaussianSplatRerunVisualizer(std::shared_ptr<core::storage::MapStore> map_store,
                                const std::string& recording_id = "shadesmar_combined",
                                const std::string& app_name = "gaussian_splats",
                                const std::string& host = "localhost",
                                uint16_t port = 9999);

    ~GaussianSplatRerunVisualizer();

    // Initialize and connect to Rerun client
    bool initialize();
    bool isConnected() const;
    void disconnect();
    void clear();
    void setTimestamp(double timestamp);

    // Notification mechanism for gs_processor
    void notifyNewSplatBatch();

    // Enable/disable visualization
    void setEnabled(bool enabled);
    bool isEnabled() const;

private:
    // Thread for async visualization updates
    void visualizationWorker();
    
    // Query and visualize latest splat batches
    void visualizeLatestSplatBatches();
    
    // Initialize or attach to existing Rerun client
    bool initializeRerunClient();

    std::shared_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<RerunVisualizer> rerun_visualizer_;
    
    std::string recording_id_;
    std::string app_name_;
    std::string host_;
    uint16_t port_;
    
    // Async processing
    std::thread visualization_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> enabled_;
    std::mutex notification_mutex_;
    std::condition_variable notification_cv_;
    
    // Keep track of processed batches to avoid duplicates
    std::vector<uint64_t> processed_batch_ids_;
    std::mutex processed_batches_mutex_;
    
    double current_timestamp_;
};

}  // namespace viz