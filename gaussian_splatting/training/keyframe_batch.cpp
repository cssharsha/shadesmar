#include "keyframe_batch.hpp"
#include <algorithm>
#include <chrono>
#include <logging/logging.hpp>
#include <opencv2/opencv.hpp>
#include <random>

namespace gaussian_splatting {
namespace training {

bool KeyframeBatch::isValid() const {
    // Add the logs printing all the checks being done here
    LOG(INFO) << "Batch size: " << batch_size << " Keyframe IDs: " << keyframe_ids.size()
              << " Keyframes: " << keyframes.size() << " Images: " << images.defined() << " "
              << images.sizes() << " Camera poses: " << camera_poses.defined() << " "
              << camera_poses.sizes() << " Camera intrinsics: " << camera_intrinsics.defined()
              << " " << camera_intrinsics.sizes() << " Depths: " << depths.defined();
    // return batch_size > 0 && keyframe_ids.size() == batch_size && keyframes.size() == batch_size
    // &&
    //        images.defined() && images.size(0) == static_cast<int64_t>(batch_size) &&
    //        camera_poses.defined() && camera_poses.size(0) == static_cast<int64_t>(batch_size) &&
    //        camera_intrinsics.defined() &&
    //        camera_intrinsics.size(0) == static_cast<int64_t>(batch_size);
    return batch_size > 0 && keyframe_ids.size() == batch_size && images.defined() &&
           images.size(0) == static_cast<int64_t>(batch_size) && camera_poses.defined() &&
           camera_poses.size(0) == static_cast<int64_t>(batch_size) &&
           camera_intrinsics.defined() &&
           camera_intrinsics.size(0) == static_cast<int64_t>(batch_size);
}

KeyframeBatchLoader::KeyframeBatchLoader(std::shared_ptr<core::storage::MapStore> map_store,
                                         const TrainingConfig& config, torch::Device device)
    : map_store_(map_store),
      config_(config),
      device_(device),
      current_batch_index_(0),
      next_batch_id_(1),
      max_cache_size_(100) {  // Cache up to 100 keyframes

    LOG(INFO) << "KeyframeBatchLoader initialized with batch size: " << config_.keyframes_per_batch
              << ", device: " << (device_.is_cuda() ? "CUDA" : "CPU");
}

void KeyframeBatch::print() {
    std::stringstream ss;
    ss << "Keyframe bath " << batch_id << "( " << batch_size << ") info:\n"
       << "Images: " << images.sizes() << "\n"
       << "Camera poses: " << camera_poses.sizes() << "\n"
       << "Camera info: " << camera_intrinsics.sizes();
    LOG(INFO) << ss.str();
}

bool KeyframeBatchLoader::initialize() {
    LOG(INFO) << "Initializing KeyframeBatchLoader...";

    if (!map_store_) {
        LOG(ERROR) << "MapStore is null";
        return false;
    }

    // Load available keyframes from MapStore
    if (!loadAvailableKeyframes()) {
        LOG(ERROR) << "Failed to load available keyframes";
        return false;
    }

    // Initialize shuffled order
    shuffled_keyframes_ = available_keyframes_;
    shuffleKeyframes();

    LOG(INFO) << "KeyframeBatchLoader initialized with " << available_keyframes_.size()
              << " keyframes, " << getTotalBatches() << " batches";

    return true;
}

bool KeyframeBatchLoader::loadAvailableKeyframes() {
    available_keyframes_.clear();

    auto all_poses = map_store_->getAllKeyFramePoses();
    LOG(INFO) << "Found " << all_poses.size() << " keyframes in MapStore";

    for (const auto& [keyframe_id, pose] : all_poses) {
        // Load keyframe to validate it has required data
        auto keyframe = map_store_->getKeyFrame(keyframe_id);
        if (validateKeyframe(keyframe)) {
            available_keyframes_.push_back(keyframe_id);
        } else {
            LOG(WARNING) << "Keyframe " << keyframe_id << " failed validation, skipping";
        }
    }

    // This might not actuallyt be required
    std::sort(available_keyframes_.begin(), available_keyframes_.end());

    LOG(INFO) << "Loaded " << available_keyframes_.size() << " valid keyframes for training";
    return !available_keyframes_.empty();
}

bool KeyframeBatchLoader::validateKeyframe(const core::types::KeyFrame::Ptr& keyframe) const {
    if (!keyframe) {
        return false;
    }

    // Check for required data
    if (!keyframe->hasColorImage()) {
        LOG(WARNING) << "Keyframe " << keyframe->id << " missing color image";
        return false;
    }

    // Validate image dimensions
    const auto& image = keyframe->getColorImage();
    if (image.width <= 0 || image.height <= 0) {
        LOG(WARNING) << "Keyframe " << keyframe->id
                     << " has invalid image dimensions: " << image.width << "x" << image.height;
        return false;
    }

    // Check pose validity
    if (!keyframe->pose.position.allFinite() || !keyframe->pose.orientation.coeffs().allFinite()) {
        LOG(WARNING) << "Keyframe " << keyframe->id << " has invalid pose";
        return false;
    }

    return true;
}

bool KeyframeBatchLoader::loadBatch(const std::vector<uint64_t>& keyframe_ids,
                                    KeyframeBatch& batch) {
    if (keyframe_ids.empty()) {
        LOG(ERROR) << "Empty keyframe ID list provided";
        return false;
    }

    if (keyframe_ids.size() > config_.keyframes_per_batch) {
        LOG(WARNING) << "Requested batch size " << keyframe_ids.size()
                     << " exceeds configured batch size " << config_.keyframes_per_batch;
    }

    LOG(INFO) << "Loading batch with " << keyframe_ids.size() << " keyframes";

    batch.clear();
    batch.batch_id = next_batch_id_++;
    batch.batch_size = keyframe_ids.size();
    batch.device = device_;
    batch.keyframe_ids = keyframe_ids;

    // Load keyframes
    batch.keyframes.reserve(keyframe_ids.size());
    for (uint64_t kf_id : keyframe_ids) {
        // Check cache first
        auto cache_it = keyframe_cache_.find(kf_id);
        core::types::KeyFrame::Ptr keyframe;

        if (cache_it != keyframe_cache_.end()) {
            keyframe = cache_it->second;
            LOG(INFO) << "Using cached keyframe " << kf_id;
        } else {
            keyframe = map_store_->getKeyFrame(kf_id);
            if (!validateKeyframe(keyframe)) {
                LOG(ERROR) << "Failed to load or validate keyframe " << kf_id;
                return false;
            }
            updateKeyframeCache(kf_id, keyframe);
        }

        batch.keyframes.push_back(keyframe);
    }

    // Convert keyframes to tensors
    if (!convertKeyframesToTensors(batch.keyframes, batch)) {
        LOG(ERROR) << "Failed to convert keyframes to tensors";
        return false;
    }

    // Validate batch
    if (!batch.isValid()) {
        LOG(ERROR) << "Generated invalid batch";
        return false;
    }

    LOG(INFO) << "Successfully loaded batch " << batch.batch_id << " with " << batch.batch_size
              << " keyframes, image size: " << batch.image_height << "x" << batch.image_width;

    return true;
}

bool KeyframeBatchLoader::getNextBatch(KeyframeBatch& batch) {
    if (!hasMoreBatches()) {
        LOG(INFO) << "No more batches available in current epoch";
        return false;
    }

    // Calculate batch keyframes
    size_t start_idx = current_batch_index_ * config_.keyframes_per_batch;
    size_t end_idx = std::min(start_idx + config_.keyframes_per_batch, shuffled_keyframes_.size());

    std::vector<uint64_t> batch_keyframes(shuffled_keyframes_.begin() + start_idx,
                                          shuffled_keyframes_.begin() + end_idx);

    bool success = loadBatch(batch_keyframes, batch);
    if (success) {
        current_batch_index_++;
    }

    return success;
}

void KeyframeBatchLoader::resetBatchIterator() {
    current_batch_index_ = 0;
    LOG(INFO) << "Reset batch iterator to beginning";
}

void KeyframeBatchLoader::shuffleKeyframes() {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(shuffled_keyframes_.begin(), shuffled_keyframes_.end(), g);

    LOG(INFO) << "Shuffled " << shuffled_keyframes_.size() << " keyframes for new epoch";
    resetBatchIterator();
}

size_t KeyframeBatchLoader::getTotalBatches() const {
    if (available_keyframes_.empty())
        return 0;
    return (available_keyframes_.size() + config_.keyframes_per_batch - 1) /
           config_.keyframes_per_batch;
}

bool KeyframeBatchLoader::hasMoreBatches() const {
    return current_batch_index_ < getTotalBatches();
}

bool KeyframeBatchLoader::convertKeyframesToTensors(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes, KeyframeBatch& batch) {
    if (keyframes.empty()) {
        LOG(ERROR) << "No keyframes to convert";
        return false;
    }

    // Determine image dimensions from first keyframe
    const auto& first_image = keyframes[0]->getColorImage();
    batch.image_height = first_image.height;
    batch.image_width = first_image.width;

    // Initialize tensors
    batch.images = torch::zeros(
        {static_cast<int64_t>(keyframes.size()), 3, batch.image_height, batch.image_width},
        torch::dtype(torch::kFloat32).device(device_));

    batch.camera_poses = torch::zeros({static_cast<int64_t>(keyframes.size()), 4, 4},
                                      torch::dtype(torch::kFloat32).device(device_));

    batch.camera_intrinsics = torch::zeros({static_cast<int64_t>(keyframes.size()), 3, 3},
                                           torch::dtype(torch::kFloat32).device(device_));

    // Convert each keyframe
    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto& keyframe = keyframes[i];

        // Convert image
        torch::Tensor image_tensor = convertImageToTensor(keyframe->getColorImage());
        if (!image_tensor.defined()) {
            LOG(ERROR) << "Failed to convert image for keyframe " << keyframe->id;
            return false;
        }
        batch.images[i] = image_tensor.to(device_);

        // Extract camera pose
        torch::Tensor pose_tensor = extractCameraPose(keyframe);
        if (!pose_tensor.defined()) {
            LOG(ERROR) << "Failed to extract camera pose for keyframe " << keyframe->id;
            return false;
        }
        batch.camera_poses[i] = pose_tensor.to(device_);

        // Extract camera intrinsics
        torch::Tensor intrinsics_tensor = extractCameraIntrinsics(keyframe);
        if (!intrinsics_tensor.defined()) {
            LOG(ERROR) << "Failed to extract camera intrinsics for keyframe " << keyframe->id;
            return false;
        }
        batch.camera_intrinsics[i] = intrinsics_tensor.to(device_);
    }

    return true;
}

torch::Tensor KeyframeBatchLoader::extractCameraPose(const core::types::KeyFrame::Ptr& keyframe) {
    // Create 4x4 transformation matrix from position and orientation
    torch::Tensor pose = torch::eye(4, torch::dtype(torch::kFloat32));

    // Extract rotation matrix from quaternion
    const auto& q = keyframe->pose.orientation;
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

    // Fill rotation part
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pose[i][j] = static_cast<float>(rotation_matrix(i, j));
        }
    }

    // Fill translation part
    const auto& pos = keyframe->pose.position;
    pose[0][3] = static_cast<float>(pos.x());
    pose[1][3] = static_cast<float>(pos.y());
    pose[2][3] = static_cast<float>(pos.z());

    return pose;
}

torch::Tensor KeyframeBatchLoader::extractCameraIntrinsics(
    const core::types::KeyFrame::Ptr& keyframe) {
    // Create 3x3 camera intrinsic matrix
    torch::Tensor K = torch::zeros({3, 3}, torch::dtype(torch::kFloat32));

    // Use camera parameters if available, otherwise use reasonable defaults
    if (keyframe->hasCameraInfo()) {
        const auto& camera_info = keyframe->getCameraInfo();

        // Extract intrinsics from K matrix (stored in row-major order)
        if (camera_info.k.size() == 9) {
            float fx = static_cast<float>(camera_info.k[0]);  // Focal length X
            float fy = static_cast<float>(camera_info.k[4]);  // Focal length Y
            float cx = static_cast<float>(camera_info.k[2]);  // Principal point X
            float cy = static_cast<float>(camera_info.k[5]);  // Principal point Y

            K[0][0] = fx;
            K[1][1] = fy;
            K[0][2] = cx;
            K[1][2] = cy;
            K[2][2] = 1.0f;
        } else {
            LOG(ERROR) << "Invalid camera K matrix size: " << camera_info.k.size()
                       << " for keyframe " << keyframe->id;
            return torch::Tensor();  // Return invalid tensor
        }
    } else {
        // Get image dimensions for default camera parameters
        const auto& image = keyframe->getColorImage();
        float image_width = static_cast<float>(image.width);
        float image_height = static_cast<float>(image.height);

        // Default camera parameters based on image size
        float fx = image_width * 0.8f;  // Reasonable focal length
        float fy = image_height * 0.8f;
        float cx = image_width * 0.5f;
        float cy = image_height * 0.5f;

        K[0][0] = fx;
        K[1][1] = fy;
        K[0][2] = cx;
        K[1][2] = cy;
        K[2][2] = 1.0f;

        LOG(WARNING) << "Using default camera intrinsics for keyframe " << keyframe->id;
    }

    return K;
}

torch::Tensor KeyframeBatchLoader::convertImageToTensor(const core::types::Image& image) {
    // Convert image to OpenCV Mat
    cv::Mat cv_image = image.toCvMat();

    if (cv_image.empty()) {
        LOG(ERROR) << "Failed to convert image to cv::Mat";
        return torch::Tensor();
    }

    // Ensure RGB format
    if (cv_image.channels() == 3) {
        cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
    } else if (cv_image.channels() == 1) {
        cv::cvtColor(cv_image, cv_image, cv::COLOR_GRAY2RGB);
    } else {
        LOG(ERROR) << "Unsupported image format: " << cv_image.channels() << " channels";
        return torch::Tensor();
    }

    // Convert to float and normalize to [0, 1]
    cv_image.convertTo(cv_image, CV_32F, 1.0 / 255.0);

    // Convert to torch tensor [H, W, C] -> [C, H, W]
    torch::Tensor tensor =
        torch::from_blob(cv_image.data, {cv_image.rows, cv_image.cols, cv_image.channels()},
                         torch::dtype(torch::kFloat32));

    // Permute to [C, H, W] format
    tensor = tensor.permute({2, 0, 1}).contiguous();

    return tensor;
}

void KeyframeBatchLoader::updateKeyframeCache(uint64_t keyframe_id,
                                              core::types::KeyFrame::Ptr keyframe) {
    keyframe_cache_[keyframe_id] = keyframe;

    // Prune cache if it exceeds maximum size
    if (keyframe_cache_.size() > max_cache_size_) {
        pruneKeyframeCache();
    }
}

void KeyframeBatchLoader::pruneKeyframeCache() {
    // Simple LRU approximation - remove first half of cache
    auto it = keyframe_cache_.begin();
    size_t remove_count = keyframe_cache_.size() / 2;

    for (size_t i = 0; i < remove_count && it != keyframe_cache_.end(); ++i) {
        it = keyframe_cache_.erase(it);
    }

    LOG(INFO) << "Pruned keyframe cache, new size: " << keyframe_cache_.size();
}

// TrainingBatchIterator implementation
TrainingBatchIterator::TrainingBatchIterator(std::shared_ptr<KeyframeBatchLoader> loader)
    : loader_(loader), current_epoch_(0) {
    stats_ = {};
    stats_.total_epochs = 0;
    stats_.total_batches_processed = 0;
    stats_.current_epoch = 0;
    stats_.current_batch_in_epoch = 0;
    stats_.avg_batch_load_time_ms = 0.0;

    LOG(INFO) << "TrainingBatchIterator initialized";
}

bool TrainingBatchIterator::hasNext() const {
    return loader_->hasMoreBatches();
}

bool TrainingBatchIterator::getNext(KeyframeBatch& batch) {
    if (!hasNext()) {
        return false;
    }

    last_batch_start_ = std::chrono::high_resolution_clock::now();

    bool success = loader_->getNextBatch(batch);
    if (success) {
        stats_.total_batches_processed++;
        stats_.current_batch_in_epoch = loader_->getCurrentBatchIndex();
        updateLoadTime();
    }

    return success;
}

void TrainingBatchIterator::reset() {
    loader_->resetBatchIterator();
    stats_.current_batch_in_epoch = 0;
    LOG(INFO) << "TrainingBatchIterator reset to beginning of epoch";
}

void TrainingBatchIterator::startNewEpoch() {
    loader_->shuffleKeyframes();
    current_epoch_++;
    stats_.total_epochs = current_epoch_;
    stats_.current_epoch = current_epoch_;
    stats_.current_batch_in_epoch = 0;

    LOG(INFO) << "Started new training epoch " << current_epoch_;
}

size_t TrainingBatchIterator::getTotalBatches() const {
    return loader_->getTotalBatches();
}

size_t TrainingBatchIterator::getCurrentBatchInEpoch() const {
    return loader_->getCurrentBatchIndex();
}

void TrainingBatchIterator::updateLoadTime() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - last_batch_start_);

    // Update running average
    double new_time = static_cast<double>(duration.count());
    if (stats_.total_batches_processed == 1) {
        stats_.avg_batch_load_time_ms = new_time;
    } else {
        stats_.avg_batch_load_time_ms = (stats_.avg_batch_load_time_ms * 0.9) + (new_time * 0.1);
    }
}

}  // namespace training
}  // namespace gaussian_splatting
