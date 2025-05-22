#include <core/storage/map_store.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>
#include <logging/logging.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip>
#include <sstream>

struct KeypointVisualizationConfig {
    std::string output_directory = "/tmp/keypoint_visualizations";
    int max_images_per_row = 4;
    int image_scale_factor = 2;  // Scale down factor for display
    int min_detections = 3;      // Minimum detections to visualize a keypoint
    int max_keypoints = 50;      // Maximum keypoints to process
    cv::Scalar keypoint_color = cv::Scalar(0, 255, 0);     // Green
    cv::Scalar connection_color = cv::Scalar(255, 0, 0);   // Blue
    cv::Scalar text_color = cv::Scalar(255, 255, 255);     // White
    int circle_radius = 8;
    int line_thickness = 2;
    int font_scale = 1;
};

struct ImageDetection {
    uint64_t keyframe_id;
    cv::Point2f pixel_location;
    cv::Mat image;
    double timestamp;
    std::string frame_id;

    ImageDetection(uint64_t kf_id, const cv::Point2f& pixel, const cv::Mat& img,
                   double ts, const std::string& fid)
        : keyframe_id(kf_id), pixel_location(pixel), image(img), timestamp(ts), frame_id(fid) {}
};

class MapKeypointVisualizer {
private:
    core::storage::MapStore& map_store_;
    KeypointVisualizationConfig config_;

public:
    MapKeypointVisualizer(core::storage::MapStore& store, const KeypointVisualizationConfig& config)
        : map_store_(store), config_(config) {}

    bool visualizeAllMapKeypoints() {
        LOG(INFO) << "Starting map keypoint visualization";

        // Create output directory
        if (!std::filesystem::create_directories(config_.output_directory)) {
            if (!std::filesystem::exists(config_.output_directory)) {
                LOG(ERROR) << "Failed to create output directory: " << config_.output_directory;
                return false;
            }
        }

        // Get all map keypoints
        auto all_keypoints = map_store_.getAllKeyPoints();
        if (all_keypoints.empty()) {
            LOG(WARNING) << "No keypoints found in map store";
            return false;
        }

        LOG(INFO) << "Found " << all_keypoints.size() << " total keypoints in map store";

        // Filter keypoints by minimum detections
        std::vector<core::types::Keypoint> filtered_keypoints;
        for (const auto& keypoint : all_keypoints) {
            if (keypoint.locations.size() >= config_.min_detections) {
                filtered_keypoints.push_back(keypoint);
            }
        }

        LOG(INFO) << "Filtered to " << filtered_keypoints.size() << " keypoints with >= "
                  << config_.min_detections << " detections";

        // Sort by number of detections (most detected first)
        std::sort(filtered_keypoints.begin(), filtered_keypoints.end(),
                  [](const auto& a, const auto& b) {
                      return a.locations.size() > b.locations.size();
                  });

        // Limit to max keypoints for performance
        if (filtered_keypoints.size() > config_.max_keypoints) {
            filtered_keypoints.resize(config_.max_keypoints);
            LOG(INFO) << "Limited to " << config_.max_keypoints << " keypoints for performance";
        }

        // Process each keypoint
        int processed_count = 0;
        int success_count = 0;

        for (const auto& keypoint : filtered_keypoints) {
            processed_count++;
            LOG(INFO) << "Processing keypoint " << keypoint.id() << " (" << processed_count
                      << "/" << filtered_keypoints.size() << ") with "
                      << keypoint.locations.size() << " detections";

            if (visualizeSingleKeypoint(keypoint)) {
                success_count++;
            }
        }

        LOG(INFO) << "Keypoint visualization complete: " << success_count << "/"
                  << processed_count << " keypoints successfully visualized";
        LOG(INFO) << "Visualizations saved to: " << config_.output_directory;

        return success_count > 0;
    }

private:
    bool visualizeSingleKeypoint(const core::types::Keypoint& keypoint) {
        try {
            // Collect all detections for this keypoint
            std::vector<ImageDetection> detections;

            for (const auto& location : keypoint.locations) {
                // Get keyframe from map store
                auto keyframe = map_store_.getKeyFrame(location.keyframe_id);
                if (!keyframe) {
                    LOG(WARNING) << "Keyframe " << location.keyframe_id << " not found for keypoint "
                                 << keypoint.id();
                    continue;
                }

                // Check if keyframe has image data
                if (!keyframe->color_data.has_value()) {
                    LOG(WARNING) << "Keyframe " << location.keyframe_id << " has no image data";
                    continue;
                }

                // Convert image data to OpenCV Mat
                cv::Mat image = convertImageData(keyframe->color_data.value());
                if (image.empty()) {
                    LOG(WARNING) << "Failed to convert image data for keyframe "
                                 << location.keyframe_id;
                    continue;
                }

                // Create detection entry with properly scaled coordinates
                cv::Point2f pixel_pos(location.x, location.y);

                // If image was scaled in convertImageData, scale the pixel coordinates accordingly
                if (config_.image_scale_factor > 1) {
                    pixel_pos.x /= config_.image_scale_factor;
                    pixel_pos.y /= config_.image_scale_factor;
                }

                detections.emplace_back(location.keyframe_id, pixel_pos, image,
                                      keyframe->pose.timestamp, location.frame_id);
            }

            if (detections.empty()) {
                LOG(WARNING) << "No valid detections found for keypoint " << keypoint.id();
                return false;
            }

            // Sort detections by keyframe ID for consistent ordering
            std::sort(detections.begin(), detections.end(),
                      [](const auto& a, const auto& b) {
                          return a.keyframe_id < b.keyframe_id;
                      });

            // Create consolidated visualization
            cv::Mat visualization = createKeypointVisualization(keypoint, detections);
            if (visualization.empty()) {
                LOG(ERROR) << "Failed to create visualization for keypoint " << keypoint.id();
                return false;
            }

            // Save visualization
            std::string filename = "keypoint_" + std::to_string(keypoint.id()) +
                                 "_detections_" + std::to_string(detections.size()) + ".png";
            std::string filepath = config_.output_directory + "/" + filename;

            if (!cv::imwrite(filepath, visualization)) {
                LOG(ERROR) << "Failed to save visualization to " << filepath;
                return false;
            }

            LOG(INFO) << "Saved keypoint " << keypoint.id() << " visualization: " << filepath;
            return true;

        } catch (const std::exception& e) {
            LOG(ERROR) << "Exception processing keypoint " << keypoint.id() << ": " << e.what();
            return false;
        }
    }

    cv::Mat convertImageData(const core::types::Image& image_data) {
        try {
            cv::Mat image;

            if (image_data.encoding == "rgb8") {
                cv::Mat rgb_image = image_data.data.clone();
                cv::cvtColor(rgb_image, image, cv::COLOR_RGB2BGR);
            } else if (image_data.encoding == "bgr8") {
                image = image_data.data.clone();
            } else if (image_data.encoding == "mono8" || image_data.encoding == "8UC1") {
                cv::Mat gray_image = image_data.data.clone();
                cv::cvtColor(gray_image, image, cv::COLOR_GRAY2BGR);
            } else {
                LOG(WARNING) << "Unsupported image encoding: " << image_data.encoding;
                return cv::Mat();
            }

            // Scale down image if requested
            if (config_.image_scale_factor > 1) {
                cv::Mat scaled_image;
                cv::resize(image, scaled_image, cv::Size(),
                          1.0 / config_.image_scale_factor, 1.0 / config_.image_scale_factor);
                return scaled_image;
            }

            return image;

        } catch (const std::exception& e) {
            LOG(ERROR) << "Exception converting image data: " << e.what();
            return cv::Mat();
        }
    }

    cv::Mat createKeypointVisualization(const core::types::Keypoint& keypoint,
                                       const std::vector<ImageDetection>& detections) {
        if (detections.empty()) return cv::Mat();

        // Calculate layout dimensions
        int num_images = detections.size();
        int images_per_row = std::min(num_images, config_.max_images_per_row);
        int num_rows = (num_images + images_per_row - 1) / images_per_row;

        // Get image dimensions (images are already scaled in convertImageData)
        cv::Size image_size = detections[0].image.size();
        int scaled_width = image_size.width;
        int scaled_height = image_size.height;

        // Calculate canvas dimensions with margins
        int margin = 20;
        int text_height = 60;
        int canvas_width = images_per_row * scaled_width + (images_per_row + 1) * margin;
        int canvas_height = num_rows * (scaled_height + text_height) + (num_rows + 1) * margin + 100; // Extra space for title

        // Create canvas
        cv::Mat canvas = cv::Mat::zeros(canvas_height, canvas_width, CV_8UC3);
        canvas.setTo(cv::Scalar(50, 50, 50)); // Dark gray background

        // Add title
        std::string title = "Map Keypoint ID: " + std::to_string(keypoint.id()) +
                           " | Detections: " + std::to_string(detections.size()) +
                           " | 3D Position: [" + std::to_string(keypoint.position.x()) + ", " +
                           std::to_string(keypoint.position.y()) + ", " +
                           std::to_string(keypoint.position.z()) + "]";

        cv::Point title_pos(margin, 40);
        cv::putText(canvas, title, title_pos, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                   config_.text_color, 2);

        // Place images and draw keypoints
        std::vector<cv::Point> keypoint_centers;

        for (int i = 0; i < num_images; ++i) {
            int row = i / images_per_row;
            int col = i % images_per_row;

            int x_offset = margin + col * (scaled_width + margin);
            int y_offset = 80 + margin + row * (scaled_height + text_height + margin);

            // Place image (already scaled in convertImageData)
            const cv::Mat& scaled_image = detections[i].image;

            // Copy image to canvas
            cv::Rect roi(x_offset, y_offset, scaled_width, scaled_height);
            scaled_image.copyTo(canvas(roi));

            // Draw keypoint on image
            cv::Point2f scaled_pixel = detections[i].pixel_location;
            // Note: pixel coordinates are already scaled to match the image scale

            cv::Point keypoint_pos = cv::Point(x_offset + scaled_pixel.x, y_offset + scaled_pixel.y);
            cv::circle(canvas, keypoint_pos, config_.circle_radius, config_.keypoint_color, -1);
            cv::circle(canvas, keypoint_pos, config_.circle_radius + 2, cv::Scalar(0, 0, 0), 2);

            keypoint_centers.push_back(keypoint_pos);

            // Add image info text
            std::stringstream info_text;
            info_text << "KF:" << detections[i].keyframe_id
                     << " | TS:" << std::fixed << std::setprecision(2) << detections[i].timestamp
                     << " | Pixel:(" << std::setprecision(0) << detections[i].pixel_location.x
                     << "," << detections[i].pixel_location.y << ")";

            cv::Point text_pos(x_offset, y_offset + scaled_height + 20);
            cv::putText(canvas, info_text.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                       config_.text_color, 1);
        }

        // Draw connection lines between keypoints
        if (keypoint_centers.size() > 1) {
            for (size_t i = 0; i < keypoint_centers.size() - 1; ++i) {
                cv::line(canvas, keypoint_centers[i], keypoint_centers[i + 1],
                        config_.connection_color, config_.line_thickness);

                // Add small arrow at the end of each line to show direction
                cv::Point direction = keypoint_centers[i + 1] - keypoint_centers[i];
                double length = cv::norm(direction);
                if (length > 0) {
                    direction.x = static_cast<int>(direction.x * 10.0 / length);
                    direction.y = static_cast<int>(direction.y * 10.0 / length);

                    cv::Point arrow_tip = keypoint_centers[i + 1];
                    cv::Point arrow_p1 = arrow_tip - cv::Point(direction.x + direction.y/2,
                                                              direction.y - direction.x/2);
                    cv::Point arrow_p2 = arrow_tip - cv::Point(direction.x - direction.y/2,
                                                              direction.y + direction.x/2);

                    cv::line(canvas, arrow_tip, arrow_p1, config_.connection_color, 2);
                    cv::line(canvas, arrow_tip, arrow_p2, config_.connection_color, 2);
                }
            }
        }

        return canvas;
    }
};

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <map_base_path> [options]\n"
              << "\nArguments:\n"
              << "  map_base_path          Path to map store directory (contains .dat, .idx, .meta files)\n"
              << "\nOptions:\n"
              << "  -o, --output DIR       Output directory for visualizations (default: /tmp/keypoint_visualizations)\n"
              << "  -r, --rows N           Max images per row (default: 4)\n"
              << "  -s, --scale N          Image scale down factor (default: 2)\n"
              << "  -m, --min-detections N Minimum detections per keypoint (default: 3)\n"
              << "  -k, --max-keypoints N  Maximum keypoints to process (default: 50)\n"
              << "  -h, --help             Show this help message\n"
              << "\nExample:\n"
              << "  " << program_name << " /data/robots -o /tmp/vis -r 3 -s 2 -m 4\n";
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string map_base_path = argv[1];
    KeypointVisualizationConfig config;

    // Parse command line arguments
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];

        if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            config.output_directory = argv[++i];
        } else if ((arg == "-r" || arg == "--rows") && i + 1 < argc) {
            config.max_images_per_row = std::stoi(argv[++i]);
        } else if ((arg == "-s" || arg == "--scale") && i + 1 < argc) {
            config.image_scale_factor = std::stoi(argv[++i]);
        } else if ((arg == "-m" || arg == "--min-detections") && i + 1 < argc) {
            config.min_detections = std::stoi(argv[++i]);
        } else if ((arg == "-k" || arg == "--max-keypoints") && i + 1 < argc) {
            config.max_keypoints = std::stoi(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            LOG(WARNING) << "Unknown argument: " << arg;
        }
    }

    LOG(INFO) << "Map Keypoint Visualizer Starting";
    LOG(INFO) << "Map base path: " << map_base_path;
    LOG(INFO) << "Output directory: " << config.output_directory;
    LOG(INFO) << "Max images per row: " << config.max_images_per_row;
    LOG(INFO) << "Image scale factor: " << config.image_scale_factor;
    LOG(INFO) << "Min detections: " << config.min_detections;
    LOG(INFO) << "Max keypoints: " << config.max_keypoints;

    // Load map store
    core::storage::MapStore map_store(map_base_path);
    if (!map_store.loadMap()) {
        LOG(ERROR) << "Failed to load map from " << map_base_path;
        LOG(ERROR) << "Ensure the directory contains .dat, .idx, and .meta files";
        return 1;
    }

    // Create visualizer and process
    MapKeypointVisualizer visualizer(map_store, config);

    if (!visualizer.visualizeAllMapKeypoints()) {
        LOG(ERROR) << "Keypoint visualization failed";
        return 1;
    }

    LOG(INFO) << "Keypoint visualization completed successfully";
    return 0;
}