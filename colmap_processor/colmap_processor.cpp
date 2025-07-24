#include <glog/logging.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rmw/rmw.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// A simple struct to hold camera intrinsics
struct CameraIntrinsics {
    std::string camera_model;
    double fx, fy, cx, cy;
    uint32_t width, height;
};

class ColmapDataExtractor {
public:
    ColmapDataExtractor(const std::string& bag_path, const std::string& image_topic,
                        const std::string& camera_info_topic, const std::string& output_dir)
        : bag_path_(bag_path),
          image_topic_(image_topic),
          camera_info_topic_(camera_info_topic),
          temp_dir_(output_dir) {
        images_dir_ = temp_dir_ / "images";
    }

    bool extractData() {
        if (!setupDirectories()) {
            return false;
        }

        rosbag2_cpp::Reader reader;
        try {
            rosbag2_storage::StorageOptions storage_options({bag_path_, "sqlite3"});
            rosbag2_cpp::ConverterOptions converter_options(
                {rmw_get_serialization_format(), rmw_get_serialization_format()});
            reader.open(storage_options, converter_options);
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to open rosbag: " << e.what();
            return false;
        }

        auto metadata = reader.get_metadata();
        uint32_t total_messages = 0;
        for (const auto& topic : metadata.topics_with_message_count) {
            if (topic.topic_metadata.name == image_topic_) {
                LOG(INFO) << " - " << topic.topic_metadata.name << " (" << topic.message_count
                          << " messages, type: " << topic.topic_metadata.type << ")";
                total_messages = topic.message_count;
            }
        }

        if (total_messages == 0) {
            LOG(ERROR) << "There must be some messages available";
            return false;
        }

        LOG(INFO) << "Reading rosbag: " << bag_path_;
        LOG(INFO) << "Image topic: " << image_topic_;
        LOG(INFO) << "Camera info topic: " << camera_info_topic_;

        uint32_t process_every = static_cast<uint32_t>(total_messages / 500);
        uint32_t current_read_count = 0;
        while (reader.has_next()) {
            auto msg = reader.read_next();
            current_read_count++;
            if (current_read_count % process_every != 0) {
                continue;
            }

            if (msg->topic_name == camera_info_topic_) {
                processCameraInfo(msg);
            } else if (msg->topic_name == image_topic_) {
                processImage(msg);
            }
        }

        if (!intrinsics_written_) {
            LOG(WARNING) << "Camera info was not found or processed. Cannot create camera config "
                            "for COLMAP.";
            return false;
        }

        LOG(INFO) << "Finished processing rosbag.";
        LOG(INFO) << "Extracted " << image_count_ << " images to " << images_dir_;
        LOG(INFO) << "Wrote camera intrinsics to " << temp_dir_ / "cameras.txt";

        return true;
    }

private:
    bool setupDirectories() {
        try {
            if (!std::filesystem::exists(temp_dir_)) {
                std::filesystem::create_directories(images_dir_);
                LOG(INFO) << "Created temporary directory: " << temp_dir_;
            } else {
                LOG(INFO) << "Output directory already exists: " << temp_dir_;
            }
        } catch (const std::filesystem::filesystem_error& e) {
            LOG(ERROR) << "Failed to create directories: " << e.what();
            return false;
        }
        return true;
    }

    template <typename T>
    void deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_msg,
                     T& ros_msg) {
        rclcpp::Serialization<T> serialization;
        rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
        serialization.deserialize_message(&serialized_msg, &ros_msg);
    }

    void processCameraInfo(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg) {
        if (intrinsics_written_)
            return;

        sensor_msgs::msg::CameraInfo camera_info_msg;
        deserialize(msg, camera_info_msg);

        CameraIntrinsics intrinsics;
        intrinsics.width = camera_info_msg.width;
        intrinsics.height = camera_info_msg.height;
        intrinsics.fx = camera_info_msg.k[0];
        intrinsics.fy = camera_info_msg.k[4];
        intrinsics.cx = camera_info_msg.k[2];
        intrinsics.cy = camera_info_msg.k[5];

        intrinsics.camera_model = "PINHOLE";

        writeIntrinsicsToFile(intrinsics);
        intrinsics_written_ = true;
    }

    void processImage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg) {
        sensor_msgs::msg::Image image_msg;
        deserialize(msg, image_msg);

        cv::Mat image;
        try {
            cv::Mat temp_image(image_msg.height, image_msg.width, CV_8UC3, image_msg.data.data());
            image = temp_image.clone();
        } catch (const std::exception& e) {
            LOG(ERROR) << "OpenCV conversion error: " << e.what();
            return;
        }

        std::string image_filename = "frame_" + std::to_string(image_count_++) + ".png";
        std::filesystem::path image_path = images_dir_ / image_filename;

        if (!cv::imwrite(image_path.string(), image)) {
            LOG(ERROR) << "Failed to write image: " << image_path;
        }
    }

    void writeIntrinsicsToFile(const CameraIntrinsics& intrinsics) {
        std::ofstream file(temp_dir_ / "cameras.txt");
        if (!file.is_open()) {
            LOG(ERROR) << "Failed to open cameras.txt for writing.";
            return;
        }
        file << "1 " << intrinsics.camera_model << " " << intrinsics.width << " "
             << intrinsics.height << " " << intrinsics.fx << " " << intrinsics.fy << " "
             << intrinsics.cx << " " << intrinsics.cy << std::endl;
        LOG(INFO) << "Wrote intrinsics to cameras.txt";
    }

    std::string bag_path_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::filesystem::path temp_dir_;
    std::filesystem::path images_dir_;
    bool intrinsics_written_ = false;
    int image_count_ = 0;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    if (argc != 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <path-to-rosbag> <image-topic> <camera-info-topic> <output-dir>"
                  << std::endl;
        return 1;
    }

    std::string bag_path = argv[1];
    std::string image_topic = argv[2];
    std::string camera_info_topic = argv[3];
    std::string output_dir = argv[4];

    ColmapDataExtractor extractor(bag_path, image_topic, camera_info_topic, output_dir);

    if (extractor.extractData()) {
        LOG(INFO) << "Data extraction successful.";
    } else {
        LOG(ERROR) << "Data extraction failed.";
        return 1;
    }

    return 0;
}
