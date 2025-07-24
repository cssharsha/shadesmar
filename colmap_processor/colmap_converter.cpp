#include "colmap_converter.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include "core/types/keyframe.hpp"
#include "core/types/keypoint.hpp"
#include "utils/stf/transform_tree.hpp"

namespace gs {

ColmapConverter::ColmapConverter(const std::string& colmap_path, const std::string& map_path)
    : colmap_path_(colmap_path),
      map_path_(map_path),
      map_store_(map_path, core::storage::ProcessRole::DUAL) {
    reconstruction_.ReadBinary(colmap_path_);
    map_store_.syncIndexFromDisk();
}

ColmapConverter::ColmapConverter(const std::string& colmap_path)
    : colmap_path_(colmap_path),
      map_path_(colmap_path),
      map_store_(colmap_path_ + "/map/map", core::storage::ProcessRole::DUAL) {
    auto T_R_C = Eigen::Isometry3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d(0, 0, 0);
    Eigen::AngleAxisd r(-M_PI / 2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd p(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd y(-M_PI / 2, Eigen::Vector3d::UnitZ());
    T_R_C.linear() = Eigen::Matrix3d(r * p * y);
    T_R_C.translation() = t;
    T_R_C_.orientation = Eigen::Quaterniond(T_R_C.rotation());
    T_R_C_.position = T_R_C.translation();
    LOG(INFO) << "T_R_C: " << T_R_C.matrix();
    reconstruction_.ReadBinary(colmap_path_);
    LOG(INFO) << "Converter to start new map from: " << colmap_path_;
    map_store_.syncIndexFromDisk();
    map_store_.printFilePaths();
}

void ColmapConverter::writeDebugPoseJson(const std::string& stage, uint64_t image_id,
                                         const Eigen::Quaterniond& q_xyzw, const Eigen::Vector3d& t,
                                         const Eigen::Matrix3d& R_world_to_cam,
                                         const Eigen::Isometry3d& world_T_cam,
                                         const Eigen::Isometry3d& cam_from_world) {
    namespace fs = std::filesystem;
    try {
        fs::path debug_dir = fs::path(colmap_path_) / "debug";
        fs::create_directories(debug_dir);
        fs::path out_path = debug_dir / (stage + "_" + std::to_string(image_id) + ".json");
        std::ofstream out(out_path);
        if (!out) {
            LOG(ERROR) << "Failed to open debug JSON: " << out_path;
            return;
        }
        auto write_mat3 = [&](const Eigen::Matrix3d& M) {
            out << "[\n";
            for (int i = 0; i < 3; ++i) {
                out << "    [" << std::setprecision(12) << M(i, 0) << ", " << M(i, 1) << ", "
                    << M(i, 2) << "]" << (i < 2 ? "," : "") << "\n";
            }
            out << "  ]";
        };
        auto write_vec3 = [&](const Eigen::Vector3d& v) {
            out << "[" << std::setprecision(12) << v.x() << ", " << v.y() << ", " << v.z() << "]";
        };

        out << std::fixed;
        out << "{\n";
        out << "  \"stage\": \"" << stage << "\",\n";
        out << "  \"image_id\": " << image_id << ",\n";
        out << "  \"input\": {\n";
        out << "    \"quaternion_xyzw\": [" << q_xyzw.x() << ", " << q_xyzw.y() << ", "
            << q_xyzw.z() << ", " << q_xyzw.w() << "],\n";
        out << "    \"translation\": ";
        write_vec3(t);
        out << "\n  },\n";
        out << "  \"derived\": {\n";
        out << "    \"R_world_to_cam\": ";
        write_mat3(R_world_to_cam);
        out << ",\n";
        out << "    \"world_T_cam\": {\n";
        out << "      \"R\": ";
        write_mat3(world_T_cam.rotation());
        out << ",\n      \"t\": ";
        write_vec3(world_T_cam.translation());
        out << "\n    },\n";
        out << "    \"cam_from_world\": {\n";
        out << "      \"R\": ";
        write_mat3(cam_from_world.rotation());
        out << ",\n      \"t\": ";
        write_vec3(cam_from_world.translation());
        out << "\n    }\n  }\n}";

        out.close();
        LOG(INFO) << "Wrote debug pose JSON: " << out_path;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception writing debug JSON: " << e.what();
    }
}

void ColmapConverter::debugStartImagesAggregate() {
    debug_images_agg_.str("");
    debug_images_agg_.clear();
    debug_images_first_ = true;
    debug_images_agg_open_ = true;
    debug_images_agg_ << "{\n  \"stage\": \"parse_image_file\",\n  \"entries\": [\n";
}

void ColmapConverter::debugAppendImageEntry(uint64_t image_id, const Eigen::Quaterniond& q_xyzw,
                                            const Eigen::Vector3d& t,
                                            const Eigen::Matrix3d& R_world_to_cam,
                                            const Eigen::Isometry3d& world_T_cam,
                                            const Eigen::Isometry3d& cam_from_world) {
    if (!debug_images_agg_open_) {
        debugStartImagesAggregate();
    }
    auto write_mat3 = [&](const Eigen::Matrix3d& M) {
        debug_images_agg_ << "[\n";
        for (int i = 0; i < 3; ++i) {
            debug_images_agg_ << "          [" << std::setprecision(12) << M(i, 0) << ", "
                              << M(i, 1) << ", " << M(i, 2) << "]" << (i < 2 ? "," : "") << "\n";
        }
        debug_images_agg_ << "        ]";
    };
    auto write_vec3 = [&](const Eigen::Vector3d& v) {
        debug_images_agg_ << "[" << std::setprecision(12) << v.x() << ", " << v.y() << ", " << v.z()
                          << "]";
    };

    if (!debug_images_first_) {
        debug_images_agg_ << ",\n";
    }
    debug_images_first_ = false;

    debug_images_agg_ << "    {\n";
    debug_images_agg_ << "      \"image_id\": " << image_id << ",\n";
    debug_images_agg_ << "      \"input\": {\n";
    debug_images_agg_ << "        \"quaternion_xyzw\": [" << q_xyzw.x() << ", " << q_xyzw.y()
                      << ", " << q_xyzw.z() << ", " << q_xyzw.w() << "],\n";
    debug_images_agg_ << "        \"translation\": ";
    write_vec3(t);
    debug_images_agg_ << "\n      },\n";
    debug_images_agg_ << "      \"derived\": {\n";
    debug_images_agg_ << "        \"R_world_to_cam\": ";
    write_mat3(R_world_to_cam);
    debug_images_agg_ << ",\n";
    debug_images_agg_ << "        \"world_T_cam\": {\n";
    debug_images_agg_ << "          \"R\": ";
    write_mat3(world_T_cam.rotation());
    debug_images_agg_ << ",\n          \"t\": ";
    write_vec3(world_T_cam.translation());
    debug_images_agg_ << "\n        },\n";
    debug_images_agg_ << "        \"cam_from_world\": {\n";
    debug_images_agg_ << "          \"R\": ";
    write_mat3(cam_from_world.rotation());
    debug_images_agg_ << ",\n          \"t\": ";
    write_vec3(cam_from_world.translation());
    debug_images_agg_ << "\n        }\n";
    debug_images_agg_ << "      }\n";
    debug_images_agg_ << "    }";
}

void ColmapConverter::debugFinishImagesAggregate() {
    if (!debug_images_agg_open_)
        return;
    debug_images_agg_ << "\n  ]\n}\n";
    debug_images_agg_open_ = false;

    namespace fs = std::filesystem;
    try {
        fs::path debug_dir = fs::path(colmap_path_) / "debug";
        fs::create_directories(debug_dir);
        fs::path out_path = debug_dir / "parse_image_file.json";
        std::ofstream out(out_path);
        if (!out) {
            LOG(ERROR) << "Failed to open aggregate debug JSON: " << out_path;
            return;
        }
        out << debug_images_agg_.str();
        out.close();
        LOG(INFO) << "Wrote aggregate image parsing debug JSON: " << out_path;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception writing aggregate debug JSON: " << e.what();
    }
}

bool ColmapConverter::convertImageToKeyframe(const uint32_t& image_id,
                                             core::types::KeyFrame::Ptr& keyframe) {
    keyframe = std::make_shared<core::types::KeyFrame>();
    const auto& image = reconstruction_.Image(image_id);
    LOG(INFO) << "Converting image " << image.Name() << " to keyframe";
    keyframe->id = image.ImageId();
    LOG(INFO) << "Image ID: " << image.ImageId() << "keyframe ID: " << keyframe->id;
    LOG(INFO) << "Image has pose: " << image.HasPose();
    auto& pose = keyframe->pose;
    if (!image.HasPose()) {
        LOG(WARNING) << "Image " << image.Name() << " has no pose";
        return false;
    } else {
        LOG(INFO) << "Image has pose";
    }
    if (!image.HasFramePtr()) {
        LOG(WARNING) << "Image " << image.Name() << " has no frame";
        return false;
    } else {
        LOG(INFO) << "Image has frame: " << image.FrameId();
    }
    // Should have got from this directly but throws some sensor comparison
    // error
    // const colmap::Rigid3d& cam_from_world = image.CamFromWorld();
    auto& cam_from_world = image.FramePtr()->RigFromWorld();
    pose.position = cam_from_world.translation;
    pose.orientation = cam_from_world.rotation;
    LOG(INFO) << "Pose: " << pose.position.transpose() << " with timestamp: " << pose.timestamp;
    // populate dummy timestamp
    keyframe->pose.timestamp = 12345.;
    // Should just be one camera so set the frame to colmap something
    keyframe->pose.frame_id = "colmap_camera";
    LOG(INFO) << "Pose: " << pose.position.transpose() << " with timestamp: " << pose.timestamp;

    const auto& camera = reconstruction_.Camera(image.CameraId());
    core::types::CameraInfo camera_info;
    camera_info.frame_id = image.Name();
    camera_info.width = camera.width;
    camera_info.height = camera.height;
    camera_info.distortion_model = colmap::CameraModelIdToName(camera.model_id);
    // Check if colmap stores it in row major later
    // The camera params in colmap has 4 values which needs to be converted to
    // a 3x3 matrix and copied to a row major vector
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0, 0) = camera.params[0];
    K(1, 1) = camera.params[1];
    K(0, 2) = camera.params[2];
    K(1, 2) = camera.params[3];
    camera_info.k = std::vector<double>(K.data(), K.data() + K.size());
    // camera_info.k = camera.params;
    LOG(INFO) << "Camera info: " << camera_info.width << "x" << camera_info.height
              << " distortion model: " << camera_info.distortion_model
              << " k: " << camera_info.k.size();

    auto image_path = colmap_path_ + "/images/" + image.Name();
    cv::Mat test_image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (test_image.empty()) {
        LOG(ERROR) << "Failed to load image: " << image.Name() << " from " << image_path;
        return false;
    }
    core::types::Image img =
        core::types::Image::fromCvMat(test_image, "bgr8", keyframe->pose.frame_id);
    keyframe->color_data = img;
    keyframe->camera_info = camera_info;

    // Debug: dump pose from this stage
    {
        Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
        Tcw.rotate(cam_from_world.rotation);
        Tcw.pretranslate(cam_from_world.translation);
        Eigen::Isometry3d Twc = Tcw.inverse();
        writeDebugPoseJson("convert_image_to_keyframe", image.ImageId(), cam_from_world.rotation,
                           cam_from_world.translation, cam_from_world.rotation.toRotationMatrix(),
                           Twc, Tcw);
    }

    return true;
}

bool ColmapConverter::parseCameraFile(const std::string& camera_file,
                                      core::types::CameraInfo& camera_info) {
    std::ifstream cameras_file(colmap_path_ + "/text/cameras.txt");
    if (!cameras_file.is_open()) {
        LOG(ERROR) << "Failed to open cameras.txt for reading.";
        return false;
    }
    std::string line;
    if (!std::getline(cameras_file, line)) {
        LOG(ERROR) << "Failed to read first line of cameras.txt";
        return false;
    }
    std::cout << "Line: " << line << std::endl;
    if (!std::getline(cameras_file, line)) {
        LOG(ERROR) << "Failed to read second line of cameras.txt";
        return false;
    }
    std::cout << "Line: " << line << std::endl;

    if (!std::getline(cameras_file, line)) {
        LOG(ERROR) << "Failed to read third line of cameras.txt";
        return false;
    }
    std::cout << "Line: " << line << std::endl;
    if (!std::getline(cameras_file, line)) {
        LOG(ERROR) << "Failed to read third line of cameras.txt";
        return false;
    }
    std::cout << "Line: " << line << std::endl;
    std::stringstream line_stream(line);
    parseCameraInfo(line_stream, camera_info);
    std::cout << "Camera info: " << camera_info.width << "x" << camera_info.height
              << " distortion model: " << camera_info.distortion_model
              << " k: " << camera_info.k.size() << std::endl;
    std::cout << "Camera info: k: \n" << camera_info.getKInEigen() << std::endl;
    if (!camera_info.d.empty()) {
        std::cout << "Camera info: " << camera_info.d.size() << std::endl;
        for (const auto& d : camera_info.d) {
            std::cout << "Distortion coeff: " << d << std::endl;
        }
    }
    return true;
}

bool ColmapConverter::parseCameraInfo(std::stringstream& line,
                                      core::types::CameraInfo& camera_info) {
    // Split line into values
    std::vector<std::string> cam_info;
    std::string val_str;

    // Camera list with one line of data per camera:
    //   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
    // For simple radisl the params are f, cx, cy, k (k is radial distortion coeff)
    // Split the line into values with delimiter ' '
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse camera info line";
        return false;
    }
    std::cout << "Current read string: " << val_str << std::endl;
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse camera model ";
        return false;
    }
    std::cout << "Current read string: " << val_str << std::endl;
    camera_info.distortion_model = val_str;

    if (val_str == "SIMPLE_RADIAL") {
        // Simple radial model
        if (!std::getline(line, val_str, ' ')) {
            LOG(ERROR) << "Failed to parse camera info line ";
            return false;
        }
        std::cout << "Current read string: " << val_str << std::endl;
        camera_info.width = std::stoi(val_str);
        if (!std::getline(line, val_str, ' ')) {
            LOG(ERROR) << "Failed to parse height ";
            return false;
        }
        std::cout << "Current read string: " << val_str << std::endl;
        camera_info.height = std::stoi(val_str);
        if (!std::getline(line, val_str, ' ')) {
            LOG(ERROR) << "Failed to parse focal length ";
            return false;
        }
        std::cout << "Current read string: " << val_str << std::endl;
        camera_info.k = std::vector<double>(9, 0.0);
        camera_info.k.at(0) = std::stod(val_str);
        camera_info.k.at(4) = std::stod(val_str);

        if (!std::getline(line, val_str, ' ')) {
            LOG(ERROR) << "Failed to parse cx ";
            return false;
        }
        std::cout << "Current read string: " << val_str << std::endl;
        camera_info.k.at(2) = std::stod(val_str);
        if (!std::getline(line, val_str, ' ')) {
            LOG(ERROR) << "Failed to parse cy: ";
            return false;
        }
        std::cout << "Current read string: " << val_str << std::endl;
        camera_info.k.at(5) = std::stod(val_str);
        if (!std::getline(line, val_str, ' ')) {
            LOG(ERROR) << "Failed to parse distortion coeffs ";
            return false;
        }
        std::cout << "Current read string: " << val_str << std::endl;
        camera_info.d.push_back(std::stod(val_str));
    } else {
        LOG(ERROR) << "Unsupported camera model: ";
        return false;
    }

    return true;
}

bool ColmapConverter::parseImageLine(std::stringstream& line,
                                     core::types::KeyFrame::Ptr& keyframe) {
    // Image list with two lines of data per image:
    //   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    //   POINTS2D[] as (X, Y, POINT3D_ID)
    std::string val_str;

    // Image ID
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    uint64_t image_id = std::stoi(val_str);

    // Read rotation
    Eigen::Quaterniond q;
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    q.w() = std::stod(val_str);
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    q.x() = std::stod(val_str);
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    q.y() = std::stod(val_str);
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    q.z() = std::stod(val_str);

    // Read translation
    Eigen::Vector3d t;
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    t.x() = std::stod(val_str);
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    t.y() = std::stod(val_str);
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    t.z() = std::stod(val_str);
    Eigen::Isometry3d cam_from_world = Eigen::Isometry3d::Identity();
    cam_from_world.rotate(q);
    // cam_from_world.linear() = q.toRotationMatrix();
    cam_from_world.pretranslate(t);
    // cam_from_world.translation() = t;
    auto world_T_cam = cam_from_world.inverse();
    // world_T_cam = T_R_C_.getEigenIsometry() * world_T_cam;

    auto R_world_to_cam = q.toRotationMatrix();
    auto t_cam_in_world = -R_world_to_cam.transpose() * t;
    auto T_cam_in_world = Eigen::Isometry3d::Identity();
    T_cam_in_world.rotate(q);
    T_cam_in_world.pretranslate(t);
    std::cout << "Computed from manual math: \n" << T_cam_in_world.matrix() << std::endl;
    std::cout << "Computed from inverse:\n" << world_T_cam.matrix() << std::endl;

    // Debug aggregate: append this entry (images.txt parsing)
    debugAppendImageEntry(image_id, q, t, R_world_to_cam, world_T_cam, cam_from_world);
    // 21 Eigen::Matrix3d R_world_to_cam = q_world_to_cam.toRotationMatrix();
    // 22 23   // 1. GET ROTATION IN WORLD FRAME
    //     24  // The camera's orientation in the world is the transpose (inverse)
    //     25  // of the original world-to-camera rotation matrix.
    //     26 Eigen::Matrix3d R_cam_in_world = R_world_to_cam.transpose();
    // 27 28   // 2. GET POSITION IN WORLD FRAME
    //     29  // The camera's position in the world is C = -R' * t
    //     30 Eigen::Vector3d t_cam_in_world = -R_world_to_cam.transpose() * t_world_to_cam;

    core::types::Pose pose;
    // pose.orientation = Eigen::Quaterniond(T_cam_in_world.rotation());
    pose.orientation = Eigen::Quaterniond(world_T_cam.rotation());  // Same as above
    // pose.position = T_cam_in_world.translation();
    pose.position = world_T_cam.translation();

    pose.frame_id = "colmap_camera";
    pose.timestamp = 0.0;

    std::cout << "Pose: " << pose.position.transpose() << " with timestamp: " << pose.timestamp
              << std::endl;

    // Skip the camera ID
    std::getline(line, val_str, ' ');
    // Read image name
    if (!std::getline(line, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse image line: ";
        return false;
    }
    std::string image_path = colmap_path_ + "/images/" + val_str;
    std::cout << "Image path: " << image_path << std::endl;
    // Read image
    cv::Mat test_image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (test_image.empty()) {
        LOG(ERROR) << "Failed to load image from " << image_path;
        return false;
    }
    core::types::Image img = core::types::Image::fromCvMat(test_image, "bgr8", pose.frame_id);

    keyframe = std::make_shared<core::types::KeyFrame>();
    keyframe->id = image_id;
    keyframe->pose = pose;
    keyframe->color_data = img;

    return true;
}

bool ColmapConverter::parseImageFile(
    const std::string& image_file, const core::types::CameraInfo& camera_info,
    std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframes) {
    std::ifstream image_file_stream(image_file);
    std::cout << "Image file stream is open: " << image_file_stream.is_open() << std::endl;
    if (!image_file_stream.is_open()) {
        LOG(ERROR) << "Failed to open image file for reading: " << image_file;
        return false;
    }
    std::string line;
    // Skip the first four lines
    std::getline(image_file_stream, line);
    std::getline(image_file_stream, line);
    std::getline(image_file_stream, line);
    std::getline(image_file_stream, line);
    debugStartImagesAggregate();
    while (std::getline(image_file_stream, line)) {
        std::cout << "Read line: " << line << std::endl;
        std::stringstream line_stream(line);
        core::types::KeyFrame::Ptr kf;
        if (!parseImageLine(line_stream, kf)) {
            LOG(ERROR) << "Failed to parse image line: ";
            return false;
        }
        kf->camera_info = camera_info;
        map_store_.addKeyFrame(kf);

        LOG(INFO) << "Parsed image: " << kf->id << " with pose: " << kf->pose.position.transpose()
                  << " with timestamp: " << kf->pose.timestamp;
        keyframes[kf->id] = std::move(kf);
        std::cout << "Added keyframe to map: " << keyframes.size() << std::endl;
        // Skip the next line since it contains points
        std::getline(image_file_stream, line);
        // std::cout << "Read line: " << line << std::endl;
    }
    std::cout << "Finished reading images" << std::endl;
    debugFinishImagesAggregate();
    return true;
}

bool ColmapConverter::parsePointLine(std::stringstream& point_file) {
    // 3D point list with one line of data per point:
    //   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
    std::string val_str;
    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        return false;
    }
    core::types::Keypoint keypoint(std::stoi(val_str));
    Eigen::Vector3d point3D;

    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        return false;
    }
    point3D.x() = std::stod(val_str);
    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        return false;
    }
    point3D.y() = std::stod(val_str);
    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        return false;
    }
    point3D.z() = std::stod(val_str);

    // The frame of colmap is x is right, y is down, z is forward,
    // but the pointcloud is x is forward, y is right, z is up.
    keypoint.position = point3D;
    // keypoint.position = T_R_C_.getEigenIsometry() * point3D;

    // Read color
    // Converting Eigen::Vector3ub to cv::Mat
    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        val_str = "0";
    }
    keypoint.color.x() = std::stoi(val_str);
    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        val_str = "0";
    }
    keypoint.color.y() = std::stoi(val_str);
    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        val_str = "0";
    }
    keypoint.color.z() = std::stoi(val_str);
    std::cout << "Keypoint Color: " << keypoint.color.transpose() << std::endl;

    // Read error - not used
    if (!std::getline(point_file, val_str, ' ')) {
        LOG(ERROR) << "Failed to parse point line: " << val_str;
        val_str = "0";
    }

    // Read location (or as colmap says track)
    core::types::Location loc;
    while (std::getline(point_file, val_str, ' ')) {
        loc.keyframe_id = std::stoi(val_str);

        loc.frame_id = "colmap_camera";

        if (!std::getline(point_file, val_str, ' ')) {
            LOG(ERROR) << "Failed to parse point line: ";
            return false;
        }
        auto point2D_idx = std::stoi(val_str);
        // The point2D_idx is the zero based index of the point in the image but
        // in order to get the correct pixel you'd have to read from the image.txt
        // second line and then match it with here which all seeemed a tiny bit
        // convoluted, so Im just using the recnstruction_ to get the pixel
        auto pixel = reconstruction_.Image(loc.keyframe_id).Point2D(point2D_idx);
        loc.x = pixel.xy.x();
        loc.y = pixel.xy.y();

        std::cout << "Added location: " << loc.keyframe_id << " " << loc.frame_id << " " << loc.x
                  << " " << loc.y << std::endl;
    }
    std::cout << "Keypoint: " << keypoint.id() << " has " << keypoint.locations.size()
              << " locations" << std::endl;
    map_store_.addKeyPoint(keypoint);

    return true;
}

bool ColmapConverter::parsePointFile(const std::string& point_file) {
    std::ifstream point_file_stream(point_file);
    if (!point_file_stream.is_open()) {
        LOG(ERROR) << "Failed to open point file for reading: ";
        return false;
    }
    std::string line;
    // skiip the first three lines
    std::getline(point_file_stream, line);
    std::getline(point_file_stream, line);
    std::getline(point_file_stream, line);
    while (std::getline(point_file_stream, line)) {
        std::stringstream line_stream(line);
        if (!parsePointLine(line_stream)) {
            LOG(ERROR) << "Failed to parse point line: ";
            return false;
        }
    }
    map_store_.saveChanges();
    return true;
}

bool ColmapConverter::convertFromText() {
    auto text_path = colmap_path_ + "/text";
    if (!std::filesystem::exists(text_path)) {
        LOG(ERROR) << "Text path does not exist: " << text_path;
        return false;
    }

    // There should be just one camera in the cameras.txt file in this example
    core::types::CameraInfo camera_info;
    if (!parseCameraFile(text_path + "/cameras.txt", camera_info)) {
        LOG(ERROR) << "Failed to parse cameras.txt";
        return false;
    }
    std::cout << "Camera info: " << camera_info.width << "x" << camera_info.height
              << " distortion model: " << camera_info.distortion_model
              << " k: " << camera_info.k.size() << std::endl;

    // Reading all the images as keyframes
    auto image_path = text_path + "/images.txt";
    if (!std::filesystem::exists(image_path)) {
        LOG(ERROR) << "Image path does not exist: " << image_path;
        return false;
    }
    std::unordered_map<uint64_t, core::types::KeyFrame::Ptr> keyframes;
    if (!parseImageFile(image_path, camera_info, keyframes)) {
        LOG(ERROR) << "Failed to parse image file";
        return false;
    }
    std::cout << "Ding ding" << std::endl;
    std::cout << "Parsed " << keyframes.size() << " keyframes" << std::endl;
    map_store_.saveChanges();

    // Read points
    auto point_path = text_path + "/points3D.txt";
    if (!std::filesystem::exists(point_path)) {
        LOG(ERROR) << "Point path does not exist: " << point_path;
        return false;
    }
    if (!parsePointFile(point_path)) {
        LOG(ERROR) << "Failed to parse point file";
        return false;
    }

    return true;
}

bool ColmapConverter::convert() {
    LOG(INFO) << "Converting from COLMAP to map from: " << colmap_path_ << " with the\n"
              << " Num of rigs: " << reconstruction_.NumRigs() << "\n"
              << " Num of cameras: " << reconstruction_.NumCameras() << "\n"
              << "Num of frames: " << reconstruction_.NumFrames() << "\n"
              << " Num of images: " << reconstruction_.NumImages() << "\n"
              << " Num of points: " << reconstruction_.NumPoints3D();
    reconstruction_.WriteText(colmap_path_ + "/text/");
    auto all_keypoints_before_conversion = map_store_.getAllKeyPoints();
    LOG(INFO) << "Total keypoints before conversion: " << all_keypoints_before_conversion.size();
    std::unordered_map<uint64_t, core::types::KeyFrame::Ptr> keyframes;

    for (const auto& rgbpoint : reconstruction_.Points3D()) {
        const auto& point = rgbpoint.second;
        LOG(INFO) << "Point: " << rgbpoint.first << " Pos: " << point.xyz.transpose() << " RGB"
                  << point.color.transpose();
        core::types::Keypoint keypoint(rgbpoint.first);
        keypoint.position = point.xyz;
        // Converting Eigen::Vector3ub to cv::Mat
        cv::Mat rgb_descriptor = cv::Mat::zeros(1, 3, CV_8UC1);
        rgb_descriptor.at<uint8_t>(0, 0) = point.color[0];
        rgb_descriptor.at<uint8_t>(0, 1) = point.color[1];
        rgb_descriptor.at<uint8_t>(0, 2) = point.color[2];
        keypoint.descriptor = rgb_descriptor;
        for (const auto& element : point.track.Elements()) {
            auto pixel = reconstruction_.Image(element.image_id).Point2D(element.point2D_idx);
            core::types::Location loc{element.image_id, "camera", static_cast<float>(pixel.xy.x()),
                                      static_cast<float>(pixel.xy.y())};
            if (keyframes.find(element.image_id) == keyframes.end()) {
                LOG(INFO) << "Adding a new keyframe";
                core::types::KeyFrame::Ptr kf;
                if (!convertImageToKeyframe(element.image_id, kf)) {
                    LOG(ERROR) << "Failed to convert image " << element.image_id;
                    continue;
                }
                keyframes[element.image_id] = kf;
                LOG(INFO) << "Added keyframe " << kf->id
                          << " with pose: " << kf->pose.position.transpose()
                          << " with timestamp: " << kf->pose.timestamp
                          << " frame: " << kf->pose.frame_id
                          << " image: " << kf->color_data->data.size()
                          << " camera info: " << kf->camera_info->k.size();

                map_store_.addKeyFrame(kf);
            }
            keypoint.locations.emplace_back(std::move(loc));
        }
        if (keypoint.locations.size() == 0) {
            LOG(WARNING) << "No locations for keypoint " << keypoint.id();
            continue;
        }
        keypoint.needs_triangulation = false;
        map_store_.addKeyPoint(keypoint);
    }
    map_store_.saveChanges();

    auto all_keypoints = map_store_.getAllKeyPoints();
    LOG(INFO) << "Total keypoints: " << all_keypoints.size();

    return true;
}

}  // namespace gs
