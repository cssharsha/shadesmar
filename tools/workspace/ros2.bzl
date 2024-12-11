load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("//tools/workspace:ros2/deps.bzl", "ros2_deps_installer")

# List of required ROS2 packages with their versions and system dependencies
ROS2_PACKAGES = {
    "common_interfaces": {
        "org": "ros2",
        "version": "4.2.3",
        "deps": ["geometry_msgs", "nav_msgs", "sensor_msgs"],
        "system_deps": ["ros-humble-geometry-msgs", "ros-humble-nav-msgs", "ros-humble-sensor-msgs"],
    },
    "rclcpp": {
        "org": "ros2",
        "version": "16.0.4",
        "deps": ["rclcpp", "rclcpp_components"],
        "system_deps": ["ros-humble-rclcpp", "ros-humble-rclcpp-components"],
    },
    "vision_opencv": {
        "org": "ros-perception",
        "version": "3.2.1",
        "deps": ["cv_bridge"],
        "system_deps": ["ros-humble-cv-bridge", "libopencv-dev"],
    },
    "rosbag2": {
        "org": "ros2",
        "version": "0.15.7",
        "deps": ["rosbag2_cpp", "rosbag2_storage"],
        "system_deps": ["ros-humble-rosbag2-cpp", "ros-humble-rosbag2-storage", "sqlite3"],
    },
    "message_filters": {
        "org": "ros2",
        "version": "4.3.2",
        "deps": ["message_filters"],
        "system_deps": ["ros-humble-message-filters"],
    },
    "rcpputils": {
        "org": "ros2",
        "version": "2.4.1",
        "deps": ["rcpputils"],
        "system_deps": ["ros-humble-rcpputils"],
    },
    "rcutils": {
        "org": "ros2",
        "version": "5.1.3",
        "deps": ["rcutils"],
        "system_deps": ["ros-humble-rcutils"],
    },
}

def ros2_repositories():
    all_deps = []
    for pkg in ROS2_PACKAGES.values():
        all_deps.extend(pkg["system_deps"])

    ros2_deps_installer(
        name = "ros2_system_deps",
        system_deps = all_deps,
    )

    for pkg_name, pkg_info in ROS2_PACKAGES.items():
        http_archive(
            name = "ros2_" + pkg_name,
            build_file = "//third_party:ros2/{}.BUILD".format(pkg_name),
            urls = ["https://github.com/{}/{}/archive/refs/tags/{}.tar.gz".format(
                pkg_info["org"],
                pkg_name,
                pkg_info["version"]
            )],
            strip_prefix = "{}-{}".format(pkg_name, pkg_info["version"]),
        )