load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "colmap_viz",
    srcs = ["colmap_viz.cpp"],
    hdrs = ["colmap_viz.hpp"],
    includes = ["colmap_processor"],
    linkopts = [
        "-lstdc++fs",
    ],
    deps = [
        "//core/storage",
        "//viz:rerun_viz",
        "//utils:stf",
    ],
)
cc_binary(
    name = "colmap_viz_main",
    srcs = ["colmap_viz_main.cpp"],
    deps = [
        ":colmap_viz",
    ],
)

cc_library(
    name = "colmap_converter",
    srcs = ["colmap_converter.cpp"],
    hdrs = ["colmap_converter.hpp"],
    includes = ["colmap_processor"],
    linkopts = [
        "-lstdc++fs",
    ],
    deps = [
        "@colmap//:colmap",
        "//core/storage",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc", # For cvtColor
    ],
)

cc_binary(
    name = "cconverter",
    srcs = ["converter.cpp"],
    deps = [
        ":colmap_converter",
    ],
)

ros2_cpp_binary(
    name = "colmap_processor",
    srcs = ["colmap_processor.cpp"],
    deps = [
        "//common:logging",
        "@ros2_rosbag2//:rosbag2_cpp",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_geometry2//:cpp_tf2_msgs",
        "@ros2_rclcpp//:rclcpp",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc", # For cvtColor
        "@ros2_rmw//:rmw",
    ],
    set_up_ament = True,
)
