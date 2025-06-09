load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_library", "ros2_cpp_binary", "ros2_cpp_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = ["include/viz/interface.hpp"],
    includes = ["include"],
    deps = [
        "//core/types",
        "//core/graph:graph",
        "@opencv//:core",
    ],
)

cc_library(
    name = "config_loader",
    srcs = ["src/config_loader.cpp"],
    hdrs = ["include/viz/config_loader.hpp"],
    includes = ["include"],
    deps = [
        "//ros:rosbag_reader",
        "//core/graph:graph",
        "@nlohmann_json//:json",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "rerun_viz",
    srcs = ["src/rerun_viz.cpp"],
    hdrs = ["include/viz/rerun_viz.hpp", "include/viz/rerun_eigen_adapters.hpp"],
    includes = ["include"],
    deps = [
        ":interface",
        "//core/types",
        "//core/graph:graph",
        "//utils:stf",
        "@rerun_sdk//:rerun",
        "//common:logging",
        "@opencv//:core",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "rerun_viz_test",
    srcs = ["src/test_rerun.cpp"],
    deps = ["@rerun_sdk//:rerun"],
)

ros2_cpp_binary(
    name = "visualize_rosbag",
    srcs = ["examples/visualize_rosbag.cpp"],
    deps = [
        ":rerun_viz",
        ":config_loader",
        # ":config_files",
        "//ros:rosbag_reader",
        "@ros2_rosbag2//:rosbag2_cpp",
        "@ros2_rosbag2//:rosbag2_storage",
    ],
    set_up_ament = True,
)

ros2_cpp_binary(
    name = "visualize_live",
    srcs = ["examples/visualize_live.cpp"],
    deps = [
        ":rerun_viz",
        "//ros:ros_adapter",
    ],
    set_up_ament = True,
)

# Configuration files
filegroup(
    name = "config_files",
    srcs = glob(["config/*.json"]),
    visibility = ["//visibility:public"],
)
