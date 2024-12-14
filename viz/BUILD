load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = ["include/viz/interface.hpp"],
    includes = ["include"],
    deps = [
        "//core:core",
        "@foxglove_bridge//:foxglove_bridge_component",
    ],
)

cc_library(
    name = "rerun_viz",
    srcs = ["src/rerun_viz.cpp"],
    hdrs = ["include/viz/rerun_viz.hpp", "include/viz/rerun_eigen_adapters.hpp"],
    includes = ["include"],
    deps = [
        ":interface",
        "@rerun_sdk//:rerun",
    ],
)

# cc_library(
#     name = "foxglove_viz",
#     srcs = ["src/foxglove_viz.cpp"],
#     hdrs = [
#         "include/viz/foxglove_viz.hpp",
#     ],
#     includes = ["include"],
#     deps = [
#         ":interface",
#         "@foxglove_bridge//:foxglove_bridge",
#     ],
# )

# cc_test(
#     name = "viz_test",
#     srcs = ["test/viz_test.cpp"],
#     deps = [
#         ":rerun_viz",
#         ":foxglove_viz",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

ros2_cpp_binary(
    name = "visualize_rosbag",
    srcs = ["examples/visualize_rosbag.cpp"],
    deps = [
        ":rerun_viz",
        "//ros:rosbag_reader",
    ],
)
