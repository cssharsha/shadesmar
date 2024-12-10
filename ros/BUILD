load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "ros",
    srcs = glob([
        "src/**/*.cpp",
    ]),
    hdrs = glob([
        "include/ros/**/*.hpp",
    ]),
    includes = ["include"],
    deps = [
        "//core",
        "@ros_core//:roscpp",
        "@ros_core//:cv_bridge",
        "@ros_core//:rosbag",
    ],
    visibility = ["//visibility:public"],
)

# Example application using the ROS wrapper
cc_binary(
    name = "process_bag",
    srcs = ["src/process_bag.cpp"],
    deps = [
        ":ros",
        "//core",
    ],
)