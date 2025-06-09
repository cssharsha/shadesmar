load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "kitti_reader",
    srcs = [
        "src/kitti_reader.cpp",
        "src/conversions.cpp",
    ],
    hdrs = [
        "include/kitti/kitti_reader.hpp",
        "include/kitti/conversions.hpp",
        "include/kitti/config.hpp",
    ],
    copts = [
        "-std=c++17",
        "-pthread",
        "-I.",  # Add the current directory to the include path
    ],
    includes = ["include"],  # This helps with include path resolution
    linkopts = [],
    visibility = ["//visibility:public"],
    deps = [
        "//core/graph:graph",
        "//core/storage:storage",
        "//core:core",
        "//viz:rerun_viz",
        "//utils:stf",
        # "@kitti_devkit//:kitti_devkit_parsing", # Uncomment if using a devkit target
        "@eigen//:eigen",          # Assuming Eigen is available as @eigen//:eigen
        "@opencv//:core",
        "@opencv//:imgproc",
        "@opencv//:imgcodecs",       # For visualization
    ],
)

# Example of how you might create a cc_binary for a test/demo application
cc_binary(
    name = "kitti_reader_example",
    srcs = ["examples/kitti_example.cpp"],
    copts = [
        "-std=c++17",
    ],
    deps = [
        ":kitti_reader",
    ],
)
