load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "tracking",
    srcs = [
        "2d/orb_tracker.cpp",
        "2d/reconstruction.cpp",
        "2d/gtsam_reconstruction.cpp",
    ],
    hdrs = [
        "2d/orb_tracker.hpp",
        "2d/reconstruction.hpp",
        "2d/gtsam_reconstruction.hpp",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "//core/types",
        "//core/storage",
        "//utils:stf",
        "@opencv//:core",
        "@opencv//:features2d",
        "@opencv//:calib3d",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "//common:logging",
        "@gtsam//:gtsam",
    ],
)

cc_test(
    name = "reconstruct_test",
    srcs = glob(["2d/test/*_test.cpp"]),
    deps = [
        ":tracking",
        "@com_google_googletest//:gtest_main",
    ],
)
