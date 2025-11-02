load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "tracking",
    srcs = [
        "2d/orb_tracker.cpp",
        # "2d/klt_tracker.cpp",
        "2d/reconstruction.cpp",
        "2d/gtsam_reconstruction.cpp",
        "2d/triangulation_common.cpp",
        "2d/sift_tracker.cpp",
        "2d/mast3r_tracker.cpp",
    ],
    hdrs = [
        "2d/orb_tracker.hpp",
        # "2d/klt_tracker.hpp",
        "2d/reconstruction.hpp",
        "2d/gtsam_reconstruction.hpp",
        "2d/triangulation_common.hpp",
        "2d/sift_tracker.hpp",
        "2d/mast3r_tracker.hpp",
    ],
    data = [
        "2d/mast3r_inference_service.py",
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
        # "@opencv//:video",
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
