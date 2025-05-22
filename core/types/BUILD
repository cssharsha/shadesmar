load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "types",
    hdrs = glob(["include/core/types/*.hpp"]),
    includes = [
        "include",
        "$(GENDIR)",
    ],
    deps = [
        "//core/proto:geometry_cc_proto",
        "//core/proto:sensor_data_cc_proto",
        "//core/proto:keyframe_cc_proto",
        "//core/proto:keypoint_cc_proto",
        "//core/proto:factor_cc_proto",
        "//core/proto:imu_cc_proto",
        "@eigen",
        "@opencv//:core",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "types_test",
    srcs = glob(["test/*_test.cpp"]),
    deps = [
        ":types",
        "@com_google_googletest//:gtest_main",
    ],
)
