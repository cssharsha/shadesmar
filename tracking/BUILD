load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "tracking",
    srcs = [
        "2d/orb_tracker.cpp",
    ],
    hdrs = [
        "2d/orb_tracker.hpp",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "//core/types",
        "//utils:stf",
        "@opencv//:core",
        "@opencv//:features2d",
        "@opencv//:calib3d",
        "//common:logging",
    ],
)
