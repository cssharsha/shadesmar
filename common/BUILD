load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "logging",
    srcs = ["logging/logging.hpp"],
    hdrs = ["logging/logging.hpp"],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
    ],
    # defines = select({
    #     "//conditions:default": [],
    #     "//config:enable_debug": ["ENABLE_DEBUG_LOGS"],
    # }),
)
