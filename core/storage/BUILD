load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "storage",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    includes = [
        "$(GENDIR)",
    ],
    deps = [
        "//core/types",
        "//core/proto:map_cc_proto",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "storage_test",
    srcs = glob(["test/*_test.cpp"]),
    deps = [
        ":storage",
        "@com_google_googletest//:gtest_main",
    ],
)
