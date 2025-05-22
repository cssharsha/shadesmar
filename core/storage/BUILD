load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "storage",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob(["include/core/storage/*.hpp"]),
    includes = [
        "include",
        "$(GENDIR)",
    ],
    deps = [
        "//core/types",
        "//core/proto:map_cc_proto",
        "//core/proto:map_storage_index_cc_proto",
        "//common:logging",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "storage_test",
    srcs = glob(["test/*_test.cpp"]),
    deps = [
        ":storage",
        "@opencv//:imgcodecs",
        "@com_google_googletest//:gtest_main",
    ],
)
