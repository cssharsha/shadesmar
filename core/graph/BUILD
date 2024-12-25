load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "graph",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob([
        "include/core/graph/*.hpp",
        "include/core/graph/util.hpp",
    ]),
    includes = ["include"],
    deps = [
        "//core/types",
        "@gtsam",
        "@gtsam//:gtsam_unstable",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "graph_test",
    srcs = glob(["test/*_test.cpp"]),
    deps = [
        ":graph",
        "@com_google_googletest//:gtest_main",
        "@tbb",
    ],
)
