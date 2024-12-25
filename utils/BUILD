load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stf",
    srcs = ["stf/transform_tree.cpp"],
    hdrs = ["stf/transform_tree.hpp"],
    includes = ["."],
    deps = [
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "transform_tree_test",
    srcs = ["stf/test/transform_tree_test.cpp"],
    deps = [
        ":stf",
        "@com_google_googletest//:gtest_main",
    ],
)
