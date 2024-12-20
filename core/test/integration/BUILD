load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "integration_tests",
    srcs = glob(["*.cpp"]),
    deps = [
        "//core",
        "@com_google_googletest//:gtest_main",
    ],
)
