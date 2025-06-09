load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stf",
    srcs = [
        "stf/transform_tree.cpp",
        "stf/transform_utils.cpp",
    ],
    hdrs = [
        "stf/transform_tree.hpp",
        "stf/transform_utils.hpp",
    ],
    includes = ["."],
    deps = [
        "@eigen//:eigen",
        "//core/types",
        "//common:logging",
    ],
)

cc_library(
    name = "message_synchronizer",
    hdrs = ["message_synchronizer/message_synchronizer.hpp"],
    includes = ["."],
    deps = [
        "//core/types",
        "//common:logging",
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

# cc_test(
#     name = "message_synchronizer_test",
#     srcs = [
#         "message_synchronizer/test/message_synchronizer_test.cpp",
#     ],
#     deps = [
#         ":message_synchronizer",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

# cc_test(
#     name = "message_synchronizer_mock_test",
#     srcs = [
#         "message_synchronizer/test/message_synchronizer_mock.hpp",
#         "message_synchronizer/test/message_synchronizer_mock_test.cpp",
#     ],
#     deps = [
#         ":message_synchronizer",
#         "@com_google_googletest//:gtest_main",
#     ],
# )
