package(default_visibility = ["//visibility:public"])

filegroup(
    name = "all_files",
    srcs = glob(
        [
            "**/*.h",
            "**/*.cc",
            "**/*.cpp",
            "**/*.hpp",
            "**/*.proto",
            "**/*.BUILD",
            "**/*.bazel",
            "**/*.bzl",
        ],
        exclude = [
            "external/**",
            "bazel-*/**",
            "bazel-out/**",
        ],
    ),
    visibility = ["//visibility:public"],
)

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "shadesman",
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "debug_map_store",
    srcs = ["debug_map_store.cpp"],
    deps = [
        "//core/storage:storage",
        "//core/types:types",
        "//common:logging",
    ],
    visibility = ["//visibility:public"],
)
