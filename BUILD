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
