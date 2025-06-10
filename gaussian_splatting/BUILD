load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gaussian_splatting",
    srcs = ["gs_processor.cpp"],
    hdrs = ["gs_processor.hpp"],
    deps = [
        "//core/storage",
        "//core/types",
        "//core/proto:gaussian_splat_cc_proto",
        "//utils:stf",
        "//common:logging",
        "@eigen",
    ],
)

cc_binary(
    name = "gs_processor",
    srcs = ["gs_processor_main.cpp"],
    deps = [
        ":gaussian_splatting",
        "//common:logging",
    ],
)