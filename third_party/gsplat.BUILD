load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "glm",
    hdrs = glob([
        "gsplat/cuda/csrc/third_party/glm/glm/**/*.h",
        "gsplat/cuda/csrc/third_party/glm/glm/**/*.hpp",
        "gsplat/cuda/csrc/third_party/glm/glm/**/*.inl",
    ]),
    includes = [
        "gsplat/cuda/csrc/third_party/glm",
    ],
    visibility = ["//visibility:public"],
    copts = [
        "--std=c++17",
    ],
)

cuda_library(
    name = "gsplat",
    srcs = glob([
        "gsplat/cuda/csrc/*.cpp",
        "gsplat/cuda/csrc/*.cu",
    ]),
    hdrs = glob([
        "gsplat/cuda/csrc/*.h",
        "gsplat/cuda/csrc/*.cuh",
        "gsplat/cuda/include/*.h",
        "gsplat/cuda/include/*.cuh",
    ]),
    includes = [
        # "gsplat/cuda/csrc/",
        "gsplat/cuda/include/",
        ".",
    ],
    visibility = ["//visibility:public"],
    copts = [
        "--std=c++17",
        "-I/usr/include/python3.10",
    ],
    deps = [
        ":glm",
        "@libtorch//:torch",
        "@eigen",
    ],
)
