cc_library(
    name = "torch",
    srcs = glob([
        "lib/*.so",
        "lib/*.so.*",
    ], exclude = [
        "lib/libnnapi_backend.so",
        "lib/libtorch_python.so",
        "lib/libjitbackend_test.so",
        "lib/libtorchbind_test.so",
        "lib/libc10d_cuda_test.so",
    ]),
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
        "include/**/*.cuh",
    ]),
    includes = [
        "include",
        "include/torch/csrc/api/include",
    ],
    linkopts = [
        "-Llib",
        "-ltorch",
        "-ltorch_cpu",
        "-ltorch_cuda",
        "-lc10",
        "-lc10_cuda",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "torch_cuda",
    srcs = glob([
        "lib/libtorch_cuda.so",
        "lib/libc10_cuda.so",
    ]),
    hdrs = glob([
        "include/torch/**/*.h",
        "include/ATen/**/*.h",
        "include/c10/**/*.h",
    ]),
    includes = [
        "include",
    ],
    linkopts = [
        "-Llib",
        "-ltorch_cuda",
        "-lc10_cuda",
    ],
    visibility = ["//visibility:public"],
    deps = [":torch"],
)

# Main library that most users will depend on
cc_library(
    name = "libtorch",
    visibility = ["//visibility:public"],
    deps = [
        ":torch",
        ":torch_cuda",
    ],
)
