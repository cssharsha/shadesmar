cc_library(
    name = "torch",
    srcs = glob([
        "lib/libtorch.so*",
        "lib/libtorch_cpu.so*",
        "lib/libc10.so*",
        "lib/libgomp*.so*",
        "lib/libtorch_global_deps.so*",
        "lib/libshm.so*",
        "lib/libtorch_cuda.so*",
        "lib/libc10_cuda.so*",
        "lib/libcudart*.so*",
        "lib/libtorch_cuda_linalg.so*",
        "lib/libnvToolsExt*.so*",
        "lib/libcublas*.so*", 
        "lib/libcublasLt*.so*",
        "lib/libcudnn*.so*",
        "lib/libnvrtc*.so*",
        "lib/libnvfuser_codegen.so*",
        "lib/libcaffe2_nvrtc.so*",
    ]),
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
    ]),
    includes = [
        "include",
        "include/torch/csrc/api/include",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "torch_cuda",
    srcs = glob([
        "lib/libtorch_cuda.so*",
        "lib/libc10_cuda.so*",
    ]),
    hdrs = glob([
        "include/torch/**/*.h",
        "include/ATen/**/*.h",
        "include/c10/**/*.h",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [":torch"],
    linkopts = [
        "-ltorch_cuda",
        "-lc10_cuda",
    ],
)

# Main library that most users will depend on
cc_library(
    name = "libtorch",
    visibility = ["//visibility:public"],
    deps = [":torch"],
)