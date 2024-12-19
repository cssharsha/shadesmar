cc_library(
    name = "cuda_headers",
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cuda_runtime",
    srcs = ["lib64/libcudart.so"],
    deps = [":cuda_headers"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cublas",
    srcs = ["lib64/libcublas.so"],
    deps = [":cuda_runtime"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cusolver",
    srcs = ["lib64/libcusolver.so"],
    deps = [":cuda_runtime", ":cublas"],
    visibility = ["//visibility:public"],
)

# Specialized config for Gaussian Splatting CUDA kernels
cuda_library(
    name = "gaussian_splat_cuda",
    srcs = ["cuda/gaussian_splat.cu"],
    hdrs = ["cuda/gaussian_splat.cuh"],
    deps = [
        ":cuda_runtime",
        ":cublas",
        "@eigen",
    ],
    gpu_arch = "sm_75",  # Adjust based on your GPU
)
