cc_library(
    name = "eigen",
    hdrs = glob(["Eigen/**", "unsupported/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
    defines = [
        "EIGEN_AVOID_STL_ARRAY",
        "EIGEN_USE_THREADS=0",
        "EIGEN_DONT_VECTORIZE",
    ],
    deps = select({
        "@rules_cuda//cuda:is_enabled": ["@local_cuda//:cuda_headers"],
        "//conditions:default": [],
    }),
)
