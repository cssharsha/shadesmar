cc_binary(
    name = "torch_test",
    srcs = ["torch_test.cpp"],
    deps = ["@libtorch//:libtorch"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "cuda_test",
    srcs = ["cuda_test.cpp"],
    deps = ["@local_cuda//:cuda_runtime"],
    visibility = ["//visibility:public"],
)