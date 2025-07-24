package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ceres",
    hdrs = glob(["include/ceres/**/*.h"]),
    srcs = glob(["lib/x86_64-linux-gnu/libceres.so*"]),
    includes = ["include"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_github_gflags_gflags//:gflags",
        "@eigen//:eigen",
    ],
)
