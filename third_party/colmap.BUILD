package(default_visibility = ["//visibility:public"])

cc_library(
    name = "colmap",
    hdrs = glob(["include/colmap/**/*.h"]),
    srcs = glob(["lib/libcolmap*.a"]),
    includes = ["include"],
    deps = [
        "@ceres//:ceres",
        "@eigen//:eigen",
        "@boost//:boost",
        "@freeimage//:freeimage",
        "@curl//:curl",
        "@openssl//:crypto",
    ],
    linkopts = [
        "-fopenmp",
        "-lglog",
        "-lgflags",
    ],
)
