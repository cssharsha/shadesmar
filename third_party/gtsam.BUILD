cc_library(
    name = "gtsam",
    srcs = glob([
        "lib/libgtsam.so*",
    ]),
    hdrs = glob([
        "include/gtsam/**/*.h",
        "include/gtsam/**/*.hpp",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen",
        "@boost//:boost",
        "@tbb",
    ],
)

cc_library(
    name = "gtsam_unstable",
    srcs = glob([
        "lib/libgtsam_unstable.so*",
    ]),
    hdrs = glob([
        "include/gtsam_unstable/**/*.h",
        "include/gtsam_unstable/**/*.hpp",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        ":gtsam",
        "@eigen",
        "@boost//:boost",
        "@tbb",
    ],
)
