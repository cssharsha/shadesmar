cc_library(
    name = "rcpputils",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.hpp",
    ]),
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_rcutils//:rcutils",
    ],
)