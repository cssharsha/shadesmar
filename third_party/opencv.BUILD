cc_library(
    name = "core",
    srcs = glob([
        "lib/x86_64-linux-gnu/libopencv_core.so*",
    ]),
    hdrs = glob([
        "include/opencv4/opencv2/**/*.hpp",
        "include/opencv4/opencv2/**/*.h",
    ]),
    includes = ["include/opencv4"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "imgproc",
    srcs = glob([
        "lib/x86_64-linux-gnu/libopencv_imgproc.so*",
    ]),
    hdrs = glob([
        "include/opencv4/opencv2/**/*.hpp",
        "include/opencv4/opencv2/**/*.h",
    ]),
    includes = ["include/opencv4"],
    visibility = ["//visibility:public"],
    deps = [":core"],
)

# You might also need these depending on what cv_bridge requires
cc_library(
    name = "highgui",
    srcs = glob([
        "lib/x86_64-linux-gnu/libopencv_highgui.so*",
    ]),
    hdrs = glob([
        "include/opencv4/opencv2/**/*.hpp",
        "include/opencv4/opencv2/**/*.h",
    ]),
    includes = ["include/opencv4"],
    visibility = ["//visibility:public"],
    deps = [":core"],
)