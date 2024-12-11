cc_library(
    name = "cv_bridge",
    srcs = glob([
        "cv_bridge/src/**/*.cpp",
        "cv_bridge/src/**/*.hpp",
    ]),
    hdrs = glob(["cv_bridge/include/**/*.hpp", "cv_bridge/include/**/*.h"]),
    strip_include_prefix = "cv_bridge/include",
    includes = ["cv_bridge/include"],
    copts = [
        "-I$(GENDIR)/external/ros2_common_interfaces/",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:sensor_msgs",
        "@opencv//:core",
        "@opencv//:imgproc",
    ],
)