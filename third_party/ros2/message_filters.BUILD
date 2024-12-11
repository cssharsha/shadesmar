cc_library(
    name = "message_filters",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.hpp",
    ]),
    hdrs = glob(["include/**/*.h", "include/**/*.hpp"]),
    strip_include_prefix = "include",
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rcpputils//:rcpputils",
    ],
)