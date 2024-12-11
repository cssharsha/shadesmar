cc_library(
    name = "rclcpp",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.hpp",
    ]),
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:nav_msgs",
        "@ros2_common_interfaces//:sensor_msgs",
    ],
)