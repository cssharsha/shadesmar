cc_library(
    name = "ros_core",
    srcs = glob([
        "lib/lib*.so*",
    ]),
    hdrs = glob([
        "include/rclcpp/**/*.hpp",
        "include/rcl/**/*.h",
        "include/rmw/**/*.h",
        "include/rosidl_runtime_cpp/**/*.hpp",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@boost//:system",
        "@boost//:thread",
        "@boost//:filesystem",
    ],
)

genrule(
    name = "rosidl_generator",
    srcs = [],
    outs = ["rosidl_generator_tool"],
    cmd = """
        echo '#!/bin/bash' > "$@"
        echo 'exec /opt/ros/humble/lib/rosidl_generator_cpp/rosidl_generator_cpp "$$@"' >> "$@"
        chmod +x "$@"
    """,
    executable = True,
    visibility = ["//visibility:public"],
)