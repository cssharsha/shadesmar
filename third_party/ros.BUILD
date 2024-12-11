cc_library(
    name = "tracetools",
    srcs = glob([
        "lib/libtracetools*.so*",
    ]),
    hdrs = glob([
        "include/tracetools/**/*.h",
        "include/tracetools/**/*.hpp",
    ]),
    includes = [
        "include",
        "include/tracetools",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "rcl_interfaces",
    srcs = glob(["lib/librcl_interfaces__rosidl*.so*"]),
    hdrs = glob(["include/rcl_interfaces/**/*"]),
    includes = [
        "include",
        "include/rcl_interfaces",
    ],
    visibility = ["//visibility:public"],
    deps = [":rcl_base"],
)

cc_library(
    name = "libstatistics_collector",
    srcs = glob([
        "lib/libstatistics_collector*.so*",
    ]),
    hdrs = glob([
        "include/libstatistics_collector/**/*.h",
        "include/libstatistics_collector/**/*.hpp",
        "include/statistics_msgs/**/*.h",
    ]),
    includes = [
        "include",
        "include/libstatistics_collector",
        "include/statistics_msgs",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "rcl_base",
    srcs = glob([
        "lib/librcl*.so*",
        "lib/librmw*.so*",
        "lib/librcutils*.so*",
        "lib/librcpputils*.so*",
        "lib/librosidl*.so*",
        "lib/librosidl_typesupport_cpp*.so*",
        "lib/librcl_yaml_param_parser*.so*",
        "lib/librosidl_typesupport_cpp*.so*",
        "lib/librosidl_typesupport_introspection_cpp*.so*",
        "lib/librosidl_typesupport_c*.so*",
        "lib/librosidl_typesupport_introspection_c*.so*",
    ]),
    hdrs = glob([
        "include/builtin_interfaces/**/*.h",
        "include/rcl/**/*.h",
        "include/rmw/**/*.h",
        "include/rclcpp/**/*.hpp",
        "include/rclcpp/**/*.h",
        "include/rcutils/**/*.h",
        "include/rcpputils/**/*.h",
        "include/rosidl_runtime_c/**/*.h",
        "include/rosidl_runtime_cpp/**/*.h",
        "include/rosidl_typesupport_interface/**/*.h",
        "include/rosidl_typesupport_cpp/**/*.hpp",
        "include/rosidl_typesupport_cpp/**/*.h",
        "include/rosidl_typesupport_introspection_cpp/**/*.h",
        "include/rosidl_typesupport_c/**/*.h",
        "include/rosidl_typesupport_introspection_c/**/*.h",
        "include/rcl_yaml_param_parser/**/*.h",
    ]),
    includes = [
        "include",
        "include/builtin_interfaces",
        "include/rcl",
        "include/rmw",
        "include/rclcpp",
        "include/rcutils",
        "include/rcpputils",
        "include/rosidl_runtime_c",
        "include/rosidl_runtime_cpp",
        "include/rosidl_typesupport_interface",
        "include/rosidl_typesupport_cpp",
        "include/rosidl_typesupport_introspection_cpp",
        "include/rosidl_typesupport_c",
        "include/rosidl_typesupport_introspection_c",
        "include/rcl_yaml_param_parser",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":tracetools",
        ":libstatistics_collector",
        "@boost//:system",
        "@boost//:thread",
    ],
)
cc_library(
    name = "core",
    srcs = glob([
        "lib/librclcpp*.so*",
        "lib/libtracetools.so*",
        "lib/libstatistics*.so*",
    ]),
    hdrs = glob([
        "include/**/*.hpp",
        "include/**/*.h",
    ]),
    visibility = ["//visibility:public"],
    deps = [
        ":rcl_base",
        ":rcl_interfaces",
        "@boost//:filesystem",
    ],
)

cc_library(
    name = "std_msgs",
    srcs = glob(["lib/libstd_msgs__rosidl*.so*"]),
    hdrs = glob(["include/std_msgs/**/*"]),
    includes = [
        "include",
        "include/std_msgs",
    ],
    visibility = ["//visibility:public"],
    deps = [":core"],
)

cc_library(
    name = "geometry_msgs",
    srcs = glob(["lib/libgeometry_msgs__rosidl*.so*"]),
    hdrs = glob(["include/geometry_msgs/**/*"]),
    includes = [
        "include",
        "include/geometry_msgs",
    ],
    visibility = ["//visibility:public"],
    deps = [":core"],
)

cc_library(
    name = "nav_msgs",
    srcs = glob(["lib/libnav_msgs__rosidl*.so*"]),
    hdrs = glob(["include/nav_msgs/**/*"]),
    includes = ["include", "include/nav_msgs"],
    visibility = ["//visibility:public"],
    deps = [":core", ":geometry_msgs", ":std_msgs"],
)

cc_library(
    name = "sensor_msgs",
    srcs = glob(["lib/libsensor_msgs__rosidl*.so*"]),
    hdrs = glob(["include/sensor_msgs/**/*"]),
    includes = [
        "include",
        "include/sensor_msgs",
    ],
    visibility = ["//visibility:public"],
    deps = [":core"],
)

cc_library(
    name = "cv_bridge",
    srcs = glob(["lib/libcv_bridge*.so*"]),
    hdrs = glob(["include/cv_bridge/**/*"]),
    includes = [
        "include",
        "include/cv_bridge",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core",
        ":sensor_msgs",
        "@opencv//:core",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "message_filters",
    srcs = glob(["lib/libmessage_filters*.so*"]),
    hdrs = glob(["include/message_filters/**/*"]),
    includes = [
        "include",
        "include/message_filters",
    ],
    visibility = ["//visibility:public"],
    deps = [":rcl_base"],
)

cc_library(
    name = "rosbag",
    srcs = glob([
        "lib/librosbag2*.so*",
        "lib/libsqlite3_storage*.so*",
        "lib/libmcap_storage*.so*",
    ]),
    hdrs = glob([
        "include/rosbag2_cpp/**/*.hpp",
        "include/rosbag2_storage/**/*.hpp",
        "include/rosbag2_storage/**/*.h",
    ]),
    # strip_include_prefix = "include/rosbag2_cpp",
    includes = [
        "include/rosbag2_cpp",
        "include/rosbag2_storage",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core",
        ":rcl_base",
        ":nav_msgs",
        ":sensor_msgs",
        "@boost//:filesystem",
        "@boost//:iostreams",
    ],
)

# genrule(
#     name = "rosidl_generator",
#     srcs = [],
#     outs = ["rosidl_generator_tool"],
#     cmd = """
#         echo '#!/bin/bash' > "$@"
#         echo 'exec /opt/ros/humble/lib/rosidl_generator_cpp/rosidl_generator_cpp "$$@"' >> "$@"
#         chmod +x "$@"
#     """,
#     executable = True,
#     visibility = ["//visibility:public"],
# )

# genrule(
#     name = "nav_msgs_generated",
#     srcs = ["@ros_humble//:share/nav_msgs/msg/*.msg"],
#     outs = ["include/nav_msgs/msg/detail/*.hpp"],
#     tools = [":rosidl_generator"],
#     cmd = "$(location :rosidl_generator) $(SRCS) -o $(OUTS)",
# )
