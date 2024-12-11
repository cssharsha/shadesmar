load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "geometry_msgs",
    hdrs = glob(["geometry_msgs/msg/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "nav_msgs",
    hdrs = glob(["nav_msgs/msg/*.hpp"]),
    includes = ["include"],
    deps = [":geometry_msgs"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "sensor_msgs",
    hdrs = glob(["sensor_msgs/msg/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)