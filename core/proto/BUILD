load("@rules_cc//cc:defs.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "geometry_proto",
    srcs = ["geometry.proto"],
)

cc_proto_library(
    name = "geometry_cc_proto",
    deps = [":geometry_proto"],
)

proto_library(
    name = "sensor_data_proto",
    srcs = ["sensor_data.proto"],
    deps = [":geometry_proto"],
)

cc_proto_library(
    name = "sensor_data_cc_proto",
    deps = [":sensor_data_proto"],
)

proto_library(
    name = "keyframe_proto",
    srcs = ["keyframe.proto"],
    deps = [
        ":geometry_proto",
        ":sensor_data_proto",
    ],
)

cc_proto_library(
    name = "keyframe_cc_proto",
    deps = [":keyframe_proto"],
)

proto_library(
    name = "keypoint_proto",
    srcs = ["keypoint.proto"],
    deps = [
        ":geometry_proto",
    ],
)

cc_proto_library(
    name = "keypoint_cc_proto",
    deps = [":keypoint_proto"],
)

proto_library(
    name = "imu_proto",
    srcs = ["imu.proto"],
    deps = [":geometry_proto"],
)

cc_proto_library(
    name = "imu_cc_proto",
    deps = [":imu_proto"],
)

proto_library(
    name = "factor_proto",
    srcs = ["factor.proto"],
    deps = [
        ":geometry_proto",
        ":imu_proto",
    ],
)

cc_proto_library(
    name = "factor_cc_proto",
    deps = [":factor_proto"],
)

proto_library(
    name = "map_storage_index_proto",
    srcs = ["map_storage_index.proto"],
    deps = [":geometry_proto"],
)
cc_proto_library(
    name = "map_storage_index_cc_proto",
    deps = [":map_storage_index_proto"]
)
proto_library(
    name = "map_proto",
    srcs = ["map.proto"],
    deps = [
        ":keyframe_proto",
        ":keypoint_proto",
        ":factor_proto",
    ],
)

cc_proto_library(
    name = "map_cc_proto",
    deps = [":map_proto"],
)
