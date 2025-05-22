load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "tracking_test_case_proto",
    srcs = ["tracking_test_case.proto"],
    deps = [
        "//core/proto:keyframe_proto",
        "//core/proto:geometry_proto",
        "//utils/stf/proto:stf_proto",
    ],
)

cc_proto_library(
    name = "tracking_test_case_cc_proto",
    deps = [":tracking_test_case_proto"],
)