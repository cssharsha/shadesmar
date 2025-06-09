load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "stf_proto",
    srcs = ["transform_tree.proto"],
    deps = ["//core/proto:geometry_proto"],
)

cc_proto_library(
    name = "stf_cc_proto",
    deps = [":stf_proto"],
)