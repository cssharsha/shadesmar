package(default_visibility = ["//visibility:public"])

# The prebuilt C library
cc_library(
    name = "foxglove_c",
    srcs = ["foxglove/lib/libfoxglove.so"],
    hdrs = ["foxglove/include/foxglove-c/foxglove-c.h"],
    includes = ["foxglove/include"],
    visibility = ["//visibility:public"],
)

# The C++ wrapper library
cc_library(
    name = "foxglove_sdk",
    srcs = glob([
        "foxglove/src/**/*.cpp",
    ]),
    hdrs = glob([
        "foxglove/include/foxglove/**/*.hpp",
    ]),
    includes = ["foxglove/include"],
    visibility = ["//visibility:public"],
    deps = [
        ":foxglove_c",
        "@nlohmann_json//:json",
    ],
)
