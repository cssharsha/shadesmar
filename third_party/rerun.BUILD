cc_library(
    name = "rerun",
    srcs = select({
        "@platforms//cpu:x86_64": ["rerun_cpp_sdk/lib/librerun_c__linux_x64.a"],
        "@platforms//cpu:arm64": ["rerun_cpp_sdk/lib/librerun_c__linux_arm64.a"],
    }) + glob([
        "rerun_cpp_sdk/src/**/*.cpp",
    ]),
    hdrs = glob([
        "rerun_cpp_sdk/src/**/*.hpp",
        "rerun_cpp_sdk/src/**/*.h",
    ]),
    includes = ["rerun_cpp_sdk/src"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@arrow",
    ],
    copts = [
        "-DRERUN_WITH_EIGEN",
        "-DEIGEN_MPL2_ONLY",
    ],
    linkstatic = True,
)
