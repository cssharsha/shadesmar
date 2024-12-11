cc_library(
    name = "rcutils",
    srcs = select({
        "@platforms//os:linux": glob([
            "src/**/*.c",
            "src/**/*.cpp",
        ], exclude = [
            "src/**/win32/**",
            "src/**/*_win32.*",
        ]),
        "//conditions:default": [],
    }),
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
        "src/**/*.h",
    ]),
    includes = [
        "include",
        "src",
    ],
    visibility = ["//visibility:public"],
)