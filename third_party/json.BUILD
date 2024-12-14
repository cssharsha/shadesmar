package(default_visibility = ["//visibility:public"])

cc_library(
    name = "json",
    hdrs = glob([
        "single_include/nlohmann/**/*.hpp",
        "single_include/nlohmann/**/*.h",
    ]),
    includes = ["single_include"],
    strip_include_prefix = "single_include",
)
