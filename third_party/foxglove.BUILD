cc_library(
    name = "foxglove",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@boost//:boost",
        "@nlohmann_json//:json",
    ],
)
