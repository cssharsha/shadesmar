package(default_visibility = ["//visibility:public"])

BOOST_COMPONENTS = [
    "system",
    "thread",
    "filesystem",
    "serialization",
    "program_options",
    "date_time",
    "timer",
    "chrono",
    "regex",
    "iostreams",
]

[cc_library(
    name = comp,
    includes = ["include"],
    hdrs = glob(["include/boost/%s/**/*.hpp" % comp]),
    srcs = glob(["lib/libboost_%s.so*" % comp]),
    deps = [":system"] if comp != "system" else [],
) for comp in BOOST_COMPONENTS]

cc_library(
    name = "boost",
    deps = [":" + comp for comp in BOOST_COMPONENTS],
    includes = ["include"],
    hdrs = glob(["include/boost/**/*.hpp"]),
    visibility = ["//visibility:public"],
)