cc_library(
    name = "tbb",
    srcs = [
        "lib/x86_64-linux-gnu/libtbbmalloc.so.2",
        "lib/x86_64-linux-gnu/libtbbmalloc_proxy.so.2",
        "lib/x86_64-linux-gnu/libtbb.so.12",
    ],
    hdrs = glob(["include/tbb/**"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    linkopts = [
        "-ltbbmalloc",
        "-ltbbmalloc_proxy",
        "-ltbb",
        "-L/usr/lib/x86_64-linux-gnu",
        "-Wl,-rpath,/usr/lib/x86_64-linux-gnu",
    ],
    strip_include_prefix = "include",
    alwayslink = 1,
)