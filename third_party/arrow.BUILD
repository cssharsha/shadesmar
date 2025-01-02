cc_library(
    name = "arrow",
    includes = ["include/arrow"],
    hdrs = glob(["include/arrow/**/*.h"]),
    srcs = [],
    visibility = ["//visibility:public"],
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu",
        "-larrow",
        # "-larrow_dataset",
        # "-larrow_flight",
    ],
)
