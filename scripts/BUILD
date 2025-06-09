load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "trajectory_compare",
    srcs = ["trajectory_compare.cpp"],
    deps = [
        "//core/storage:storage",
        "//core/types:types",
        "//common:logging",
    ],
    linkopts = ["-lm"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "visualize_map_keypoints",
    srcs = ["visualize_map_keypoints.cpp"],
    deps = [
        "//core/storage:storage",
        "//core/types:types",
        "//common:logging",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
        "-lopencv_highgui",
        "-lstdc++fs",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "debug_map_store",
    srcs = ["debug_map_store.cpp"],
    deps = [
        "//core/storage:storage",
        "//core/types:types",
        "//common:logging",
    ],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "plot_trajectory",
    srcs = ["plot_trajectory.py"],
    main = "plot_trajectory.py",
    visibility = ["//visibility:public"],
)