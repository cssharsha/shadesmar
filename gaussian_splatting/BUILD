load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

# Training configuration
cc_library(
    name = "training_config",
    hdrs = ["training/training_config.hpp"],
    visibility = ["//visibility:public"],
)

# Keyframe batch processing
cc_library(
    name = "keyframe_batch",
    srcs = ["training/keyframe_batch.cpp"],
    hdrs = ["training/keyframe_batch.hpp"],
    deps = [
        ":training_config",
        "//core/types",
        "//core/storage",
        "@libtorch//:torch",
        "@opencv//:core",
        "@opencv//:imgproc", 
        "//common:logging",
    ],
)

# Training visualization
cc_library(
    name = "training_visualizer",
    srcs = ["visualization/training_visualizer.cpp"],
    hdrs = ["visualization/training_visualizer.hpp"],
    deps = [
        ":keyframe_batch",
        "//viz:rerun_viz",
        "//core/types",
        "@opencv//:core",
        "@opencv//:imgproc",
        "//common:logging",
    ],
)

# Batch trainer
cc_library(
    name = "batch_trainer",
    srcs = ["training/batch_trainer.cpp"],
    hdrs = ["training/batch_trainer.hpp"],
    deps = [
        ":training_config",
        ":batch_gpu_manager",
        ":parameter_transforms",
        ":loss_functions",
        ":densification_controller",
        "//core/types",
        "//core/storage",
        "@libtorch//:torch",
        "//common:logging",
    ],
)

# Parameter transforms
cc_library(
    name = "parameter_transforms",
    srcs = ["optimization/parameter_transforms.cpp"],
    hdrs = ["optimization/parameter_transforms.hpp"],
    deps = [
        "@libtorch//:torch",
        "//common:logging",
    ],
)

# Loss functions
cc_library(
    name = "loss_functions",
    srcs = ["optimization/loss_functions.cpp"],
    hdrs = ["optimization/loss_functions.hpp"],
    deps = [
        "@libtorch//:torch",
        "//common:logging",
    ],
)

# GPU batch manager
cc_library(
    name = "batch_gpu_manager",
    srcs = ["utils/batch_gpu_manager.cpp"],
    hdrs = ["utils/batch_gpu_manager.hpp"],
    deps = [
        ":training_config",
        "//core/types",
        "@libtorch//:torch",
        "//common:logging",
    ],
)

# Densification controller
cc_library(
    name = "densification_controller",
    srcs = ["optimization/densification_controller.cpp"],
    hdrs = ["optimization/densification_controller.hpp"],
    deps = [
        ":training_config",
        ":batch_gpu_manager",
        "@libtorch//:torch",
        "//common:logging",
    ],
)

cc_library(
    name = "bilateral_grid",
    srcs = ["rendering/bilateral_grid.cpp"],
    hdrs = ["rendering/bilateral_grid.hpp"],
    deps = [
        "//gaussian_splatting:training_config",
        "@libtorch//:torch",
        "@eigen",
        "//common:logging",
    ],
)

cuda_library(
    name = "spherical_harmonics",
    srcs = [
        "rendering/spherical_harmonics.cpp",
        "rendering/spherical_harmonics.cu",
    ],
    hdrs = ["rendering/spherical_harmonics.hpp"],
    deps = [
        "//gaussian_splatting:training_config",
        "@libtorch//:torch",
        "@eigen",
        "//common:logging",
    ],
    copts = [
        "--std=c++17"
    ]
)

cc_library(
    name = "rasterizer",
    srcs = ["rendering/rasterizer.cpp"],
    hdrs = ["rendering/rasterizer.hpp"],
    deps = [
        "//core/types",
        "//common:logging",
        "@libtorch//:torch",
        "@eigen",
    ],
)

# Training Gaussian Splat Processor
cc_library(
    name = "training_gs_processor",
    srcs = ["training_gs_processor.cpp"],
    hdrs = ["training_gs_processor.hpp"],
    deps = [
        ":training_config",
        ":keyframe_batch",
        ":training_visualizer",
        ":batch_trainer",
        ":densification_controller",
        ":bilateral_grid",
        ":spherical_harmonics",
        ":rasterizer",
        "//core/storage",
        "//core/types",
        "//utils:stf",
        "@libtorch//:torch",
        "@opencv//:core",
        "@opencv//:imgproc",
        "//common:logging",
    ],
)

# Streaming Gaussian Splat Processor
cc_library(
    name = "streaming_gs_processor",
    srcs = ["streaming_gs_processor.cpp"],
    hdrs = ["streaming_gs_processor.hpp"],
    deps = [
        ":training_config",
        ":keyframe_batch",
        ":training_visualizer",
        ":batch_trainer",
        ":densification_controller",
        ":bilateral_grid",
        ":rasterizer",
        "//core/storage",
        "//core/types",
        "//utils:stf",
        "@libtorch//:torch",
        "@opencv//:core",
        "@opencv//:imgproc",
        "@eigen",
        "//common:logging",
    ],
)

# Main gaussian splatting library
cc_library(
    name = "gaussian_splatting",
    srcs = ["gs_processor.cpp"],
    hdrs = ["gs_processor.hpp"],
    deps = [
        ":training_config",
        ":batch_trainer",
        ":parameter_transforms",
        ":loss_functions",
        ":batch_gpu_manager",
        ":densification_controller",
        ":bilateral_grid",
        ":spherical_harmonics",
        ":streaming_gs_processor",
        "//core/storage",
        "//core/types",
        "//core/proto:gaussian_splat_cc_proto",
        "//utils:stf",
        "//common:logging",
        "@eigen",
        "@libtorch//:torch",
    ],
)

# Training processor test
cc_test(
    name = "training_gs_processor_test",
    srcs = ["training_gs_processor_test.cpp"],
    deps = [
        ":training_gs_processor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "gs_processor",
    srcs = ["gs_processor_main.cpp"],
    deps = [
        ":gaussian_splatting",
        "//viz:visualize_gs_rerun",
        "//core/storage",
        "//common:logging",
    ],
)
