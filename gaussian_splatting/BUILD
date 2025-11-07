load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

# Common tensor configuration (central dtype control)
cc_library(
    name = "tensor_config",
    hdrs = ["common/tensor_config.hpp"],
    deps = ["@libtorch//:torch"],
    visibility = ["//visibility:public"],
)

# Training configuration
cc_library(
    name = "training_config",
    hdrs = ["training/training_config.hpp"],
    visibility = ["//visibility:public"],
)

# Keyframe batch processing
cc_library(
    name = "keyframe_batch",
    srcs = [
        "training/keyframe_batch.cpp",
        "training/keyframe_tensor.cpp",
        "training/gaussian_tensors.cpp",
    ],
    hdrs = [
        "training/keyframe_batch.hpp",
        "training/keyframe_tensor.hpp",
        "training/gaussian_tensors.hpp",
    ],
    deps = [
        ":tensor_config",
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
        ":utils",
        "//viz:rerun_viz",
        "//core/types",
        "@opencv//:core",
        "@opencv//:imgproc",
        "//common:logging",
    ],
)

# Foxglove renderer
cc_library(
    name = "foxglove_renderer",
    srcs = ["visualization/foxglove_renderer.cpp"],
    hdrs = ["visualization/foxglove_renderer.hpp"],
    deps = [
        ":keyframe_batch",
        ":rasterizer",
        "//viz:foxglove_server",
        "//core/types",
        "//common:logging",
        "@libtorch//:torch",
        "@opencv//:core",
        "@opencv//:imgproc",
        "@eigen",
    ],
)

# Batch trainer
cc_library(
    name = "batch_trainer",
    srcs = ["training/batch_trainer.cpp"],
    hdrs = ["training/batch_trainer.hpp"],
    deps = [
        ":tensor_config",
        ":training_config",
        ":utils",
        ":parameter_transforms",
        ":loss_functions",
        ":rasterizer",
        ":strategy",
        "//core/types",
        "//core/storage",
        "@libtorch//:torch",
        "//common:logging",
        ":training_visualizer",
    ],
)

# Parameter transforms
cc_library(
    name = "parameter_transforms",
    srcs = ["optimization/parameter_transforms.cpp"],
    hdrs = ["optimization/parameter_transforms.hpp"],
    deps = [
        ":tensor_config",
        "@libtorch//:torch",
        "//common:logging",
    ],
)

# Fused SSIM
cuda_library(
    name = "fused_ssim",
    srcs = [
        "optimization/ssim/fused_ssim.cpp",
        "optimization/ssim/ssim.cu",
    ],
    hdrs = [
        "optimization/ssim/fused_ssim.hpp",
        "optimization/ssim/ssim.h",
    ],
    deps = [
        "@libtorch//:torch",
        "//common:logging",
    ],
    copts = [
        "--std=c++17"
    ],
)

# Loss functions
cc_library(
    name = "loss_functions",
    srcs = ["optimization/loss_functions.cpp"],
    hdrs = ["optimization/loss_functions.hpp"],
    deps = [
        ":tensor_config",
        ":fused_ssim",
        ":utils",
        "@libtorch//:torch",
        "//common:logging",
    ],
)

# Optimizer
cc_library(
    name = "optimizer",
    srcs = ["optimization/optimizer.cpp"],
    hdrs = ["optimization/optimizer.hpp"],
    deps = [
        ":tensor_config",
        "@libtorch//:torch",
        ":keyframe_batch",
    ],
)

# Strategy
cc_library(
    name = "strategy",
    srcs = [
        "optimization/scheduler.cpp",
        "optimization/strategy.cpp",
        "optimization/strategy_policies.cpp",
    ],
    hdrs = [
        "optimization/scheduler.hpp",
        "optimization/strategy.hpp",
        "optimization/strategy_policies.hpp",
    ],
    deps = [
        ":tensor_config",
        ":keyframe_batch",
        ":optimizer",
        ":rasterizer",
        "//gaussian_splatting/gsplat:gsplat",
        "@libtorch//:torch",
        "//common:logging",
    ],
)

cc_library(
    name = "utils",
    srcs = [
        "utils/batch_gpu_manager.cpp",
        "utils/initializers.cpp",
        "utils/image_utils.cpp",
        "utils/point_cloud_utils.cpp",
        "utils/splat_logger.cpp",
    ],
    hdrs = [
        "utils/batch_gpu_manager.hpp",
        "utils/initializers.hpp",
        "utils/torch_utils.hpp",
        "utils/image_utils.hpp",
        "utils/point_cloud_utils.hpp",
        "utils/splat_logger.hpp",
    ],
    deps = [
        ":training_config",
        ":keyframe_batch",
        "//core/types",
        "//core/storage",
        "@libtorch//:torch",
        "//common:logging",
        "@opencv//:core",
        "@opencv//:imgproc",
        "@opencv//:imgcodecs",
        "@pcl//:kdtree",
    ],
)

# Densification controller
cc_library(
    name = "densification_controller",
    srcs = ["optimization/densification_controller.cpp"],
    hdrs = ["optimization/densification_controller.hpp"],
    deps = [
        ":tensor_config",
        ":training_config",
        ":utils",
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
    srcs = [
        "rendering/rasterizer.cpp",
    ],
    hdrs = [
        "rendering/rasterizer.hpp",
    ],
    deps = [
        ":tensor_config",
        ":utils",
        ":keyframe_batch",
        "//core/types",
        "//common:logging",
        "@libtorch//:torch",
        "@eigen",
        "//gaussian_splatting/gsplat:gsplat",
    ],
)

cc_test(
    name = "projection_test",
    srcs = ["rendering/test/projection_test.cpp"],
    deps = [
        ":rasterizer",
        "@com_google_googletest//:gtest_main",
    ],
)

# # Training Gaussian Splat Processor
# cc_library(
#     name = "training_gs_processor",
#     srcs = ["training_gs_processor.cpp"],
#     hdrs = ["training_gs_processor.hpp"],
#     deps = [
#         ":training_config",
#         ":keyframe_batch",
#         ":training_visualizer",
#         ":batch_trainer",
#         ":densification_controller",
#         ":bilateral_grid",
#         ":spherical_harmonics",
#         ":rasterizer",
#         "//core/storage",
#         "//core/types",
#         "//utils:stf",
#         "@libtorch//:torch",
#         "@opencv//:core",
#         "@opencv//:imgproc",
#         "//common:logging",
#     ],
# )

# Streaming Gaussian Splat Processor
cc_library(
    name = "streaming_gs_processor",
    srcs = ["streaming_gs_processor.cpp"],
    hdrs = ["streaming_gs_processor.hpp"],
    deps = [
        ":training_config",
        ":keyframe_batch",
        ":training_visualizer",
        ":foxglove_renderer",
        ":batch_trainer",
        ":bilateral_grid",
        ":rasterizer",
        ":utils",
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
        ":utils",
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
        "//gaussian_splatting/gsplat:gsplat",
    ],
)

cc_library(
    name = "standalone_gs",
    srcs = ["standalone_gs_processor.cpp"],
    hdrs = ["standalone_gs_processor.hpp"],
    deps = [
        ":training_config",
        ":keyframe_batch",
        ":training_visualizer",
        ":batch_trainer",
        ":densification_controller",
        ":bilateral_grid",
        ":rasterizer",
        ":utils",
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
# # Training processor test
# cc_test(
#     name = "training_gs_processor_test",
#     srcs = ["training_gs_processor_test.cpp"],
#     deps = [
#         ":training_gs_processor",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

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

cc_binary(
    name = "standalone_gs_processor",
    srcs = ["gs_processor_standalone_main.cpp"],
    deps = [
        ":standalone_gs",
        "//viz:visualize_gs_rerun",
        "//core/storage",
        "//common:logging",
    ],
)
cc_binary(
    name = "streaming_gs_processor_main",
    srcs = ["gs_processor_streaming_main.cpp"],
    deps = [
        ":streaming_gs_processor",
        "//viz:visualize_gs_rerun",
        "//core/storage",
        "//common:logging",
    ],
)
