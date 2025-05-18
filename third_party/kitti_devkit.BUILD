# third_party/kitti_devkit.BUILD
# This is a HYPOTHETICAL BUILD file.
# Its content depends on the actual structure of the KITTI devkit.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kitti_devkit_parsing",
    srcs = glob([
        "cpp/src/parsers/*.cpp", # Assuming devkit has C++ parsers here
        # Add other relevant source files
    ]),
    hdrs = glob([
        "cpp/include/kitti_parsers/*.h", # Assuming headers are here
        # Add other relevant header files
    ]),
    copts = ["-std=c++17"],
    # Specify include paths if headers are not directly in the globbed paths relative to package root
    # includes = ["cpp/include/"],
    deps = [
        # Dependencies of the devkit code, if any (e.g., Eigen)
        # "@eigen//:eigen",
    ],
)

# You might have other targets for calibration tools, visualizers, etc.