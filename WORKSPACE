workspace(name = "shadesmar")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# Load workspace rules
load("//tools/workspace:eigen.bzl", "eigen_repository")
load("//tools/workspace:gtsam.bzl", "gtsam_repository")
load("//tools/workspace:ros.bzl", "ros_repository")
# load("//tools/workspace:ros2.bzl", "ros2_repositories")
load("//tools/workspace:opencv.bzl", "opencv_repository")
load("//tools/workspace:boost.bzl", "boost_repository")
load("//tools/workspace:tbb.bzl", "tbb_repository")

# Protocol Buffers
http_archive(
    name = "com_google_protobuf",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.19.4.tar.gz"],
    strip_prefix = "protobuf-3.19.4",
)
load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
protobuf_deps()

# Google Test
http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/release-1.11.0.tar.gz"],
    strip_prefix = "googletest-release-1.11.0",
)

# # Boost
# http_archive(
#     name = "com_github_nelhage_rules_boost",
#     url = "https://github.com/nelhage/rules_boost/archive/refs/heads/master.tar.gz",
#     strip_prefix = "rules_boost-master",
# )

# load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
# boost_deps()

# CUDA Support
http_archive(
    name = "local_cuda",
    build_file = "//third_party:cuda.BUILD",
    urls = ["https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run"],
    sha256 = "...",  # Add appropriate SHA
)

# JSON for Modern C++
http_archive(
    name = "nlohmann_json",
    build_file = "//third_party:json.BUILD",
    urls = ["https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz"],
    sha256 = "...",  # Add appropriate SHA
)

# Rerun SDK
http_archive(
    name = "rerun_sdk",
    build_file = "//third_party:rerun.BUILD",
    urls = ["https://github.com/rerun-io/rerun/releases/download/0.9.0/rerun_cpp_sdk_0.9.0.tar.gz"],
    sha256 = "...",  # Add appropriate SHA
)

# Initialize repositories
eigen_repository()
gtsam_repository()
ros_repository()
# ros2_repositories()
opencv_repository()
boost_repository()
tbb_repository()

# Rules for building C/C++ dependencies
http_archive(
    name = "rules_foreign_cc",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.9.0.tar.gz",
    strip_prefix = "rules_foreign_cc-0.9.0",
)
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")
rules_foreign_cc_dependencies()

new_local_repository(
    name = "ros_humble",
    path = "/opt/ros/humble",
    build_file_content = """
filegroup(
    name = "share",
    srcs = glob(["share/**/*"]),
    visibility = ["//visibility:public"],
)
""",
)
