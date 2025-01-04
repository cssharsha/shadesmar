workspace(name = "shadesmar")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# Add rules_cc
http_archive(
    name = "rules_cc",
    urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.0.9/rules_cc-0.0.9.tar.gz"],
    sha256 = "2037875b9a4456dce4a79d112a8ae885bbc4aad968e6587dca6e64f3a0900cdf",
    strip_prefix = "rules_cc-0.0.9",
)

load("@rules_cc//cc:repositories.bzl", "rules_cc_dependencies")
rules_cc_dependencies()

# Add bazel_features
http_archive(
    name = "bazel_features",
    sha256 = "9fcb3d7cbe908772462aaa52f02b857a225910d30daa3c252f670e3af6d8036d",
    strip_prefix = "bazel_features-1.0.0",
    url = "https://github.com/bazel-contrib/bazel_features/releases/download/v1.0.0/bazel_features-v1.0.0.tar.gz",
)

# Load workspace thirdparty deps
load("//tools/workspace:eigen.bzl", "eigen_repository")
load("//tools/workspace:gtsam.bzl", "gtsam_repository")
load("//tools/workspace:opencv.bzl", "opencv_repository")
load("//tools/workspace:boost.bzl", "boost_repository")
load("//tools/workspace:tbb.bzl", "tbb_repository")
load("//tools/workspace:arrow.bzl", "arrow_repository")

eigen_repository()
gtsam_repository()
opencv_repository()
boost_repository()
tbb_repository()
arrow_repository()

http_archive(
    name = "bazel_skylib",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.3.0/bazel-skylib-1.3.0.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.3.0/bazel-skylib-1.3.0.tar.gz",
    ],
    sha256 = "74d544d96f4a5bb630d465ca8bbcfe231e3594e5aae57e1edbf17a6eb3ca2506",
)
load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

# Add platform definitions
http_archive(
    name = "platforms",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/platforms/releases/download/0.0.7/platforms-0.0.7.tar.gz",
        "https://github.com/bazelbuild/platforms/releases/download/0.0.7/platforms-0.0.7.tar.gz",
    ],
    sha256 = "3a561c99e7bdbe9173aa653fd579fe849f1d8d67395780ab4770b1f381431d51",
)
register_execution_platforms("@platforms//:all")

# Python rules setup
http_archive(
    name = "rules_python",
    sha256 = "7a005bc3f1938fb04643b6637445d4d62c2dee545b2b18a17c6c561c67d2b174",
    strip_prefix = "rules_python-0.36.0",
    url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.36.0.zip",
)

# Initialize Python rules and load necessary functions
load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")
py_repositories()

# Register Python toolchain
python_register_toolchains(
    name = "python3_10",
    python_version = "3.10",
)

# Load pip_parse
load("@rules_python//python:pip.bzl", "pip_parse")

# Load the interpreter path for later use
load("@python3_10//:defs.bzl", "interpreter")

http_archive(
    name = "rules_license",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_license/releases/download/1.0.0/rules_license-1.0.0.tar.gz",
        "https://github.com/bazelbuild/rules_license/releases/download/1.0.0/rules_license-1.0.0.tar.gz",
    ],
    sha256 = "26d4021f6898e23b82ef953078389dd49ac2b5618ac564ade4ef87cced147b38",
)

# Protobuf
http_archive(
    name = "rules_proto",
    sha256 = "0e5c64a2599a6e26c6a03d6162242d231ecc0de219534c38cb4402171def21e8",
    strip_prefix = "rules_proto-7.0.2",
    url = "https://github.com/bazelbuild/rules_proto/releases/download/7.0.2/rules_proto-7.0.2.tar.gz",
)

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies")
rules_proto_dependencies()

load("@rules_proto//proto:setup.bzl", "rules_proto_setup")
rules_proto_setup()

http_archive(
    name = "com_google_protobuf",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.12.4.tar.gz"],
    strip_prefix = "protobuf-3.12.4",
    sha256 = "512e5a674bf31f8b7928a64d8adf73ee67b8fe88339ad29adaa3b84dbaa570d8",
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
protobuf_deps()

# Google Test
http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/release-1.11.0.tar.gz"],
    strip_prefix = "googletest-release-1.11.0",
)

# CUDA Support
http_archive(
    name = "local_cuda",
    build_file = "//third_party:cuda.BUILD",
    urls = ["https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run"],
    # sha256 = "...",
)

# JSON for Modern C++
http_archive(
    name = "nlohmann_json",
    urls = ["https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz"],
    strip_prefix = "json-3.11.3",
)

# Add Rerun SDK
http_archive(
    name = "rerun_sdk",
    urls = ["https://github.com/rerun-io/rerun/releases/download/0.21.0/rerun_cpp_sdk.zip"],
    build_file = "//third_party:rerun.BUILD",
)

# ROS2 dependencies
http_archive(
    name = "com_github_mvukov_rules_ros2",
    sha256 = "32bc90fa947328dfc42f46242d3241ae930acd00a9ccbcbea64918aa6d94ba91",
    strip_prefix = "rules_ros2-19d0a447d5d1226897ae032ca6f31548c0b02075",
    url = "https://github.com/mvukov/rules_ros2/archive/19d0a447d5d1226897ae032ca6f31548c0b02075.tar.gz",
)

load("@com_github_mvukov_rules_ros2//repositories:repositories.bzl", "ros2_workspace_repositories", "ros2_repositories")
ros2_workspace_repositories()
ros2_repositories()

load("@com_github_mvukov_rules_ros2//repositories:deps.bzl", "ros2_deps")
ros2_deps()

# Parse pip dependencies for ROS2
pip_parse(
    name = "rules_ros2_pip_deps",
    python_interpreter_target = interpreter,
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()

# Hedron's Compile Commands Extractor for Bazel
# https://github.com/hedronvision/bazel-compile-commands-extractor
http_archive(
    name = "hedron_compile_commands",
    url = "https://github.com/mikael-s-persson/bazel-compile-commands-extractor/archive/fix/syntax_only_hdr_processing.tar.gz",
    strip_prefix = "bazel-compile-commands-extractor-fix-syntax_only_hdr_processing",
)
load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")
hedron_compile_commands_setup()
load("@hedron_compile_commands//:workspace_setup_transitive.bzl", "hedron_compile_commands_setup_transitive")
hedron_compile_commands_setup_transitive()
load("@hedron_compile_commands//:workspace_setup_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive")
hedron_compile_commands_setup_transitive_transitive()
load("@hedron_compile_commands//:workspace_setup_transitive_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive_transitive")
hedron_compile_commands_setup_transitive_transitive_transitive()

# Add gflags (required by glog)
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
    strip_prefix = "gflags-2.2.2",
    urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
)

# Add glog
http_archive(
    name = "com_github_google_glog",
    sha256 = "8a83bf982f37bb70825df71a9709fa90ea9f4447fb3c099e1d720a439d88bad6",
    strip_prefix = "glog-0.6.0",
    urls = ["https://github.com/google/glog/archive/v0.6.0.tar.gz"],
)
