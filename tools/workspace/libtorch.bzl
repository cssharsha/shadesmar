load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def libtorch_repository():
    http_archive(
        name = "libtorch",
        urls = ["https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcu118.zip"],
        sha256 = "7796249faa9828a53b72d3f616fc97a1d9e87e6a35ac72b392ca1ddc7b125188", 
        strip_prefix = "libtorch",
        build_file = "//third_party:libtorch.BUILD",
    )