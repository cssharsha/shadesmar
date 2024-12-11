load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _ros2_deps_installer_impl(repository_ctx):
    repository_ctx.file("BUILD", """
sh_binary(
    name = "install_deps",
    srcs = ["install.sh"],
    visibility = ["//visibility:public"],
)
""")

    script_content = """#!/bin/bash
set -e
apt-get update
apt-get install -y {}
rosdep update
rosdep install --from-paths . -y
""".format(" ".join(repository_ctx.attr.system_deps))

    repository_ctx.file("install.sh", script_content, executable=True)

ros2_deps_installer = repository_rule(
    implementation = _ros2_deps_installer_impl,
    attrs = {
        "system_deps": attr.string_list(),
    },
)