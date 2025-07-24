load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def gsplat_repository():
    git_repository(
        name = "gsplat",
        remote = "https://github.com/nerfstudio-project/gsplat.git",
        tag = "v1.5.3",
        init_submodules = True,
        build_file = "//third_party:gsplat.BUILD",
    )
