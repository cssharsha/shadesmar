def gtsam_repository():
    native.new_local_repository(
        name = "gtsam",
        build_file = "//third_party:gtsam.BUILD",
        path = "/usr/local",
    )
