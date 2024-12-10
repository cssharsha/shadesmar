def tbb_repository():
    native.new_local_repository(
        name = "tbb",
        build_file = "//third_party:tbb.BUILD",
        path = "/usr",
    )
