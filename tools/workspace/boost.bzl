def boost_repository():
    native.new_local_repository(
        name = "boost",
        build_file = "//third_party:boost.BUILD",
        path = "/usr",  # Root of Boost installation
    )