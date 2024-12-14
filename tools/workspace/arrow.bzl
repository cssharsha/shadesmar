def arrow_repository():
    native.new_local_repository(
        name = "arrow",
        build_file = "//third_party:arrow.BUILD",
        path = "/usr",
    )
