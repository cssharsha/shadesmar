def eigen_repository():
    native.new_local_repository(
        name = "eigen",
        build_file = "//third_party:eigen.BUILD",
        path = "/usr/include/eigen3",
    )
