def ceres_repository():
    """Adds the pre-installed ceres library to the workspace."""
    native.new_local_repository(
        name = "ceres",
        path = "/usr",
        build_file = "//third_party:ceres.BUILD",
    )
