def colmap_repository():
    """Adds the pre-installed colmap library to the workspace."""
    native.new_local_repository(
        name = "colmap",
        path = "/usr/local",
        build_file = "//third_party:colmap.BUILD",
    )
