def freeimage_repository():
    """Adds the pre-installed freeimage library to the workspace."""
    native.new_local_repository(
        name = "freeimage",
        path = "/usr",
        build_file = "//third_party:freeimage.BUILD",
    )
