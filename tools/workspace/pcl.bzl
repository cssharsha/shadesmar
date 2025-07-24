def pcl_repository():
    """Adds the pre-installed PCL library to the workspace.

    This expects PCL to be available under the system prefix (default /usr).
    The accompanying BUILD file at //third_party:pcl.BUILD exposes subsets
    like :common and :kdtree, plus header-only nanoflann (if present).
    """
    native.new_local_repository(
        name = "pcl",
        build_file = "//third_party:pcl.BUILD",
        path = "/usr",
    )

