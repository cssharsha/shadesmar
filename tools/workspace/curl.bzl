def curl_repository():
    """Adds the pre-installed libcurl library to the workspace."""
    native.new_local_repository(
        name = "curl",
        path = "/usr",
        build_file = "//third_party:curl.BUILD",
    )
