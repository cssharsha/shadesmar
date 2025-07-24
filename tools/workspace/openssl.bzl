def openssl_repository():
    """Adds the pre-installed openssl library to the workspace."""
    native.new_local_repository(
        name = "openssl",
        path = "/usr",
        build_file = "//third_party:openssl.BUILD",
    )
