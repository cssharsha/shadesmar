def opencv_repository():
    native.new_local_repository(
        name = "opencv",
        build_file = "//third_party:opencv.BUILD",
        path = "/usr",  # Changed to root of OpenCV installation
    )