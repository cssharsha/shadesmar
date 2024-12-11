def ros_repository():
    native.new_local_repository(
        name = "ros_core",
        build_file = "//third_party:ros.BUILD",
        path = "/opt/ros/humble",
    )

def ros_message_library(name, srcs, deps = None):
    """Rule for generating ROS2 message libraries."""
    if deps == None:
        deps = ["@ros_core//:core"]
    else:
        deps = deps + ["@ros_core//:core"]

    native.genrule(
        name = name + "_gen",
        srcs = srcs,
        outs = [name + ".hpp", name + ".cpp"],
        tools = ["@ros_core//:rosidl_generator"],
        cmd = """
            $(location @ros_core//:rosidl_generator) \
                --output-dir $(GENDIR) \
                --template-dir /opt/ros/humble/share/rosidl_generator_cpp \
                $(SRCS)
        """,
    )

    native.cc_library(
        name = name,
        srcs = [name + ".cpp"],
        hdrs = [name + ".hpp"],
        deps = deps,
    )