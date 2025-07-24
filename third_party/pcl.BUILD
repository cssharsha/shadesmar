package(default_visibility = ["//visibility:public"])

# Expose a minimal subset of PCL required initially:
# - pcl_common (base types/utilities)
# - pcl_kdtree (KdTree functionality)
# - headers include glob covers all PCL headers so downstream users can include
#   other modules later without changing the include roots.

PCL_INCLUDE_DIRS = [
    # PCL installs headers under a versioned include dir. List common versions
    # so includes like <pcl/...> work across environments.
    "include/pcl-1.10",
    "include/pcl-1.11",
    "include/pcl-1.12",
    "include/pcl-1.13",
]

PCL_HEADER_GLOBS = [
    "include/pcl-*/pcl/**/*.h",
    "include/pcl-*/pcl/**/*.hpp",
]

cc_library(
    name = "common",
    srcs = glob([
        # Typical multi-arch lib path on Debian/Ubuntu
        "lib/x86_64-linux-gnu/libpcl_common.so*",
        # Fallbacks for other distros/layouts (non-fatal if they don't exist)
        "lib/libpcl_common.so*",
    ]),
    hdrs = glob(PCL_HEADER_GLOBS),
    includes = PCL_INCLUDE_DIRS,
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "kdtree",
    srcs = glob([
        "lib/x86_64-linux-gnu/libpcl_kdtree.so*",
        "lib/libpcl_kdtree.so*",
    ]),
    hdrs = glob(PCL_HEADER_GLOBS),
    includes = PCL_INCLUDE_DIRS,
    deps = [
        ":common",
        "@eigen",
    ],
)

# PCL often vendors nanoflann as headers. Expose them (if present) so projects
# can include <pcl/3rdparty/nanoflann.hpp> directly. This target is header-only.
cc_library(
    name = "nanoflann_headers",
    hdrs = glob([
        "include/pcl-*/pcl/3rdparty/nanoflann.hpp",
    ]),
    includes = PCL_INCLUDE_DIRS,
)
