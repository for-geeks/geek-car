load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "librealsense_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":realsense_component_lib",
    ],
)

cc_library(
    name = "realsense_component_lib",
    srcs = ["realsense_component.cc"],
    hdrs = [
        "realsense_component.h",
    ],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        ":realsense_api",
        ":realsense_motion",
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors/device:d435_lib",
        "//modules/sensors/device:device_base",
        "//modules/sensors/device:t265_lib",
        "//modules/sensors/proto:sensors_cc_proto",
        "@eigen",
        "@opencv",
        "@pcl",
    ],
)

cc_library(
    name = "realsense_api",
    srcs = ["realsense.cc"],
    hdrs = ["realsense.h"],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        "//cyber",
        "@opencv",
        "@pcl",
    ],
)

cc_library(
    name = "realsense_motion",
    srcs = ["realsense_motion.h"],
    linkopts = [
        "-lrealsense2",
    ],
)

cpplint()
