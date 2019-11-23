load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "librealsense_component.so",
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
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
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        ":realsense_api",
        ":realsense_motion",
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors/proto:sensors_cc_proto",
        "//modules/perception/proto:perception_cc_proto",
        "//modules/sensors/realsense:d435i_lib",
        "//modules/sensors/realsense:device_base",
        "//modules/sensors/realsense:t265_lib",
        "//modules/sensors/realsense:d435_lib",
    ],
)

cc_library(
    name = "realsense_api",
    srcs = ["realsense.cc"],
    hdrs = ["realsense.h"],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
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
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    linkopts = [
        "-lrealsense2",
    ],
)

cc_binary(
    name = "libnooploop_component.so",
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":nooploop_component_lib",
    ],
)

cc_library(
    name = "nooploop_component_lib",
    srcs = ["nooploop_component.cc"],
    hdrs = [
        "nooploop_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/common:uart",
        "//modules/sensors/nooploop:linktrack_tag_frame0",
        "//modules/sensors/nooploop:ncommon",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cpplint()
