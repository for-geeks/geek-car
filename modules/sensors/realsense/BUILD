package(default_visibility = ["//visibility:public"])

cc_library(
    name = "d435_lib",
    srcs = ["realsense_d435.cc"],
    hdrs = [
        "realsense_d435.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        ":device_base",
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors:realsense_api",
        "//modules/sensors:realsense_motion",
        "//modules/sensors/proto:sensors_cc_proto",
        "@eigen",
        "@pcl",
        "@opencv",
    ],
)

cc_library(
    name = "d435i_lib",
    srcs = ["realsense_d435i.cc"],
    hdrs = [
        "realsense_d435i.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        ":device_base",
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors:realsense_api",
        "//modules/sensors:realsense_motion",
        "//modules/sensors/proto:sensors_cc_proto",
        "@eigen",
        "@pcl",
        "@vtk",
        "@opencv",
    ],
)

cc_library(
    name = "t265_lib",
    srcs = ["realsense_t265.cc"],
    hdrs = [
        "realsense_t265.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        ":device_base",
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors:realsense_api",
        "//modules/sensors:realsense_motion",
        "//modules/sensors/proto:sensors_cc_proto",
        "@opencv",
    ],
)

cc_library(
    name = "device_base",
    srcs = ["device_base.h"],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        "//cyber",
        "//modules/control/proto:control_cc_proto",
        "@opencv",
        "@eigen",
    ],
)
