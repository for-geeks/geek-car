package(default_visibility = ["//visibility:public"])

cc_library(
    name = "d435_lib",
    srcs = ["realsense_d435i.cc"],
    hdrs = [
        "realsense_d435i.h",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_calib3d",
        "-lopencv_highgui",
        "-lrealsense2",
    ],
    deps = [
        ":device_base",
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/control/proto:control_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
        "//modules/sensors:realsense_motion",
        "@eigen",
        "@pcl",
    ],
)

cc_library(
    name = "device_base",
    srcs = ["device_base.h"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_calib3d",
        "-lopencv_highgui",
        "-lrealsense2",
    ],
    deps = [
        "//cyber",
    ],
)
