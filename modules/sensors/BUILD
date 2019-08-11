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
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_calib3d",
        "-lopencv_highgui",
        "-lrealsense2",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/control/proto:control_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
	":realsense_api",
    ],
)

cc_library(
    name = "realsense_api",
    srcs = ["realsense.h"],
    
)

cc_binary(
    name = "connect_check",
    srcs = ["connect_check.cc"],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        "//modules/common:global_gflags",
    ],
)

cc_binary(
    name = "image_save",
    srcs = ["image_save.cc"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cpplint()
