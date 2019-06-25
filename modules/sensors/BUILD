load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "librealsense_component.so",
    #linkopts = ["-shared"],
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
        "realsense.h",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_videoio",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
        "-lopencv_calib3d",
        "-lrealsense2",
    ],
    deps = [
        "//cyber",
        "//modules/sensors/proto:sensors_cc_proto",
    ]
)

cpplint()