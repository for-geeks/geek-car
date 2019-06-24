load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "realsense_component",
    srcs = ["realsense_component.cc"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_videoio",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
        "-lopencv_calib3d",
        "-lrealsense2",
    ],
    hdrs = [
        "realsense_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/sensors/proto:sensors_cc_proto",
    ]
)

cc_library(
    name = "realsense",
    srcs = ["realsense.cc"],
    linkopts = [
        "-lrealsense2",
    ],
    hdrs = ["realsense.h"],
    deps = [
        
    ]

)


cpplint()