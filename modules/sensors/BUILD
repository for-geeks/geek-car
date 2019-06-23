load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "realsense",
    srcs = ["realsense.cc"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_videoio",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
        "-lrealsense2",
    ],
    deps = [
        "//cyber",
        "//modules/sensors/proto:sensors_cc_proto",
    ]
)


cpplint()