load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "pose_reader",
    srcs = [
	"reader_pose.cc",
	],
    deps = [
	"//cyber",
	":io",
    ],
)
cc_library(
    name = "io",
    srcs = ["io.h"],
)

cc_binary(
    name = "pose_listener",
    srcs = ["pose_listener.cc"],
    deps = [
	"//cyber",
	"//modules/sensors/proto:sensors_cc_proto",
    ],
)

cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/control/proto:control_cc_proto",
    ],
)

cc_binary(
    name = "libcalibration_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":lib_control_calibration",
    ],
)

cc_library(
    name = "lib_control_calibration",
    srcs = [
        "control_calibration.cc",
    ],
    hdrs = [
        "control_calibration.h",
    ],
    deps = [
        "//cyber",
        "//modules/control/proto:control_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cpplint()
