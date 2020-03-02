load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "pose_reader",
    srcs = [
        "pose_reader.cc",
    ],
    copts = [
        "-DMODULE_NAME=\\\"control\\\"",
    ],
    deps = [
        "//cyber",
    ],
)

cc_binary(
    name = "pose_listener",
    srcs = ["pose_listener.cc"],
    copts = [
        "-DMODULE_NAME=\\\"control\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    copts = [
        "-DMODULE_NAME=\\\"control\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/control/proto:control_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "libcalibration_component.so",
    copts = [
        "-DMODULE_NAME=\\\"control\\\"",
    ],
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
    copts = [
        "-DMODULE_NAME=\\\"control\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/control/proto:control_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cpplint()
