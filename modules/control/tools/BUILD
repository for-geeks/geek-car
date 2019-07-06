load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/control/proto:control_cc_proto",
    ],
)

# cc_binary(
#     name = "control_calibration",
#     deps = [
#         ":lib_control_calibration",
#     ],
# )

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
