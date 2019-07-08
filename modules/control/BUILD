load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":control_lib",
    ],
)

cc_binary(
    name = "control_reference_gen",
    #linkshared = True,
    #linkstatic = False,
    srcs = [
        "control_reference_gen.cc",
    ],
    deps = [
        "//cyber",
        "//modules/control/proto:control_cc_proto",
    ],
)

cc_binary(
    name = "libchassis_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":chassis_lib",
    ],
)

cc_library(
    name = "control_lib",
    srcs = [
        "control_component.cc",
    ],
    hdrs = [
        "control_component.h",
    ],
    deps = [
        ":control_gflags",
        "//cyber",
        "//modules/control/proto:control_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cc_library(
    name = "chassis_lib",
    srcs = [
        "Uart.cpp",
        "chassis_component.cc",
    ],
    hdrs = [
        "Uart.h",
        "chassis_component.h",
    ],
    deps = [
        ":control_gflags",
        "//cyber",
        "//modules/control/proto:control_cc_proto",
    ],
)

cc_library(
    name = "control_gflags",
    srcs = ["control_gflags.cc"],
    hdrs = [
        "control_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
