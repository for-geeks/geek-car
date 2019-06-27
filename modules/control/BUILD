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

cc_library(
    name = "control_lib",
    srcs = [
        "Uart.cpp",
        "control_component.cc",
    ],
    hdrs = [
        "Uart.h",
        "control_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/control/proto:control_cc_proto",
    ],
)

cpplint()
