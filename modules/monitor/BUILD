load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libmonitor_component.so",
    copts = [
        "-DMODULE_NAME=\\\"monitor\\\"",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":monitor_component_lib",
    ],
)

cc_library(
    name = "monitor_component_lib",
    srcs = ["monitor_component.cc"],
    hdrs = [
        "monitor_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"monitor\\\"",
    ],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/monitor/proto:status_cc_proto",
        "//modules/sensors:realsense_api",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cc_binary(
    name = "realsense_monitor",
    srcs = ["realsense_monitor.cc"],
    copts = [
        "-DMODULE_NAME=\\\"monitor\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cpplint()
