load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hello-greet",
    srcs = ["hello-greet.cc"],
    hdrs = ["hello-greet.h"],
)

cc_binary(
    name = "hello-world",
    srcs = ["hello-world.cc"],
    deps = [
        ":hello-greet",
    ],
)

cc_binary(
    name = "libexample.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":example_lib"],
)

cc_library(
    name = "example_lib",
    srcs = [
        "example_component.cc",
    ],
    hdrs = [
        "example_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

cc_binary(
    name = "talker",
    srcs = ["talker.cc"],
    deps = [
        "//cyber",
        "//modules/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "performance_talker",
    srcs = ["performance_talker.cc"],
    deps = [
        "//cyber",
    ],
)

cc_binary(
    name = "listener",
    srcs = ["listener.cc"],
    deps = [
        "//cyber",
        "//modules/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "performance_listener",
    srcs = ["performance_listener.cc"],
    deps = [
        "//cyber",
    ],
)

cc_binary(
    name = "paramserver",
    srcs = ["paramserver.cc"],
    deps = [
        "//cyber",
        "//cyber/parameter",
    ],
)

cc_binary(
    name = "service",
    srcs = ["service.cc"],
    deps = [
        "//cyber",
        "//modules/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "record",
    srcs = ["record.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
    ],
)

cc_binary(
    name = "camera",
    srcs = ["camera.cc"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//cyber",
        "//modules/examples/proto:examples_cc_proto",
    ],
)

cpplint()
