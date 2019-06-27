package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "control_cc_proto",
    deps = [
        ":chassis_proto",
        ":control_proto",
    ],
)

proto_library(
    name = "chassis_proto",
    srcs = [
        "chassis.proto",
    ],
)

proto_library(
    name = "control_proto",
    srcs = [
        "control.proto",
    ],
)
