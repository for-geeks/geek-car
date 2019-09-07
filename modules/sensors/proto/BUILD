package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "sensors_cc_proto",
    deps = [
        ":point_cloud_proto",
        ":sensors_proto",
    ],
)

proto_library(
    name = "sensors_proto",
    srcs = [
        "sensors.proto",
    ],
)

proto_library(
    name = "point_cloud_proto",
    srcs = [
        "point_cloud.proto",
    ],
)
