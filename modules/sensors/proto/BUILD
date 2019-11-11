package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "sensors_cc_proto",
    deps = [
        ":sensors_proto",
    ],
)

proto_library(
    name = "sensors_proto",
    srcs = [
        "mobileye.proto",
        "nooploop.proto",
        "pointcloud.proto",
        "radar.proto",
        "sensor_image.proto",
        "sensors.proto",
        "ultrasonic_radar.proto",
    ],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
