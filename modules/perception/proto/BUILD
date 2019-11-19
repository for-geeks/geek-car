package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "perception_cc_proto",
    deps = [
        ":perception_proto_lib",
    ],
)

proto_library(
    name = "perception_proto_lib",
    srcs = [
        "perception.proto",
        "perception_obstacle.proto",
    ],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
