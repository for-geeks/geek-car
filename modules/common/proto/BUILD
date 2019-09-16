package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "error_code_proto",
    deps = [
        ":error_code_proto_lib",
    ],
)

proto_library(
    name = "error_code_proto_lib",
    srcs = [
        "error_code.proto",
    ],
)

cc_proto_library(
    name = "header_proto",
    deps = [
        ":header_proto_lib",
    ],
)

proto_library(
    name = "header_proto_lib",
    srcs = [
        "header.proto",
    ],
    deps = [
        ":error_code_proto_lib",
    ],
)

cc_proto_library(
    name = "geometry_proto",
    deps = [
        ":geometry_proto_lib",
    ],
)

proto_library(
    name = "geometry_proto_lib",
    srcs = [
        "geometry.proto",
    ],
)

cc_proto_library(
    name = "pnc_point_proto",
    deps = [
        ":pnc_point_proto_lib",
    ],
)

proto_library(
    name = "pnc_point_proto_lib",
    srcs = [
        "pnc_point.proto",
    ],
)
