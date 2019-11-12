load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "rs_connection_check",
    srcs = ["rs_connection_check.cc"],
    linkopts = [
        "-lrealsense2",
    ],
    deps = [
        "//modules/common:global_gflags",
    ],
)

cc_binary(
    name = "image_save",
    srcs = ["image_save.cc"],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/sensors/proto:sensors_cc_proto",
        "@opencv",
    ],
)

cpplint()
