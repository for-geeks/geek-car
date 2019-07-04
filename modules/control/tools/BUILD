package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/control/proto:control_cc_proto",
    ],
)

cpplint()
