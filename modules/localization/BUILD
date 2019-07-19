load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map",
    srcs = [
        "map.cc",
    ],
    linkopts = [
        "-lrealsense",
    ],
)

cpplint()
