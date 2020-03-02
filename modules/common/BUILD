load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "global_gflags",
    srcs = [
        "global_gflags.cc",
    ],
    hdrs = [
        "global_gflags.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"common\\\"",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "curve_fitting",
    srcs = ["curve_fitting.h"],
    copts = [
        "-DMODULE_NAME=\\\"common\\\"",
    ],
)

cc_library(
    name = "uart",
    srcs = ["uart.cc"],
    hdrs = ["uart.h"],
    copts = [
        "-DMODULE_NAME=\\\"common\\\"",
    ],
    deps = [
        "//cyber",
    ],
)

cpplint()
