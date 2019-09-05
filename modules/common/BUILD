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
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "curve_fitting",
    srcs = ["curve_fitting.h"],
)

cpplint()
