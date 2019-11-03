load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libperception_component.so",
    copts = [
        "-DMODULE_NAME=\\\"perception\\\"",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":perception_lib",
    ],
)

cc_library(
    name = "perception_lib",
    srcs = [
        "perception_component.cc",
    ],
    hdrs = [
        "perception_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"perception\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/perception/proto:perception_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
        "@pcl",
    ],
)

cc_binary(
    name = "pcd_read",
    srcs = ["pcd_read_test.cc"],
    copts = [
        "-DMODULE_NAME=\\\"perception\\\"",
    ],
    linkopts = [
        # "-lrealsense2",
    ],
    deps = [
        "@pcl",
    ],
)

cc_binary(
    name = "euclidean_segmentation",
    srcs = ["euclidean_segmentation.cc"],
    copts = [
        "-DMODULE_NAME=\\\"perception\\\"",
    ],
    linkopts = [
        "-llz4",
    ],
    deps = [
        "@eigen",
        "@pcl",
    ],
)

cpplint()
