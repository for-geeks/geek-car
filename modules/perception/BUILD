load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libperception_component.so",
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
    linkopts = [
        "-llz4",
    ],
    deps = [
        "@pcl",
        "@eigen",
    ],
)

cpplint()
