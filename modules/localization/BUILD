load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

#cc_binary(
#    name = "map",
#    srcs = [
#        "map.cc",
#    ],
#    linkopts = [
#        "-lrealsense",
#    ],
#)

cc_binary(
    name = "liblocalization_component.so",
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":localization_lib",
    ],
)

cc_library(
    name = "localization_lib",
    srcs = [
        "localization_component.cc",
    ],
    hdrs = [
        "localization_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
        "@apriltag",
        "@opencv",
    ],
)

cc_binary(
    name = "libapriltag_component.so",
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":apriltag_lib",
    ],
)


cc_library(
    name = "apriltag_lib",
    srcs = [
        "apriltag_component.cc",
    ],
    hdrs = [
        "apriltag_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    linkopts = [
        "-lrealsense2",
        "-lapriltag",
    ],
    deps = [
        "//cyber",
        "//modules/common:global_gflags",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
        "@apriltag",
        "@opencv",
    ],
)

cpplint()
