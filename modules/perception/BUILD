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
        # ":control_gflags",
        "//cyber",
        "//modules/perception/proto:perception_cc_proto",
        "//modules/sensors/proto:sensors_cc_proto",
    ],
)

# cc_library(
#     name = "control_gflags",
#     srcs = ["control_gflags.cc"],
#     hdrs = [
#         "control_gflags.h",
#     ],
#     deps = [
#         "//external:gflags",
#     ],
# )

cpplint()
