load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])



cc_library(
    name = "ncommon",
    srcs = [
        "ncommon.cc",
    ],
    hdrs = [
        "ncommon.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
)

cc_library(
    name = "linktrack_anchor_frame0",
    srcs = [
        "nlink_linktrack_anchor_frame0.cc",
    ],
    hdrs = [
        "nlink_linktrack_anchor_frame0.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        ":ncommon",
    ],
)

# cc_library(
#     name = "linktrack_node_frame0",
#     srcs = [
#         "nlink_linktrack_node_frame0.cc",
#     ],
#     hdrs = [
#         "nlink_linktrack_node_frame0.h",
#     ],
#     copts = [
#         "-DMODULE_NAME=\\\"sensors\\\"",
#     ],
#     deps = [
#         ":ncommon",
#     ],
# )

# cc_library(
#     name = "linktrack_node_frame1",
#     srcs = [
#         "nlink_linktrack_node_frame1.cc",
#     ],
#     hdrs = [
#         "nlink_linktrack_node_frame1.h",
#     ],
#     copts = [
#         "-DMODULE_NAME=\\\"sensors\\\"",
#     ],
#     deps = [
#         ":ncommon",
#     ],
# )

# cc_library(
#     name = "linktrack_node_frame2",
#     srcs = [
#         "nlink_linktrack_node_frame2.cc",
#     ],
#     hdrs = [
#         "nlink_linktrack_node_frame2.h",
#     ],
#     copts = [
#         "-DMODULE_NAME=\\\"sensors\\\"",
#     ],
#     deps = [
#         ":ncommon",
#     ],
# )

cc_library(
    name = "linktrack_tag_frame0",
    srcs = [
        "nlink_linktrack_tag_frame0.cc",
    ],
    hdrs = [
        "nlink_linktrack_tag_frame0.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"sensors\\\"",
    ],
    deps = [
        ":ncommon",
    ],
)