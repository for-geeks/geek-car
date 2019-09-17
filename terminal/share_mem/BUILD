load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "test_share_memory_receiver",
    srcs = [
        "test2.cpp",
        "share_memory.h",
            ],
    copts = [
        '-Wall',
    ],
    deps = [
        ":share_memory_lib",
        #"-Icyber_bridge/exercise1/listener.h",
    ],
    linkopts = [ "-lpthread" ],
)

cc_binary(
    name = "test_share_memory_sender",
    srcs = [
        "test1.cpp",
        "share_memory.h",
            ],
    copts = [
        '-Wall',
    ],
    deps = [
        ":share_memory_lib",
        #"-Icyber_bridge/exercise1/listener.h",
    ],
    linkopts = [ "-lpthread -lumb" ],

)

cc_library(
    name = "share_memory_lib",
    srcs = ["share_memory.cpp"],
    hdrs = ["share_memory.h"],
    copts = [
        "-L/usr/lib -pthread -fpic -lrt",
    ],
    linkstatic = True,
    deps = [
        #"//cyber",
        #"//cyber_bridge/exercise1/proto:sensors_cc_proto",
        #"//gui_core/exercise1:exercise1",
    ],
    linkopts = [ "-lpthread" ],
)

cpplint()
