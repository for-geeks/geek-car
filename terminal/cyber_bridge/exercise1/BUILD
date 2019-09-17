load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "image_reader",
    srcs = [
        "image_reader.cc",
        "listener.h",
            ],
    copts = [
        '-fpic -g -Wall',
    ],
    linkopts = [ "-lpthread -lnddsc -lnddscore -lumb -ldl -lnsl -lm -lrt" ],
    deps = [
        ":image_listener_lib",
    ],
)

cc_library(
    name = "image_listener_lib",
    srcs = ["listener.cc",

            ],
    hdrs = [
            "listener.h",

           ],
    #linkstatic = True,
    copts = [
        '-fpic -g -Wall',
    ],
    linkopts = [ "-lpthread -lnddsc -lnddscore -lumb -ldl -lnsl -lm -lrt" ],
    deps = [
        "//cyber",
        "//cyber_bridge/exercise1/proto:sensors_cc_proto",
        "//cyber_bridge/exercise1/proto:control_cc_proto",
        "//cyber_bridge/exercise1/proto:localization_cc_proto",
        "//cyber_bridge/exercise1/proto:planning_cc_proto",
        #"//cyber_bridge/exercise1/proto:chassis_cc_proto",
        "//share_mem:share_memory_lib",
        "//gui_core:umb_standard"
        #"-Ishare_mem/share_memory.h",
    ],

)

cpplint()
