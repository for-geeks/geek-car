load("//tools/build_rules:qt.bzl", "qt_cc_library", "qt_ui_library")

qt_ui_library(
    name = "exercise1ui",
    ui = "exercise1.ui",
    deps = ["@qt//:qt_widgets"],
)

qt_cc_library(
    name = "exercise1",
    src = "exercise1.cpp",
    normal_hdrs = ["gui_core/umb_standard.hpp"],
    hdr = "exercise1.h",

    copts = [
        '-fpic -Wall',
        #"-L/usr/lib/ -lprotobuf",
    ],

    deps = [
        "@qt//:qt_core",
        "@qt//:qt_widgets",
        "@qt//:qt_gui",
    	":exercise1ui",
        "//share_mem:share_memory_lib",
        #"//cyber",
        #"//cyber_bridge/exercise1/proto:sensors_cc_proto",
    ],
)
