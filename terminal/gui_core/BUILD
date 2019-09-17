load("//tools/build_rules:qt.bzl", "qt_cc_library", "qt_ui_library")
package(default_visibility = ["//visibility:public"])
cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    #hdrs = ["cyber/cyber.h"],
    copts = [
        '-fpic -Wall',
        #"-Icyber/cyber",
    ],
    deps = [
        ":mainwindow",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "mainwindow",
    src = "mainwindow.cpp",
    hdr = "mainwindow.h",
    #normal_hdrs = ["exercise1.h",],
    copts = [
        '-fpic -Wall',
        "-Igui_core/exercise1",
    ],
    deps = [
        "@qt//:qt_core",
        "@qt//:qt_widgets",
        "@qt//:qt_gui",
        ":mainwindowui",
        "//gui_core/exercise1:exercise1",
    ],
)

qt_ui_library(
    name = "mainwindowui",
    ui = "mainwindow.ui",
    deps = ["@qt//:qt_widgets"],
)

cc_library(
    name = "umb_standard",
    srcs = ["umb_standard.cpp"],
    hdrs = ["umb_standard.h"],
    copts = [
        '-fpic -Wall -lnddsc -lnddscore -lumb -ldl -lnsl -lm -lrt',
        #"-Icyber/cyber",
    ],
    deps = [

    ],
)
