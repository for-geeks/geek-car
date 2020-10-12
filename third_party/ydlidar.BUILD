licenses(["notice"])

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "ydlidar",
    srcs = glob([
        "local/lib/libydlidar_*.a",
    ]),
    hdrs = glob([
        "local/include/core/*/*.h",
        "local/include/src/*.h",
    ]),
    includes = ([
        "local/include/core",
        "local/include/src",
    ]),
)
