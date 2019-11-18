package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "vtk",
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu/",
        "-lvtkCommonCore-6.3",
    ],
    hdrs = glob([
        "include/vtk-6.3/*.h",
    ]),
    includes = ["include/vtk-6.3"],
)
