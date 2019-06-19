load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hello-greet",
    srcs = ["hello-greet.cc"],
    hdrs = ["hello-greet.h"],
)

cc_binary(
    name = "hello-world",
    srcs = ["hello-world.cc"],
    deps = [
        ":hello-greet",
    ],
)

cc_binary(
    name = "libexample.so",
    linkopt = ["-shared"],
    linkstatic = False,
    deps = [":example_lib"]
)

cc_library(
    name = "example_lib",
    srcs = [
        "example_component.cc",
    ],
    hdrs = [
        "example_component.h",
    ],
    deps = [
        "//cyber",
        "//cyber/example/proto:example_cc_proto",
    ],
)


cpplint()