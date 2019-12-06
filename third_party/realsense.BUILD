licenses(["notice"])

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "realsense",
    includes = ["."],
    linkopts = select(
        {
            ":x86_mode": [
                "-L/usr/lib/x86_64-linux-gnu/",
            ],
            ":arm_mode": [
                "-L/usr/lib/aarch64-linux-gnu/",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ) + [
        "-lrealsense2",
    ],
)


config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)
