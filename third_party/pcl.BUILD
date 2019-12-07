licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pcl",
    defines = ["PCL_NO_PRECOMPILE"],
    includes = [
        ".",
    ],
    linkopts = select(
        {
            ":x86_mode": [
                "-L/usr/local/lib/",
            ],
            ":arm_mode": [
                "-L/usr/lib/aarch64-linux-gnu/",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ) + [
        "-lboost_system",
        "-lpcl_common",
        "-lpcl_features",
        "-lpcl_filters",
        "-lpcl_io_ply",
        "-lpcl_io",
        "-lpcl_kdtree",
        "-lpcl_keypoints",
        "-lpcl_ml",
        "-lpcl_octree",
        "-lpcl_outofcore",
        "-lpcl_people",
        "-lpcl_recognition",
        "-lpcl_registration",
        "-lpcl_sample_consensus",
        "-lpcl_search",
        "-lpcl_segmentation",
        "-lpcl_stereo",
        "-lpcl_surface",
        "-lpcl_tracking",
        "-lpcl_visualization",
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
