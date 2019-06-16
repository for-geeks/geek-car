workspace(name = "apollo")

# googletest (GTest and GMock)
new_http_archive(
    name = "gtest",
    build_file = "third_party/gtest.BUILD",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",
    strip_prefix = "googletest-release-1.8.0",
    url = "https://github.com/google/googletest/archive/release-1.8.0.tar.gz",
)

# gflags
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "466c36c6508a451734e4f4d76825cf9cd9b8716d2b70ef36479ae40f08271f88",
    strip_prefix = "gflags-2.2.0",
    url = "https://github.com/gflags/gflags/archive/v2.2.0.tar.gz",
)

bind(
    name = "gflags",
    actual = "@com_github_gflags_gflags//:gflags",
)

# glog
new_http_archive(
    name = "glog",
    build_file = "third_party/glog.BUILD",
    sha256 = "7580e408a2c0b5a89ca214739978ce6ff480b5e7d8d7698a2aa92fadc484d1e0",
    strip_prefix = "glog-0.3.5",
    url = "https://github.com/google/glog/archive/v0.3.5.tar.gz",
)

# Google Benchmark
new_http_archive(
    name = "benchmark",
    build_file = "third_party/benchmark.BUILD",
    sha256 = "e7334dd254434c6668e33a54c8f839194c7c61840d52f4b6258eee28e9f3b20e",
    strip_prefix = "benchmark-1.1.0",
    url = "https://github.com/google/benchmark/archive/v1.1.0.tar.gz",
)

# cpplint from google style guide
new_git_repository(
    name = "google_styleguide",
    build_file = "third_party/google_styleguide.BUILD",
    commit = "159b4c81bbca97a9ca00f1195a37174388398a67",
    remote = "https://github.com/google/styleguide.git",
)

# OpenCV 2.4.13.2
new_http_archive(
    name = "opencv2",
    build_file = "third_party/opencv2.BUILD",
    strip_prefix = "opencv-2.4.13.2",
    url = "https://github.com/opencv/opencv/archive/2.4.13.2.zip",
)

# glew
new_local_repository(
    name = "glew",
    build_file = "third_party/glew.BUILD",
    path = "/usr/include",
)

#protobuf 3.6.1
#http_archive(
#    name = "com_google_protobuf",
#    strip_prefix = "protobuf-3.6.1",
#    url = "file:///tmp/protobuf-cpp-3.6.1.tar.gz",
#)

#protobuf 3.3
http_archive(
    name = "com_google_protobuf",
    strip_prefix = "protobuf-3.5.1",
    url = "https://github.com/google/protobuf/releases/download/v3.5.1/protobuf-cpp-3.5.1.tar.gz",
)

# rtps
#new_http_archive(
#    name = "fastrtps",
#    build_file = "third_party/fastrtps.BUILD",
#    #strip_prefix = "Fast-RTPS-1.7.2",
#    url = "https://github.com/eProsima/Fast-RTPS/archive/v1.7.2.tar.gz",
#)

new_local_repository(
    name = "fastrtps",
    build_file = "third_party/fastrtps.BUILD",
    path = "/usr/local/fast-rtps",
)
#bind(
#    name = "fastrtps",
#    actual = "@fastrtps//:Fast-RTPS-1.7.2",
#)

# python
new_local_repository(
    name = "python27",
    build_file = "third_party/python27.BUILD",
    path = "/usr/local/Cellar/python@2/2.7.16/Frameworks/Python.framework/Versions/2.7/",
)
