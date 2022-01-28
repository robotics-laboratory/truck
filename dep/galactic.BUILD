cc_library(
    name = "all_galactic",
    srcs = glob(["lib/**/*.so"], exclude=["lib/python*/**/*.so"]),
    hdrs = glob(["include/**"]),
    strip_include_prefix = "include/",
    visibility = ["//visibility:public"],
    linkopts = ["-lpthread"],
)

# for now all_galactic will do, but in future we should generate/write up correct dependencies here
#
# cc_library(
#     name = "rclcpp",
#     srcs = ["lib/librclcpp.so"],
#     hdrs = glob(["include/rc*/**"]),
#     strip_include_prefix = "include/",
#     visibility = ["//visibility:public"],
#     deps = [
#         "@galactic//:rmw",
#         "@galactic//:rosidl",
#         "@galactic//:builtin_interfaces",
#         "@galactic//:tracetools",
#         "@galactic//:libstatistics_collector",
#     ],
# )

# cc_library(
#     name = "rmw",
#     srcs = glob(["lib/librmw*.so"]),
#     hdrs = glob(["include/rmw*/**"]),
#     strip_include_prefix = "include/",
#     visibility = ["//visibility:public"],
# )

# cc_library(
#     name = "rosidl",
#     srcs = glob(["lib/librosidl_*.so"]),
#     hdrs = glob(["include/rosidl_*/**"]),
#     strip_include_prefix = "include/",
#     visibility = ["//visibility:public"],
# )

# cc_library(
#     name = "builtin_interfaces",
#     hdrs = glob(["include/builtin_interfaces/**"]),
#     strip_include_prefix = "include/",
#     visibility = ["//visibility:public"],
# )

# cc_library(
#     name = "tracetools",
#     srcs = glob(["lib/libtracetools*.so"]),
#     hdrs = glob(["include/tracetools/**"]),
#     strip_include_prefix = "include/",
#     visibility = ["//visibility:public"],
# )

# cc_library(
#     name = "libstatistics_collector",
#     srcs = ["lib/liblibstatistics_collector.so"],
#     hdrs = glob(["include/libstatistics_collector/**"]),
#     strip_include_prefix = "include/",
#     visibility = ["//visibility:public"],    
# )