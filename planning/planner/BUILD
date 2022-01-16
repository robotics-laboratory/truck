cc_library(
    name = "nlohmann_json_lib",
    hdrs = ["@nlohmann_json//file"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "/file",
    include_prefix = "json",
    linkstatic = True,
)

cc_library(
    name = "float_comparison_lib",
    hdrs = ["@float_comparison//file"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "/file",
    include_prefix = "float_comparison",
    linkstatic = True,
)

cc_binary(
    name = "planner",
    srcs = ["main.cpp"],
    deps = ["//planning/planner:nlohmann_json_lib", "//planning/planner:float_comparison_lib"],
)
