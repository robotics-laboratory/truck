load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_file")

new_local_repository(
    name = "galactic",
    path = "/opt/ros/galactic",
    build_file = "dep/galactic.BUILD",
)

http_file(
  name = "nlohmann_json",
  downloaded_file_path = "json.hpp",
  sha256 = "e832d339d9e0c042e7dff807754769d778cf5d6ae9730ce21eed56de99cb5e86",
  urls = ["https://github.com/nlohmann/json/releases/download/v3.10.5/json.hpp"],
)

http_file(
  name = "float_comparison",
  downloaded_file_path = "float_comparison.hpp",
  sha256 = "02c4a1920f41ce204ee490586157760eefa75732442d99976675e5de373a1d7b",
  urls = ["https://raw.githubusercontent.com/packedbread/float_comparison/master/float_comparison.hpp"]
)
