// Unified mowbot images. Context and tags are set by docker/build.sh.
group "default" {
  targets = ["runtime", "devel"]
}

target "runtime" {
  context    = "."
  dockerfile = "docker/Dockerfile"
  target     = "runtime"
}

target "devel" {
  context    = "."
  dockerfile = "docker/Dockerfile"
  target     = "devel"
}
