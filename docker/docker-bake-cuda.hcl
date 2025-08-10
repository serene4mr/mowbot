group "default" {
  targets = [
    "main-dev-cuda",
    "main-runtime-cuda"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-main-dev-cuda" {}
target "docker-metadata-action-main-runtime-cuda" {}

target "main-dev-cuda" {
  inherits = ["docker-metadata-action-main-dev-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "main-dev-cuda"
}

target "main-runtime-cuda" {
  inherits = ["docker-metadata-action-main-runtime-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "main-runtime-cuda"
}