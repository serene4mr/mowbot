group "default" {
  targets = [
    "main-dev-cuda",
    "main-cuda"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-main-dev-cuda" {}
target "docker-metadata-action-main-cuda" {}

target "main-dev-cuda" {
  inherits = ["docker-metadata-action-main-dev-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "main-dev-cuda"
}

target "main-cuda" {
  inherits = ["docker-metadata-action-main-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "main-cuda"
}