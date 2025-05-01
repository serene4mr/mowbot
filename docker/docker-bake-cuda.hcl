group "default" {
  targets = [
    "mowbot-devel-cuda",
    "mowbot-cuda"
  ]
}

// For docker/metadata-action
target "docker-metadata-action-mowbot-devel-cuda" {}
target "docker-metadata-action-mowbot-cuda" {}

target "mowbot-devel-cuda" {
  inherits = ["docker-metadata-action-mowbot-devel-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "mowbot-devel-cuda"
}

target "mowbot-cuda" {
  inherits = ["docker-metadata-action-mowbot-cuda"]
  dockerfile = "docker/Dockerfile"
  target = "mowbot-cuda"
}