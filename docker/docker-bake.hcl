group "default" {
  targets = [
    "mowbot-devel",
    "mowbot"
  ]
}


// For docker/metadata-action
target "docker-metadata-action-mowbot-devel" {}
target "docker-metadata-action-mowbot" {}

target "mowbot-devel" {
  inherits = ["docker-metadata-action-mowbot-devel"]
  dockerfile = "docker/Dockerfile"
  target = "mowbot-devel"
}

target "mowbot" {
  inherits = ["docker-metadata-action-mowbot"]
  dockerfile = "docker/Dockerfile"
  target = "mowbot"
}