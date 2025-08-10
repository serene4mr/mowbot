group "default" {
  targets = [
    "main-dev",
    "main-runtime"
  ]
}


// For docker/metadata-action
target "docker-metadata-action-main-dev" {}
target "docker-metadata-action-main-runtime" {}

target "main-dev" {
  inherits = ["docker-metadata-action-main-dev"]
  dockerfile = "docker/Dockerfile"
  target = "main-dev"
}

target "main-runtime" {
  inherits = ["docker-metadata-action-main-runtime"]
  dockerfile = "docker/Dockerfile"
  target = "main-runtime"
}