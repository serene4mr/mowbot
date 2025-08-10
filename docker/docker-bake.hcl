group "default" {
  targets = [
    "main-dev",
    "main"
  ]
}


// For docker/metadata-action
target "docker-metadata-action-main-dev" {}
target "docker-metadata-action-main" {}

target "main-dev" {
  inherits = ["docker-metadata-action-main-dev"]
  dockerfile = "docker/Dockerfile"
  target = "main-dev"
}

target "main" {
  inherits = ["docker-metadata-action-main"]
  dockerfile = "docker/Dockerfile"
  target = "main"
}