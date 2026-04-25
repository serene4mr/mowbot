# GitHub Workflows

## `docker-build.yml`

Builds and pushes **linux/amd64** images only (Jetson images are built manually on device).

**Triggers**: changes under `docker/`, `ansible/`, `setup-dev-env.sh`, `env/`, `ansible-galaxy-requirements.yaml`, `mowbot.repos`.

**Tags produced** (examples):

- `ghcr.io/serene4mr/mowbot:runtime-amd64-<sha-or-tag>`
- `ghcr.io/serene4mr/mowbot:devel-amd64-<sha-or-tag>`
- On `main`, also: `runtime-amd64-latest`, `devel-amd64-latest` (aliases via `docker buildx imagetools create`)

**PRs**: build with `--load` only (no push).

There is **no** multi-arch manifest for a single tag; Jetson uses separate `*-jetson-l4t-r36.4-*` tags when built with `./docker/build.sh --platform jetson`.

See the [docker README](../docker/README.md) and [root README](../README.md) for the full tag layout.
