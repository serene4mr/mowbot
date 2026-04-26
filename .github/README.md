# GitHub Workflows

See [docker/README.md](../docker/README.md) and [README.md](../README.md) for the full image tag layout.

## `docker-build.yml`

Builds and pushes **linux/amd64** images only. Jetson (ARM64) images are built manually on-device with `./docker/build.sh --platform jetson`.

### Triggers

Runs on push/PR to `main` when any of the following paths change:

- `docker/**`
- `ansible/**`
- `ansible/playbooks/**`
- `setup-dev-env.sh`
- `env/**`
- `ansible-galaxy-requirements.yaml`
- `mowbot.repos`

Can also be triggered manually via `workflow_dispatch`.

### Tags produced

| Tag | When |
|---|---|
| `ghcr.io/serene4mr/mowbot:runtime-amd64-<sha>` | every push / PR build |
| `ghcr.io/serene4mr/mowbot:devel-amd64-<sha>` | every push / PR build |
| `ghcr.io/serene4mr/mowbot:runtime-amd64-latest` | push to `main` only |
| `ghcr.io/serene4mr/mowbot:devel-amd64-latest` | push to `main` only |

On a version tag (`v*`) the `<sha>` is replaced by the tag name.  
PRs build the image but do **not** push. There is no multi-arch manifest; Jetson uses separate tags.

### Authentication

| Purpose | Mechanism |
|---|---|
| Clone private repos during `docker build` | SSH key via `webfactory/ssh-agent` |
| Push images to GHCR | `GITHUB_TOKEN` (built-in, no PAT needed) |

---

## Setup

### Required secrets

| Secret | Purpose |
|---|---|
| `SSH_PRIVATE_KEY` | Cloning private repos during Docker build |

No PAT token is needed — `GITHUB_TOKEN` has `packages: write` granted directly in the workflow.

### 1. Generate an SSH key pair

```bash
ssh-keygen -t ed25519 -C "github-actions-mowbot" -f ~/.ssh/id_ed25519_mowbot_ci -N ""
```

### 2. Add the public key to your GitHub account

- Go to <https://github.com/settings/keys>
- Click **New SSH key**
- Title: `mowbot-github-actions`, Key type: Authentication Key
- Paste the contents of `~/.ssh/id_ed25519_mowbot_ci.pub`

### 3. Add the private key as a repository secret

- Go to <https://github.com/serene4mr/mowbot> → Settings → Secrets and variables → Actions
- Click **New repository secret**, Name: `SSH_PRIVATE_KEY`
- Paste the contents of `~/.ssh/id_ed25519_mowbot_ci`

### 4. Grant workflow write permissions

- Go to <https://github.com/serene4mr/mowbot> → Settings → Actions → General
- Under **Workflow permissions**, select **Read and write permissions**

---

## Key files

| File | Purpose |
|---|---|
| `.github/workflows/docker-build.yml` | CI workflow |
| `docker/build.sh` | Build script (`--platform`, `--target`, `--version`, `--push`) |
| `mowbot.repos` | vcstool manifest with SSH URLs for private repos |

`mowbot.repos` uses SSH URLs so the agent works during `vcs import`:

```yaml
repositories:
  core/mowbot_sdk:
    type: git
    url: git@github.com:serene4mr/mowbot_sdk.git
    version: main
  launcher/mowbot_launch:
    type: git
    url: git@github.com:serene4mr/mowbot_launch.git
    version: main
  sensor_component/mowbot_ext:
    type: git
    url: git@github.com:serene4mr/mowbot_ext.git
    version: main
  sensor_component/realsense-ros:
    type: git
    url: git@github.com:serene4mr/realsense-ros.git
    version: main
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `remote: Repository not found` during build | SSH key not configured | Verify `SSH_PRIVATE_KEY` secret and that the public key is added to your GitHub account |
| `denied: permission_denied: write_package` | Workflow permissions not set | Enable "Read and write permissions" in repo Actions settings |
| `invalid empty ssh agent socket` | `SSH_PRIVATE_KEY` secret missing or empty | Check the secret value and `webfactory/ssh-agent` step logs |
| Container fails to start locally (`render` group missing) | Host has no `render` group | Remove `--group-add render` from `.devcontainer/amd64/devcontainer.json` `runArgs` |
