# GitHub Workflows

This directory contains GitHub Actions workflows for the mowbot project.

## Workflows

### 1. `docker-build.yml`
**Purpose**: Build and push Docker images to GitHub Container Registry

**Triggers**:
- Push to `main` branch (when Docker, Ansible, or setup files change)
- Pull requests to `main` branch
- Manual trigger (`workflow_dispatch`)

**Jobs**:
- `build-base`: Builds base images (base, base-cuda)
- `build-main`: Builds main development and production images
- `build-cuda`: Builds CUDA-enabled images

**Features**:
- Multi-platform builds (amd64, arm64)
- Automatic tagging based on branch/PR/commit
- Conditional pushing (only pushes on main branch, not PRs)

### 2. `test.yml`
**Purpose**: Run tests, linting, and security scans

**Triggers**:
- Push to `main` branch
- Pull requests to `main` branch
- Manual trigger

**Jobs**:
- `lint`: YAML and Ansible linting
- `test-docker`: Docker build testing (no push)
- `security-scan`: Vulnerability scanning with Trivy

### 3. `release.yml`
**Purpose**: Create releases and push tagged images

**Triggers**:
- Push of version tags (e.g., `v1.0.0`)

**Features**:
- Builds all image variants for release
- Creates GitHub release
- Pushes images with semantic version tags

### 4. `dependabot.yml`
**Purpose**: Auto-merge Dependabot PRs

**Triggers**:
- Pull requests from Dependabot

**Features**:
- Auto-merges patch version updates
- Requires manual review for major/minor updates

## Configuration Files

### `.yamllint.yml`
YAML linting configuration for consistent formatting.

### `dependabot.yml`
Dependabot configuration for automated dependency updates.

## Usage

### Manual Workflow Execution
You can manually trigger workflows from the GitHub Actions tab:
1. Go to Actions tab in your repository
2. Select the workflow you want to run
3. Click "Run workflow"
4. Choose branch and parameters

### Creating Releases
To create a release:
```bash
git tag v1.0.0
git push origin v1.0.0
```

### Viewing Workflow Results
- Check the Actions tab for workflow status
- View logs for debugging
- Monitor security scan results in the Security tab

## Permissions

The workflows use the following permissions:
- `contents: read` - Read repository contents
- `packages: write` - Push to GitHub Container Registry
- `contents: write` - Create releases (release workflow only)

## Secrets

The workflows use the built-in `GITHUB_TOKEN` secret for authentication. No additional secrets are required.
