# GitHub Workflows

This directory contains GitHub Actions workflows for the mowbot project.

## Current Workflows

### `docker-build.yml` - Docker Image Builder
**Purpose**: Automatically build and push Docker images to GitHub Container Registry using your existing `docker/build.sh` script

**Triggers**:
- Push to `main` branch (when Docker, Ansible, or setup files change)
- Pull requests to `main` branch (builds but doesn't push)
- Manual trigger (`workflow_dispatch`)

**What it builds**:
1. **Base Images** (`base`, `base-cuda`)
   - Foundation images with ROS and basic tools
   - Multi-platform: amd64, arm64

2. **Main Images** (`main-dev`, `main`)
   - Development and production images
   - Multi-platform: amd64, arm64

3. **CUDA Images** (`main-dev-cuda`, `main-cuda`)
   - GPU-enabled images for CUDA workloads
   - Platform: amd64 only

**Build Method**:
- Uses your existing `docker/build.sh` script
- Leverages Docker Bake for advanced build features
- Maintains consistency with local development workflow

**Image Tags**:
- `ghcr.io/serene4mr/mowbot:base-latest`
- `ghcr.io/serene4mr/mowbot:main-dev-latest`
- `ghcr.io/serene4mr/mowbot:main-latest`
- `ghcr.io/serene4mr/mowbot:main-dev-cuda-latest`
- `ghcr.io/serene4mr/mowbot:main-cuda-latest`

**Smart Features**:
- ✅ Uses your proven build script
- ✅ Only builds when relevant files change
- ✅ Only pushes on main branch (not PRs)
- ✅ Multi-platform support
- ✅ Consistent with local development

## How This Solves Your Devcontainer Problem

1. **Automated Builds**: Every push to main builds fresh images
2. **Available Images**: Images are pushed to `ghcr.io/serene4mr/mowbot:main-dev-latest`
3. **Devcontainer Ready**: Your devcontainer can now pull from the registry
4. **No More Local Builds**: No need to build images manually

## Usage

### Manual Trigger
1. Go to Actions tab in your repository
2. Select "Build and Push Docker Images"
3. Click "Run workflow"
4. Choose branch and click "Run workflow"

### Automatic Trigger
The workflow runs automatically when you:
- Push to `main` branch with changes to:
  - `docker/` directory
  - `ansible/` directory
  - `setup-dev-env.sh`
  - Environment files (`*.env`)
  - `ansible-galaxy-requirements.yaml`

### Viewing Results
- Check the Actions tab for build status
- View logs for debugging
- Images are available at `ghcr.io/serene4mr/mowbot`

## Future Workflows (Planned)

### 1. Testing Workflow
- YAML linting
- Ansible validation
- Docker build testing
- Security scanning

### 2. Release Workflow
- Create GitHub releases
- Tagged Docker images
- Release notes

### 3. Dependency Management
- Automated dependency updates
- Security vulnerability scanning

## Permissions

The workflow uses:
- `contents: read` - Read repository contents
- `packages: write` - Push to GitHub Container Registry

## Secrets

Uses the built-in `GITHUB_TOKEN` secret - no additional setup required.

## Troubleshooting

### Build Failures
1. Check the Actions tab for error logs
2. Verify Dockerfile syntax
3. Check build arguments
4. Ensure all required files exist

### Image Not Found
1. Wait for workflow to complete
2. Check if images were pushed successfully
3. Verify image tags in registry

### Permission Issues
1. Ensure repository has package write permissions
2. Check if `GITHUB_TOKEN` is available
3. Verify workflow permissions are set correctly
