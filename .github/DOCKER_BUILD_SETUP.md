# Docker Build Setup Journey

This document chronicles the complete setup of automated Docker image building and pushing to GitHub Container Registry (GHCR) for the Mowbot project.

## 🎯 Initial Problem

**Devcontainer Build Error**: `Failed to install Cursor server: Failed to run devcontainer command: ... An error occurred building the image.`

**Root Cause**: Devcontainer trying to pull non-existent Docker images from GHCR.

**Solution**: Set up GitHub Actions CI/CD to build and push Docker images to GHCR.

## 🔍 Problems Encountered & Solutions

### Problem 1: Repository Access in GitHub Actions

**Issue**: `fatal: repository 'https://github.com/serene4mr/mowbot_sdk.git/' not found`

**Root Cause**: Private repositories not accessible with default authentication.

**Attempted Solutions**:
- ❌ PAT token with HTTPS URLs (vcstool strips auth tokens)
- ❌ Git URL rewriting (inconsistent behavior)
- ✅ **Final Solution**: SSH authentication with account-level SSH key

### Problem 2: SSH Authentication Setup

**Issue**: `Could not determine ref type of version: remote: Repository not found`

**Root Cause**: SSH key not properly configured for GitHub Actions.

**Solution**: 
- Generated Ed25519 SSH key pair
- Added public key to GitHub account SSH keys
- Added private key to repository secrets as `SSH_PRIVATE_KEY`

### Problem 3: Git Configuration Issues

**Issue**: SSH URLs in `mowbot.repos` but HTTPS URLs being used.

**Root Cause**: Uncommitted changes to `mowbot.repos` file.

**Solution**: Committed SSH URLs to `mowbot.repos` file.

### Problem 4: Docker Build SSH Agent

**Issue**: `ERROR: failed to convert agent config {default [] false}: invalid empty ssh agent socket`

**Root Cause**: SSH agent not available to Docker Buildx.

**Solution**: Export `SSH_AUTH_SOCK` environment variable.

### Problem 5: Docker Build SSH Privileges

**Issue**: `ERROR: additional privileges requested`

**Root Cause**: Docker Buildx needs explicit permission for SSH forwarding.

**Solution**: Added `--allow=ssh` flag to all `docker buildx bake` commands.

### Problem 6: Package Registry Permissions

**Issue**: `denied: permission_denied: write_package`

**Root Cause**: GITHUB_TOKEN lacks package write permissions.

**Solution**: Hybrid authentication approach:
- SSH key for repository access
- PAT token for Docker registry access

### Problem 7: Disk Space Issues

**Issue**: `ERROR: write /blobs/sha256/...: no space left on device`

**Root Cause**: GitHub Actions runner running out of disk space during large Docker builds.

**Solution**: Added disk cleanup steps and build optimizations.

## 🏗️ Final Architecture

### Authentication Strategy

**Hybrid Approach**:
- **SSH Key**: Repository cloning (secure, reliable)
- **PAT Token**: Docker registry access (required for GHCR)

### Build Process

**Sequential Builds**:
- Build AMD64 platform → Push → Cleanup
- Build ARM64 platform → Push → Cleanup
- Create multi-platform manifests

**Disk Space Management**:
- Cleanup before builds
- Cleanup between platform builds
- Cleanup after builds

### Security

- **SSH keys** stored as repository secrets
- **PAT token** with minimal required permissions
- **No credentials in code** or Docker images

## 📋 Setup Instructions

### 1. Generate SSH Key Pair

```bash
ssh-keygen -t ed25519 -C "github-actions" -f ~/.ssh/id_ed25519_github_actions -N ""
```

### 2. Add Public Key to GitHub Account

- Go to: https://github.com/settings/keys
- Click: "New SSH key"
- Title: `mowbot-github-actions`
- Key type: Authentication Key
- Key: Copy content of `~/.ssh/id_ed25519_github_actions.pub`

### 3. Add Private Key to Repository Secrets

- Go to: https://github.com/serene4mr/mowbot → Settings → Secrets and variables → Actions
- Click: "New repository secret"
- Name: `SSH_PRIVATE_KEY`
- Value: Copy content of `~/.ssh/id_ed25519_github_actions`

### 4. Configure PAT Token

- Go to: https://github.com/settings/tokens
- Generate new token with scopes:
  - `repo` (Full control of private repositories)
  - `read:packages` (Download packages)
  - `write:packages` (Upload packages)
- Add as repository secret: `PAT_TOKEN`

### 5. Repository Permissions

- Go to: https://github.com/serene4mr/mowbot → Settings → Actions → General
- Set: "Read and write permissions" for workflows

## 🔧 Key Files

### `.github/workflows/docker-build.yml`
Main workflow file with:
- SSH authentication setup
- Hybrid authentication (SSH + PAT)
- Disk space management
- Multi-platform builds

### `docker/build.sh`
Build script with:
- `--allow=ssh` flags for Docker builds
- Platform-specific builds
- Version tagging

### `mowbot.repos`
Repository configuration with SSH URLs:
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
```

## 🎯 Expected Results

### Successful Workflow Run

1. ✅ **Repository cloning** with SSH authentication
2. ✅ **Docker builds** with actual source code
3. ✅ **Package registry access** with PAT token
4. ✅ **Multi-platform images** pushed to GHCR
5. ✅ **Manifests created** for cross-platform support

### Available Images

```
ghcr.io/serene4mr/mowbot:base              # Multi-arch (AMD64 + ARM64)
ghcr.io/serene4mr/mowbot:base-latest       # Latest multi-arch
ghcr.io/serene4mr/mowbot:main-dev          # Multi-arch (AMD64 + ARM64)
ghcr.io/serene4mr/mowbot:main-dev-latest   # Latest multi-arch
ghcr.io/serene4mr/mowbot:main-dev-cuda     # AMD64 only
ghcr.io/serene4mr/mowbot:main-dev-cuda-latest # Latest CUDA
```

## 🚀 Benefits

### Security
- SSH authentication for repository access
- No credentials in Docker images
- Minimal PAT token permissions

### Reliability
- Hybrid authentication approach
- Disk space management
- Sequential builds with cleanup

### Maintainability
- Clear separation of concerns
- Well-documented setup process
- Reusable workflow configuration

## 🔄 Troubleshooting

### Common Issues

1. **SSH Authentication Fails**
   - Verify SSH key is added to GitHub account
   - Check SSH_PRIVATE_KEY secret is correct
   - Ensure mowbot.repos has SSH URLs

2. **Package Registry Access Fails**
   - Verify PAT_TOKEN secret has correct permissions
   - Check repository workflow permissions
   - Ensure PAT token has `write:packages` scope

3. **Disk Space Issues**
   - Workflow includes automatic cleanup
   - Builds are sequential to minimize space usage
   - Monitor disk usage in workflow logs

### Debug Steps

1. Check workflow logs for specific error messages
2. Verify all secrets are properly configured
3. Test SSH access manually if needed
4. Check GitHub repository permissions

---

**Last Updated**: August 2024  
**Status**: ✅ Complete and Working
