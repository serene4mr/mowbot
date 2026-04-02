#!/usr/bin/env bash

set -e

# ---------------------------------------------------------
# Usage: ./publish-manifest.sh [TAG] [TARGET]
# Example: ./publish-manifest.sh latest main-cuda
# ---------------------------------------------------------

TAG=${1:-latest}
TARGET=${2:-main-cuda}
REGISTRY="ghcr.io/serene4mr/mowbot"

echo "⚙️ Creating multi-arch manifest for ${REGISTRY}:${TARGET}-${TAG}..."

# Create and push the versioned multi-arch manifest
docker manifest create ${REGISTRY}:${TARGET}-${TAG} \
  ${REGISTRY}:${TARGET}-${TAG}-amd64 \
  ${REGISTRY}:${TARGET}-${TAG}-arm64

docker manifest push ${REGISTRY}:${TARGET}-${TAG}

# If the tag is exactly "latest", also promote it as the root alias (e.g. mowbot:main-cuda)
if [ "$TAG" = "latest" ]; then
  echo "🚀 Promoting to root latest alias: ${REGISTRY}:${TARGET}..."
  docker manifest create ${REGISTRY}:${TARGET} \
    ${REGISTRY}:${TARGET}-${TAG}-amd64 \
    ${REGISTRY}:${TARGET}-${TAG}-arm64
  docker manifest push ${REGISTRY}:${TARGET}
fi

echo "✅ Successfully linked and pushed multi-arch manifest to GitHub!"
