#!/usr/bin/env bash
# Deprecated: multi-arch manifests for a single tag are no longer used.
# Each platform has an explicit tag (e.g. runtime-amd64-<ver>, devel-jetson-l4t-r36.4-<ver>).
# See README.md and docker/README.md.
echo "publish-manifest.sh is deprecated. Remove any CI/cron references to this script." >&2
exit 1
