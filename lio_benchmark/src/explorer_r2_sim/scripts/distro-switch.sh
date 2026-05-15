#!/usr/bin/env bash
# Switch the SDF plugin filenames between Gazebo Harmonic (gz-sim-*-system)
# and Gazebo Fortress (ignition-gazebo-*-system).
#
# Usage:
#   scripts/distro-switch.sh harmonic   # → gz-sim-* (default)
#   scripts/distro-switch.sh fortress   # → ignition-gazebo-*
#
# Operates in-place on worlds/*.sdf and models/explorer_r2/model.sdf.
set -euo pipefail

PKG_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET="${1:?usage: distro-switch.sh harmonic|fortress}"

case "$TARGET" in
  harmonic)
    FROM='ignition-gazebo'
    TO='gz-sim'
    ;;
  fortress)
    FROM='gz-sim'
    TO='ignition-gazebo'
    ;;
  *)
    echo "unknown target: $TARGET (expected harmonic|fortress)" >&2
    exit 1
    ;;
esac

cd "$PKG_ROOT"
files=( worlds/cave.sdf worlds/tunnel.sdf models/explorer_r2/model.sdf )
for f in "${files[@]}"; do
  [[ -f "$f" ]] || continue
  sed -i "s|filename=\"$FROM-\([a-z-]*\)-system\"|filename=\"$TO-\1-system\"|g" "$f"
  echo "patched $f"
done
echo
echo "Now: ln -sf env/$( [ "$TARGET" = harmonic ] && echo jazzy-harmonic.env || echo humble-fortress.env ) .env"
echo "Then: docker compose build sim"
