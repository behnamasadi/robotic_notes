#!/bin/bash
# Script to run GitHub Actions workflow locally using act

# Run the workflow (this will take a while as it builds everything)
# Use -W to run in the current directory instead of /github/workspace
# Use --container-architecture linux/amd64 to match GitHub runners
# Use -P to specify the runner image

echo "Running GitHub Actions workflow locally..."
echo "This will take a while as it builds all dependencies..."
echo ""

# Run with workflow_dispatch event (manual trigger)
act workflow_dispatch \
  -W . \
  --container-architecture linux/amd64 \
  -P ubuntu-latest=catthehacker/ubuntu:act-latest \
  --verbose

# Alternative: Run just the build job
# act -j build -W . --container-architecture linux/amd64 -P ubuntu-latest=catthehacker/ubuntu:act-latest
