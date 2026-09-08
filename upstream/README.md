# Upstream Version Tracking

This directory tracks versions of upstream dependencies used in this project.

## Purpose

Instead of committing upstream source code (which would bloat the repository), we only track version information. The actual upstream repositories are cloned by scripts when needed.

## Files

Each `.version` file contains:
- Repository URL
- Commit hash or tag that was tested
- Notes about compatibility

## Current Upstream Dependencies

- `picamera2.version` - Picamera2 library version
- `yolov5-demo.version` - YOLOv5 reference version (if applicable)

## Usage

Scripts read these version files to:
1. Clone the correct upstream version
2. Apply patches
3. Verify compatibility

## Updating Versions

When upstream is updated:
1. Update the `.version` file with new commit/tag
2. Test patches against new version
3. Update patches if needed
