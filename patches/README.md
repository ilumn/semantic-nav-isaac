# Patches Directory

This directory contains replayable patches for upstream dependencies.

## Purpose

Patches are used to modify upstream projects (like Picamera2) without maintaining a fork. This keeps the repository clean and makes it easy to update when upstream changes.

## Structure

Each upstream project has its own subdirectory:
- `picamera2/` - Patches for the Picamera2 library

## Patch Files

Patch files follow naming convention: `0001-description.patch`, `0002-description.patch`, etc.

## Usage

Patches can be applied manually or via setup scripts. See `docs/patches/` for detailed documentation.

## Creating Patches

```bash
cd /path/to/upstream/repo
git format-patch -N <base-commit> --stdout > patches/<project>/0001-description.patch
```
