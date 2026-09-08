# Implementation Checklist

This file tracks the concrete build-out of the monocular semantic mapping MVP in this repository.

Current target stack:

- Python 3.12 in `.venv-models`
- `pycolmap` for reconstruction
- Ultralytics `YOLO-World` for open-vocabulary detection
- FastAPI for the demo backend API
- Static frontend with an in-repo 3D viewer

## Foundation

- [x] Read and distill `Monocular Semantic Mapping Plan.md`
- [x] Choose an initial MVP stack that can run locally in this workspace
- [x] Stand up a dedicated Python 3.12 environment for the proper model stack
- [x] Install the intended core stack (`pycolmap`, `ultralytics`, `fastapi`, `uvicorn`, `pyyaml`)
- [x] Verify `pycolmap` and `YOLOWorld` import successfully
- [x] Create the repository layout for backend, data, outputs, and demo assets
- [x] Define ontology and schema-aligned JSON artifacts
- [x] Document the MVP scope versus later COLMAP and YOLO-World upgrades

## Backend Pipeline

- [x] Implement video ingest and processing job management
- [x] Implement frame extraction and video metadata capture
- [x] Implement keyframe selection
- [x] Implement sparse monocular reconstruction baseline
- [x] Implement detector abstraction
- [x] Implement YOLO-World detector integration
- [x] Implement short-term 2D tracklets
- [x] Implement 3D grounding from 2D observations into the reconstruction frame
- [x] Implement persistent entity memory
- [x] Implement place inference
- [x] Implement relation inference
- [x] Implement scene graph assembly and JSON export
- [x] Implement visual debug artifact export
- [x] Add scene-aware prompt presets for indoor, outdoor, and aerial footage
- [x] Add adaptive prompt resolution so generic uploads can specialize to aerial or outdoor scenes
- [x] Add YOLO-World prompt batching and tiled inference for small-object scenes
- [x] Export support-point clouds and richer grounding metadata for detailed inspection

## Demo Interface

- [x] Implement static web app shell
- [x] Implement video upload flow
- [x] Implement processing-status polling
- [x] Implement map summary and entity inspection panels
- [x] Implement interactive 3D viewer for cameras, sparse points, and semantic entities
- [x] Implement artifact browsing for exported JSON and annotated previews
- [x] Add viewer scene-rotation controls for portrait or sideways reconstructions
- [x] Project sampled video frames into the 3D preview for spatial grounding
- [x] Refine the UI layout so the workstation fits more cleanly on screen
- [x] Upgrade the viewer with keyboard-selectable control modes
- [x] Replace the rough prototype styling with a more professional operator-console visual design
- [x] Add prompt preset controls to the upload workflow
- [x] Add viewer toggles for camera frustums, relations, labels, path, and support clouds
- [x] Add alternate viewer keymaps for different navigation preferences
- [x] Draw entity extents, place extents, and support points in the 3D canvas
- [x] Convert the preview switcher into accessible Scene/Graph tabs with keyboard navigation and URL-backed state
- [x] Fix the recurring preview pane fill regression and reduce overlay chrome for a cleaner viewer-first layout

## Verification

- [x] Run backend smoke tests
- [x] Run end-to-end processing on a sample video
- [x] Run targeted detector validation on an aerial neighborhood frame with the adaptive prompt pipeline
- [ ] Verify the 3D viewer loads generated artifacts correctly in a live browser session
- [x] Write README instructions for setup, run, and upgrade path
- [x] Write a system report with flowcharts explaining architecture and runtime behavior
