---
title: MonoSafe ACC Video Generator
colorFrom: blue
colorTo: green
sdk: docker
app_port: 7860
suggested_hardware: t4-small
---

# MonoSafe ACC Video Generator (Space v1)

This folder is a **separate Hugging Face Space scaffold** (Docker SDK) for ACC-only interactive video generation.

## What It Does

- Accepts user parameters for ACC synthesis (`threshold` or `basis`, fps/dpi/sample, optional device id).
- Lets users override ACC grid specs at runtime:
  - `states.eta` (resolution),
  - `states.lb` (lower bounds),
  - `states.ub` (upper bounds).
- Runs pFaces synthesis in a queued background job.
- Generates and displays the resulting MP4 video in the UI.

## Runtime Requirements

- Dedicated GPU Space runtime (recommended: `t4-small` or better) for best performance.
- CPU-only runtimes are supported via POCL OpenCL CPU backend with a CL1.2 compiler override.
- pFaces demo license limits still apply (`Max PEs` check is enforced).

## How To Deploy (Separate Repo)

1. Create a new Hugging Face Space repo with **Docker SDK**.
2. Copy the contents of this folder (`hf-space-acc-v1/`) to the root of that Space repo.
3. Push to Hugging Face.

The Space image will:
- install dependencies,
- install `pFaces 1.4.0d` Linux build,
- install POCL CPU OpenCL runtime,
- build the local kernel driver,
- start the Gradio app on port `7860`.

## Notes

- This v1 Space intentionally includes only ACC workflow for predictable latency and reliability.
- The app includes a pipeline-stage abstraction in `app.py` so additional stages can be added later without breaking the UI contract.
