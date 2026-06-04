# RMCS Mac Dev Container

Use this dev container on `macOS + Apple Silicon + Docker Desktop` when you want a local development environment for editing, indexing, and building RMCS.

## What it supports

- VSCode Dev Containers
- ROS2/C++ development toolchain
- `clangd` indexing
- `build-rmcs` and `clean-rmcs`
- default development image build without OpenVINO or Livox SDK bootstrap
- Ubuntu ARM apt source defaults to `https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports`
- ROS apt source defaults to `https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu`

## What it does not try to replicate

- Linux `--network host`
- `/dev` passthrough for USB or serial hardware
- X11 / Wayland GUI forwarding
- Full on-device robot runtime parity
- optional SDK bootstrap steps that depend on flaky external downloads during image build

## How to open

1. Open the repository in VSCode.
2. Run `Dev Containers: Open Folder in Container...`.
3. Select `.devcontainer/mac/devcontainer.json`.

When the container is ready, copy the recommended editor settings if needed:

```bash
cp .vscode/settings.default.json .vscode/settings.json
```
