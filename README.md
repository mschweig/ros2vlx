# ROS2VLX - ROS2 Vision Language eXecutor

ROS2VLX is a natural language interface for ROS2, enabling users to interact with ROS2 using voice or text commands. It leverages LangChain, NVIDIA AI endpoints, and OpenAI/Azure OpenAI to provide a seamless and intelligent experience.

> **⚠️ This project is currently under heavy development. Expect frequent updates and changes.**

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.12-blue.svg)](https://www.python.org/downloads/release/python-3120/)
[![Contributions Welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](CONTRIBUTING.md)
[![Issues](https://img.shields.io/github/issues/mschweig/ros2vlx)](https://github.com/mschweig/ros2vlx/issues)

## Features
- List ROS2 topics
- Echo ROS2 topics
- Launch TurtleBot3 simulation
- Record ROS2 bag files
- Extensible and modular design

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/mschweig/ros2vlx.git
   cd ros2vlx
   ```

2. Create and activate a virtual environment using [uv](https://docs.astral.sh/uv/)
   ```bash
   uv venv .ros2vlx-venv --python 3.12
   source .ros2vlx-venv/bin/activate
   ```

3. Install dependencies
   ```bash
   uv pip install -e .
   ```

4. Use with cli.py:
   ```bash
   python3 scripts/cli.py "Echo the chatter topic"
   ```