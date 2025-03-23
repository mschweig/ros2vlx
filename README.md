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

2. Ensure ROS2 is properly installed and sourced on your machine. Follow the official [ROS2 installation guide](https://docs.ros.org/en/rolling/Installation.html) for your platform. After installation, source the ROS2 setup script:
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   ```
   Replace `<ros2-distro>` with your installed ROS2 distribution (e.g., `humble`, `rolling`).

3. Set up Azure OpenAI API information:
   - Create a `.env` file in the project root directory.
   - Add the following environment variables to the `.env` file:
     ```env
      AZURE_OPENAI_API_KEY=<your-api-key>
      AZURE_OPENAI_ENDPOINT=<your-endpoint>
      AZURE_OPENAI_API_VERSION="2023-06-01-preview"
      AZURE_OPENAI_TEMPERATURE=0
      AZURE_DEPLOYMENT_MODEL_NAME="gpt-4o"
     ```
   Replace `<your-api-key>` and `<your-endpoint>` with your Azure OpenAI API credentials.

4. Create and activate a virtual environment using [uv](https://docs.astral.sh/uv/):
   ```bash
   uv venv .ros2vlx-venv --python 3.12
   source .ros2vlx-venv/bin/activate
   ```

5. Install dependencies:
   ```bash
   uv pip install -e .
   ```

6. Use with `cli.py`:
   ```bash
   python3 scripts/cli.py "Echo the chatter topic"
   ```