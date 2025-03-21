# ROS2VLX - ROS2 Vision Language eXecutor

ROS2VLX is a natural language interface for ROS2, enabling users to interact with ROS2 using voice or text commands. It leverages LangChain, NVIDIA AI endpoints, and OpenAI/Azure OpenAI to provide a seamless and intelligent experience.

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

2. Create and activate a virtual environment using [uv](https://docs.astral.sh/uv/)
   ```bash
   uv venv .ros2vlx-venv --python 3.12
   source .ros2vlx-venv/bin/activate
   ```

3. Install dependencies
   ```bash
   uv pip install -e .
   ```