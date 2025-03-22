from langchain.tools import tool
import subprocess
from utils.logger import logger

@tool
def list_topics():
    """Returns a list of active ROS2 topics."""
    command = "ros2 topic list"
    try:
        result = subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        logger.info(f"Executed command: {command}")
        return result.stdout
    except subprocess.CalledProcessError as e:
        logger.error(f"Error executing command: {command}. Error: {e.stderr}")
        return f"Error: {e.stderr}"

@tool
def launch_turtlebot3_simulation():
    """Launches the TurtleBot3 simulation in Gazebo."""
    command = "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
    try:
        result = subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        logger.info(f"Executed command: {command}")
        return result.stdout
    except subprocess.CalledProcessError as e:
        logger.error(f"Error executing command: {command}. Error: {e.stderr}")
        return f"Error: {e.stderr}"

@tool
def record_bag(topic: str):
    """Records a ROS2 bag for the specified topic."""
    command = f"ros2 bag record {topic}"
    try:
        result = subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        logger.info(f"Executed command: {command}")
        return result.stdout
    except subprocess.CalledProcessError as e:
        logger.error(f"Error executing command: {command}. Error: {e.stderr}")
        return f"Error: {e.stderr}"