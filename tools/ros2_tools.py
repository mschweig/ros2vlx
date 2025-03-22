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
    
@tool
def echo_topic(topic: str):
    """
    Subscribes to a ROS2 topic and displays its messages in real-time.
    
    Args:
        topic (str): The name of the ROS2 topic to echo.
    """
    command = f"ros2 topic echo {topic}"
    process = None
    
    try:
        # Use subprocess.Popen to stream output in real-time
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Stream output line by line
        for line in process.stdout:
            print(line, end="")
        
        # Wait for the process to complete
        process.wait()
        
        if process.returncode != 0:
            logger.error(f"Error executing command: {command}. Error: {process.stderr.read()}")
            return f"Error: {process.stderr.read()}"
        
        logger.info(f"Executed command: {command}")
        return "Topic echo completed."
    
    except KeyboardInterrupt:
        # Handle Ctrl+C interrupt
        if process is not None:
            process.terminate()  # Gracefully terminate the subprocess
            process.wait()       # Wait for the process to terminate
            logger.info("Process terminated by user (Ctrl+C).")
        return "Topic echo stopped by user."
    
    except Exception as e:
        logger.error(f"Error executing command: {command}. Error: {e}")
        return f"Error: {e}"


