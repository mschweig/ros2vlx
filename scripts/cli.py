import argparse
from agents.ros2_agent import agent
from utils.logger import logger

def main():
    parser = argparse.ArgumentParser(description="ROS2VLX - ROS2 Vision Language eXecutor")
    parser.add_argument("command", type=str, help="The natural language command to execute")
    args = parser.parse_args()

    try:
        response = agent.run(args.command)
        print(response)
    except Exception as e:
        logger.error(f"Error processing command: {e}")

if __name__ == "__main__":
    main()