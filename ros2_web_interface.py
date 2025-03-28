import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change to your message type
from fastapi import FastAPI
import uvicorn
import threading
from typing import Optional


class ROSFastAPI(Node):
    """
    A ROS2 node that combines a message subscriber with a FastAPI server.
    """

    def __init__(self):
        super().__init__("ros_fastapi_node")

        # ROS2 Subscriber
        self.subscription = self.create_subscription(
            String,
            "chatter",
            self.listener_callback,
            10,
        )

        # Store the latest message
        self.latest_message: Optional[str] = None

        # FastAPI setup
        self.app = FastAPI(title="ROS2 FastAPI Integration")
        self.setup_routes()

        self.create_timer(1.0, self.get_topics)

        self.get_logger().info("ROS2 FastAPI node initialized")

    def get_topics(self):
        self.topics = self.get_topic_names_and_types()
        self.get_logger().info("Retrieving topics...")
        self.destroy_node()  # Optional: auto-shutdown after listing

    def listener_callback(self, msg):
        """ROS2 subscriber callback"""
        self.latest_message = msg.data

    def setup_routes(self):
        """Configure FastAPI routes"""

        @self.app.get("/")
        async def root():
            return {"message": "ROS2 FastAPI Server Running"}

        @self.app.get("/list_topics")
        async def list_topics():
            self.get_logger().info("Received command to list topics")
            return {
                "latest_topics": self.topics,
                "status": ("success" if self.topics else "no topics"),
            }

        @self.app.get("/get_data")
        async def execute(topic: str):
            self.get_logger().info(f"Received command for topic: {topic}")
            return {
                "latest_message": self.latest_message,
                "status": ("success" if self.latest_message else "no messages"),
            }


def run_fastapi(app: FastAPI):
    """Run FastAPI server in a separate thread"""
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


def main(args=None):
    rclpy.init(args=args)

    # Create the combined node
    node = ROSFastAPI()

    # Start FastAPI in a separate thread
    fastapi_thread = threading.Thread(target=run_fastapi, args=(node.app,), daemon=True)
    fastapi_thread.start()

    try:
        # Run ROS2 node
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
