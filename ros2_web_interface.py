#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change to your message type
from fastapi import FastAPI
import uvicorn
import threading
from typing import Optional
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ROSFastAPI(Node):
    """
    A ROS2 node that combines a message subscriber with a FastAPI server.
    """
    
    def __init__(self):
        super().__init__('ros_fastapi_node')
        
        # ROS2 Subscriber
        self.subscription = self.create_subscription(
            String,                      # Message type
            'chatter',                 # Topic name
            self.listener_callback,      # Callback function
            10)                         # Queue size
        
        # Store the latest message
        self.latest_message: Optional[str] = None
        
        # FastAPI setup
        self.app = FastAPI(title="ROS2 FastAPI Integration")
        self.setup_routes()
        
        logger.info("ROS2 FastAPI node initialized")

    def listener_callback(self, msg):
        """ROS2 subscriber callback"""
        self.latest_message = msg.data
        self.get_logger().info(f'Received: {msg.data}')

    def setup_routes(self):
        """Configure FastAPI routes"""
        
        @self.app.get("/")
        async def root():
            return {"message": "ROS2 FastAPI Server Running"}
        
        @self.app.get("/latest_message")
        async def get_latest_message():
            return {
                "latest_message": self.latest_message,
                "status": "success" if self.latest_message else "no messages received yet"
            }

def run_fastapi(app: FastAPI):
    """Run FastAPI server in a separate thread"""
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )

def main(args=None):
    rclpy.init(args=args)
    
    # Create the combined node
    node = ROSFastAPI()
    
    # Start FastAPI in a separate thread
    fastapi_thread = threading.Thread(
        target=run_fastapi,
        args=(node.app,),
        daemon=True
    )
    fastapi_thread.start()
    
    try:
        # Run ROS2 node
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()