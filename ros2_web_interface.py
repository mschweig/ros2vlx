import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change this to your message type
import threading
from fastapi import FastAPI
import uvicorn
from pydantic import BaseModel
import json

class FastAPIThread(threading.Thread):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.app = FastAPI()
        self.latest_message = None
        
        # Define a simple API model
        class MessageResponse(BaseModel):
            message: str
            timestamp: float
            
        # Setup routes
        @self.app.get("/")
        async def read_root():
            return {"status": "ROS2-FastAPI server running"}
            
        @self.app.get("/latest_message", response_model=MessageResponse)
        async def get_latest_message():
            if self.latest_message:
                return {
                    "message": json.dumps(self.latest_message),
                    "timestamp": self.ros_node.get_clock().now().nanoseconds / 1e9
                }
            return {"message": "No messages received yet", "timestamp": 0.0}
    
    def update_message(self, msg):
        self.latest_message = msg
        
    def run(self):
        uvicorn.run(self.app, host="0.0.0.0", port=8000)

class RosSubscriberFastAPI(Node):
    def __init__(self):
        super().__init__('ros_subscriber_fastapi')
        
        # Create subscriber (change topic and message type as needed)
        self.subscription = self.create_subscription(
            String,
            'chatter',  # Change this to your topic
            self.listener_callback,
            10)
        
        # Initialize and start FastAPI thread
        self.fastapi_thread = FastAPIThread(self)
        self.fastapi_thread.start()
        
        self.get_logger().info('ROS2 subscriber with FastAPI server started')
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        
        # Update the latest message in FastAPI thread
        self.fastapi_thread.update_message(msg.data)
        
        # You can add your message processing logic here

def main(args=None):
    rclpy.init(args=args)
    
    node = RosSubscriberFastAPI()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.fastapi_thread.join()  # Wait for FastAPI thread to finish
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()