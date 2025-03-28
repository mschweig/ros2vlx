import rclpy
from rclpy.node import Node
import threading
import asyncio
from fastapi import FastAPI, HTTPException, Query
import uvicorn
import traceback
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

app = FastAPI()


class OneShotDynamicSubscriber(Node):
    def __init__(self):
        super().__init__('one_shot_dynamic_subscriber')
        self.lock = threading.Lock()
        self.latest_msg = None
        self.subscription_event = threading.Event()

    def get_topic_type(self, topic_name: str) -> str:
        topic_list = self.get_topic_names_and_types()
        for name, types in topic_list:
            if name == topic_name:
                if not types:
                    raise ValueError(f"Topic '{topic_name}' has no types.")
                return types[0]  # Use the first type
        raise ValueError(f"Topic '{topic_name}' not found.")

    def import_message_class(self, type_str: str):
        module_name, _, msg_name = type_str.split('/')
        return get_message(f"{module_name}/msg/{msg_name}")

    def one_shot_subscribe(self, topic_name: str, timeout: float = 5.0):
        with self.lock:
            self.latest_msg = None
            self.subscription_event.clear()

            # Detect type
            topic_type = self.get_topic_type(topic_name)
            self.get_logger().info(f"Detected topic type: {topic_type}")

            # Load message type
            msg_class = self.import_message_class(topic_type)

            # Subscribe
            subscription = self.create_subscription(
                msg_class,
                topic_name,
                self.callback,
                qos_profile=10
            )

            # Wait for message
            received = self.subscription_event.wait(timeout=timeout)
            self.destroy_subscription(subscription)

            if not received:
                raise TimeoutError(f"Timeout waiting for message on {topic_name}")

            return self.latest_msg

    def callback(self, msg):
        self.get_logger().info("Received a message.")
        if self.latest_msg is None:
            self.latest_msg = msg
            self.subscription_event.set()


node: OneShotDynamicSubscriber = None

@app.get("/get_data")
def get_data(topic: str = Query(..., description="Topic to subscribe to"),
             timeout: float = Query(5.0, description="Timeout in seconds (default=5)")):
    if not topic.startswith("/"):
        raise HTTPException(status_code=400, detail="Topic name must start with '/'")
    
    try:
        msg = node.one_shot_subscribe(topic, timeout)
    except TimeoutError as e:
        raise HTTPException(status_code=504, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        print(traceback.format_exc())
        raise HTTPException(status_code=500, detail=f"Internal Error: {str(e)}")

    msg_dict = message_to_ordereddict(msg)
    return {"topic": topic, "data": msg_dict}


@app.get("/list_topics")
def list_topics():
    topic_list = node.get_topic_names_and_types()
    topics = [{"name": name, "types": types} for name, types in topic_list]
    return {"topics": topics}


def ros_spin():
    rclpy.spin(node)


def main(args=None):
    global node
    rclpy.init(args=args)
    node = OneShotDynamicSubscriber()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    uvicorn.run(app, host="0.0.0.0", port=8000)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
