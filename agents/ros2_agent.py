from langchain.agents import AgentExecutor, ZeroShotAgent
from langchain.agents import Tool
from tools.ros2_tools import list_topics, launch_turtlebot3_simulation, record_bag, echo_topic
from utils.chat_model_factory import ChatModelFactory
from config.settings import settings

# Initialize the LLM using the factory
llm = ChatModelFactory.get_chat_model(settings.CHAT_MODEL)

# Define the tools
tools = [
    Tool(
        name="list_topics",
        func=list_topics,
        description="Useful for listing active ROS2 topics."
    ),
    Tool(
        name="launch_turtlebot3_simulation",
        func=launch_turtlebot3_simulation,
        description="Useful for launching the TurtleBot3 simulation in Gazebo."
    ),
    Tool(
        name="record_bag",
        func=record_bag,
        description="Useful for recording a ROS2 bag for a specified topic."
    ),
    Tool(
        name="echo_topic",
        func=echo_topic,
        description="Useful for subscribing to a ROS2 topic and displaying its messages in real-time."
    )
]

# Create the ZeroShotAgent
agent = ZeroShotAgent.from_llm_and_tools(llm=llm, tools=tools)

# Wrap the agent in an AgentExecutor
agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)