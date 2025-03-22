from langchain.agents import initialize_agent, Tool
from langchain_openai import ChatOpenAI, AzureChatOpenAI
from tools.ros2_tools import list_topics, launch_turtlebot3_simulation, record_bag
from config.settings import settings

# Initialize the LLM
llm = AzureChatOpenAI(
    azure_deployment="gpt-4o",
    api_version="2023-06-01-preview", 	
    temperature=0
)

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
    )
]

# Initialize the agent
agent = initialize_agent(tools, llm, agent="zero-shot-react-description", verbose=True)