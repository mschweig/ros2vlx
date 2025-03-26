import subprocess
from fastapi import FastAPI, WebSocket
from fastapi.responses import JSONResponse
import uvicorn
from typing import Optional

app = FastAPI(title="ROS 2 Web Service")

def execute_command(command: str, timeout: int = 2) -> str:
    """Generic command executor with timeout (does NOT prepend 'ros2')"""
    try:
        result = subprocess.run(
            command.split(),
            capture_output=True,
            text=True,
            timeout=timeout
        )
        if result.returncode != 0:
            return f"Error: {result.stderr.strip()}"
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return "Timeout: Command took too long."
    except Exception as e:
        return f"Execution failed: {str(e)}"

# ROS2-specific endpoints
@app.get("/list_topics")
def list_topics():
    """List all ROS 2 topics (uses ros2 internally)"""
    output = execute_command("ros2 topic list")
    return {"topics": output.splitlines()}

@app.get("/topic_echo/{topic_name}")
def get_topic_message(topic_name: str, timeout: Optional[int] = 2):
    """Get latest message from a topic (uses ros2 internally)"""
    output = execute_command(f"ros2 topic echo --once {topic_name}", timeout)
    return {"topic": topic_name, "message": output}

# Generic endpoint (raw command execution)
@app.get("/run_command")
def run_command(command: str, timeout: Optional[int] = 2):
    """
    Execute ANY system command (does NOT prepend 'ros2')
    Example: /run_command?command=ls -l
    """
    output = execute_command(command, timeout)
    return {"command": command, "output": output.splitlines()}

# WebSocket for ROS2 topic streaming
@app.websocket("/ws/topic_echo/{topic_name}")
async def websocket_topic_echo(websocket: WebSocket, topic_name: str):
    await websocket.accept()
    process = subprocess.Popen(
        ["ros2", "topic", "echo", topic_name],
        stdout=subprocess.PIPE,
        text=True
    )
    try:
        while True:
            line = process.stdout.readline()
            if not line:
                break
            await websocket.send_text(line)
    finally:
        process.kill()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)