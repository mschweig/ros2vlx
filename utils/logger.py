import logging
import logging.config
import yaml
import os

def setup_logging():
    """Set up logging using the YAML configuration file."""
    config_path = os.path.join(os.path.dirname(__file__), "../config/logging_config.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f.read())
        logging.config.dictConfig(config)

# Initialize logging
setup_logging()
logger = logging.getLogger("ros2vlx")