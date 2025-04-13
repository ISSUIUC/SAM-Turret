
import yaml
from pathlib import Path

def load_config(config_path):
    """
    Loads a YAML config file and returns a dictionary.

    Args:
        config_path (str | Path): Path to the YAML file.

    Returns:
        dict: Parsed configuration data.
    """
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return config

# Example config.yaml structure:
#
# turret:
#   lat: 40.0
#   lon: -88.0
#   alt: 250.0
# motors:
#   pitch:
#     step_size: 1.8
#     deadzone: [0.5, 1.0]
#   yaw:
#     step_size: 1.8
#     deadzone: [0.5, 1.0]
# pid:
#   pitch: [1.0, 0.0, 0.1]
#   yaw: [1.0, 0.0, 0.1]
