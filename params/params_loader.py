import os
import yaml


def _load_yaml() -> dict:
    config_path = os.path.join(
        # os.path.dirname(__file__), '../../../../config/camera_info.yaml')
    with open(os.path.abspath(config_path), 'r') as f:
        return yaml.safe_load(f)['ros__parameters']


_params = _load_yaml()

