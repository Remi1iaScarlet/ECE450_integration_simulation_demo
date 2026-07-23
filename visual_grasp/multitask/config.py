"""Load the multitask pipeline configuration (multitask/config.yaml)."""
import pathlib

import yaml

_DEFAULT = pathlib.Path(__file__).resolve().parent / "config.yaml"


def load_config(path=None):
    """Return the config as a nested dict."""
    with open(path or _DEFAULT) as f:
        return yaml.safe_load(f)
