from pathlib import Path
import os
import yaml


def project_root():
    return Path(os.environ.get("PEG_SORTING_ROOT", Path(__file__).resolve().parents[3]))


def config(name):
    with (project_root() / "config" / f"{name}.yaml").open() as stream:
        return yaml.safe_load(stream)
