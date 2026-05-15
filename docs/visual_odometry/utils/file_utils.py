from pathlib import Path


def project_root() -> Path:
    # utils/ -> project root (go up 1)
    return Path(__file__).resolve().parent.parent


def resource_path(*parts: str) -> Path:
    """
    Build a path under the project root in an OS-safe way.
    Example: resource_path("config", "config.yaml")
    """
    return project_root().joinpath(*parts)


if __name__ == "__main__":
    print(Path(__file__).resolve().parent.parent)
    cfg = resource_path("config", "config.yaml")
    print(cfg)
