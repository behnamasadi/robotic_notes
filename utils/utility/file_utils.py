# utility/file_utils.py
from pathlib import Path
import sys
import importlib.resources as ir

_PACKAGE_NAME = "utility"


def project_root() -> Path:
    """
    Dev root (dir with pyproject.toml) if found; else package dir.
    This is kept for non-'models' paths you might use.
    """
    here = Path(__file__).resolve()
    for p in (here, *here.parents):
        if (p / "pyproject.toml").exists():
            return p
    return here.parent


def _entry_script_dir() -> Path | None:
    """
    Directory of the entry script (e.g., .../serialization_saving_loading/scripts).
    Works when running 'python foo.py' or similar.
    Returns None when running in test environment (pytest).
    """
    try:
        p = Path(sys.argv[0]).resolve()
        # Detect if we're running in a test environment (pytest, unittest, etc.)
        # In test environments, we should fall back to project_root behavior
        script_name = p.name.lower()
        if any(test_tool in script_name for test_tool in ['pytest', 'py.test', 'unittest']):
            return None
        # Also check if the path is in site-packages (likely a test runner)
        if 'site-packages' in str(p):
            return None
        return p.parent if p.exists() else None
    except Exception:
        return None


def resource_path(*parts: str) -> Path:
    """
    If the first part is 'models', always resolve under the folder next to the entry script:
        <entry_script_dir>/models/...
    Otherwise:
        - prefer a path relative to the entry script directory (so scripts can
          be run from anywhere and still resolve resources consistently)
        - fall back to dev layout under <project_root>/...
        - finally, when installed (no repo), fall back to package resources.
    """
    if parts and parts[0] == "models":
        tail = parts[1:]
        entry_dir = _entry_script_dir()
        # Always anchor to the entry script's 'models' folder
        if entry_dir is not None:
            return (entry_dir / "models" / Path(*tail)).resolve()

        # Fallbacks if there's no entry script (e.g., REPL):
        # Try repo layout:
        return (
            project_root()
            / "serialization_saving_loading"
            / "scripts"
            / "models"
            / Path(*tail)
        ).resolve()

    # Non-'models' behavior
    # 1) Prefer being relative to the entry script directory so scripts can be
    #    executed from any working directory.
    entry_dir = _entry_script_dir()
    if entry_dir is not None:
        return (entry_dir / Path(*parts)).resolve()

    # 2) Fallback: repo/dev layout under project_root
    dev_root = project_root()
    if dev_root:
        return dev_root.joinpath(*parts).resolve()

    # 3) Installed fallback via importlib.resources (rarely hit if you only use 'models')
    candidate = ir.files(_PACKAGE_NAME).joinpath(*parts)
    with ir.as_file(candidate) as p:
        return Path(p).resolve()
