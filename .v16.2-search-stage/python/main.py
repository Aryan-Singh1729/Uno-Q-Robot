"""Arduino App Lab Python entry point.

App Lab deploys this file and sketch/sketch.ino as one application. The shared
Python modules remain at the app root so they are also usable by desktop tests.
"""

from pathlib import Path
import sys

APP_ROOT = Path(__file__).resolve().parents[1]
if str(APP_ROOT) not in sys.path:
    sys.path.insert(0, str(APP_ROOT))

from main import app_lab_main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(app_lab_main())
