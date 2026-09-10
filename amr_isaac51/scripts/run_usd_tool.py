#!/usr/bin/env python3
"""Run an offline USD tool with Isaac's registered USD and PhysX plugins."""
from pathlib import Path
import runpy
import sys
import traceback

if len(sys.argv) < 2:
    raise SystemExit("Usage: python.sh scripts/run_usd_tool.py TOOL.py [arguments]")
tool = Path(sys.argv[1]).resolve()
if not tool.is_file():
    raise SystemExit(f"USD tool not found: {tool}")
sys.argv = [str(tool), *sys.argv[2:]]
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "fast_shutdown": True})
code = 0
try:
    runpy.run_path(str(tool), run_name="__main__")
except SystemExit as exc:
    code = exc.code if isinstance(exc.code, int) else (1 if exc.code else 0)
except Exception:
    traceback.print_exc()
    code = 1
finally:
    sys.stdout.flush()
    sys.stderr.flush()
    app.app.post_quit(code)
    app.close()
raise SystemExit(code)
