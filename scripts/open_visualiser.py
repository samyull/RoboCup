# PlatformIO extra script: opens test/visualiser.py after every upload.
#
#  - Before uploading, closes the visualiser this script last opened, so it
#    isn't holding the Teensy's serial port when the uploader needs it.
#  - After uploading, starts a fresh visualiser (port auto-detected, no R reset
#    since the robot has just rebooted anyway). Its output goes to
#    .pio/visualiser.log.
#
# Turn off with `custom_open_visualiser = no` in platformio.ini.
# Uses `python` from PATH; set the VISUALISER_PYTHON environment variable to
# point at a different interpreter (it needs pyserial, matplotlib and numpy).

import os
import shutil
import subprocess

Import("env")  # noqa: F821 - provided by PlatformIO

PROJECT_DIR = env["PROJECT_DIR"]  # noqa: F821
VISUALISER = os.path.join(PROJECT_DIR, "test", "visualiser.py")
PID_FILE = os.path.join(PROJECT_DIR, ".pio", "visualiser.pid")
LOG_FILE = os.path.join(PROJECT_DIR, ".pio", "visualiser.log")


def enabled():
    value = env.GetProjectOption("custom_open_visualiser", "yes")  # noqa: F821
    return str(value).strip().lower() not in ("no", "false", "0", "off")


def close_previous(*_args, **_kwargs):
    if not os.path.exists(PID_FILE):
        return
    try:
        with open(PID_FILE) as f:
            pid = int(f.read().strip())
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(pid), "/F"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            os.kill(pid, 15)
    except (OSError, ValueError):
        pass  # already closed
    finally:
        try:
            os.remove(PID_FILE)
        except OSError:
            pass


def open_visualiser(*_args, **_kwargs):
    python = os.environ.get("VISUALISER_PYTHON") or shutil.which("python") or shutil.which("py")
    if not python:
        print("open_visualiser: no Python found on PATH - set VISUALISER_PYTHON")
        return
    flags = 0
    if os.name == "nt":
        flags = subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP
    log = open(LOG_FILE, "w")
    proc = subprocess.Popen([python, VISUALISER, "--no-reset"], cwd=PROJECT_DIR,
                            stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, creationflags=flags)
    with open(PID_FILE, "w") as f:
        f.write(str(proc.pid))
    print(f"Visualiser opened (log: {LOG_FILE})")


if enabled():
    env.AddPreAction("upload", close_previous)  # noqa: F821
    env.AddPostAction("upload", open_visualiser)  # noqa: F821
