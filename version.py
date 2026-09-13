import subprocess
from datetime import datetime

try:
    ver = subprocess.check_output(
        ["git", "describe", "--tags", "--always", "--dirty"],
        stderr=subprocess.DEVNULL,
        text=True
    ).strip()
except Exception:
    ver = "unknown"

date = datetime.now().strftime("%Y-%m-%dT%H:%M")

print(
    f'-DAIRBRIDGE_VERSION=\\"{ver}\\" '
    f'-DAIRBRIDGE_BUILD_DATE=\\"{date}\\"'
)