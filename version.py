import subprocess


def git_output(args, default="unknown"):
    try:
        return subprocess.check_output(
            ["git", *args],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return default


# Keep these values stable between builds so PlatformIO can reuse its object
# files. A wall-clock timestamp here changes the compiler flags every minute
# and forces a near-full rebuild even when the source code did not change.
ver = git_output(["describe", "--tags", "--always", "--dirty"])
date = git_output(["show", "-s", "--format=%cd", "--date=format:%Y-%m-%dT%H:%M", "HEAD"], "unknown")

print(
    f'-DAIRBRIDGE_VERSION=\\"{ver}\\" '
    f'-DAIRBRIDGE_BUILD_DATE=\\"{date}\\"'
)
