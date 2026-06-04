"""Inject FW_VERSION into compile flags (git tag / env VERSION / describe)."""
Import("env")
import os

version = os.environ.get("VERSION", "").strip()
if not version:
    import subprocess

    try:
        r = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        if r.returncode == 0 and r.stdout.strip():
            version = r.stdout.strip()
    except Exception:
        pass

if not version:
    version = "dev"

# Escape for CPP define string
version = version.replace("\\", "\\\\").replace('"', '\\"')
env.Append(CPPDEFINES=[("FW_VERSION", '\\"' + version + '\\"')])
