"""Copy include/User_Setup.h into TFT_eSPI libdeps so TOUCH_CS/ST7796 pins match the board."""

Import("env")
import shutil
from pathlib import Path

project_dir = Path(env.subst("$PROJECT_DIR"))
pioenv = env["PIOENV"]
src = project_dir / "include" / "User_Setup.h"
dst = project_dir / ".pio" / "libdeps" / pioenv / "TFT_eSPI" / "User_Setup.h"
if not src.is_file():
    print(f"TFT_eSPI: skip copy — missing {src}")
else:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    print(f"TFT_eSPI: synced User_Setup.h -> {dst.relative_to(project_dir)}")
