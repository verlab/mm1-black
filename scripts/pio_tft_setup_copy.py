"""Copy include/User_Setup.h into TFT_eSPI libdeps so library + firmware share touch pins."""
Import("env")

import shutil
from pathlib import Path

project_dir = Path(env["PROJECT_DIR"])
pioenv = env["PIOENV"]
src = project_dir / "include" / "User_Setup.h"
dst = project_dir / ".pio" / "libdeps" / pioenv / "TFT_eSPI" / "User_Setup.h"

if src.is_file() and dst.parent.is_dir():
    shutil.copy2(src, dst)
    print(f"TFT_eSPI: synced User_Setup.h -> {dst.relative_to(project_dir)}")
elif src.is_file():
    print(f"TFT_eSPI: skip sync (install deps first): {dst.parent}")
