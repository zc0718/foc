#!/usr/bin/env python3
"""Build `foc_controller.fmu` from `fmu_wrapper.c` and this directory's `modelDescription.xml`.

The FMI headers come from FMPy's `c-code` directory (the standard headers, unmodified), so nothing here
invents an interface: the wrapper is a translation layer over `foc_api.h`, and `modelDescription.xml` is the
external face of `foc_hal_callbacks_t`.

    python3 api/fmu/build.py [--fmpy PATH] [--lib PATH] [--out PATH]
"""

from __future__ import annotations

import argparse
import pathlib
import shutil
import subprocess
import sys
import zipfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = ROOT / "api" / "fmu"


def find_fmpy_headers(explicit: str | None) -> pathlib.Path:
    if explicit:
        return pathlib.Path(explicit)
    candidates = sorted(ROOT.glob("workspace/materials/FMPy/src/fmpy/c-code"))
    # An installed FMPy ships the same headers beside its Python package (`fmpy/c-code`), and on a machine
    # without the materials checkout that copy is the only one there is -- CI is such a machine, where
    # `pip install fmpy` is the whole of its FMPy. Searching only under `workspace/` made the gate unreachable
    # there while `gates.py` had already proved an importable FMPy existed, so it failed for the one reason it
    # should not: the header was on disk, one `import` away.
    # 已安装的 FMPy 自带同一份头文件（fmpy/c-code）；没有 materials 检出的机器上，这是唯一一份。
    try:
        import fmpy
    except ImportError:
        pass
    else:
        candidates.append(pathlib.Path(fmpy.__file__).resolve().parent / "c-code")
    for path in candidates:
        if (path / "fmi2Functions.h").is_file():
            return path
    raise SystemExit(
        "fmi2Functions.h not found; pass --fmpy <dir with fmi2Functions.h>, or install FMPy "
        "(`python3 -m pip install fmpy`) so that its `fmpy/c-code` directory is searched"
    )


def find_library(explicit: str | None) -> pathlib.Path:
    """The C ABI archive, newest first.

    `libfoc_c.a` is the C interface and `libfoc_cpp.a` the implementation behind it; both are needed, and
    they must come from the same package or the symbols disagree.
    `libfoc_c.a` 是 C 接口，`libfoc_cpp.a` 是它背后的实现；两个都要，且必须来自同一个包，否则符号对不上。
    """
    if explicit:
        return pathlib.Path(explicit)
    candidates = sorted(
        pathlib.Path.home().glob(".conan2/p/b/foc*/p/lib/libfoc_c.a"), key=lambda path: path.stat().st_mtime
    )
    if not candidates:
        raise SystemExit("libfoc_c.a not found; run `conan create .` first, or pass --lib")
    return candidates[-1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fmpy")
    parser.add_argument("--lib")
    parser.add_argument("--out", default=str(HERE / "foc_controller.fmu"))
    args = parser.parse_args()

    headers = find_fmpy_headers(args.fmpy)
    library = find_library(args.lib)
    out = pathlib.Path(args.out)

    platform = ROOT / "api" / "fmu" / "binaries" / "linux64"
    platform.mkdir(parents=True, exist_ok=True)
    module = platform / "foc_controller.so"

    command = [
        "gcc",
        "-std=c11",
        "-O2",
        "-fPIC",
        "-shared",
        str(HERE / "fmu_wrapper.c"),
        "-I",
        str(headers),
        "-I",
        str(ROOT / "include"),
        str(library),
        str(library.with_name("libfoc_cpp.a")),
        "-lstdc++",
        # The library is built with coverage instrumentation here, so the archive references the gcov
        # runtime. A release archive would not need this line; keeping it costs nothing.
        # 这里的库带覆盖率插桩，归档里引用了 gcov 运行时。release 归档不需要这一行；留着也不花钱。
        "-lgcov",
        "-lm",
        "-o",
        str(module),
    ]
    print(" ".join(command))
    subprocess.run(command, check=True)

    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.write(HERE / "modelDescription.xml", "modelDescription.xml")
        archive.write(module, f"binaries/linux64/{module.name}")
    print(f"wrote {out} ({out.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
