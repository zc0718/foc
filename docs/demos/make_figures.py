"""Build the demo assets that the Sphinx "Demos" section shows.

The pipeline is three steps, and every step calls the library itself -- nothing is drawn by hand:

1. compile `sil_export.cpp` against the repository sources (same headers a user gets);
2. run it: it drives the SIL bench through three scenarios and writes a trace and a metrics CSV;
3. plot the traces into `docs/images/OUT_*.png` (the docs pipeline copies `OUT*` into the Sphinx
   image tree) and the metrics into `docs/sphinx/source/demos/metrics_*.csv` (rendered as tables).

Why C++ and not the Python module: `foc_python` (like the whole C API) runs on the stub HAL, whose
current feedback is a fixed script -- the traces would be flat lines. Real curves need the SIL
plant, which only exists on the C++ side.

Usage / 用法:

    source ~/venv/build/bin/activate        # matplotlib + numpy
    python3 docs/demos/make_figures.py      # writes docs/images/OUT_*.png + demos/metrics_*.csv
"""

import csv
import pathlib
import subprocess
import sys
import tempfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
IMAGES = ROOT / "docs" / "images"
DEMOS = ROOT / "docs" / "sphinx" / "source" / "demos"

# `src/core/version.cpp` is left out on purpose: the recipe wraps the C headers in `extern "C"`
# when it exports them, the raw repository headers are not wrapped, so that one C++ wrapper does
# not link against `version.c` outside Conan. Nothing in the exporter calls it.
SKIP_SOURCES = {"version.cpp"}

COLORS = {"measured": "#1f77b4", "reference": "#d62728", "aux": "#2ca02c"}


def compile_exporter(target: pathlib.Path) -> None:
    sources = [str(p) for p in sorted((ROOT / "src").rglob("*.cpp")) if p.name not in SKIP_SOURCES]
    command = [
        "g++", "-std=c++17", "-O2",
        f"-I{ROOT / 'include'}", f"-I{ROOT / 'test_package' / 'test' / 'unit'}",
        str(ROOT / "docs" / "demos" / "sil_export.cpp"), *sources,
        "-o", str(target),
    ]
    subprocess.run(command, check=True)


def read_trace(path: pathlib.Path) -> dict[str, list[dict[str, float]]]:
    series: dict[str, list[dict[str, float]]] = {}
    with path.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            series.setdefault(row["scenario"], []).append(
                {key: float(value) for key, value in row.items() if key != "scenario"})
    return series


def column(rows: list[dict[str, float]], key: str) -> list[float]:
    return [row[key] for row in rows]


def save(fig, name: str) -> pathlib.Path:
    IMAGES.mkdir(parents=True, exist_ok=True)
    target = IMAGES / f"OUT_{name}.png"
    fig.tight_layout()
    fig.savefig(target, dpi=150)
    plt.close(fig)
    print(f"  figure {target.relative_to(ROOT)}")
    return target


def plot_current_step(rows: list[dict[str, float]]) -> None:
    t = [1000.0 * x for x in column(rows, "t_s")]  # seconds -> milliseconds
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(7.2, 4.2))
    top.plot(t, column(rows, "reference"), "--", color=COLORS["reference"], label="iq reference")
    top.plot(t, column(rows, "iq_a"), color=COLORS["measured"], label="iq measured")
    top.set_ylabel("iq (A)")
    top.legend(loc="lower right")
    bottom.plot(t, column(rows, "id_a"), color=COLORS["aux"])
    bottom.set_ylabel("id (A)")
    bottom.set_xlabel("time (ms)")
    bottom.axhline(0.0, color="black", linewidth=0.6, alpha=0.4)
    for axis in (top, bottom):
        axis.grid(True, which="both", alpha=0.3)
    top.set_title("Current step: +1 A then -1 A, SIL bench, 20 kHz")
    save(fig, "demo_current_step")


def plot_speed_step(rows: list[dict[str, float]]) -> None:
    t = column(rows, "t_s")
    fig, top = plt.subplots(figsize=(7.2, 3.6))
    top.plot(t, column(rows, "reference"), "--", color=COLORS["reference"], label="speed reference")
    top.plot(t, column(rows, "speed_rad_s"), color=COLORS["measured"], label="speed measured")
    top.set_xlabel("time (s)")
    top.set_ylabel("speed (rad/s)")
    top.grid(True, alpha=0.3)
    top.legend(loc="lower right")
    bottom = top.twinx()
    bottom.plot(t, column(rows, "iq_a"), color=COLORS["aux"], alpha=0.55, linewidth=1.0, label="iq")
    bottom.set_ylabel("iq (A)")
    bottom.legend(loc="center right")
    top.set_title("Speed step 0 -> 100 rad/s: speed loop drives iq")
    save(fig, "demo_speed_step")


def plot_sine(rows: list[dict[str, float]]) -> None:
    t = column(rows, "t_s")
    fig, top = plt.subplots(figsize=(7.2, 3.6))
    top.plot(t, column(rows, "reference"), "--", color=COLORS["reference"], label="speed reference")
    top.plot(t, column(rows, "speed_rad_s"), color=COLORS["measured"], label="speed measured")
    top.set_xlabel("time (s)")
    top.set_ylabel("speed (rad/s)")
    top.grid(True, alpha=0.3)
    top.legend(loc="upper right")
    top.set_title("Sine tracking: 50 rad/s at 1 Hz")
    save(fig, "demo_sine_tracking")


def write_tables(metrics_path: pathlib.Path) -> None:
    DEMOS.mkdir(parents=True, exist_ok=True)
    grouped: dict[str, list[dict[str, str]]] = {}
    with metrics_path.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            grouped.setdefault(row["scenario"], []).append(row)
    for scenario, rows in grouped.items():
        target = DEMOS / f"metrics_{scenario}.csv"
        with target.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["metric", "value", "unit"])
            writer.writeheader()
            for row in rows:
                writer.writerow({"metric": row["metric"], "value": row["value"], "unit": row["unit"]})
        print(f"  table  {target.relative_to(ROOT)}")


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="foc-demo-") as workdir:
        binary = pathlib.Path(workdir) / "sil_export"
        print("compiling the exporter against include/ + src/ ...")
        compile_exporter(binary)
        trace_path = pathlib.Path(workdir) / "trace.csv"
        metrics_path = pathlib.Path(workdir) / "metrics.csv"
        print("running the SIL bench ...")
        subprocess.run([str(binary), str(trace_path), str(metrics_path)], check=True)
        series = read_trace(trace_path)
        print("plotting ...")
        plot_current_step(series["current_step"])
        plot_speed_step(series["speed_step"])
        plot_sine(series["sine_tracking"])
        print("writing tables ...")
        write_tables(metrics_path)
    print("done: figures in docs/images/OUT_demo_*.png, tables in docs/sphinx/source/demos/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
