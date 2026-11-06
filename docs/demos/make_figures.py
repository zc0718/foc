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


def build(source: pathlib.Path, target: pathlib.Path) -> None:
    """Compile one demo program against the repository sources.

    The slice examples under `docs/doxygen/dox/demos/` are built by this same
    function, so a figure cannot come from code the reader is not shown: the
    snippet in the docs and the curve in the figure are one translation unit.
    切片示例用同一个函数编译，所以图不可能来自读者看不到的代码：文档里的片段与图里的曲线
    是同一个翻译单元。
    """
    sources = [str(p) for p in sorted((ROOT / "src").rglob("*.cpp")) if p.name not in SKIP_SOURCES]
    command = [
        "g++", "-std=c++17", "-O2",
        f"-I{ROOT / 'include'}", f"-I{ROOT / 'test_package' / 'test' / 'unit'}",
        str(source), *sources,
        "-o", str(target),
    ]
    subprocess.run(command, check=True)


def compile_exporter(target: pathlib.Path) -> None:
    build(ROOT / "docs" / "demos" / "sil_export.cpp", target)


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


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    """Rows as strings: one slice CSV carries a text column (`state`), so the
    cast has to happen per column in the plotter that knows what it asked for.
    行按字符串读：其中一片的 CSV 带文本列（`state`），所以转换必须发生在知道列含义的那个
    绘图函数里。"""
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def plot_slice_s1(rows: list[dict[str, str]]) -> None:
    """Hold, ramp, hand over: the two currents and the forced angle."""
    stride = max(1, len(rows) // 4000)
    sample = rows[::stride]
    t = [1000.0 * float(row["t_s"]) for row in sample]
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(7.2, 4.4))
    # The routine reports the angle inside [0, 2*pi). Plotted as-is that reads as a
    # sawtooth, and "the vector keeps jumping back to zero" is the opposite of what
    # the ramp does -- so unwrap it, for display only.
    # 例行把角度报在 [0, 2*pi) 内。照原样画出来像锯齿，而"矢量不断跳回零"与斜坡实际在做的
    # 事恰好相反——所以只在显示上做展开。
    unwrapped: list[float] = []
    total = 0.0
    previous: float | None = None
    for row in sample:
        value = float(row["theta_e_rad"])
        if previous is not None:
            step = value - previous
            step -= 6.28318530718 * round(step / 6.28318530718)
            total += step
        unwrapped.append(total)
        previous = value
    top.plot(t, unwrapped, color=COLORS["measured"], label="forced theta_e (unwrapped)")
    top.set_ylabel("theta_e (rad)")
    top.legend(loc="lower right")
    bottom.plot(t, [float(row["id_a"]) for row in sample], color=COLORS["aux"], label="i_d command")
    bottom.set_ylabel("i_d (A)")
    bottom.set_xlabel("time (ms)")
    bottom.legend(loc="lower right")
    for axis in (top, bottom):
        axis.grid(True, which="both", alpha=0.3)
    # The two decisions the routine makes, marked rather than described.
    states = [row["state"] for row in sample]
    for index in range(1, len(states)):
        if states[index] != states[index - 1]:
            for axis in (top, bottom):
                axis.axvline(t[index], color="black", linewidth=0.8, alpha=0.5, linestyle="--")
    top.set_title("Open-loop start: hold 1.5 A, ramp the vector, hand over at 60 rad/s elec")
    save(fig, "slice_s1_startup")


def plot_slice_s6(rows: list[dict[str, str]]) -> None:
    """One position move: the velocity the profile allows, and where it lands."""
    stride = max(1, len(rows) // 4000)
    sample = rows[::stride]
    t = [1000.0 * float(row["t_s"]) for row in sample]
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(7.2, 4.4))
    top.plot(t, [float(row["velocity_rad_s"]) for row in sample], color=COLORS["aux"], label="velocity command")
    top.set_ylabel("velocity (rad/s)")
    top.legend(loc="upper right")
    bottom.plot(t, [float(row["position_rad"]) for row in sample], color=COLORS["measured"], label="position")
    bottom.plot(t, [float(row["target_rad"]) for row in sample], "--", color=COLORS["reference"], label="target")
    bottom.set_ylabel("position (rad)")
    bottom.set_xlabel("time (ms)")
    bottom.legend(loc="lower right")
    for axis in (top, bottom):
        axis.grid(True, which="both", alpha=0.3)
    top.set_title("One move, shaped: accel limit sets the ramp, the gain sets the tail")
    save(fig, "slice_s6_position")


def plot_slice_s5(rows: list[dict[str, str]]) -> None:
    """Duty span against the requested voltage, at two angles.

    The modulator's limit is a hexagon, not a circle, so the two sweeps clamp at
    two different amplitudes. The clamp point is *read from the data* (the first
    row where the modulator reports saturation) rather than asserted here.
    调制器的边界是六边形而不是圆，所以两条扫描钳位在两个不同幅值。钳位点是**从数据里读**
    （调制器第一次报饱和的那一行），不在这里断言。
    """
    fig, axis = plt.subplots(figsize=(7.2, 3.8))
    for angle, colour in ((0.0, COLORS["measured"]), (30.0, COLORS["reference"])):
        series = [row for row in rows if float(row["angle_deg"]) == angle]
        if not series:
            continue
        inside = [(float(r["v_pu"]), float(r["duty_span"])) for r in series if r["saturated"] == "0"]
        outside = [(float(r["v_pu"]), float(r["duty_span"])) for r in series if r["saturated"] != "0"]
        axis.plot([p[0] for p in inside], [p[1] for p in inside], color=colour, label=f"request at {angle} deg")
        axis.plot([p[0] for p in outside], [p[1] for p in outside], "--", color=colour, alpha=0.6,
                  label=f"{angle} deg, clamped")
        if outside:
            edge = min(p[0] for p in outside)
            axis.axvline(edge, color=colour, linewidth=0.8, alpha=0.4)
            axis.annotate(f"clamp at |v|/v_bus = {edge:.2f}", xy=(edge, 0.55), xytext=(edge - 0.005, 0.55),
                          rotation=90, fontsize=8, color=colour, ha="right", va="center")
    axis.set_xlabel("|v| / v_bus")
    axis.set_ylabel("max duty - min duty")
    axis.set_ylim(0.0, 1.08)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="lower right", fontsize=8)
    axis.set_title("SVPWM: one requested amplitude, two limits -- sector centre vs sector face")
    save(fig, "slice_s5_modulation")


def run_slice(name: str, workdir: pathlib.Path, plotter) -> None:
    """Build one slice example, run it in CSV mode, plot it.

    The examples live where the docs include them, so there is exactly one copy
    of the code that produces the figure and the snippet.
    切片示例就住在文档引入它们的地方，所以"产生图的代码"与"片段"只有一份。
    """
    source = ROOT / "docs" / "doxygen" / "dox" / "demos" / f"{name}.cxx"
    binary = workdir / name
    out = workdir / f"{name}.csv"
    print(f"building the slice example {source.name} ...")
    build(source, binary)
    subprocess.run([str(binary), "--csv", str(out)], check=True, stdout=subprocess.DEVNULL)
    plotter(read_csv(out))


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
        print("slice figures ...")
        run_slice("slice-s1-startup", pathlib.Path(workdir), plot_slice_s1)
        run_slice("slice-s6-position", pathlib.Path(workdir), plot_slice_s6)
        run_slice("slice-s5-modulation", pathlib.Path(workdir), plot_slice_s5)
    print("done: figures in docs/images/OUT_demo_*.png, tables in docs/sphinx/source/demos/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
