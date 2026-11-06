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
# `version.cpp` is left out because the recipe wraps the C headers in `extern "C"`
# when it exports them, and the raw headers are not wrapped -- so that one C++
# wrapper does not link outside Conan. The C sources are linked in: the C ABI demo
# calls `foc_api.c`, and a demo that cannot link the path it documents is a lie.
SKIP_SOURCES = {"version.cpp", "version.c"}

COLORS = {"measured": "#1f77b4", "reference": "#d62728", "aux": "#2ca02c"}


def build(source: pathlib.Path, target: pathlib.Path) -> None:
    """Compile one demo program against the repository sources.

    The slice examples under `docs/doxygen/dox/demos/` are built by this same
    function, so a figure cannot come from code the reader is not shown: the
    snippet in the docs and the curve in the figure are one translation unit.
    切片示例用同一个函数编译，所以图不可能来自读者看不到的代码：文档里的片段与图里的曲线
    是同一个翻译单元。
    """
    sources = [str(p) for p in sorted((ROOT / "src").rglob("*.cpp"))
               if p.name not in SKIP_SOURCES] + [str(p) for p in sorted((ROOT / "src").rglob("*.c"))
                                                 if p.name not in SKIP_SOURCES]
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


def plot_slice_s9(rows: list[dict[str, str]]) -> None:
    """What crosses the C ABI: the command, the speed the host reports, and one duty."""
    stride = max(1, len(rows) // 4000)
    sample = rows[::stride]
    t = [1000.0 * float(row["t_s"]) for row in sample]
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(7.2, 4.4))
    top.plot(t, [float(row["iq_command_a"]) for row in sample], "--", color=COLORS["reference"], label="iq command")
    top.plot(t, [float(row["iq_measured_a"]) for row in sample], color=COLORS["measured"], label="iq measured")
    top.set_ylabel("iq (A)")
    top.legend(loc="lower right")
    bottom.plot(t, [float(row["duty_span"]) for row in sample], color=COLORS["aux"], label="duty span (written back)")
    bottom.set_ylabel("max duty - min duty")
    bottom.set_xlabel("time (ms)")
    bottom.legend(loc="upper right")
    for axis in (top, bottom):
        axis.grid(True, which="both", alpha=0.3)
    top.set_title("Through the C ABI: the command in, the state and the duties back out")
    save(fig, "slice_s9_c_abi")


def plot_slice_s4(rows: list[dict[str, str]]) -> None:
    """Where the currents go above base speed, and what torque is left."""
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 4.0))
    id_a = [float(row["id_a"]) for row in rows]
    iq_a = [float(row["iq_a"]) for row in rows]
    ratio = [float(row["speed_ratio"]) for row in rows]
    torque = [float(row["torque_nm"]) for row in rows]
    limited = [row["voltage_limited"] == "1" for row in rows]

    first = next((i for i, flag in enumerate(limited) if flag), None)
    if first is not None:
        left.plot(id_a[:first + 1], iq_a[:first + 1], color=COLORS["aux"], linewidth=2.4,
                  label="current-limited (MTPA)")
        left.plot(id_a[first:], iq_a[first:], color=COLORS["reference"], linewidth=2.0,
                  label="voltage-limited")
        left.plot([id_a[first]], [iq_a[first]], "o", color="black", markersize=4)
        left.annotate("base speed", xy=(id_a[first], iq_a[first]),
                      xytext=(id_a[first] + 0.08, iq_a[first] - 0.35), fontsize=8,
                      arrowprops={"arrowstyle": "->", "linewidth": 0.8})
    else:
        left.plot(id_a, iq_a, color=COLORS["measured"], linewidth=1.2)
    # The asymptote the sweep runs to: it is the last sample's id, not a constant
    # written here -- the figure may show it, the page may not assert it.
    # 扫描趋向的那条渐近线：取最后一个采样点的 id，而不是在这里写一个常数。
    left.axvline(id_a[-1], color="grey", linestyle=":", linewidth=0.9)
    left.annotate("runs to -psi_f/Ld", xy=(id_a[-1], min(iq_a)), xytext=(id_a[-1] + 0.06, min(iq_a) + 0.12),
                  fontsize=8, color="dimgrey")
    left.set_ylim(0.0, max(iq_a) * 1.18)
    left.set_xlabel("id (A)")
    left.set_ylabel("iq (A)")
    left.set_title("the current path leaves MTPA and runs to -psi_f/Ld")
    left.grid(True, alpha=0.3)
    left.legend(loc="upper right", fontsize=8)

    right.plot(ratio, torque, color=COLORS["measured"])
    if first is not None:
        right.axvline(ratio[first], color=COLORS["reference"], linewidth=0.9, linestyle="--")
        right.annotate(f"voltage binds at {ratio[first]:.2f} x base", xy=(ratio[first], max(torque) * 0.5),
                       xytext=(ratio[first] + 0.8, max(torque) * 0.5), fontsize=8, color=COLORS["reference"])
    right.set_xlabel("electrical speed / base speed")
    right.set_ylabel("torque (N.m)")
    right.set_title("torque left above base speed")
    right.set_ylim(bottom=0.0)
    right.grid(True, alpha=0.3)
    fig.tight_layout()
    save(fig, "slice_s4_field_weakening")


def plot_slice_s7(rows: list[dict[str, str]]) -> None:
    """Sampling and the power stage: when a single shunt can measure, and what
    the dead-time correction looks like once it is read in the alpha-beta frame.

    Neither panel asserts a threshold of its own: the "needed" window and
    `deadtime_v` both arrive in the CSV's `limit` column, straight out of the
    configuration the program ran with.
    两块图都不自己断言阈值：窗口所需的时长与 `deadtime_v` 都来自 CSV 的 `limit` 列，
    直接就是程序运行时的配置。
    """
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 3.9))

    # Left: the window a single shunt leaves you, against the modulation index,
    # at three angles. A point where the library said "not measurable" is drawn
    # as a hollow marker on the axis, so the flat-at-zero series is visible rather
    # than absent.
    # 左：单电阻留下的窗口随调制比的变化，三个角度。库判为"测不了"的点画成空心标记压在
    # 横轴上，这样"恒为零"的那条线是看得见的，而不是不存在。
    merged: list[list[tuple[float, float, float]]] = []
    for angle, colour in ((0, COLORS["reference"]), (20, COLORS["measured"]), (30, COLORS["aux"])):
        series = [row for row in rows if row["kind"] == f"shunt{angle}"]
        if not series:
            continue
        measurable = [(float(r["x"]), float(r["value"])) for r in series if r["flag"] == "0"]
        refused = [(float(r["x"]), float(r["value"])) for r in series if r["flag"] != "0"]
        if measurable:
            left.plot([p[0] for p in measurable], [p[1] for p in measurable], color=colour, linewidth=1.8,
                      label=f"{angle} deg")
        if refused:
            # Hollow markers, not clipped: every sample of the 0 deg series is
            # refused, so those markers *are* the series and they land exactly on
            # the axis. Drawn as clipped markers it would look missing.
            # 空心标记且不裁剪：0 度那条线每个采样都被判不可测，这些标记**就是**那条线，
            # 而且正好落在横轴上。按裁剪画会看起来像根本没这条线。
            left.plot([p[0] for p in refused], [p[1] for p in refused], "^", color=colour, markersize=4.0,
                      alpha=0.7, markerfacecolor="white", clip_on=False, zorder=5,
                      label=f"{angle} deg (refused)")
        merged.append([(float(r["x"]), float(r["value"]), float(r["limit"])) for r in series])

        # Where the window stops growing is the modulator clamping, not the shunt:
        # read the knee from the data instead of naming a modulation index.
        # 窗口不再变大的地方是调制器钳位，不是采样电阻：拐点从数据里读，不在这里
        # 写一个调制比。
        points = [(float(r["x"]), float(r["value"])) for r in series]
        knee = next((points[i][0] for i in range(1, len(points))
                     if points[i][1] == points[i - 1][1] and points[i - 1][1] > 0.0), None)
        if knee is not None and angle == 30:
            left.axvline(knee, color=colour, linestyle=":", linewidth=0.9)
            left.annotate("flattens where the modulator clamps", xy=(knee, 6.0), xytext=(knee + 0.03, 11.0),
                          fontsize=8, color=colour)

    needed = next((point[2] for series in merged for point in series if point[2] > 0.0), 0.0)
    if needed > 0.0:
        left.axhline(needed, color="black", linestyle="--", linewidth=0.9)
        left.annotate(f"settle + sample = {needed:.2f} us", xy=(0.02, needed), xytext=(0.05, needed + 0.6),
                      fontsize=8)
    left.set_xlabel("modulation index m")
    left.set_ylabel("sampling window (us)")
    left.set_ylim(bottom=0.0)
    left.grid(True, alpha=0.3)
    left.legend(loc="upper left", fontsize=7, title="vector angle (hollow = refused)", title_fontsize=7)
    left.set_title("one shunt: the window depends on the angle")

    # Right: the correction the library commands, read as the alpha-beta-visible
    # voltage. The dashed line is the per-phase loss that was asked for; neither
    # curve has to touch it, and the page says why.
    # 右：库下达的修正，按 αβ 可见电压读数。虚线是要求的单相损失；两条曲线都不必碰到它，
    # 原因在正文里。
    for kind, colour, style, label in (("dead", COLORS["reference"], "-", "step (i_fade = 0)"),
                                       ("ramp", COLORS["measured"], "-", "ramp (i_fade > 0)")):
        series = [row for row in rows if row["kind"] == kind]
        if not series:
            continue
        right.plot([float(r["x"]) for r in series], [float(r["value"]) for r in series], style, color=colour,
                   linewidth=1.8, label=label)
    limit = next((float(r["limit"]) for r in rows if r["kind"] in ("dead", "ramp")), 0.0)
    if limit > 0.0:
        right.axhline(limit, color="black", linestyle="--", linewidth=0.9)
        right.annotate(f"deadtime_v = {limit:.3f} V (per phase)", xy=(0.11, limit), xytext=(0.045, limit + 0.03),
                       fontsize=8)
    right.set_xlabel("|i| on the measured phase (A)")
    right.set_ylabel("correction, alpha-beta visible (V)")
    right.set_xlim(0.0, 0.14)
    right.set_ylim(bottom=0.0)
    right.grid(True, alpha=0.3)
    right.legend(loc="lower right", fontsize=8)
    right.set_title("dead time: step vs ramp")

    fig.tight_layout()
    save(fig, "slice_s7_sampling")


def plot_slice_s2(rows: list[dict[str, str]]) -> None:
    """Zero speed: whether the estimate moves at all, and why a surface-mount
    rotor never gets one.

    The left panel is the angle error against time for the three runs the example
    makes; the right panel is the library's own sensitivity function, whose zero
    at a saliency ratio of one is the whole reason the first run is flat.
    左：三次运行的角度误差随时间；右：库自己的灵敏度函数——它在凸极比为一时恰为零，
    这正是第一次运行那条平线的原因。
    """
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 3.9))

    # A run that never locks and a run that never started look the same on this
    # axis, so the lock flag decides the line style rather than the shape.
    # 从不锁定的运行与从未开始的运行在这条轴上看起来一样，所以用锁定标志决定线型，
    # 而不是靠形状。
    for kind, colour, style, label in (("spm", COLORS["reference"], "--", "surface-mount rotor"),
                                       ("slow-control", COLORS["aux"], "--", "carrier the rate cannot resolve"),
                                       ("ipm", COLORS["measured"], "-", "interior rotor")):
        series = [row for row in rows if row["kind"] == kind]
        if not series:
            continue
        locked = any(row["flag"] == "1" for row in series)
        # On a linear axis the decay is over before the first tick, so the panel is
        # capped at the window where something happens and the error is plotted on a
        # log axis. Zero is clipped to the floor: a perfect lock has no error to plot.
        # 线性轴上衰减在第一个刻度之前就结束了，所以横轴截到有事发生的窗口，纵轴取对数。
        # 零被裁剪到下限：完美锁定本身没有误差可画。
        # The two runs that never lock sit on exactly the same value for the whole
        # window -- they are the same line, which is the point: "no signal" and "the
        # rate cannot resolve the carrier" are indistinguishable from the outside. The
        # first is drawn wide and pale so the second is visible on top of it.
        # 两条从不锁定的运行在整个窗口上落在完全相同的值上——它们是同一条线，而这正是要点：
        # “没有信号”与“控制率分辨不了载波”从外面看分不开。第一条画粗画淡，好让第二条压在它
        # 上面仍然可见。
        width = 1.7 if locked else (3.2 if kind == "spm" else 1.1)
        alpha = 1.0 if locked else (0.35 if kind == "spm" else 1.0)
        left.plot([float(r["x"]) for r in series],
                  [max(float(r["value"]), 1e-6) for r in series], style, color=colour, linewidth=width,
                  alpha=alpha, label=f"{label}" + ("" if locked else " (never locks)"))
    left.set_xlabel("time (ms)")
    left.set_ylabel("|angle estimate - truth| (rad)")
    left.set_xlim(0.0, 100.0)
    left.set_yscale("log")
    left.set_ylim(1e-6, 1.5)
    left.grid(True, alpha=0.3)
    left.annotate("flat: the estimate never moved", xy=(70.0, 0.72), xytext=(34.0, 2.0e-3), fontsize=8,
                  arrowprops={"arrowstyle": "->", "linewidth": 0.8})
    left.legend(loc="upper right", fontsize=7)
    left.set_title("standstill: does the estimate move at all")

    # Which carrier a control rate can resolve, straight from the library.
    # 一个控制率能分辨多高的载波，直接问库。
    series = [row for row in rows if row["kind"] == "gain"]
    if series:
        ratio = [float(r["x"]) for r in series]
        gain = [float(r["value"]) for r in series]
        right.plot(ratio, gain, color=COLORS["measured"], linewidth=1.8)
        crossing = next((r for r, g in zip(ratio, gain) if g != 0.0), None)
        if crossing is not None:
            right.axvline(crossing, color=COLORS["reference"], linestyle=":", linewidth=1.0)
            right.annotate("no saliency, no signal", xy=(crossing, min(gain) * 0.98), xytext=(crossing + 0.03, min(gain) * 0.98),
                           fontsize=8, color=COLORS["reference"], rotation=90, va="bottom")
        right.set_xlabel("Lq / Ld")
        right.set_ylabel("demodulation sensitivity (A)")
        right.grid(True, alpha=0.3)
        right.set_title("what the method has to measure")

    fig.tight_layout()
    save(fig, "slice_s2_zero_speed")


def plot_slice_s8(rows: list[dict[str, str]]) -> None:
    """Protection: how long an overload is allowed to last, and the two numbers
    that look like one decision but are not.

    Left: the i2t budget against current -- infinite at the continuous rating,
    finite above it, and it never reaches the hard trip because the hard trip
    answers first. Right: the continuous limit against temperature, with the trip
    threshold drawn on top of it, where the limit is still not zero.
    左：i2t 预算随电流——额定处为无穷，往上有限，而且永远到不了硬跳闸，因为硬跳闸先回答。
    右：连续限幅随温度，跳闸阈值画在它上面，而那里限幅还不为零。
    """
    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 3.9))

    series = [row for row in rows if row["kind"] == "i2t" and row["flag"] == "1"]
    if series:
        current = [float(r["x"]) for r in series]
        seconds = [float(r["value"]) for r in series]
        left.plot(current, seconds, color=COLORS["measured"], linewidth=1.8)
        # The curve is read, not drawn: the shortest and longest times in the run
        # are annotated, and neither is a constant typed here.
        left.annotate(f"{seconds[0]:.2f} s", xy=(current[0], seconds[0]), xytext=(current[0] + 0.05, seconds[0]),
                      fontsize=8, va="center")
        left.annotate(f"{seconds[-1]:.2f} s", xy=(current[-1], seconds[-1]), xytext=(current[-1] - 0.02, seconds[-1] * 1.25),
                      fontsize=8, ha="right")
        left.set_ylabel("time to the i2t trip (s)")
        left.set_xlabel("steady current (A)")
        left.set_yscale("log")
        left.grid(True, alpha=0.3)
        left.set_title("how long an overload lasts")

    series = [row for row in rows if row["kind"] == "derate"]
    if series:
        temperature = [float(r["x"]) for r in series]
        limit = [float(r["value"]) for r in series]
        right.plot(temperature, limit, color=COLORS["measured"], linewidth=1.8, label="continuous limit")
        # The trip is a separate number, and it comes from its own row -- drawing it
        # where the curve reaches zero would show the derating curve's end instead of
        # the threshold, and those are two different temperatures.
        # 跳闸是另一个数，来自它自己那一行——画在曲线到零的地方，画出来的会是降额曲线的终点而
        # 不是阈值，而那是两个不同的温度。
        trip = next((r for r in rows if r["kind"] == "trip"), None)
        if trip is not None:
            trip_c = float(trip["x"])
            right.axvline(trip_c, color=COLORS["reference"], linestyle="--", linewidth=1.2)
            right.plot([trip_c], [float(trip["value"])], "o", color="black", markersize=4)
            right.annotate(f"trip above {trip_c:.0f} C,\nlimit still {float(trip['value']):.2f} A",
                           xy=(trip_c, float(trip["value"])),
                           xytext=(trip_c - 58.0, float(trip["value"]) + 0.32), fontsize=8,
                           arrowprops={"arrowstyle": "->", "linewidth": 0.8})
        right.set_xlabel("winding temperature (C)")
        right.set_ylabel("allowed current (A)")
        right.set_ylim(bottom=0.0)
        right.grid(True, alpha=0.3)
        right.set_title("the limit and the trip")

    fig.tight_layout()
    save(fig, "slice_s8_protection")


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
        run_slice("slice-s9-c-abi", pathlib.Path(workdir), plot_slice_s9)
        run_slice("slice-s4-field-weakening", pathlib.Path(workdir), plot_slice_s4)
        run_slice("slice-s7-sampling", pathlib.Path(workdir), plot_slice_s7)
        run_slice("slice-s2-zero-speed", pathlib.Path(workdir), plot_slice_s2)
        run_slice("slice-s8-protection", pathlib.Path(workdir), plot_slice_s8)
    print("done: figures in docs/images/OUT_demo_*.png, tables in docs/sphinx/source/demos/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
