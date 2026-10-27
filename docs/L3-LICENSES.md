# L3 · Licence matrix — can this be used, and how?

**Purpose / 用途**：before any upstream FOC project is read closely, vendored, or copied from, look
it up here. The library is **Apache-2.0 and meant to be linked into products**, so a single copyleft
source file would relicense the whole thing.
在细读、引入或摘抄任何一个上游 FOC 项目之前，先查这张表。本库是 **Apache-2.0，并且要被链接进产品**，
一个 copyleft 源文件就会把整个库的许可改掉。

**Enforced / 由门禁强制**：`python3 .github/misc/gates.py license` checks every file under
`include/` and `src/` against `.github/misc/.license-guard.txt`, and is part of `gates.py all`.
Self-tested: a `SPDX-License-Identifier: GPL-3.0-or-later` line, a "GNU General Public License"
header, and an `AGPL` mention all fail; `Apache-2.0` / `MIT` / `BSD-*` / `ISC` / `CC0` / `Unlicense`
pass.
`python3 .github/misc/gates.py license` 会对 `include/` 与 `src/` 下每个文件按 `.license-guard.txt`
检查，并已并入 `gates.py all`。已自测：GPL 的 SPDX 标识、GPL 抬头原文、`AGPL` 字样都会失败；
`Apache-2.0`/`MIT`/`BSD-*`/`ISC`/`CC0`/`Unlicense` 通过。

---

## 1. The matrix / 许可矩阵

| Project / 项目 | Licence / 许可 | Evidence / 证据 | Vendor into this library? / 可否引入本库 | How we may use it / 我们可以怎么用 |
|---|---|---|---|---|
| **SimpleFOC** | **MIT** | `LICENSE` 原文（Copyright 2020 Antun Skuric），2026-10-27 审计核实 | ✅ yes, keep the notice / 可以，保留声明 | Behaviour spec + the HAL breadth checklist. **Not code**: its Arduino/global-object model conflicts with ADR-001 (`include/` must be allocation-free, exception-free) and ADR-002 (policy composition, no globals). / 行为规格 + HAL 广度清单。**不引代码**：其 Arduino/全局对象模型与 ADR-001/002 冲突 |
| **moteus** | **Apache-2.0** | `README` 正文，2026-10-27 审计核实 | ✅ yes, keep `NOTICE` / 可以，保留 `NOTICE` | Best reference for the *command surface* (field semantics, `NaN` = hold position, gain scales) and for field-weakening calibration flow. Same licence family as ours. / 命令面语义与弱磁标定流程的最佳参考，且与本库同族许可 |
| **VESC (bldc)** | **GPLv3** | `README` 徽章 + 正文 | ❌ **never** — copyleft is viral and would end the Apache-2.0 licence and any commercial use / **绝不可**，传染性会终结本库的 Apache-2.0 与商业可用性 | Capability/parameter **semantics only**. Do not copy code *or comments*; write the behaviour down ourselves and test it. / 只对标能力与参数语义；**连注释都不要抄**，自己写成测试 |
| **miniFOC** | **AGPL-3.0** | `README` 正文 | ❌ **never** — the network clause is poison for a library / **绝不可**，网络条款对库是毒药 | Reference for "low-cost hardware, no current loop" only. / 仅作低成本硬件、无电流环的参考 |
| **SguanFOC** | **MIT** | `LICENSE` 原文（Copyright 2026 星必尘Sguan） | ✅ yes | Per D8/D29: thinking reference only, never in the build, never in the library. / 按 D8/D29：只作思路参考，不进构建、不进库 |
| **ODrive (firmware)** | **UNVERIFIED / 未核实** | ⚠️ three attempts on 2026-10-27 all failed: `raw.githubusercontent.com/.../LICENSE` (content extraction failed), the GitHub repo page (failed), `docs.odriverobotics.com` (site renders no licence). **Do not vendor until somebody reads the actual `LICENSE` file.** / 三次抓取均失败，**在有人真正读到 `LICENSE` 之前不得引入** | ❓ unknown | Behaviour spec only, which is licence-safe regardless: spinout detection, input modes, anticogging semantics. / **行为规格**（无论许可如何都安全）：spinout 检测、输入模式、anticogging 语义 |
| **ST MCSDK / TI InstaSPIN / FpOC** | unverified / 未核实 | not fetched in the 2026-10-27 audit / 本次审计未抓取 | ❓ | Treat like ODrive: behaviour first, code never, until verified. / 与 ODrive 同处理 |

> **Two ways to "integrate", and only one of them is about code / 两种"接入"，只有一种涉及代码**
>
> 1. **Behaviour integration (always allowed)** — write our own test that encodes what the other
>    project guarantees, then satisfy it. No licence question at all, and the test is the artefact
>    that survives.
>    行为接入（永远可行）：把对方承诺的行为写成我们自己的测试，然后满足它。完全不涉及许可问题，
>    而且留下的产物就是测试。
> 2. **Code integration (only for permissive)** — copy, keep the notice, extend `NOTICE`, and let
>    `gates.py license` keep everyone honest. A vendored subtree that is legitimately copyleft gets a
>    scope line in `.license-guard.txt` and an entry in `NOTICE`; the library default stays permissive.
>    代码接入（仅限宽松许可）：复制、保留声明、扩 `NOTICE`，并让 `gates.py license` 守住。确实需要
>    copyleft 子目录时，在 `.license-guard.txt` 加作用域行并在 `NOTICE` 登记；库本身默认仍是宽松许可。

## 2. Hardware is not code / 硬件不是代码

Driving **hardware** designed for another project needs no licence at all — that is the cheapest real
integration available: implement the five HAL interfaces (`Pwm`, `CurrentSense`, `PositionSensor`,
`DcLink`, `Clock`) for the board and the library serves it.
为别人的项目设计的**硬件**写驱动不涉及任何许可——这是最便宜的真实接入：为那块板子实现五个 HAL 接口
（`Pwm`、`CurrentSense`、`PositionSensor`、`DcLink`、`Clock`），库就能服务它。

## 3. Checklist before touching an upstream project / 动手前的检查单

- [ ] Its row exists above and the licence column is not ❓ — otherwise stop and verify first.
      表里有这一行且许可栏不是 ❓，否则先核实。
- [ ] I am reading it for **behaviour**, or for **code that is permissive**.
      我是在看**行为**，还是在看**宽松许可的代码**。
- [ ] If code: `NOTICE` updated, `gates.py license` still green.
      若引代码：更新 `NOTICE`，`gates.py license` 仍绿。
- [ ] Numbers I "remember" about it are re-checked, because this file is evidence-based on purpose.
      关于它我"记得"的数字都要重新核对——本文件刻意以证据为准。
