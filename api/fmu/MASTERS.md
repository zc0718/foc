# Which FMI masters this FMU has actually been loaded in

**Why this page exists / 这页为什么存在**

"Standard compliance" is not a property we can assert about our own code, because the only thing that
can demonstrate it is somebody else's tool. So the claim is narrowed to something checkable: *these
masters, these versions, this run*. A matrix with one verified cell is honest; a sentence saying "FMI 2.0
compliant" is not.
"合规"不是我们对自己代码能下的断言——唯一能证明它的是别人的工具。所以主张被收窄成可核对的东西：
**这些主控、这些版本、这一次运行**。一个只有一个有效格的矩阵是诚实的；一句"符合 FMI 2.0"不是。

**Every "verified" cell names the command that reproduces it.** A cell that cannot be reproduced is not
verified, whatever it says.
**每个"已验证"格都写着复现它的命令。** 复现不了的格子不是已验证，不管它写的是什么。

---

## 1. The matrix / 矩阵

| Master | Version | FMI 2.0 load | 50 µs fixed step | Refuses a wrong step | Reproduce with |
|---|---|---|---|---|---|
| **FMPy** (Python, `FMU2Slave`) | see `workspace/materials/FMPy` | ✅ verified | ✅ verified | ✅ verified | `python3 .github/project/gates.py fmu` |
| **FMPy** (`fmpy.simulate`, the high-level API) | same | ⚠️ not run | — | — | not attempted; the low-level path is what a master actually does |
| OpenModelica / OMSimulator | — | ❌ not run | — | — | — |
| SimulationX | — | ❌ not run | — | — | — |
| Dymola | — | ❌ not run | — | — | — |
| MATLAB / Simulink FMI import | — | ❌ not run | — | — | — |
| `fmpy` on Windows (`.dll`) | — | ❌ not run | — | — | the wrapper is portable C99; the *build* is not tested there |

"❌ not run" is not a failure — it is the absence of evidence, which is what a matrix is for. It becomes a
failure the moment somebody writes the word "compliant" without filling a cell in.
"❌ 未运行"不是失败——它是证据的缺失，而这正是矩阵的用途。只有在有人没填格子就写下"合规"两个字时，它才变成失败。

## 2. What the verified cell actually asserts / 那个已验证格到底断言了什么

`api/fmu/smoke.py` drives 2000 ticks at 50 µs with the master owning the rotor and reporting the measured
currents, then reads back the duties. Three separate assertions, each on its own line so a red run says
which part of the contract broke:

1. the controller reaches `Run` with `error == 0` after being enabled through the FMI interface;
2. the duties leave 0.5 (the loop actually modulated) and stay inside `[0, 1]`;
3. **a doubled communication step is refused with `fmi2Error`, and the correct step at the same timeline
   point is accepted immediately afterwards** — both, in that order, because "refuses everything" would
   satisfy the first half alone.

Item 3 is the one added on 2026-11-06 (F-1). Before it, `modelDescription.xml` declared
`canHandleVariableCommunicationStepSize="true"` while `fmi2DoStep` fed `communicationStepSize` straight
into `foc_tick`: a variable-step master would have run a controller tuned for a different period, with no
error and no warning. The declaration is now `false`, the wrapper enforces it, and this matrix's third
column is what makes the enforcement observable from outside.
第 3 条是 2026-11-06（F-1）加的。在这之前 `modelDescription.xml` 声明
`canHandleVariableCommunicationStepSize="true"`，而 `fmi2DoStep` 把 `communicationStepSize` 直接喂给
`foc_tick`：一个变步长主控会开动一台按别的周期调参的控制器，无错误、无警告。现在声明是 `false`，包装器强制执行，
而本矩阵的第三列就是让那次强制执行**从外部可观察**的东西。

## 3. What this page deliberately does not do / 这页故意不做什么

- **It is not a boundaries-page row.** `boundaries.rst` pins every statement to a `TEST(Suite, Case)` in
  this repository, and the FMU's evidence is by construction *outside* the repository: the whole point of
  a master matrix is that the witness is a tool we did not write. Forcing it into a page whose evidence
  model is "a gtest case in this repo" would either break that gate's rule or make the row point at a case
  that does not demonstrate the claim. The matrix is the right home for it.
  它**不是** boundaries 页的一行。`boundaries.rst` 把每条声明钉在本仓库的一个 `TEST(Suite, Case)` 上，而 FMU 的
  证据按定义在仓库**之外**：主控矩阵的全部意义就是"证人不是我们写的工具"。把它硬塞进一个证据模型是"本仓库的 gtest
  用例"的页面里，要么破坏那道门的规则，要么让那一行指向一个证明不了该主张的用例。矩阵才是它该待的地方。
- **No FMI 3.0 column.** There is no FMI 3.0 adapter yet (`X-F2`), and an empty column would read as
  "3.0 is nearly there". When the adapter exists it gets its own matrix, with its own verified cell.
  没有 FMI 3.0 列。FMI 3.0 适配器还不存在（`X-F2`），而一个空列会被读成"3.0 快好了"。适配器出现时它会有自己的
  矩阵与自己的有效格。
- **No performance numbers.** A co-simulation step's cost belongs to the master's loop, not to this block,
  and quoting one here would be a number about somebody else's scheduler.
  没有性能数字。协同仿真一步的代价属于主控的循环而不是这个块，在这里引用它就是关于别人调度器的数字。
