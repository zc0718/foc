---
name: het-patent
description: 'Patent mining & disclosure drafting for the fcpp template. Use when: mine patentable points, draft patent disclosures (专利交底书), analyze codebase for IP, create patent documents from code. 专利挖掘：从代码库挖掘可专利创新点并生成标准格式交底书（背景/方案/替代/保护点）。'
license: Apache-2.0
argument-hint: '[project scope, e.g., "src/" or "整个项目" / 分析范围]'
user-invocable: true
---

# N8 · het-patent — Patent Mining & Disclosure Drafting（专利挖掘与交底书撰写）

> For developers. Ported from HeT-AI `het-patent` and refactored for the fcpp template: same mining methodology, fcpp-convention frontmatter, drafts land in `workspace/patents/`（git 忽略，不污染源码树）。

## What This Skill Produces

From a software project (any language/framework), this skill:
1. **Mines patentable technical innovations** — focusing on "dimensional elevation" to distinguish from public knowledge
2. **Generates a collection of patent disclosure documents** (markdown `.md` files) with the standard structure:
   - §1 Background technology, existing technology, and their shortcomings
   - §2 Technical solution (problem → full solution with PlantUML/Mermaid diagrams → beneficial effects)
   - §3 Alternative embodiments
   - §4 Key technical protection points
3. **Post-processes every draft through a de-AI writing pass** (see [de-ai-rules.md](./references/de-ai-rules.md)) — removes AI-stereotypical phrasing while preserving patent formalism, quantitative anchors, and mechanism-level claims

**Output language:** Default is **Chinese (中文)**. English is supported if the user explicitly requests it.
**Diagram format:** **PlantUML** preferred; Mermaid as fallback. Both use editable text syntax.
**Other visual elements:** Use **tables** for structured comparisons, feature matrices, and quantitative data. Use **KaTeX math** (`$...$` inline, `$$...$$` block) for formulas, equations, and algorithmic notation when they clarify the technical mechanism.

## When to Use

- User asks to find patentable points in a project
- User wants to draft patent disclosure documents (专利交底书)
- User wants to analyze a codebase for intellectual property
- User mentions "专利", "创新点", "交底书", "patent", "IP mining"

## Prerequisites

This skill uses only file-system tools (read, search, write). No external dependencies.

---

## Workflow

### Phase 1: Scope Definition

Ask the user to specify the project scope to analyze. If not provided, default to the entire workspace.

**Decision points:**
- Which directories/files to include? (e.g., `include/`, `src/`, `api/`, `benchmark/`, specific modules)
- Are there design docs, papers, or architecture documents to reference? (`.github/misc/Codegen-Starter.txt`, `docs/`, module READMEs)
- Any known innovation areas the user wants emphasized?

**Output:** A confirmed scope statement. Create a todo list tracking all phases.

### Phase 2: Deep Code Analysis

Read and analyze the project thoroughly. This is the **most critical phase** — shallow analysis produces weak patents.

**Must read (in parallel where possible):**
1. `README` / top-level documentation — understand what the project does
2. Core source files — understand the architecture and key abstractions (`include/*.hpp`, `src/*.cpp`)
3. Configuration/setup files — understand the build and integration patterns (`conanfile.py`, `metadata.json`, `CMakeLists.txt`)
4. Test cases — understand usage patterns and edge cases (`test_package/test/`)
5. Any papers or design docs referenced by the user

**Analysis dimensions to track:**
- **Memory layout & data flow:** How data moves between components, memory representation choices
- **Coupling & dependency:** How components are wired, dependency injection patterns
- **State machines & lifecycles:** Object/process lifecycles, state transitions
- **Protocols & contracts:** Interfaces between subsystems, API boundaries
- **Error handling & fault tolerance:** Where and how errors are caught and recovered
- **Concurrency & parallelism:** Thread/process models, synchronization mechanisms
- **Configuration & extensibility:** Plugin systems, configuration priority chains

### Phase 3: Innovation Point Mining

Extract patentable points using **system-layer decomposition** — identify innovations by analyzing the project at each layer of the computing stack, then elevate each finding to a framework-agnostic technical contradiction.

#### 3.1 Mining Methodology: Domain-Adaptive Layer-Decomposition + Contradiction Extraction

**Meta-Principle (applies to ALL project types):**

Every software project controls some set of **constrained resources** and traverses some **critical execution path**. To find hardcore patentable innovations, identify the project's core resource/constraint axis, then decompose along that axis. The layer table you use must match the project domain — using the wrong decomposition produces weak or irrelevant patents.

**Step 1 — Identify the project domain and select the matching decomposition table:**

*If the project spans multiple domains, run the decomposition for each domain independently.*

> **fcpp 模板适用域提示**: 本模板是 C/C++ 系统级库，按模块套用域表——`etl`（嵌入式模板库）→ Domain E；`net` → Domain C；`cpptest`/`ctest`/`doctest`（测试框架）→ Domain A；`api/python_bindings` → Domain A/C。数值/算法密集处再补 Domain D。

**Step 2 — For each layer where a non-trivial solution exists:**
1. State the **generic technical contradiction** (not tied to the project's specific domain)
2. Identify the **concrete system-behavior resolution** (what the code actually does at that layer)
3. Formulate a **patentable inventive step** that is framework/language-agnostic

---

##### Domain A: Computing Systems / ML / Data Infrastructure

Covers: MLOps platforms, distributed systems, databases, compilers, runtimes, OS-level tools.

| Layer | Probe Question | What to Extract |
|---|---|---|
| **Memory** | How does data layout avoid copy/deserialize/pointer-chase? | Memory layout inventions, zero-copy, in-place mutation |
| **Instruction** | How does the code path eliminate branches/call-stack rebuild/recompile? | Path convergence, branch-prediction optimization, cold-start avoidance |
| **I/O** | How does data transfer avoid disk/network/string-parse round-trips? | I/O elimination, serialization avoidance, streaming protocols |
| **Cache** | How does access pattern preserve cache-line locality? | Cache-friendly structures, false-sharing prevention |
| **Lock** | How does concurrency avoid mutex/transaction/barrier contention? | Lock-free structures, granularity reduction, deadlock prevention |
| **State Machine** | How does lifecycle avoid invalid-state entry/deadlock/leak? | State validation at load-time, completeness guarantees |

---

##### Domain B: GUI / Interactive Applications / Frontend

Covers: Desktop apps, web frontends, mobile UI, game engines, visualization tools.

| Layer | Probe Question | What to Extract |
|---|---|---|
| **Rendering Pipeline** | How does the draw path avoid redundant repaint/layout/reflow? | Dirty-region tracking, incremental update, layer compositing |
| **Event Routing** | How does input dispatch avoid O(n) walk or conflicting handlers? | Event delegation, capture-bubble optimization, gesture arbitration |
| **State Sync** | How does UI state stay consistent with backing data without full rebuild? | Uni-directional data flow, diff-patch, reactive dependency tracking |
| **Component Lifecycle** | How does create/update/destroy avoid leak, stale ref, or double-init? | Lifecycle guard, dispose pattern, mount/unmount ordering |
| **Asset/Resource** | How are fonts/images/shaders loaded without blocking the UI thread? | Lazy decode, texture atlas, streaming load with priority |
| **Layout Constraint** | How does constraint solving avoid exponential backtracking? | Cassowary/linear constraint solving, incremental layout propagation |

---

##### Domain C: Server / Network Services / API Backends

Covers: Web servers, microservices, API gateways, message queues, proxies.

| Layer | Probe Question | What to Extract |
|---|---|---|
| **Request Lifecycle** | How does request admission/processing/teardown avoid head-of-line blocking? | Connection pooling, pipelining, priority queuing |
| **Data Consistency** | How does state stay consistent across nodes without global lock? | CRDT, vector clocks, consensus protocol optimization |
| **Fault Isolation** | How does a single failure avoid cascading to the whole cluster? | Circuit breaker, bulkhead, graceful degradation |
| **Message Routing** | How does message dispatch avoid O(n) topic matching per message? | Trie/prefix-tree routing, subscription indexing |
| **Serialization** | How does wire format reduce schema overhead and version skew? | Schema evolution, zero-copy serialization, delta encoding |
| **Rate/Flow Control** | How does backpressure propagate without dropping critical data? | Adaptive throttling, token bucket with priority lanes |

---

##### Domain D: Algorithms / Computational Modules / Scientific Computing

Covers: Numerical libraries, optimization solvers, cryptography, signal processing, ML model components.

| Layer | Probe Question | What to Extract |
|---|---|---|
| **Complexity Reduction** | How does the algorithm avoid worst-case O(n²) or exponential blowup? | Amortized analysis, pruning, early termination, approximation guarantee |
| **Numerical Stability** | How does the computation avoid catastrophic cancellation or gradient vanish/explode? | Kahan summation, log-space computation, stable activation design |
| **Data Structure Invariant** | How does the structure maintain its property (heap/balance/sorted) under mutation? | Lazy rebalance, batch update, persistent/immutable sharing |
| **Convergence** | How does the iterative process guarantee convergence within feasible steps? | Adaptive step size, momentum, restart strategy, duality gap monitoring |
| **Precision/Quantization** | How does reduced precision avoid degrading the output beyond a threshold? | Mixed-precision, stochastic rounding, per-channel scaling |
| **Sparsity/Compression** | How does the representation exploit sparsity without dense-materialization overhead? | Block-sparse format, pruning mask, compressed storage with direct indexing |

---

##### Domain E: Embedded / IoT / Real-Time Systems

Covers: Firmware, sensor networks, robotics control loops, automotive ECUs.

| Layer | Probe Question | What to Extract |
|---|---|---|
| **Power Management** | How does the system meet latency deadlines while minimizing wake-ups? | Tickless scheduling, DMA chaining, peripheral FIFO batching |
| **Real-Time Scheduling** | How does task prioritization guarantee worst-case execution time? | Rate-monotonic, EDF, priority ceiling, lock-free preemption |
| **Sensor Fusion** | How does multi-sensor data align temporally without drift accumulation? | Timestamp interpolation, Kalman prediction, sync pulse chaining |
| **Memory Budget** | How does fixed-size allocation avoid fragmentation and OOM? | Static pool, slab allocator, buddy system with defrag |
| **Communication** | How does the radio duty-cycle balance throughput vs. battery life? | Adaptive beacon interval, burst aggregation, LBT/CCA optimization |
| **Failsafe** | How does the watchdog/reset path preserve critical state? | Brown-out save, checksum-protected NVRAM, graded shutdown |

---

**When the project does not fit any predefined domain:** Derive a custom decomposition by asking: *"What are the 3-5 constrained resources or critical paths in this system?"* Then for each, formulate a probe question that asks "How does this system avoid the naive/brute-force approach to this constraint?"

#### 3.2 Dimensional Elevation

Describe each innovation at **one level lower** than its surface functionality:

| Surface (Weak — Rejectable) | Elevated (Strong — Defensible) |
|---|---|
| "方便用户配置" | "消除配置解析的二次反序列化CPU周期消耗" |
| "提前发现错误" | "在GPU算力分配之前阻断无效试验启动路径" |
| "提高开发效率" | "避免跨内存区域拷贝以降低缓存未命中率" |

#### 3.3 Categorization

- **Base patent (系统级):** The overarching system architecture, scheduling, or orchestration mechanism — provides the foundation that other patents depend on
- **Method patents (方法级):** Specific algorithms, data structures, or protocols within the system
- **Peripheral patents (外围):** Supporting mechanisms (validation, logging, configuration, escape hatches)

#### 3.4 Quality Gate — Each Point Must Satisfy ALL Four

1. **Technical contradiction identified:** What physical/system-level problem exists in ALL known approaches? (e.g., memory separation, coupling, serialization overhead, lock contention)
2. **System-behavior resolution:** Innovation described at memory layout / instruction path / cache behavior / I/O pattern / lock granularity level — NOT at UX level
3. **Framework/language-agnostic core:** The inventive step does not depend on a specific library, language feature, or framework API
4. **Clear causal chain:** problem (system bottleneck) → mechanism (what the invention does at hardware/OS/runtime level) → effect (quantifiable improvement in system metric)

#### 3.5 Output

A table of patent points with ID, title, category, core inventive step, and priority (1=highest). **Present to user for approval** before proceeding to drafting. Do NOT skip approval — misidentified points cause the most rework.

### Phase 4: Patent Disclosure Drafting

For each approved patent point, generate a markdown file with a **semantic filename** at `workspace/patents/` (create the directory if it does not exist). Follow the [patent disclosure template](./references/patent-template.md).

**File naming:** Use a descriptive Chinese name derived from the patent title. Examples:
- `双视图同体内存表征方法.md` (not `p1.md`)
- `类加载钩子静态校验方法.md` (not `p2.md`)
- `零耦合纯标注元数据挂载机制.md` (not `p3.md`)

**Output language:** Chinese by default. Use English only if the user explicitly requests it.

**Each document must contain:**

#### §1 背景技术 (Background)
- Industry context and problem domain
- **At least 2-3 existing technology approaches** with concrete technical shortcomings
- Shortcomings described at the **system/memory/CPU level**, not the user-experience level
- A concluding paragraph stating the unresolved technical contradiction

#### §2 技术方案 (Technical Solution)
- **2.1 技术问题**: A single, precise technical problem statement
- **2.2 完整技术方案**: The core of the patent. Must include:
  - Step-by-step mechanism description with **numbered substeps** (e.g., 步骤 S1-S5)
  - **PlantUML diagrams** (preferred) or **Mermaid diagrams** (fallback) for each key mechanism. Use `@startuml`/`@enduml` blocks for PlantUML; ` ```mermaid ` for Mermaid.
  - **Tables** where structured comparison or quantitative data benefits clarity (e.g., existing-tech vs. invention comparison, resource consumption breakdown, scenario matrix)
  - **KaTeX math** (`$...$` inline, `$$...$$` block) where formulas, equations, or algorithmic notation clarify the technical mechanism (e.g., complexity bounds, constraint equations, convergence criteria)
  - Each visual element (diagram/table/formula) must have a text description sufficient to understand without viewing it
  - Cross-scenario validation: describe 2+ different application scenarios showing the mechanism works across domains
- **2.3 有益效果**: Numbered beneficial effects. Each must:
  - Map to a technical problem from §2.1
  - Use **system-behavior language**: memory addressing, cache behavior, I/O patterns, instruction paths, lock contention, etc.
  - Include **quantitative projections** where possible (e.g., "百次试验规模", "千级采样频率", "毫秒级 vs 分钟级")
  - Follow the [expert guidelines](./references/expert-guidelines.md) for language quality

#### §3 替代方案 (Alternatives)
- 4-8 alternative implementations for key mechanisms
- Cover: data structure alternatives, protocol alternatives, timing alternatives, storage alternatives
- Purpose: broaden protection scope, prevent workarounds

#### §4 技术关键点 (Key Protection Points)
- Bullet list of the precise technical elements to be protected
- Each point describes a **mechanism**, not a feature
- Format: "一种...方法/系统，其特征在于..."

**Output:** Semantically-named markdown files at `workspace/patents/` (e.g., `双视图同体内存表征方法.md`).

### Phase 5: Quality Enhancement (Post-Drafting Review)

After all drafts are generated, apply the expert review guidelines from [expert-guidelines.md](./references/expert-guidelines.md):

1. **Technicalization pass:** Scan all "有益效果" sections. Replace any subjective/UX terms with system-behavior terms:
   - ❌ "方便", "高效", "提升开发效率", "提前发现", "易于使用"
   - ✅ "消除跨内存区域拷贝", "避免GPU空转", "阻断无效算力分配", "降低缓存未命中率"

2. **Quantitative anchoring:** Ensure each beneficial effect has at least one quantitative anchor (order-of-magnitude projection).

3. **Portfolio linking:** If there are ≥3 patent points, designate one base system-level patent and add cross-reference notes linking peripheral patents back to it.

---

### Phase 6: De-AI Post-Processing

After the quality-enhancement pass, run a **de-AI writing pass** on every draft following [de-ai-rules.md](./references/de-ai-rules.md):

1. **Lock protected spans first** — numbers with their objects, step numbers (S1-S5), figure/formula references, mechanism terms, patent idioms (其特征在于/所述/实施例/优选地), and all PlantUML/Mermaid/KaTeX blocks are **never touched**.
2. **Tier-graded rewriting** — Tier 1 (delete): opening clichés ("随着……的不断发展"), value inflation ("具有重要意义"), filler intensifiers (显著/极大/高效), sourceless authority claims; Tier 2 (restructure): translation-style long chains, adjective triads, nominalizations, synonym cycling; Tier 3 (keep): patent formalism.
3. **Scope by section** — long text uses `bounded` (filler sentences go to a "suggested deletion" list; 空话进"建议删除"清单); §4 protection points use `in-place` (reword only, no sentence deletion; 只改词不删句).
4. **Fidelity readback（硬约束）** — every protected span survives forward; no new numbers/attributions appear backward; missing data is flagged, never invented.
5. **Residual audit** — final scan against the blacklist/whitelist tables; Tier 3 leftovers may stay if recorded.

**Do not colloquialize**: the target register is a restrained professional patent voice, not casual prose — strip the AI boilerplate, keep the patent boilerplate. 去的是 AI 定式表达，保的是专利定式表达。

---

## Resources

- [Patent disclosure template](./references/patent-template.md) — Section-by-section format reference with PlantUML/Mermaid diagram examples
- [Expert guidelines](./references/expert-guidelines.md) — Language quality rules, rejection-avoidance patterns, self-review checklist
- [De-AI rules](./references/de-ai-rules.md) — Phase 6 post-processing: removes AI writing patterns (methodology adapted from shuorenhua/humanizer-zh) while preserving patent professionalism

---

## Example Prompts（示例）

Load `het-patent`, then give it a scope. Where the harness supports slash invocation,
`/het-patent <scope>` is an alias. 加载 `het-patent` 后给出范围；支持 `/` 的 harness 可用 `/het-patent <scope>`。

```
het-patent src/                      # Analyze src/ directory
het-patent 整个项目                    # Analyze entire workspace
het-patent include/ src/ api/        # Analyze specific modules
```

## Notes

- Drafts are saved to `workspace/patents/` under the workspace root. The directory is created if it does not exist. `/workspace/` is git-ignored — drafts never pollute the tracked source tree.
- Files use **semantic Chinese names** derived from the patent title (e.g., `双视图同体内存表征方法.md`), not serial numbers.
- The skill works with any programming language — the focus is on technical mechanisms, not language-specific syntax.
- Each draft targets ~3000-5000 words of substantive technical content.
- Visual elements: **PlantUML** (preferred) or **Mermaid** (fallback) for diagrams; **tables** for structured data; **KaTeX** for formulas when beneficial.
- Output is in Chinese by default. Switch to English only on explicit user request.
- Every draft goes through the Phase 6 de-AI pass — AI-stereotypical phrasing is removed; patent formalism, numbers, step references, and mechanism claims are preserved verbatim.

