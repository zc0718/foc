# De-AI Post-Processing Rules for Patent Disclosures / 专利交底书去 AI 味后处理规则

> This file is the last stage of the het-patent generation chain (Phase 6). The methodology merges two open-source de-AI-writing projects and adapts them for the patent domain:
> - [shuorenhua / 说人话](https://github.com/MrGeDiao/shuorenhua) — fixed processing pipeline, protected-span locking, hit-strength tiers, long-text scope control, fidelity readback
> - [humanizer-zh](https://github.com/op7418/humanizer-zh) — 24 AI writing pattern categories and a high-frequency AI vocabulary list (adapted from Wikipedia's "Signs of AI writing")
>
> **Position statement**: this stage is not meant to defeat any AI detector — it makes the disclosure read like the work of an experienced patent engineer: removing template feel and performance feel while **never compromising patent professionalism or factual integrity**. 本层不用于欺骗任何 AI 检测器，而是让交底书读起来像有经验的专利工程师写的——去掉模板感与表演感，同时绝不损害专利的专业性与事实完整性。

---

## 1. Core Principle: Patent Scene vs. Generic Scene（核心原则）

Generic de-AI tools target chat, blogs, and marketing copy, and allow casual and personal registers ("用我", "允许混乱"). **A patent disclosure is the opposite**:

| Dimension | Generic scene (shuorenhua/humanizer default) | Patent disclosure (this rulebook) |
|---|---|---|
| Register | casual / personal allowed | **formal written Chinese, no colloquialization** |
| Deletion strength | whole filler sentences may be deleted | long text `bounded`; §4 protection points `in-place` |
| Fact protection | numbers, versions, commands, attribution | all of the left **plus step numbers / figure numbers / formulas / patent idioms / quantitative anchors** |
| Style goal | vivid, opinionated | restrained, precise, mechanism-level; strip hype and inflation |
| Biggest risk | false positives on normal text | **deleting patent idioms as if they were AI boilerplate** (e.g., mistaking "其特征在于" for a cliché) |

**One-line principle: strip the AI boilerplate, keep the patent boilerplate — the two kinds of boilerplate must be strictly distinguished.** 去的是"AI 的定式表达"，保的是"专利的定式表达"——两类定式要严格区分。

---

## 2. Processing Pipeline（处理流水线，固定顺序）

Run per disclosure, in fixed order (borrowed from shuorenhua's six steps, adapted for patents):

### Step 1 — Scene Judgment（场景判定）

Classify the whole disclosure as **`public-writing` × `technical-doc` (patent sub-scene)** and apply conservative strength. Per-section adjustments:

| Section | Scene | Strength |
|---|---|---|
| §1 Background | docs (technical doc) | conservative; opening boilerplate may be deleted sentence-wise (see §3 Tier-1 opening) |
| §2 Technical solution | docs + claim precursor | conservative; steps S1-S5, diagrams, formulas protected as whole blocks |
| §3 Alternatives | docs | conservative |
| §4 Protection points | **claim style** | `in-place`: reword only, no sentence deletion, never touch the "其特征在于" pattern |

### Step 2 — Protected-Span Locking（保护片段锁定）

Circle every **untouchable** span before rewriting; skip them throughout:

1. **Numbers with their objects**: `百次试验规模`, `千级采样频率`, `毫秒级 vs 分钟级`, percentages, complexity bounds ($O(n^2)$) — never summarized as "明显提升", never add new numbers
2. **Quantitative anchors**: all order-of-magnitude projections in §2.3, kept verbatim
3. **Mechanism language**: system-behavior terms (memory addressing, cache miss rate, instruction paths, lock contention, zero-copy, I/O round-trips)
4. **Structural references**: step numbers (步骤 S1-S5), figure numbers, formula numbers, table numbers
5. **Patent idioms (whitelist, never touched)**: 所述、其特征在于、包括但不限于、优选地、进一步地、在本实施例中、在另一实施例中、本领域技术人员、有益效果在于
6. **Diagram & formula blocks**: `@startuml/@enduml`, ```mermaid```, `$...$` / `$$...$$` blocks are **frozen verbatim** (including all text inside)
7. **Factual relations**: cause-effect, conditions, negation, scope, completion, direction, intensity — relations must not be rewritten ("展示了潜力" ≠ "已实现")

### Step 3 — Hit-Strength Tiers（命中强度分级）

| Tier | Definition | Action |
|---|---|---|
| **Tier 1 (must-fix)** | opening boilerplate, value inflation, filler intensifiers, sourceless authority | delete, or reduce to a concrete action/fact |
| **Tier 2 (scene-dependent)** | translation-style long chains, adjective triads, nominalizations, synonym cycling | restructure sentences, revert to verbs, keep one consistent name per object |
| **Tier 3 (keep/light)** | patent idioms, cross-embodiment connectives | keep by default; merge only if repeated 3+ times in one paragraph |

### Step 4 — Sentence/Paragraph Patterns First, Phrase Tables as Fallback（句式段落优先，短语表兜底）

Handle sentence/paragraph patterns first (opening boilerplate, value-inflation passages, long translation-style chains), then scan phrase tables. Phrase hits are judged per tier — **never before the sentence-level pass**.

### Step 5 — Fidelity Readback（保真回读，硬约束）

Read back in both directions; any failure means rework:
- **Forward**: every protected span in the input (numbers / steps / terms / relations) is recoverable in the output, unchanged
- **Backward**: every new conclusion/relation/number in the output traces to input evidence; **no invented numbers, no changed attribution, no new sourceless conclusions**
- Missing information is flagged as a gap ("此处未提供数据"), never filled in

### Step 6 — Residual Audit（残留自检）

After rewriting, do a final scan against the §4 vocabulary tables and the §5 checklist. Tier-3 leftovers may remain if the reason is recorded.

---

## 3. AI-Boilerplate Blacklist for the Patent Scene（专利场景 AI 定式黑名单）

### Tier 1 — Must-Delete / Must-Fix（必删/必改）

| Category | Typical phrasing | Handling |
|---|---|---|
| **Opening boilerplate** | "随着科技的不断发展", "近年来", "当今社会", "在……的时代背景下", "众所周知" | delete the lead-in; start directly with prior-art facts and defects |
| **Value inflation** | "具有重要意义", "深远影响", "极大地推动了", "为……提供了强有力的支撑", "展现了……的价值" | delete; a patent states technical problems and effects, not industry eulogies |
| **Filler intensifiers** | 显著、极大、高效、方便、快捷、灵活、强大、完美、无缝、鲁棒（without quantitative support） | delete or replace with a verifiable mechanism description ("降低缓存未命中率" beats "显著提升性能") |
| **Sourceless authority** | "业内人士普遍认为", "公认的", "权威研究表明"（no citation） | delete; §1 prior art must point to concrete technology |

### Tier 2 — Sentence Restructuring（按上下文处理）

| Category | Typical phrasing | Handling |
|---|---|---|
| **Translation-style long chains** | "基于……的方式，通过……实现了对……的优化，从而……进而……" (chains >3 links) | split into short sentences; one subject-verb-object per sentence; rewrite as steps |
| **Adjective triads** | "快速、高效、稳定" triple-stacked adjectives | keep only the one that is verifiable, delete the rest |
| **Nominalizations** | "进行了优化", "实现了对……的提升" | revert to verbs: "优化了……" |
| **Synonym cycling** | the same object called by different near-synonyms across paragraphs (e.g., mixing "该机制/上述方案/本系统") | unify the name; patent idioms excepted |

### Tier 3 — Keep（专利定式，禁改）

所述、其特征在于、包括但不限于、优选地、进一步地、实施例、本领域技术人员、有益效果在于、步骤 S1-S5、figure/formula references.

### High-Frequency AI Vocabulary（高频 AI 词表）

- From humanizer-zh: 此外、至关重要、深入探讨、强调、持久的、增强、培养、获得、突出、相互作用、复杂/复杂性、格局、关键性的、展示、证明、宝贵的、充满活力
- Patent-scene additions: 赋能、抓手、闭环、沉淀、维度、颗粒度、范式、一站式、无缝衔接、深度耦合（→ rewrite as the concrete mechanism）、痛点、打造、助力
- Handling: judge each hit per tier; keep words with literal meaning in a mechanism context (e.g., "获得锁" keeps 获得)

---

## 4. Order vs. Existing Quality Gates（与既有质量门禁的顺序）

De-AI is the **last step** — the order is fixed (semantics first, language last):

```text
Phase 4 initial draft
  → Phase 5 Quality Enhancement (technicalization pass: UX terms → system-behavior language; quantitative anchoring; portfolio linking)
  → Phase 6 De-AI Post-Processing (this rulebook: protected-span locking → tier-graded rewriting → readback → residual audit)
```

First make sure the mechanism is right, then make sure it does not sound AI-generated. 先保证"说的是对的机制"，再保证"话不说成 AI 腔"。

---

## 5. Before / After Example（正反例）

**Before (AI-flavored, typical in patents)**

> 随着信息技术的不断发展，数据处理效率问题日益突出。本方案显著提升了系统的整体性能，为行业发展提供了强有力的支撑，具有重要的应用价值。基于缓存机制的方式，通过减少了跨内存区域的拷贝操作，从而实现了对数据访问效率的极大优化，进而达到了降低缓存未命中率的目的。

**After (patent register)**

> 现有方案中，数据访问需跨内存区域拷贝，每次拷贝引入一次缓存未命中，访问延迟随数据规模线性增长。本方案将数据固定于单一内存区域，访问路径不再触发拷贝，缓存未命中次数由每次访问 1 次降为 0 次；在百次试验规模下，访问延迟由毫秒级降至微秒级。

**What changed**
- Opening boilerplate and value inflation removed（"随着……发展"、"显著提升"、"为行业……支撑"、"重要的应用价值"）
- Translation-style long chain（基于……通过……从而……进而……达到……目的）split into two factual sentences
- Numbers and mechanisms preserved and readback-verifiable: 跨内存区域拷贝、缓存未命中、百次/毫秒级/微秒级
- No patent idiom deleted; "降低缓存未命中率" kept as a mechanism description

---

## 6. Self-Checklist（自检清单）

Run per disclosure after Phase 6:

- [ ] Opening boilerplate and value-inflation passages deleted (§1 opens with prior-art facts)
- [ ] Filler intensifiers deleted or replaced with verifiable mechanism descriptions
- [ ] Every quantitative anchor readable back forward; no new numbers
- [ ] Step numbers S1-S5, figure/formula references intact without drift
- [ ] PlantUML/Mermaid/KaTeX blocks verbatim
- [ ] Patent idioms (其特征在于 etc.) not deleted
- [ ] Adjective triads and nominalizations handled per Tier 2
- [ ] Patent idioms repeated ≤2 times per paragraph
- [ ] Backward readback passes: no sourceless conclusions, attribution unchanged
- [ ] Residual audit passed, or leftovers recorded with reasons
