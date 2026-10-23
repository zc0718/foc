# Expert Review Guidelines for Patent Drafting

These guidelines prevent common rejection patterns and elevate the technical depth of patent disclosures.

---

## 1. Language Technicalization (语言技术化)

### The Rule: system-behavior language

**Never describe a technical effect as "convenience" or "ease of use".** Always translate to system-behavior language.

### Forbidden Terms & Replacements

| ❌ Forbidden (Subjective/UX) | ✅ Replacement (System-Behavior) |
| --- | --- |
| 方便用户配置 | 消除配置文件的二次反序列化CPU周期消耗 |
| 提升开发效率 | 减少跨内存区域拷贝次数，降低内存总线带宽占用 |
| 提前发现错误 | 在GPU算力分配之前阻断无效试验启动路径 |
| 易于扩展 | 热插拔替换无需进程重启，保护GPU显存状态连续性 |
| 简化流程 | 将N次I/O往返收敛为单次内存原地覆写 |
| 提高准确性 | 消除字符串到数值的类型转换指令开销 |
| 自动完成 | 将人工分支判断收敛于引擎内部单一路径，降低分支预测失败率 |
| 避免重复工作 | 避免函数重编译与引擎冷启动开销 |
| 降低学习成本 | 消除对特定HPO引擎API的导入依赖与符号冲突风险 |

### Causal Chain Pattern

Each beneficial effect should form a complete causal chain:

```text
本发明通过[机制] → 在底层[系统层面]消除了[具体开销/瓶颈] 
→ 在[典型规模]下，[定量对比] → 最终使[系统指标]改善
```

---

## 2. Quantitative Anchoring (定量锚定)

### The Rule: every effect needs a number

Every beneficial effect must have at least one **order-of-magnitude quantitative projection**.

### Quantification Dimensions

| Dimension | Example Anchor |
| --- | --- |
| **Scale** | "百次试验规模", "千级采样频率", "每秒数千次请求" |
| **Time** | "毫秒级 vs 分钟级", "类加载时(ms) vs 试验运行时(min-h)", "5-10秒 vs 毫秒级" |
| **Memory** | "1GB序列化", "64字节缓存行", "32-64字节元数据结构体" |
| **Count** | "1000次跨对象拷贝 → 1000次原地覆写", "数百次无效GPU试验" |
| **Resource** | "每周24-48 GPU小时浪费", "内存总线事务数降低约50%" |
| **Instruction** | "15-20个CPU周期的流水线冲刷", "分支预测误报率" |

### Quantification Template

```text
在[典型规模]场景下：
- 现有技术：[X次/单位] [操作类型]，累计[资源消耗]
- 本发明：[Y次/单位] [操作类型]，[资源消耗]
- 差异：约Z%的[指标]降低 / [时间]缩短至[时间]
```

---

## 3. Portfolio Defense (合案布局)

### The Rule: base patent + dependent claims

When drafting ≥3 related patents, designate one **base system-level patent** and link the others as embeddable dependent claims.

### Implementation

1. **Identify the base patent:** The one covering the overarching system architecture, scheduling mechanism, or orchestration layer.
2. **In the base patent's §4 (Key Protection Points):** Add a final point stating how other cases can be embedded as dependent claims.
3. **In each peripheral patent's §2.3 (Beneficial Effects):** Add a note linking back to the base patent.

### Example (from het-ai case)

In the base patent (p10 — scheduling system):
> **合案布局说明**: 本案为系统级基础专利。p1-p9所述技术方案可作为本案的从属权利要求嵌入——p1(统一表征)依赖于本系统调度层的内存管理机制；p2(静态校验)依赖于本系统的类加载拦截点；...

---

## 4. Common Rejection Patterns & Fixes

| Rejection Risk | Cause | Fix |
| --- | --- | --- |
| **缺乏创造性 (Lack of inventiveness)** | Described as "convenience feature" | Rephrase as solving a **technical contradiction** at system level |
| **纯商业方法 (Pure business method)** | No hardware/system mechanism described | Anchor every claim in a **concrete system behavior** (memory layout, instruction path, I/O pattern) |
| **不清楚 (Unclear)** | Vague terms like "等", "相关" | Use precise technical terms and numbered steps |
| **不支持 (Unsupported)** | Claims broader than disclosed | Ensure each claim maps to a specific mechanism described in §2.2 |

---

## 5. Self-Review Checklist

After drafting each document, verify:

- [ ] §1 cites ≥2 existing technical approaches with concrete shortcomings at system level
- [ ] §2.1 states a precise **technical** problem (not a functional goal)
- [ ] §2.2 includes numbered steps and PlantUML (preferred) or Mermaid diagrams with text descriptions
- [ ] §2.2 uses **tables** where structured comparison or quantitative data benefits clarity
- [ ] §2.2 uses **KaTeX formulas** where mathematical notation clarifies the mechanism (with symbol explanations)
- [ ] §2.3 each effect has: causal chain + quantitative anchor + system-behavior language
- [ ] §2.3 contains NO forbidden terms (方便, 高效, 提升效率, etc.)
- [ ] §3 has 4-8 alternative implementations
- [ ] §4 each protection point describes a mechanism, not a feature
- [ ] Cross-scenario validation (§2.2) covers ≥2 different application domains
- [ ] Portfolio linking done if ≥3 patents in batch

---

## 6. The "Dimensional Elevation" Principle

To avoid overlap with public knowledge, describe each innovation at **one level lower** than its surface functionality:

| Surface Level (容易被现有技术覆盖) | Elevated Level (区分于现有技术) |
| --- | --- |
| 统一接口 | 单向依赖拓扑消除循环引用导致的内存泄漏 |
| 配置管理 | 环境变量优先级链消除进程重启与模型权重丢失 |
| 实验上报 | 血缘双路径解决非表格数据无法生成标准哈希的溯源断裂 |
| 自动触发 | 触发即溯源消除跨网络查询与独立溯源服务的事务锁开销 |
| 开放集支持 | 开放集转义路径消除强制类型推断导致的进程崩溃与内存泄漏 |

