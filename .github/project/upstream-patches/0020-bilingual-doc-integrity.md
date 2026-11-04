> **MERGED -- do not apply.** Upstream shipped this in `fcpp` v0.3.4: `88f796b3 fix(:bug:): stop the bilingual
> filter from deleting whole doc paragraphs` (plus `2f17bedd`, which states the three shapes). Verified after the
> merge by running both versions' `_language_filter` over all 61 public headers in both languages: **122/122
> byte-identical outputs**, so the adoption changed no documentation content. The file below is kept as the record
> of what was asked for; `scaffold.json` is pinned to v0.3.4 with `tolerated` empty.

# 0020 · 双语文档流水线：重组可以搬字，不能删字

**一句话**：`docs/build.py` 把双语文档块重新分组时，**没有任何机制证明文字没丢** ——
一页少一段话，构建照样绿，直到有人读出来。本补丁加上"丢字即抛异常"的断言、
未闭合注释块检测，以及三种注释形态（紧凑 `///<`、缩进块、续行）的结构保留。

- 补丁：`0020-bilingual-doc-integrity.patch`（**1 文件 / +109−11 / 12 hunks**）
- 基线：`HeT-FTI/fcpp@v0.3.3` = `6d69c85`
- 目标仓库：`HeT-FTI/fcpp`（**上游**）
- 方向：**本仓库领先**。这不是"派生项目的私域改动"，而是本仓库在文档流水线上
  修掉了一批静默失败后往回提。

```bash
git apply --check workspace/scaffold/0020-bilingual-doc-integrity.patch
```

## 1. 四个静默失败（每一个都先有现象，后有修法）

| # | 现象 | 根因 | 修法 |
|---|---|---|---|
| 1 | 某页少了一整段，构建绿 | 重组块时把 `ch['raw']` 里的文字丢了，只留下解析后的行 | `_assert_no_loss()`：按"承载含义的字符"计数（去空白、去行首 `*`、去 `[en]`/`[zh]` 标记），**只允许搬家，不允许减少** |
| 2 | 过滤后 `*/` 落单，后面的 C 代码被当成注释吃掉 | 语言过滤只按"哪些行含语言标记"取舍，不认注释结构行 | `_comment_structure_re` 让 `/**`、`/*!`、`*/` 这类**只有结构没有内容**的行永远保留 |
| 3 | 块注释吞掉了文件尾 | 同一根因的另一半：`*/` 被当作"无语言标记的空行"删除 | 过滤后统计 `_depth`，不为 0 就抛 `left N block comment(s) unclosed` |
| 4 | 缩进块里没有语言标记的续行被整行丢弃 | 续行判断只看前缀 `*`，不看 `* ` 之后是否还有正文 | `if _c_lead.strip(): _push(...)` |

这四条的共同点：**它们都不会让构建报错**，只会让产物安静地变短或变怪。
一个只在"没丢字"时才算通过的断言，是唯一能在 CI 里抓住这类问题的方式。

## 2. 事实（本仓库实测）

- 文档产物：`docs/doxygen/build/{en,zh}/v<ver>/build_sub/html/` 与
  `docs/sphinx/build/html`（`html` 中文、`html_en` 英文）。
- 三语形态都在用：成员单行 `///< [en] gain. [zh] 增益。`、类内缩进块
  `* @param x [en] ...` + `*        [zh] ...`、标记独占一行。
  上游 v0.3.3 的版本能处理前两种的常见情形，第三种与续行会掉字。
- 本仓库 `docs` 门（`project/gates.py docs`）在补丁后仍全绿，说明断言没有
  把正常路径误伤 —— 这一点必须验证，否则"加了检查"和"加了阻碍"分不清。

## 3. 为什么该上游

双语文档流水线是模板自带的：`doxygen/Doxyfile` 里有 `OUTPUT_LANGUAGE` 分支、
`docs/build.py` 里有 `Language` 枚举与 `_language_filter`。**它一旦丢字，
每个派生项目的文档都丢字**，而丢字不产生任何信号 —— 派生项目往往到发版
才从读者那里知道。

代码里的 `_assert_no_loss` 只对一个块做一次字典比较，开销可忽略。

## 4. 替代方案（都更差）

- **写一个"文档行数不比上次少"的检查**：重组本来就会改行数，行数少不等于丢字，
  行数不少也可能丢字（两行并成一行）。按**字符多重集**比较才是对的粒度。
- **只修第 4 条（续行）**：前三条仍在，而且前三条的后果更严重。
- **保持本仓库 fork**：就是现在的状态 —— 一个 120 行的 fork，每次上游改
  `docs/build.py` 都要手工合，且合错的表现正是"文档安静地变短"。

## 5. 保护点

1. 断言按**字符计数**，按语言标记归一，因此不限制重组策略（实现可自由改）。
2. 抛异常用 `RuntimeError` 而不是 `assert`，避免 `-O` 下消失。
3. 未闭合检测只做**计数**，不尝试修复 —— 修复属于下游策略，模板只负责报警。
4. 补丁不改任何输出路径、不新增依赖、不加参数（纯内部一致性）。
