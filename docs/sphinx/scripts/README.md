# `docs/sphinx/scripts/` — 文档线自己的可执行脚本

这个目录只放**被 Sphinx 页面直接引用、并且必须能被执行**的东西。它不属于 doxygen 示例集
（`docs/doxygen/dox/demos/` 里的文件每个都会进 doxygen 的 Examples 索引），也不属于测试包
（`test_package/` 由 Conan 打包），所以放在文档线自己名下。

| 文件 | 作用 |
|---|---|
| `walkthrough.cxx` | 「一台电机、一个班次」贯通脚本：七个阶段，每个阶段打印该阶段的数。被 `source/guides/walkthrough.rst` 用 `literalinclude` 逐段引用 |
| `walkthrough.expected` | 建站时那一次运行的**完整输出**（48 行）。页面里"一次运行打印出的东西"这一节直接内联它，因此页面上的每个数只有一个来源 |
| `check.sh` | 编译 → 运行 → 与 `walkthrough.expected` 逐字节 `diff`。退出码非零即页面已过期 |

## 为什么这样组织

页面**不转述**数字。`walkthrough.rst` 里七个步骤各内联一段 `walkthrough.cxx` 的函数
（`literalinclude` 的 `:start-at:` / `:end-before:` 按函数名和下一阶段的分隔注释取），
然后整页唯一的数字块内联 `walkthrough.expected`。于是：

- 想知道某个数从哪来 → 看它旁边那段代码；
- 想知道代码当前是否还打印这个数 → 跑 `check.sh`。

两者都不需要人去"同步文档"。

## 跑一次

```bash
bash docs/sphinx/scripts/check.sh          # 编译 + 运行 + 比对（应当无输出差异）
```

`walkthrough.cxx` 用 `test_package/test/unit/sil_bench.hpp`（测试套件自己的 SIL 台架）作为
被控对象，因此第 4、6 阶段跑在与测试断言相同的模型上，而不是另写一个"演示用"模型。

重新录制期望输出（**只有在确认行为的变化是你想要的之后**）：

```bash
g++ -std=c++17 -O2 -Iinclude -Itest_package/test/unit docs/sphinx/scripts/walkthrough.cxx \
    $(find src -name '*.cpp' -o -name '*.c' | grep -v '/version\.') -o /tmp/walkthrough
/tmp/walkthrough > docs/sphinx/scripts/walkthrough.expected
```

## 约束

- 不要让 `walkthrough.cxx` 依赖构建产物（Conan 包、Python 模块）：站点构建时不保证它们存在。
- 不要把它搬进 `docs/doxygen/dox/demos/`：那会让它出现在 doxygen 的示例索引里，两个站点的
  示例列表就不一致了。
- 输出格式（`stage / quantity / value / unit` 四列）是页面表格的来源，改格式等于改页面。
