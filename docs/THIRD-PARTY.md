# Third-party code — who, what licence, where it is allowed, and how it is cross-checked

**Purpose / 用途**: one page that answers the four questions a customer's compliance reviewer actually asks.
`conandata.yml` says *what we depend on*; this page says *why*, *where it may appear*, and *what evidence
proves it does what we think*. A dependency nobody can find a use for is a cost the consumer pays for
nothing — that is the state this page exists to make impossible.
`conandata.yml` 说的是**依赖了什么**；这一页说的是**为什么**、**允许出现在哪**、以及**凭什么相信它做对了**。
一个谁都指不出去用途的依赖，是消费者白付的成本——这一页就是让那种状态无法继续存在。

**Enforced / 由门禁强制**: `.github/project/deps.json` carries the same list in machine-readable form
(`allowed_paths`, `forbidden_paths`, measured compile cost), and the `deps` gate (`X2`) fails on a
`#include` outside the allowed paths and on a dependency that `conandata.yml` declares but nothing uses.
`.github/project/deps.json` 以机器可读形式记录同一份清单（`allowed_paths`、`forbidden_paths`、实测编译成本），
`deps` 门（X2）会对越界的 `#include` 和"声明了却没有用法"的依赖报红。

---

## 1. The list / 清单

Every entry below is declared in `conandata.yml`. Nothing else is linked, vendored, or copied.
下面每一条都在 `conandata.yml` 里声明过。除此之外没有任何东西被链接、内置或摘抄。

| Package | Licence | Bucket `metadata.json` | Where it may appear | What it is for |
|---|---|---|---|---|
| `etl/20.47.1` | **MIT** | `cpp` | `src/**`, `test_package/test/**` — **never `include/**`** | Provenance + independent witness (see §2) |
| `gtest/1.16.0` | **BSD-3-Clause** | `infra` | `test_package/**` — never `include/**`, `src/**` | The test framework every case is written against |
| `pybind11/3.0.1` | **BSD-3-Clause** | `infra` | `api/**`, `test_package/test/python/**` | The optional Python consumer face (`enable_python_bindings`) |

`infra` means *this machine*, not *the customer's build*: GTest is dropped from the dependency list unless
tests are being built, and pybind11 unless the bindings are enabled.
`infra` 指的是**本机**，不是**客户的构建**：不构建测试时 GTest 会被摘掉，不启用绑定则 pybind11 同样被摘掉。

---

## 2. ETL — why it is here, and why it is not in the headers

**What it buys / 它带来什么.** Three things that are easy to get subtly wrong and cheap to take from a
fielded library: CRC (a wrong polynomial is *silent* — the frames still decode, they just stop being
protected), bit-stream packing for non-byte-aligned encoder frames, and byte order. The value is not
"less code to write"; it is that **a second, independently written implementation exists to compare
against ours**, and a reviewer can run that comparison.
CRC（多项式写错是**静默**的——帧照样解得开，只是不再被保护）、非字节对齐编码器帧的位流打包、字节序。
价值不在"少写代码"，而在于**存在第二份独立编写的实现可以用来和我们的对拍**，而审阅者跑得动这个对拍。

**Where it may not appear / 它不许出现在哪.** Measured on 2026-11-06 (`workspace/tools/x0_compile_cost.py`,
g++ 13.3, `-std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -O2 -fsyntax-only`, median of 7,
same TU shape as the `strict` gate):

| `etl` header | Public header it would land on | Baseline → with ETL | Headers pulled |
|---|---|---|---|
| `crc16_ccitt.h` | `foc/comm/frame.hpp` | 13.7 ms → 255.8 ms (**18.7×**) | 277 |
| `bit_stream.h` | `foc/comm/encoder_stream.hpp` | 11.8 ms → 402.9 ms (**34.2×**) | 348 |
| `to_string.h` | `foc/core/machine_map_text.hpp` | 98.7 ms → 434.1 ms (**4.4×**) | 355 |
| `endianness.h` | `foc/comm/register_map.hpp` | 15.3 ms → 115.1 ms (**7.5×**) | 149 |

A public header is paid for by **every consumer TU, on every platform, forever**. Against a 10 % budget
these are 4×–34× over, so ETL stays behind the compiled face: `src/comm/encoder_stream.cpp` (a real
translation unit) and the test trees. Consumers get the library, not the dependency's include graph.
公开头是**每个消费者 TU、每个平台、永久**在付费的。以 10 % 为预算，这些超了 4–34 倍，所以 ETL 留在编译面之后：
`src/comm/encoder_stream.cpp`（真 TU）与测试树。消费者拿到的是库，不是这个依赖的包含图。

**How it is cross-checked / 怎么对拍.**

| Standard | Our implementation | Independent witness | Case |
|---|---|---|---|
| CRC-16/CCITT-FALSE | `foc/comm/frame.hpp` `crc16_ccitt()` | `etl::crc16_ccitt_t<256>` | `test_frame.cpp` — every length 0..96 and all 768 single-bit flips of a fixed pseudorandom buffer; 865 inputs |
| Bit slicing (BiSS-C MSB-first) | `src/comm/encoder_stream.cpp` `crc6()` | `etl::bit_stream_reader` | `X4` — every bit offset |

The CRC case also pins the standard's own check value (`0x29B1` for `"123456789"`) **on both sides**, so a
disagreement can be attributed: if the witness fails that vector it is not the standard's variant and the
comparison would be comparing two wrongs.
CRC 那条在**两边**都钉了标准检查值（`"123456789"` → `0x29B1`），于是分歧可以归因：如果证人自己过不了这个向量，
它就根本不是那个变体，比较就变成了比两个错。

**Falsifiers / 伪证.** Change the polynomial in `frame.hpp` (`0x1021` → `0x8005`) and two cases go red: the
check-value case and the cross-check. Verified 2026-11-06 · 把 `frame.hpp` 的多项式改错，两条用例同时红（已实测）。

---

## 3. What is deliberately *not* here

Nine candidate packages were evaluated and rejected, with the reason and a re-open condition recorded in
`workspace/plans/2026-11-05-立项-依赖治理-标准过标-文档定型.md` §3.4 (`X-R1`…`X-R9`). The short version: the
answer to "how is byte order done" is not always "add a package" — for `units`, `safeint`, `crc32c` and
`span-lite` the honest answer was a few lines of our own plus a test, because the package either did not
fit `-fno-exceptions`, did not fit C++17, or duplicated something ETL already provides.
有九个候选被评估后拒绝，理由与重开条件记在计划的 §3.4（`X-R1`…`X-R9`）。一句话版本：对"字节序怎么做的"这个问题，
答案不总是"加一个包"——对 `units`、`safeint`、`crc32c`、`span-lite` 而言，诚实的答案是几十行自己的代码加一条测试，
因为那些包要么不适配 `-fno-exceptions`、要么不适配 C++17、要么和 ETL 已有的能力重复。
