# 快速上手

> 面向两类使用者：**C++ 用户**（`#include <foc/motor/motor.hpp>`，编译期组合策略）与
> **C 用户**（`#include <foc/api/foc_api.h>`，稳定 ABI）。两条路径**完全隔离**，见 §4。
>
> 本文只写"怎么用 + 边界在哪"。**凡本库没有验证过的事情，本文一律不声称已验证**——清单在 §7。

---

## 1. 这个库是什么

一个**硬件无关的 FOC（磁场定向控制）算法核心**。它不驱动任何具体芯片，也不假设采样、
逆变器或编码器的型号，只规定**契约**：

| 层 | 头文件 | 职责 |
|----|--------|------|
| HAL 契约 | `include/foc/hal/*.hpp` | `Pwm` / `CurrentSense` / `PositionSensor` / `DcLink` / `Clock` 五个纯虚接口 |
| 算法核心 | `include/foc/core/**` | Clarke/Park、SVPWM、PI、滤波、限幅、状态机、观测器 |
| 组合体 | `include/foc/motor/motor.hpp` | `Motor<Observer, Modulator, Controller, HalPolicy>`，电流环 + 速度环 |
| 参考实现 | `include/foc/driver/*.hpp` | 桩 HAL（`StubHal` / `StubPwm` / `ScriptedCurrent` / `SimPosition`），既是契约示范，也是 SIL 默认后端 |
| C ABI | `include/foc/api/*.h` | 不透明句柄 + 错误码 + 最小控制集 |

**为什么是"两套 API"**：C++ 模板接口直接在编译期组合策略、控制环内零虚函数、零分配；
但它的 ABI 不稳定（name mangling、模板暴露、类大小随策略变化）。C 接口是为**跨编译器、
跨语言**集成准备的防火墙，不是为了降低能力。

---

## 2. 环境与依赖

| 项 | 要求 |
|----|------|
| 依赖管理 | Conan 2.x（`conan --version`） |
| 构建 | CMake（下限见 `metadata.json` 的 `cmake_version`） |
| 编译器 | 支持 C++17 与 C11（GCC / Clang / MSVC） |
| 依赖库 | `etl`（固定容量容器，header-only）；测试用 `GTest`（仅 host） |

消费方**不需要**安装 Sphinx/Doxygen——那是本仓库自己出文档用的。

---

## 3. 引入

`conanfile.py`：

```python
def requirements(self):
    self.requires("foc/0.1.0")
```

`CMakeLists.txt`：

```cmake
find_package(foc REQUIRED)

add_executable(app main.cpp)          # C 用户写 main.c，链接方式完全一样
target_link_libraries(app PRIVATE foc::foc)
```

**链接规则（只需记一条）**：链接聚合目标 **`foc::foc`**。它同时带上 `foc::foc_c` 与
`foc::foc_cpp`，顺序由包元数据保证（`foc_c` 依赖 `foc_cpp`，见 PRD D37）。

**C 用户仍需链接 C++ 运行时（U8，不可规避）**：C ABI 是稳定的，但实现是 C++。用 CMake 时
把 C 目标设成 `LINKER_LANGUAGE CXX`（或直接加一个 `.cpp` 空文件），否则会看到一串
`undefined reference to __cxa_* / operator new`。**不要**指望 `-nostdlib++` 能开箱可用。

---

## 4. 两条路径与隔离规则

```
C++ 用户 ──► Motor<策略...>            栈/静态对象，用户自己管生命周期
C 用户   ──► foc_api.h ──► bridge.cpp  对象池（4 槽）+ 代际句柄，库自己管生命周期
```

- **严格单向**：C 路径依赖 C++ 核心；C++ 核心**从不** include `foc/api/*.h`（由
  `gates.py guard` 强制）。
- **禁止混用**：同一个 `Motor<>` 实例只能由一条路径管理。**不许**把 `foc_motor_t` 强转成
  `Motor<>*`，也不许用 C 句柄去索引 C++ 侧对象——两者的状态空间毫无关系（PRD §13.5）。
- **句柄会失效**：`foc_motor_destroy()` 之后，旧句柄的所有操作都返回 `FOC_ERR_PARAM`
  （内部代际计数，槽位复用也不会误判）。**不要**在 destroy 之后继续用句柄做任何事。

---

## 5. C++ 路径（推荐给上板用户）

```cpp
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/stub_hal.hpp>      // 换成你自己的 HalPolicy 即可上板
#include <foc/motor/motor.hpp>

using Motor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm,
                                foc::core::Pi, foc::driver::StubHal>;

int main() {
    const foc::motor::MotorConfig config{
        0.5F, 4e-4F, 4e-4F,      // rs, ld, lq  (SI: Ω, H, H)
        0.05F, 1e-4F, 1e-4F,     // kt, j, b   (N·m/A, kg·m², N·m·s)
        7U, 5.0F,                // pole_pairs, i_max [A]
        20000.0F, 500.0F};       // pwm_freq [Hz], deadtime [ns]

    // 增益由铭牌参数 + 目标带宽推导，不要手调（电流环临界阻尼；速度环抵消 B/J 极点）
    const foc::core::Pi::Config current = foc::current_pi_config(config, 2000.0F, 24.0F);
    const foc::core::Pi::Config speed = foc::speed_pi_config(config, 200.0F);

    foc::driver::StubHal hal;                        // 上板时替换成自己的 HalPolicy
    hal.dc_link.set_voltage_v(24.0F);

    Motor motor(config,
                Motor::Policies{foc::core::BackEmfObserver({config.rs, config.ld}), foc::core::Svpwm{},
                                foc::core::Pi{current}, foc::core::Pi{current}, foc::core::Pi{speed}},
                hal);

    if (motor.init() != foc::ErrorCode::Ok || motor.start() != foc::ErrorCode::Ok) {
        return 1;
    }
    motor.set_speed(100.0F);                         // 机械 rad/s；set_torque() 是转矩模式
    for (int i = 0; i < 1000; ++i) {
        (void)motor.tick(5e-5F);                     // 电流环周期
    }
    const foc::motor::Telemetry t = motor.telemetry();
    return (t.speed_mech_rad_s > 0.0F) ? 0 : 1;
}
```

要点：

- **策略是模板参数**，控制环里没有虚函数、没有堆分配；`HalPolicy` 只需要有
  `pwm` / `current_sense` / `position` / `dc_link` 四个成员（成员名即契约）。
- **`HalPolicy` 必须活得比 `Motor` 久**（`Motor` 持有引用）。
- **位置传感器报 `NaN` 速度**时，`Motor` 会自动改用观测器策略估速（角度仍取传感器）。
- 调参入口是 `current_pi_config()` / `speed_pi_config()`；`Pi` 只有**静态积分限幅**、
  没有 back-calculation，所以速度环的积分限幅取 `0.25·i_max` 而不是满量程（PRD D27）。

---

## 6. C 路径（推荐给非 C++ 工程 / 脚本 / 胶水层）

```c
#include <stdio.h>
#include <foc/api/foc_api.h>

int main(void) {
    const foc_motor_params_t params = {0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U};
    foc_motor_t motor = NULL;

    if (foc_motor_create(&params, &motor) != FOC_OK) return 1;
    if (foc_motor_enable(motor) != FOC_OK) return 1;

    foc_set_speed_g(motor, 50.0);        /* _Generic：float / double / int 都能进 */
    for (int i = 0; i < 1000; ++i) {
        if (foc_tick(motor, 5e-5F) != FOC_OK) return 1;
    }

    float speed = 0.0F, id = 0.0F, iq = 0.0F;
    (void)foc_get_state(motor, &speed, &id, &iq);
    (void)printf("speed=%f id=%f iq=%f\n", (double)speed, (double)id, (double)iq);

    (void)foc_motor_disable(motor);
    (void)foc_motor_destroy(motor);
    return 0;
}
```

| 函数 | 说明 |
|------|------|
| `foc_motor_create(&params, &out)` | 池满返回 `FOC_ERR_NO_RESOURCE`（默认 4 槽）；参数非法 / NULL 返回 `FOC_ERR_PARAM` |
| `foc_motor_enable` / `foc_motor_disable` | 启动 / 停止输出级；非法状态迁移返回 `FOC_ERR_STATE`（**不会**把实例弄死） |
| `foc_set_speed` / `foc_set_speed_f64` | 机械 rad/s；`foc_set_speed_g` 宏按实参类型分发 |
| `foc_set_torque` / `foc_set_torque_f64` | q 轴电流 A |
| `foc_tick(dt_s)` | 由调用者推进控制周期（host / SIL 驱动） |
| `foc_get_state(&speed, &id, &iq)` | 三个输出指针可任选，但至少要给一个 |

**C 路径的能力边界（重要）**：C 接口目前**只能在桩 HAL 上运行**（固定 `driver::StubHal`），
且整定与载波频率是库内默认值（20 kHz、500 ns 死区、`i_max = 5 A`、母线 24 V、电流环
2000 rad/s、速度环 200 rad/s）。**上真实硬件请走 C++ 路径**——那里可以注入自己的
`HalPolicy` 并自己决定全部增益（PRD D34）。

---

## 7. 当前不可验证项（诚实边界）

> 本库 V1 **不承诺、不假装验证**下列内容，全部留待硬件阶段。原文见 PRD §11。

| # | 项目 | 为什么现在不可验证 | 硬件阶段的验证手段（预留） |
|---|------|--------------------|---------------------------|
| U1 | 死区补偿实际效果 | 需要真实逆变器非线性 | 板级电流波形 THD 对比 |
| U2 | ADC 采样时刻与 PWM 时序对齐 | 依赖具体 MCU 定时器/触发链 | 示波器 + 板级 HAL 实现 |
| U3 | 观测器对真实电机参数偏差的鲁棒性 | plant model 的摄动是理想化的 | 实测参数辨识 + 台架 |
| U4 | EMI/EMC 行为 | 纯软件不可观测 | EMC 实验室 |
| U5 | 真实闭环噪音与振动 | 需机械负载 | 台架声学/振动测量 |
| U6 | 目标 MCU 上的实时性（ISR WCET） | 无交叉编译 profile（`.hetai/` 不含 STM32/ESP32） | het-board + 板级 benchmark/ |
| U7 | Flash/RAM 占用达标性 | 无目标芯片约束基准 | 交叉编译 size 报告 |
| U8 | **纯 C 用户无 C++ 运行时的链接** | 需剥离 `__cxa_guard` 等隐式依赖，且改变目标平台链接脚本 | 硬件阶段若需完全剥离，使用 `-nostdlib++` 并手写最小桩；**V1 明确要求 C 用户链接 C++ 运行时** |
| U9 | ~~docs 流水线本地不可执行~~ **已解决（2026-10-23）** | 系统 `doxygen` 1.9.8 链接 `libclang-cpp.so.18.1`/`libLLVM.so.18.1`，二者在 `/usr/lib/llvm-18/lib` 且不在默认 loader 路径 → 已用 `~/.venvs/foc-tools/bin/doxygen` 包装脚本补 `LD_LIBRARY_PATH`（免 root；等价 root 修复：`/etc/ld.so.conf.d/llvm-18.conf` + `ldconfig`）；sphinx 侧用 `~/venv/build`（numpy 2.4.1 + sphinx 9.1.0）。`python docs/build.py` 全绿，并借此发现并修掉 D30 | —（说明保留，便于换机复现）|

另外三条**不构成验证**的说明：

- 本库的 **SguanFOC 仅为第三层算法思路参考**，不是验证基准，也不参与任何构建或 CI；
  与其结论不一致时默认不信任它。
- SIL（`test_package/test/unit/test_sil_*`）验证的是**模型内**的闭环行为，不是硬件行为。
- 覆盖率数字只说明测试触及了哪些代码行，**不说明**控制性能。

---

## 8. 自己验证这个库

```bash
# 构建 + 跑全部用例（含 SIL 闭环、L1 解析解、L2 离线仲裁）
conan create . -s build_type=Debug --build=missing

# 质量门禁（format / tidy / guard / strict / 负向编译）
python3 .github/misc/gates.py format
python3 .github/misc/gates.py tidy
python3 .github/misc/gates.py guard
python3 .github/misc/gates.py strict
python3 .github/misc/negative_compile.py

# 双语文档（需要 sphinx；doxygen 需要能加载 LLVM 库，见 PRD U9）
python3 docs/build.py
```

验证分层（细节见 PRD §8.5）：**L1 解析解**（闭式解硬断言）→ **L2 离线仲裁**（scipy 独立
积分器生成的参考轨迹，归一化 RMSE ≤ 2%）→ **L3 仅文档参考**（不构建）。三层都只覆盖
"模型内正确性"。
