// Conan::ImportStart
#pragma once
// Conan::ImportEnd

namespace foc {

namespace hal {


/**
 * @brief [en] The power stage's fault output (`nFAULT`): one bit saying "the drive has tripped, stop".
 * @brief [zh] 功率级的故障输出（`nFAULT`）：一位，说"驱动器已经保护动作，停"。
 *
 * @details [en] This contract exists because of one default value, and that default is dangerous enough to deserve a
 *               type of its own. Everywhere else in this HAL an absent capability means "this host cannot do it" and
 *               the library degrades honestly: no clock means no timebase, no index mark means the axis cannot home. A
 *               missing *fault* input is not like that. It degrades in the direction of "the power stage is fine",
 *               which is the one answer nobody is entitled to give on the drive's behalf -- a gate driver reporting a
 *               desaturation or an under-voltage supply is the last warning before hardware is destroyed, and a
 *               controller that never asked is indistinguishable from one that was told "all clear".
 *
 *     So the absence of this member is **refused at assembly** rather than defaulted: `Motor::start()` answers
 *     `NotImpl` and the axis stays in Idle with the output stage disabled. A host that genuinely has no such pin (a
 *     simulator, an FMU, a bench with a stub power stage) says so explicitly by installing `driver::StubFaultInput` and
 *     never raising it -- the sentence "there is no fault path here" gets *written*, once, in the integrator's own
 *     code, where a reviewer can see it. That is the whole difference between a default and a decision.
 *
 *     What the pin cannot tell you: **why**. `nFAULT` is one bit, and the cause lives in the driver's own register map.
 *     The library therefore reports `FaultCode::DriveFault` -- "the power stage says it is in trouble" -- and does not
 *     pretend to guess between desaturation, driver under-voltage and driver over-temperature. Reading the cause out is
 *     the board's business, and it belongs in that board's diagnostics, not in a guess here.
 *
 * @details [zh] 这个契约的存在只为了一个默认值，而那个默认值危险到值得单独给它一个类型。在本 HAL 的其他地方，能力缺席意
 *               味着 "这个主控方做不到"，库会诚实地降级：没有时钟就没有时基，没有索引标记就是回零不了。**故障输入**不是
 *               这样。它降级的方向是"功率级没事"，而这恰恰是没有人有资格替驱动器回答的那一个答案——栅极驱动器报去饱和或
 *               驱动电源欠压，是硬件被毁之前的最后一次警告，而一个从没问过的控制器与一个被告知"一切正常"的控制器，从外
 *               面看是一样的。
 *
 *     所以这一成员的缺席是**在装配时就拒绝**，而不是取默认值：`Motor::start()` 答 `NotImpl`，轴留在 Idle、输出级保持关
 *     闭。一个确实没有这根引脚的主控方（仿真器、FMU、功率级是桩的试验台）通过装一个 `driver::StubFaultInput` 并且从不拉
 *     高来明说这件事——"这里没有故障通路"这句话要被**写下来**，只写一次，写在集成方自己的代码里，让评审看得见。这就是默
 *     认值与决定的全部差别。
 *
 *     这根引脚**说不出**的：**为什么**。`nFAULT` 是一位，成因在驱动器自己的寄存器里。所以库报 `FaultCode::DriveFault`
 *     ——"功率级说它出事了"——不去假装在去饱和、驱动欠压、驱动过温之间猜一个。把成因读出来是板子的事，属于那块板的诊断，
 *     不属于这里的一个猜测。
 *
 * @context [en] Read once per control period, before anything else: a fault outranks every other status.
 * @context [zh] 每个控制周期读一次，且在所有事情之前：故障压过其余任何状态。
 * @since 0.1.9
 * @exporter
 */
class FaultInput {
  public:
    virtual ~FaultInput() = default;

    /**
     * @brief [en] Whether the drive is reporting a fault, with the pin's polarity already applied.
     * @brief [zh] 驱动器是否在报故障，极性已经由实现处理。
     * @return [en] true = the power stage says stop. Never throws, never blocks.
     * @return [zh] true = 功率级说要停。不抛异常、不阻塞。
     * @note [en] Polarity belongs to the adapter and not to this contract: `nFAULT` is active-low on most gate drivers
     *            and active-high on some, and a library that assumed either would invert a safety signal on the other
     *            half of the boards. The adapter that knows its own hardware answers "is it faulted", not "what is the
     *            pin level".
     * @note [zh] 极性属于适配层而不属于这个契约：`nFAULT` 在多数栅极驱动器上是低有效、在少数上高有效，而任何假定其中一
     *            种的库都会在另一半板子上把安全信号取反。知道自己硬件的那个适配层回答的是"它故障了吗"，不是"引脚电平是
     *            多少 "。
     * @context [en] Control ISR.
     * @context [zh] 控制 ISR。
     * @since 0.1.9
     */
    virtual auto faulted() noexcept -> bool = 0;
};

} // namespace hal

} // namespace foc
