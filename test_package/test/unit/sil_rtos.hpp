// Conan::ImportStart
#pragma once

#include <cstddef>
#include <cstdint>

#include <foc/hal/clock.hpp>

#include "sil_plant_hal.hpp"
// Conan::ImportEnd

namespace foc {

namespace sil {


/**
 * @brief [en] A monotonic clock the schedule drives, so that the library's own overrun metric sees the scheduler's
 *             timing rather than the caller's intention.
 * @brief [zh] 由调度模型驱动的单调时钟，使库自己的过载指标看到的是**调度器**的时序，而不是调用方的意图。
 * @note [en] This is the whole trick of the item: the library already measures `max_period_us` from the platform clock.
 *            A test that hands it a virtual clock it can move is measuring the same code path a real RTOS would drive,
 *            instead of a reimplementation of it.
 * @note [zh] 这就是这一项的全部诀窍：库本来就从平台时钟量 `max_period_us`。把一块可拨动的虚拟时钟交给它，测的就是真实
 *            RTOS 会驱动的那条同样的代码路径，而不是它的一个再实现。
 * @since 0.1.9
 */
class VirtualClock final {
  public:
    auto now_us() noexcept -> std::uint64_t { return now_us_; }

    void set_us(std::uint64_t us) noexcept { now_us_ = us; }

  private:
    std::uint64_t now_us_{0U};
};


/**
 * @brief [en] The plant HAL plus a clock.
 * @brief [zh] plant HAL 加一块时钟。
 * @note [en] `PlantHal` is `final` and deletes its copy and move operations, so this holds **references** to its four
 *            members rather than containing it. That is deliberate: a copy would be a second inverter with its own held
 *            duty, and the test would then be measuring the copy.
 * @note [zh] `PlantHal` 是 `final` 且删除了拷贝与移动，所以这里**引用**它的四个成员而不是包含它。这是有意的：一份拷贝就
 *            是第二台带自己保持占空比的逆变器，而测试随后量到的会是那份拷贝。
 * @since 0.1.9
 */
class ClockedPlantHal final {
  public:
    explicit ClockedPlantHal(PlantHal& base) noexcept
        : pwm(base.pwm), current_sense(base.current_sense), position(base.position), dc_link(base.dc_link) {}

    PlantPwm& pwm;
    PlantCurrentSense& current_sense;
    PlantPosition& position;
    PlantDcLink& dc_link;
    VirtualClock clock;
};


/**
 * @brief [en] One periodic task in the model: how often it is released, how long it holds the CPU, and how important it
 *             is.
 * @brief [zh] 模型里的一个周期任务：多久释放一次、占用 CPU 多久、以及有多重要。
 * @since 0.1.9
 */
struct RtosTask final {
    std::uint32_t period_us; ///< [en] release period, us. [zh] 释放周期，us。
    std::uint32_t busy_us;   ///< [en] CPU time per release, us. [zh] 每次释放占用的 CPU 时间，us。
    std::uint8_t priority;   ///< [en] larger runs first; the control task is 1. [zh] 大的先跑；控制任务是 1。
};


/**
 * @brief [en] When the control task actually got the CPU, and whether that met its period.
 * @brief [zh] 控制任务**实际**拿到 CPU 的时刻，以及是否守住了周期。
 * @note [en] `worst_interval_us` is the number the library has to agree with, and `overruns` is the number that must
 *            not be zero when the schedule cannot meet the deadline -- a silent overrun is the failure this item exists
 *            to prevent, and silence is what a schedule model makes visible.
 * @note [zh] `worst_interval_us` 是库必须与之一致的那个数，而 `overruns` 是"调度守不住期限时必须不为零"的那个数—— 静默
 *            过载正是这一项要防的失败，而调度模型让"静默"变得可见。
 * @since 0.1.9
 */
struct RtosSchedule final {
    static constexpr std::size_t kCapacity = 256U;

    std::uint32_t start_us[kCapacity]{}; ///< [en] first instant each release held the CPU. [zh] 每次释放头一次拿到 CPU 的时刻。
    std::uint32_t count{0U};             ///< [en] releases the model ran. [zh] 模型跑过的释放次数。
    std::uint32_t overruns{0U};          ///< [en] gaps longer than one period. [zh] 长于一个周期的间隔数。
    std::uint32_t worst_interval_us{0U}; ///< [en] longest gap between consecutive starts. [zh] 相邻两次启动之间最长的间隔。
    std::uint32_t worst_lateness_us{0U}; ///< [en] worst delay past the ideal `k * period`. [zh] 相对理想 `k * period` 的最差延迟。
    bool complete{false};                ///< [en] every release ran before the horizon. [zh] 每次释放都在视界内跑到了。
};


/**
 * @brief [en] Simulate a preemptive fixed-priority schedule and report when the control task actually ran.
 * @brief [zh] 仿真一个抢占式固定优先级调度，并报告控制任务**实际**在什么时候跑。
 * @param blockers [en] higher-priority periodic tasks.
 * @param blockers [zh] 更高优先级的周期任务。
 * @param blocker_count [en] how many.
 * @param blocker_count [zh] 有几个。
 * @param control_period_us [en] the control task's period, us.
 * @param control_period_us [zh] 控制任务的周期，us。
 * @param control_busy_us [en] how long one tick holds the CPU, us.
 * @param control_busy_us [zh] 一次 tick 占用 CPU 多久，us。
 * @param releases [en] how many control periods to run.
 * @param releases [zh] 要跑多少个控制周期。
 * @return [en] the recorded starts and the two judgements.
 * @return [zh] 记录下来的启动时刻，以及两个判断。
 * @note [en] A preemptive fixed-priority model and nothing more: every task runs to completion when it holds the CPU, a
 *            higher priority preempts at its release, and there are no locks. That is enough to answer the only
 *            question this item asks -- *when does the control task get to run* -- and it is deliberately not enough to
 *            answer anything else. Blocking from shared resources would be a different model with different numbers,
 *            and pretending to cover it with this one is how a test ends up asserting something nobody measured.
 * @note [zh] 只有抢占式固定优先级这一层：任务拿到 CPU 就跑完，更高优先级在它释放时抢占，没有锁。这足以回答本项唯一要问
 *            的问题——**控制任务什么时候才跑得上**——并且刻意不足以回答别的。共享资源造成的阻塞是另一个模型、另一组数字，
 *            用这个模型假装覆盖它，正是一条用例最后断言了没人量过的东西的方式。
 * @since 0.1.9
 */
inline auto simulate_control_task(const RtosTask* blockers, std::size_t blocker_count,
                                  std::uint32_t control_period_us, std::uint32_t control_busy_us,
                                  std::uint32_t releases) noexcept -> RtosSchedule {
    RtosSchedule out{};
    constexpr std::size_t kMaxTasks = 8U;
    if ((blockers == nullptr) && (blocker_count != 0U)) {
        return out;
    }
    if ((control_period_us == 0U) || (releases == 0U) || (releases > RtosSchedule::kCapacity) ||
        (blocker_count > kMaxTasks)) {
        return out;
    }

    struct Task final {
        std::uint32_t period;
        std::uint32_t busy;
        std::uint32_t remaining;
        std::uint32_t next_release;
        std::uint8_t priority;
        bool running;
    };
    Task tasks[kMaxTasks + 1U]{};
    const std::size_t total = blocker_count + 1U;
    for (std::size_t i = 0U; i < blocker_count; ++i) {
        tasks[i] = Task{blockers[i].period_us, blockers[i].busy_us, 0U, 0U, blockers[i].priority, false};
        if ((tasks[i].period == 0U) || (tasks[i].busy > tasks[i].period)) {
            return out; // unschedulable input: a task that never finishes is not a schedule
        }
    }
    const std::size_t control = blocker_count;
    tasks[control] = Task{control_period_us, control_busy_us, 0U, 0U, 1U, false};

    std::uint64_t now = 0U;
    std::uint32_t started = 0U;
    // A horizon of 200 control periods is far past the point where anything interesting has happened, and it is what
    // turns "the schedule cannot run this" into a report instead of a hang.
    const std::uint64_t horizon = static_cast<std::uint64_t>(control_period_us) * (releases + 200U);

    while ((started < releases) && (now < horizon)) {
        for (std::size_t i = 0U; i < total; ++i) {
            if (tasks[i].next_release == 0U) {
                // The t = 0 release. The first version only wrote `next_release = period` here and never added the CPU
                // time, so no task was ready at t = 0 and the whole schedule slid forward by one period -- every
                // lateness came out as `period + offset` while still looking perfectly constant, which is exactly the
                // kind of number a criterion can absorb without noticing.
                // t = 0 的那次释放。第一版只写了 `next_release = period` 而没有加 CPU 时间，于是 t = 0 时没有任务就绪，整条调度
                // 向后滑了一个周期——每次迟到都变成 `周期 + 偏移`，而看起来仍然完美地恒定，正是判据会不知不觉吸收掉的那类数。
                tasks[i].remaining += tasks[i].busy;
                tasks[i].next_release = tasks[i].period;
            }
            while (tasks[i].next_release <= now) {
                tasks[i].remaining += tasks[i].busy;
                tasks[i].next_release += tasks[i].period;
            }
        }

        std::size_t current = total;
        for (std::size_t i = 0U; i < total; ++i) {
            if (tasks[i].remaining == 0U) {
                continue;
            }
            if ((current == total) || (tasks[i].priority > tasks[current].priority)) {
                current = i;
            }
        }
        if (current == total) {
            std::uint64_t next = horizon;
            for (std::size_t i = 0U; i < total; ++i) {
                if (static_cast<std::uint64_t>(tasks[i].next_release) < next) {
                    next = tasks[i].next_release;
                }
            }
            now = next;
            continue;
        }

        if ((current == control) && !tasks[control].running) {
            tasks[control].running = true;
            if (started < RtosSchedule::kCapacity) {
                out.start_us[started] = static_cast<std::uint32_t>(now);
            }
            ++started;
            out.count = started;
        }

        std::uint64_t until = horizon;
        for (std::size_t i = 0U; i < total; ++i) {
            if ((tasks[i].priority > tasks[current].priority) &&
                (static_cast<std::uint64_t>(tasks[i].next_release) < until)) {
                until = tasks[i].next_release;
            }
        }
        const std::uint64_t allowed = (until > now) ? (until - now) : 0U;
        const std::uint32_t run = static_cast<std::uint32_t>(
            (allowed < tasks[current].remaining) ? allowed : tasks[current].remaining);
        if (run == 0U) {
            now = (until > now) ? until : (now + 1U);
            continue;
        }
        tasks[current].remaining -= run;
        now += run;
        if (tasks[current].remaining == 0U) {
            tasks[current].running = false;
        }
    }

    out.complete = (started >= releases);
    for (std::uint32_t k = 1U; k < started; ++k) {
        const std::uint32_t interval = out.start_us[k] - out.start_us[k - 1U];
        if (interval > out.worst_interval_us) {
            out.worst_interval_us = interval;
        }
        if (interval > control_period_us) {
            ++out.overruns;
        }
        // Measured from the absolute ideal instant `k * period`, **not** from the first start: a constant offset
        // applied to every release cancels out of a first-start-relative number, which makes "every release is 20 us
        // late" indistinguishable from "nothing is late" -- and the first version of this metric reported exactly
        // that. Lateness is an absolute statement about the timeline; jitter is a relative one, and they are the
        // two things this item exists to keep apart.
        // 从绝对的理想时刻 `k * period` 量，**不是**从第一次启动量：把恒定偏移从"相对第一次启动"的数里减掉，会让"每次
        // 都晚 20 us"与"一点都不晚"无法区分——这个指标的第一版报的正是后者。迟到是关于时间轴的绝对陈述，抖动是相对
        // 的那一个，而这正是本项要分开的两件事。
        const std::uint32_t ideal = k * control_period_us;
        if ((out.start_us[k] > ideal) && ((out.start_us[k] - ideal) > out.worst_lateness_us)) {
            out.worst_lateness_us = out.start_us[k] - ideal;
        }
    }
    if (started > 0U) {
        // With one release there is no interval to speak of, and `worst_interval_us == 0` says exactly that rather
        // than inventing the period.
        // 只有一次释放时没有间隔可言，`worst_interval_us == 0` 说的正是这件事，而不是把周期编出来。
        static_cast<void>(0);
    }
    return out;
}


} // namespace sil

} // namespace foc
