// Conan::ImportStart
#include <foc/core/sm.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

constexpr std::uint8_t kStateCount = 4U;
constexpr std::uint8_t kIdle = 0U;
constexpr std::uint8_t kAlign = 1U;
constexpr std::uint8_t kRun = 2U;
constexpr std::uint8_t kFault = 3U;

/**
 * @brief kLegal[from][to]; Fault -> Idle is deliberately absent (clear_fault() owns that edge).
 */
constexpr bool kLegal[kStateCount][kStateCount] = {
    /* Idle  -> */ {false, true, false, true},
    /* Align -> */ {true, false, true, true},
    /* Run   -> */ {true, false, false, true},
    /* Fault -> */ {false, false, false, false},
};

constexpr auto to_index(MotorState state) noexcept -> std::uint8_t { return static_cast<std::uint8_t>(state); }

// The table's rows and columns are positional, so the only place the assumed order of `MotorState` is written down is
// these four constants -- and until mutation testing looked, they had no consumer at all: changing `kIdle` from 0 to 1
// changed nothing, because nothing read it. Pinning them to the enum here turns a comment into a check. If the enum is
// ever reordered, this stops compiling instead of silently permuting the transition table under the same comments.
// 表格的行与列是**按位置**的，于是 `MotorState` 假定顺序的唯一记载就是这四个常量——而在变异测试看过来之前，它们没有
// 任何消费者：把 `kIdle` 从 0 改成 1 什么都不变，因为没人读。把它们在这里钉到枚举上，把一句注释变成一个检查。若枚举
// 顺序被改动，这里会编译失败，而不是在同一批注释底下悄悄把转移表换掉。
static_assert(to_index(MotorState::Idle) == kIdle, "kLegal's row 0 is Idle");
static_assert(to_index(MotorState::Align) == kAlign, "kLegal's row 1 is Align");
static_assert(to_index(MotorState::Run) == kRun, "kLegal's row 2 is Run");
static_assert(to_index(MotorState::Fault) == kFault, "kLegal's row 3 is Fault");
static_assert(kStateCount == 4U, "kLegal is written as four rows");

} // namespace

auto StateMachine::state() const noexcept -> MotorState { return state_; }

auto StateMachine::illegal_transitions() const noexcept -> std::uint32_t { return illegal_; }


/**
 * @brief [en] Implements StateMachine::request(); the contract is documented on its declaration.
 * @brief [zh] 实现 StateMachine::request()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto StateMachine::request(MotorState target) noexcept -> ErrorCode {
    const std::uint8_t from = to_index(state_);
    const std::uint8_t to = to_index(target);
    if ((to >= kStateCount) || !kLegal[from][to]) {
        ++illegal_;
        return ErrorCode::State;
    }

    state_ = target;
    return ErrorCode::Ok;
}


/**
 * @brief [en] Implements StateMachine::clear_fault(); the contract is documented on its declaration.
 * @brief [zh] 实现 StateMachine::clear_fault()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto StateMachine::clear_fault() noexcept -> ErrorCode {
    if (state_ != MotorState::Fault) {
        ++illegal_;
        return ErrorCode::State;
    }

    state_ = MotorState::Idle;
    return ErrorCode::Ok;
}

} // namespace core

} // namespace foc
