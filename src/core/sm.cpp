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

/// kLegal[from][to]; Fault -> Idle is deliberately absent (clear_fault() owns that edge).
constexpr bool kLegal[kStateCount][kStateCount] = {
    /* Idle  -> */ {false, true, false, true},
    /* Align -> */ {true, false, true, true},
    /* Run   -> */ {true, false, false, true},
    /* Fault -> */ {false, false, false, false},
};

constexpr auto to_index(MotorState state) noexcept -> std::uint8_t { return static_cast<std::uint8_t>(state); }

} // namespace

auto StateMachine::state() const noexcept -> MotorState { return state_; }

auto StateMachine::illegal_transitions() const noexcept -> std::uint32_t { return illegal_; }


/**
 * @brief [en] Implements StateMachine::request(); the contract is documented on its declaration.
 *        [zh] 实现 StateMachine::request()；契约文档在对应的声明处。
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
 *        [zh] 实现 StateMachine::clear_fault()；契约文档在对应的声明处。
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
