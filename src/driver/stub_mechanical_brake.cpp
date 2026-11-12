// Conan::ImportStart
#include <foc/driver/stub_mechanical_brake.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_mechanical_brake.cpp
 *
 * @brief [en] The reference holding brake: state, refusals, and how often it was asked.
 * @brief [zh] 参考保持抱闸：状态、拒绝，以及被问过多少次。
 * @since 0.1.0
 */

namespace foc {

namespace driver {

auto StubMechanicalBrake::engage() noexcept -> ErrorCode {
    ++engages_;
    if (engage_fault_) {
        // A refused engage leaves the state alone: the brake did not grip, and a stub that reported "engaged anyway"
        // would hide exactly the case a sequencer has to survive.
        // 被拒绝的 engage 不改变状态：抱闸没有夹紧，而一个"照样报已抱住"的桩件，恰好会藏掉编排方必须活下来的那个情形。
        return ErrorCode::State;
    }
    engaged_ = true;
    return ErrorCode::Ok;
}

auto StubMechanicalBrake::release() noexcept -> ErrorCode {
    ++releases_;
    if (release_fault_) {
        return ErrorCode::State;
    }
    engaged_ = false;
    return ErrorCode::Ok;
}

auto StubMechanicalBrake::engaged() noexcept -> bool { return engaged_; }

void StubMechanicalBrake::set_engage_fault(bool fault) noexcept { engage_fault_ = fault; }

void StubMechanicalBrake::set_release_fault(bool fault) noexcept { release_fault_ = fault; }

auto StubMechanicalBrake::call_counts() const noexcept -> std::pair<std::uint32_t, std::uint32_t> {
    return {engages_, releases_};
}

void StubMechanicalBrake::reset() noexcept {
    engaged_ = true;
    engage_fault_ = false;
    release_fault_ = false;
    engages_ = 0U;
    releases_ = 0U;
}

} // namespace driver

} // namespace foc
