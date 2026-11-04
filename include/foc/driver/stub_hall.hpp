// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/hal/hall.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::Hall` implementation with a settable code.
 * @brief [zh] `foc::hal::Hall` 的参考实现，编码可设。
 * @note [en] It exists so the decoder and the commutation policy can be exercised without a board, and so the contract
 *            has an implementation that proves it is implementable. It deliberately does no validation: a stub that
 *            refused `0` and `7` would hide the case the decoder is supposed to catch.
 * @note [zh] 它存在的意义是让解码器与换相策略在没有板子的情况下也能被跑到，并让契约有一个能证明「可实现」的实现。它故意
 *            不做任何校验：一个会拒绝 `0` 与 `7` 的 stub，会把本该由解码器抓到的情形藏起来。
 * @since 0.1.9
 * @exporter
 */
class StubHall final : public hal::Hall {
  public:
    StubHall() noexcept = default;

    /** @brief [en] Set the reported code. @brief [zh] 设置上报的编码。 @since 0.1.9 */
    void set_states(std::uint8_t code) noexcept { code_ = static_cast<std::uint8_t>(code & 0x07U); }

    /** @brief [en] The reported code. @brief [zh] 上报的编码。 @since 0.1.9 */
    auto states() noexcept -> std::uint8_t override { return code_; }

  private:
    std::uint8_t code_{0U};
};

} // namespace driver

} // namespace foc
