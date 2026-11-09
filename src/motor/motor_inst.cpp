// Conan::ImportStart
#include <foc/motor/motor.hpp>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/stub_hal.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_motor
 * @file motor_inst.cpp
 *
 * @brief [en] Instantiates the default motor composition, so the C path links against one set of symbols.
 * @brief [zh] 显式实例化默认电机组合，使 C 路径链接到同一套符号。
 * @since 0.1.0
 */

namespace foc {

namespace motor {

/**
 * @brief [en] Stable symbols for the V1 default composition (ADR-005): back-EMF observer + SVPWM + PI + stub HAL.
 * @brief [zh] V1 默认组合的稳定符号（ADR-005）：反电动势观测器 + SVPWM + PI + 桩 HAL。
 */
template class Motor<core::BackEmfObserver, core::Svpwm, core::Pi, driver::StubHal>;

} // namespace motor

} // namespace foc
