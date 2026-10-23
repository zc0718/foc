// Conan::ImportStart
#include <foc/motor/motor.hpp>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/stub_hal.hpp>
// Conan::ImportEnd

namespace foc {

namespace motor {

// Stable symbols for the V1 default composition (ADR-005): back-EMF observer + SVPWM + PI + stub HAL.
template class Motor<core::BackEmfObserver, core::Svpwm, core::Pi, driver::StubHal>;

} // namespace motor

} // namespace foc
