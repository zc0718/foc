// Conan::ImportStart
#include <foc/core/observer/ideal_observer.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Implements IdealObserver::set_truth(); the contract is documented on its declaration.
 *        [zh] 实现 IdealObserver::set_truth()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void IdealObserver::set_truth(float theta_e_rad, float omega_e_rad_s) noexcept {
    truth_.theta_e = theta_e_rad;
    truth_.omega_e = omega_e_rad_s;
}


/**
 * @brief [en] Implements IdealObserver::update(); the contract is documented on its declaration.
 *        [zh] 实现 IdealObserver::update()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto IdealObserver::update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
    static_cast<void>(v_ab);
    static_cast<void>(i_ab);
    static_cast<void>(dt_s);
    return truth_;
}

void IdealObserver::reset() noexcept { truth_ = ObsState{0.0F, 0.0F}; }

} // namespace core

} // namespace foc
