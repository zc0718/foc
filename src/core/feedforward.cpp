// Conan::ImportStart
#include <foc/core/feedforward.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Implements the constructor; the contract is documented on its declaration.
 * @brief [zh] 实现构造函数；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
DecouplingFeedforward::DecouplingFeedforward(const Config& config) noexcept : config_(config) {}


/**
 * @brief [en] Implements generate(); the contract is documented on its declaration.
 * @brief [zh] 实现 generate()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto DecouplingFeedforward::generate(const FeedforwardInput& in) noexcept -> Dq<float> {
    const float omega = in.omega_e_rad_s;
    float v_d = 0.0F;
    float v_q = 0.0F;

    const bool cross = (config_.terms == Terms::Cross) || (config_.terms == Terms::CrossAndBemf);
    if (cross) {
        v_d -= omega * config_.lq * in.i_meas.q;
        v_q += omega * config_.ld * in.i_meas.d;
    }

    const bool bemf = (config_.terms == Terms::Bemf) || (config_.terms == Terms::CrossAndBemf);
    if (bemf) {
        v_q += omega * config_.psi_f_wb;
    }

    return Dq<float>{v_d, v_q};
}


/**
 * @brief [en] Implements config(); the contract is documented on its declaration.
 * @brief [zh] 实现 config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto DecouplingFeedforward::config() const noexcept -> const Config& { return config_; }

} // namespace core

} // namespace foc
