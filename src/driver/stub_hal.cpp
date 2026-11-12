// Conan::ImportStart
#include <foc/driver/stub_hal.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_hal.cpp
 *
 * @brief [en] Implements the HalPolicy assembled from the reference stubs, with each stub resettable on its own.
 * @brief [zh] 由参考桩件组装成的 HalPolicy 实现，每个桩件都能单独复位。
 * @since 0.1.0
 */

namespace foc {

namespace driver {


/**
 * @brief [en] Implements StubHal::reset(); the contract is documented on its declaration.
 * @brief [zh] 实现 StubHal::reset()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void StubHal::reset() noexcept {
    pwm.reset();
    current_sense.reset();
    position.set_angle_mech_rad(0.0F);
    position.set_offset_mech_rad(0.0F);
    position.set_velocity_rad_s(0.0F);
    position.set_sensor_present(true);
    dc_link.set_voltage_v(0.0F);
    // Back to "the power stage is not reporting a fault", which is the state a stub power stage is in: a reset that left
    // an injected fault asserted would make `reset()` look like it worked while the drive still refused to run.
    // 回到"功率级没在报故障"，也就是桩功率级所处的状态：一次把注入的故障留在有效状态的 reset，会让 `reset()` 看起来
    // 生效了，而驱动仍然拒绝运行。
    fault_input.reset();
    // And back to "nobody measured a temperature". A reset that left the last case's 80 degrees C in place would leak
    // one case's derating into the next one's, which is the shape of failure that takes an afternoon to find.
    // 同时回到"没有人测过温度"。一次把上个用例的 80 摄氏度留在原处的 reset，会把一个用例的降额漏进下一个用例，而那是
    // 要花一下午才能找到的失效形状。
    temperature.set_invalid();
}

} // namespace driver

} // namespace foc
