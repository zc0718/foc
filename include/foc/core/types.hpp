// Conan::ImportStart
#pragma once
#include <cstdint>
// Conan::ImportEnd

namespace foc {


/**
 * @brief [en] Error codes shared by the C++ and the C path. The numeric values are the contract
 *        mirrored by foc_err_t in <foc/api/foc_types.h>; never renumber them.
 *        [zh] C++ 与 C 路径共用的错误码。数值是与 <foc/api/foc_types.h> 中 foc_err_t 对齐的契约，
 *        不得重新编号。
 *
 * @since 0.1.0
 * @exporter
 */
enum class ErrorCode : std::int8_t {
    Ok = 0,
    Param = -1,
    NoResource = -2,
    State = -3,
    NotImpl = -4,
};


/**
 * @brief [en] Core math types shared by both paths: fixed layout, trivially copyable, no allocation.
 *        Units follow the project-wide SI rule (A, V, rad, rad/s, s, N*m).
 *        [zh] 两条路径共享的核心数学类型：布局固定、可平凡复制、无内存分配。
 *        单位遵循全项目 SI 约定（A、V、rad、rad/s、s、N·m）。
 *
 * @since 0.1.0
 */
namespace core {


/**
 * @brief [en] Stationary two-phase frame vector (Clarke output). Angle in rad, no unit tag needed.
 *        [zh] 静止两相坐标系向量（Clarke 输出）。角度单位为 rad。
 *
 * @since 0.1.0
 * @exporter
 */
template <typename T> struct Ab final {
    T alpha{};
    T beta{};
};


/**
 * @brief [en] Rotating two-phase frame vector (Park output): d = flux axis, q = torque axis.
 *        [zh] 旋转两相坐标系向量（Park 输出）：d = 磁链轴，q = 转矩轴。
 *
 * @since 0.1.0
 * @exporter
 */
template <typename T> struct Dq final {
    T d{};
    T q{};
};


/**
 * @brief [en] Three-phase quantity in SI units (A for currents, V for voltages, ratio for duty).
 *        [zh] 三相量，SI 单位（电流 A、电压 V、占空比为比值）。
 *
 * @since 0.1.0
 * @exporter
 */
template <typename T> struct Uvw final {
    T u{};
    T v{};
    T w{};
};

} // namespace core

} // namespace foc
