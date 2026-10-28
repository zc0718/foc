// Python bindings for the C API.
//
// The bindings talk to `foc/api/foc_api.h` and never to the C++ templates: the C surface is a
// stable ABI, while binding `Motor<Observer, Modulator, Controller, HalPolicy>` would need one
// binding per instantiation and would re-expose the very C++ ABI that ADR-002 hides behind the
// bridge.
// 绑定只走 C API（`foc/api/foc_api.h`），不碰 C++ 模板：模板绑定要为每个实例化写一份，还会把
// ADR-002 用桥隔离掉的 C++ ABI 重新暴露出去。
//
// The C path reports failures as return codes and keeps the control loop exception-free. The
// Python face keeps that fact but speaks Python: a failed call raises `FocError`, and its `code`
// attribute is exactly the `ErrorCode` the C API returned.
// C 路径用返回码报告失败、控制环里没有异常；Python 侧保留这个事实，但用 Python 的说法表达：
// 调用失败抛 `FocError`，异常对象的 `code` 属性就是 C API 返回的 `ErrorCode`。

#include <foc/api/foc_api.h>
#include <foc/version.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace py = pybind11;



namespace {

/// The module-level `FocError` type, published to the translator once the module is initialised.
/// 模块级的 `FocError` 类型；模块初始化时交给异常翻译器。
PyObject *g_foc_error_type = nullptr;

/**
 * [en] Raised when a C API call returns anything but FOC_OK; `code` is the raw foc_err_t.
 * [zh] 任一 C API 调用返回非 FOC_OK 时抛出；`code` 是原始的 foc_err_t。
 */
class FocError final : public std::runtime_error {
public:
    FocError(foc_err_t code, const std::string &what) : std::runtime_error(what), code_(code) {}

    [[nodiscard]] auto code() const noexcept -> foc_err_t { return code_; }

private:
    foc_err_t code_;
};


auto error_name(foc_err_t code) -> const char * {
    switch (code) {
    case FOC_OK:
        return "FOC_OK";
    case FOC_ERR_PARAM:
        return "FOC_ERR_PARAM";
    case FOC_ERR_NO_RESOURCE:
        return "FOC_ERR_NO_RESOURCE";
    case FOC_ERR_STATE:
        return "FOC_ERR_STATE";
    case FOC_ERR_NOT_IMPL:
        return "FOC_ERR_NOT_IMPL";
    case FOC_ERR_FAULT:
        return "FOC_ERR_FAULT";
    }
    return "FOC_ERR_UNKNOWN";
}


/// Turn a return code into a Python-visible exception; the C call site stays one line.
/// 把返回码变成 Python 可见的异常；C 调用点在源码里仍然只有一行。
auto check(foc_err_t code, const char *call) -> void {
    if (code == FOC_OK) {
        return;
    }
    throw FocError(code, std::string(call) + " failed: " + error_name(code));
}


/**
 * [en] RAII owner of one `foc_motor_t`. The handle is destroyed exactly once: `close()` clears it,
 * the destructor calls `close()`, and a destroyed instance cannot be used again.
 * [zh] 一个 `foc_motor_t` 的 RAII 持有者。句柄只销毁一次：`close()` 清空它，析构函数调用
 * `close()`，已销毁的实例不能再用。
 */
class Motor final {
public:
    explicit Motor(const foc_motor_params_t &params) {
        check(foc_motor_create(&params, &handle_), "foc_motor_create");
    }

    Motor(const foc_motor_params_t &params, const foc_motor_setup_t &setup) {
        check(foc_motor_create_ex(&params, &setup, &handle_), "foc_motor_create_ex");
    }

    /// [en] The setup the library actually runs with, defaults included.
    /// [zh] 库实际生效的设置（含默认值）。
    [[nodiscard]] auto setup() const -> foc_motor_setup_t {
        foc_motor_setup_t applied{};
        check(foc_motor_get_setup(handle_, &applied), "foc_motor_get_setup");
        return applied;
    }

    ~Motor() { close(); }

    Motor(const Motor &) = delete;
    auto operator=(const Motor &) -> Motor & = delete;

    Motor(Motor &&other) noexcept : handle_(other.handle_) { other.handle_ = nullptr; }

    auto operator=(Motor &&other) noexcept -> Motor & {
        if (this != &other) {
            close();
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    void enable() { check(foc_motor_enable(handle_), "foc_motor_enable"); }

    void disable() { check(foc_motor_disable(handle_), "foc_motor_disable"); }

    void clear_fault() { check(foc_motor_clear_fault(handle_), "foc_motor_clear_fault"); }

    void set_speed(double rad_per_s) { check(foc_set_speed_f64(handle_, rad_per_s), "foc_set_speed"); }

    void set_torque(double iq_a) { check(foc_set_torque_f64(handle_, iq_a), "foc_set_torque"); }

    void set_position(double position_rad) {
        check(foc_set_position_f64(handle_, position_rad), "foc_set_position");
    }

    void set_command(const foc_motor_command_t &command) {
        check(foc_set_command(handle_, &command), "foc_set_command");
    }

    [[nodiscard]] auto status() const -> py::dict {
        foc_motor_status_t raw{};
        raw.struct_size = sizeof(foc_motor_status_t);
        check(foc_motor_get_status(handle_, &raw), "foc_motor_get_status");
        py::dict out;
        out["mode"] = py::cast(static_cast<foc_motor_state_t>(raw.mode_state));
        out["error"] = py::cast(static_cast<foc_err_t>(raw.error));
        out["align_state"] = py::cast(static_cast<foc_align_state_t>(raw.align_state));
        out["align_failure"] = py::cast(static_cast<foc_align_failure_t>(raw.align_failure));
        out["align_offset_rad"] = raw.align_offset_rad;
        out["align_spread_rad"] = raw.align_spread_rad;
        out["max_period_us"] = raw.max_period_us;
        return out;
    }

    void tick(double dt_s) { check(foc_tick(handle_, static_cast<float>(dt_s)), "foc_tick"); }

    [[nodiscard]] auto telemetry() const -> py::dict {
        float speed = 0.0F;
        float id = 0.0F;
        float iq = 0.0F;
        check(foc_get_state(handle_, &speed, &id, &iq), "foc_get_state");
        py::dict out;
        out["speed_mech_rad_s"] = speed;
        out["id_a"] = id;
        out["iq_a"] = iq;
        return out;
    }

    void close() noexcept {
        if (handle_ != nullptr) {
            (void)foc_motor_destroy(handle_);
            handle_ = nullptr;
        }
    }

    [[nodiscard]] auto alive() const noexcept -> bool { return handle_ != nullptr; }

private:
    foc_motor_t handle_{nullptr};
};


/**
 * [en] RAII owner of one `foc_protector_t`, same discipline as `Motor`: destroyed exactly once.
 * [zh] 一个 `foc_protector_t` 的 RAII 持有者，纪律与 `Motor` 相同：只销毁一次。
 */
class Protector final {
public:
    explicit Protector(foc_protector_t handle) noexcept : handle_(handle) {}
    ~Protector() { close(); }

    Protector(const Protector &) = delete;
    auto operator=(const Protector &) -> Protector & = delete;
    Protector(Protector &&other) noexcept : handle_(other.handle_) { other.handle_ = nullptr; }
    auto operator=(Protector &&other) noexcept -> Protector & {
        if (this != &other) {
            close();
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    /// [en] One control period. The return is a tuple rather than a dict because the six values
    /// are positional in the C output struct and a dict would invite name drift.
    /// [zh] 一个控制周期。返回元组而非 dict，因为这六个数在 C 输出结构里是有位置的，dict 会招来
    /// 名字漂移。
    [[nodiscard]] auto step(float i_magnitude_a, float v_bus_v, float temperature_c, float omega_e_rad_s,
                            float dt_s, float flux_error_ratio, bool flux_valid) const -> py::tuple {
        foc_protection_input_t in{};
        in.struct_size = static_cast<std::uint32_t>(sizeof(in));
        in.i_magnitude_a = i_magnitude_a;
        in.v_bus_v = v_bus_v;
        in.temperature_c = temperature_c;
        in.omega_e_rad_s = omega_e_rad_s;
        in.flux_error_ratio = flux_error_ratio;
        in.flux_valid = flux_valid ? 1 : 0;

        foc_protection_output_t out{};
        out.struct_size = static_cast<std::uint32_t>(sizeof(out));
        check(foc_protector_step(handle_, &in, dt_s, &out), "foc_protector_step");

        // The fault name comes from the same table the C users read, so the two Faces cannot
        // disagree about what fault 3 is called.
        // 故障名取自 C 用户读的同一张表，于是两个"面"不会对"3 号故障叫什么"产生分歧。
        const char *name = foc_fault_name(out.fault);
        return py::make_tuple(name, out.i_limit_a, out.output_allowed != 0, out.i2t_budget,
                              out.latched_mask != 0U, out.active_mask);
    }

    void reset(bool cold) const { check(foc_protector_reset(handle_, cold ? 1 : 0), "foc_protector_reset"); }

    void close() noexcept {
        if (handle_ != nullptr) {
            (void)foc_protector_destroy(handle_);
            handle_ = nullptr;
        }
    }

    [[nodiscard]] auto alive() const noexcept -> bool { return handle_ != nullptr; }

private:
    foc_protector_t handle_{nullptr};
};

auto make_motor(float rs, float ld, float lq, float kt, float j, float b, std::uint8_t pole_pairs) -> Motor {
    foc_motor_params_t params{};
    params.rs = rs;
    params.ld = ld;
    params.lq = lq;
    params.kt = kt;
    params.j = j;
    params.b = b;
    params.pole_pairs = pole_pairs;
    return Motor(params);
}

} // namespace



PYBIND11_MODULE(foc_python, m) {
    m.doc() = "[en] Python face of the foc C API: create a motor, drive it, read the telemetry.\n"
              "[zh] foc C API 的 Python 面：创建电机、给定量、读遥测。";

    m.attr("__version__") = foc_version_string();

    py::enum_<foc_err_t>(m, "ErrorCode", "[en] Return codes, frozen and identical to the C side.\n"
                                         "[zh] 返回码，与 C 侧完全一致且数值已冻结。")
        .value("OK", FOC_OK)
        .value("PARAM", FOC_ERR_PARAM)
        .value("NO_RESOURCE", FOC_ERR_NO_RESOURCE)
        .value("STATE", FOC_ERR_STATE)
        .value("NOT_IMPL", FOC_ERR_NOT_IMPL)
        .value("FAULT", FOC_ERR_FAULT);

    py::enum_<foc_motor_state_t>(m, "MotorState",
                                 "[en] Mode state, mirrors foc::core::MotorState.\n"
                                 "[zh] 运行模式状态，与 foc::core::MotorState 对应。")
        .value("IDLE", FOC_MOTOR_IDLE)
        .value("ALIGN", FOC_MOTOR_ALIGN)
        .value("RUN", FOC_MOTOR_RUN)
        .value("FAULT", FOC_MOTOR_FAULT);

    py::enum_<foc_align_state_t>(m, "AlignState",
                                 "[en] Alignment phase, mirrors foc::core::AlignState.\n"
                                 "[zh] 对齐阶段，与 foc::core::AlignState 对应。")
        .value("IDLE", FOC_ALIGN_IDLE)
        .value("RAMP", FOC_ALIGN_RAMP)
        .value("SETTLE", FOC_ALIGN_SETTLE)
        .value("VERIFY", FOC_ALIGN_VERIFY)
        .value("DONE", FOC_ALIGN_DONE)
        .value("FAILED", FOC_ALIGN_FAILED);

    py::enum_<foc_align_failure_t>(m, "AlignFailure",
                                   "[en] Why an alignment was rejected, mirrors foc::core::AlignFailure.\n"
                                   "[zh] 对齐被拒的原因，与 foc::core::AlignFailure 对应。")
        .value("NONE", FOC_ALIGN_NO_FAILURE)
        .value("NOT_CONFIGURED", FOC_ALIGN_NOT_CONFIGURED)
        .value("NOT_SETTLED", FOC_ALIGN_NOT_SETTLED)
        .value("DIRECTION_WRONG", FOC_ALIGN_DIRECTION_WRONG);

    const py::object foc_error = py::exception<FocError>(m, "FocError", PyExc_RuntimeError);
    g_foc_error_type = foc_error.ptr();

    // a bare function pointer: pybind11 stores a translator per interpreter, so it can only take
    // one that captures nothing
    py::register_exception_translator([](std::exception_ptr failure) {
        try {
            if (failure != nullptr) {
                std::rethrow_exception(failure);
            }
        } catch (const FocError &error) {
            // hand the raw code to Python as well: `except FocError as e: e.code`
            py::object error_type = py::reinterpret_borrow<py::object>(g_foc_error_type);
            py::object instance = error_type(py::str(error.what()));
            instance.attr("code") = py::cast(error.code());
            PyErr_SetObject(g_foc_error_type, instance.ptr());
        }
    });

    py::class_<foc_motor_params_t>(m, "MotorParams",
                                    "[en] Electrical / mechanical parameters, SI units.\n"
                                    "[zh] 电气与机械参数，SI 单位。")
        .def(py::init([](float rs, float ld, float lq, float kt, float j, float b, std::uint8_t pole_pairs) {
                 foc_motor_params_t params{};
                 params.rs = rs;
                 params.ld = ld;
                 params.lq = lq;
                 params.kt = kt;
                 params.j = j;
                 params.b = b;
                 params.pole_pairs = pole_pairs;
                 return params;
             }),
             py::arg("rs") = 0.5F, py::arg("ld") = 4e-4F, py::arg("lq") = 4e-4F, py::arg("kt") = 0.05F,
             py::arg("j") = 1e-5F, py::arg("b") = 1e-6F, py::arg("pole_pairs") = static_cast<std::uint8_t>(7))
        .def_readwrite("rs", &foc_motor_params_t::rs)
        .def_readwrite("ld", &foc_motor_params_t::ld)
        .def_readwrite("lq", &foc_motor_params_t::lq)
        .def_readwrite("kt", &foc_motor_params_t::kt)
        .def_readwrite("j", &foc_motor_params_t::j)
        .def_readwrite("b", &foc_motor_params_t::b)
        .def_readwrite("pole_pairs", &foc_motor_params_t::pole_pairs)
        .def("__repr__", [](const foc_motor_params_t &p) {
            return py::str("MotorParams(rs={}, ld={}, lq={}, kt={}, j={}, b={}, pole_pairs={})")
                .format(p.rs, p.ld, p.lq, p.kt, p.j, p.b, p.pole_pairs);
        });

    py::class_<foc_motor_setup_t>(m, "MotorSetup",
                                  "[en] Optional setup: every value the library would otherwise take from\n"
                                  "its defaults. A field left at 0 keeps the default (except dead time and the\n"
                                  "loop period, where 0 is a real value).\n"
                                  "[zh] 可选设置：不填时库会自行取默认值的那些量。字段留 0 表示用默认值\n"
                                  "（死区与环路周期除外，它们的 0 是真实取值）。")
        .def(py::init([](float i_max_a, float pwm_freq_hz, float deadtime_ns, float v_bus_v,
                         float current_bandwidth_rad_s, float speed_bandwidth_rad_s, float speed_loop_period_s,
                         float position_bandwidth_rad_s, float velocity_limit_rad_s, float position_loop_period_s,
                         float position_accel_limit_rad_s2,
                         float align_current_a, float align_ramp_s, float align_hold_s, float align_tolerance_rad,
                         float align_verify_s, float align_angle_e_rad) {
                 foc_motor_setup_t setup{};
                 setup.struct_size = sizeof(foc_motor_setup_t);
                 setup.i_max_a = i_max_a;
                 setup.pwm_freq_hz = pwm_freq_hz;
                 setup.deadtime_ns = deadtime_ns;
                 setup.v_bus_v = v_bus_v;
                 setup.current_bandwidth_rad_s = current_bandwidth_rad_s;
                 setup.speed_bandwidth_rad_s = speed_bandwidth_rad_s;
                 setup.speed_loop_period_s = speed_loop_period_s;
                 setup.position_bandwidth_rad_s = position_bandwidth_rad_s;
                 setup.velocity_limit_rad_s = velocity_limit_rad_s;
                 setup.position_loop_period_s = position_loop_period_s;
                 setup.position_accel_limit_rad_s2 = position_accel_limit_rad_s2;
                 setup.align_current_a = align_current_a;
                 setup.align_ramp_s = align_ramp_s;
                 setup.align_hold_s = align_hold_s;
                 setup.align_tolerance_rad = align_tolerance_rad;
                 setup.align_verify_s = align_verify_s;
                 setup.align_angle_e_rad = align_angle_e_rad;
                 return setup;
             }),
             py::arg("i_max_a") = 0.0F, py::arg("pwm_freq_hz") = 0.0F, py::arg("deadtime_ns") = 0.0F,
             py::arg("v_bus_v") = 0.0F, py::arg("current_bandwidth_rad_s") = 0.0F,
             py::arg("speed_bandwidth_rad_s") = 0.0F, py::arg("speed_loop_period_s") = 0.0F,
             py::arg("position_bandwidth_rad_s") = 0.0F, py::arg("velocity_limit_rad_s") = 0.0F,
             py::arg("position_loop_period_s") = 0.0F, py::arg("position_accel_limit_rad_s2") = 0.0F,
             py::arg("align_current_a") = 0.0F,
             py::arg("align_ramp_s") = 0.0F, py::arg("align_hold_s") = 0.0F,
             py::arg("align_tolerance_rad") = 0.0F, py::arg("align_verify_s") = 0.0F,
             py::arg("align_angle_e_rad") = 0.0F)
        .def_readonly("struct_size", &foc_motor_setup_t::struct_size)
        .def_readwrite("i_max_a", &foc_motor_setup_t::i_max_a)
        .def_readwrite("pwm_freq_hz", &foc_motor_setup_t::pwm_freq_hz)
        .def_readwrite("deadtime_ns", &foc_motor_setup_t::deadtime_ns)
        .def_readwrite("v_bus_v", &foc_motor_setup_t::v_bus_v)
        .def_readwrite("current_bandwidth_rad_s", &foc_motor_setup_t::current_bandwidth_rad_s)
        .def_readwrite("speed_bandwidth_rad_s", &foc_motor_setup_t::speed_bandwidth_rad_s)
        .def_readwrite("speed_loop_period_s", &foc_motor_setup_t::speed_loop_period_s)
        .def_readwrite("position_bandwidth_rad_s", &foc_motor_setup_t::position_bandwidth_rad_s)
        .def_readwrite("velocity_limit_rad_s", &foc_motor_setup_t::velocity_limit_rad_s)
        .def_readwrite("position_loop_period_s", &foc_motor_setup_t::position_loop_period_s)
        .def_readwrite("position_accel_limit_rad_s2", &foc_motor_setup_t::position_accel_limit_rad_s2)
        .def_readwrite("align_current_a", &foc_motor_setup_t::align_current_a)
        .def_readwrite("align_ramp_s", &foc_motor_setup_t::align_ramp_s)
        .def_readwrite("align_hold_s", &foc_motor_setup_t::align_hold_s)
        .def_readwrite("align_tolerance_rad", &foc_motor_setup_t::align_tolerance_rad)
        .def_readwrite("align_verify_s", &foc_motor_setup_t::align_verify_s)
        .def_readwrite("align_angle_e_rad", &foc_motor_setup_t::align_angle_e_rad)
        .def("__repr__", [](const foc_motor_setup_t &s) {
            return py::str("MotorSetup(i_max_a={}, pwm_freq_hz={}, deadtime_ns={}, v_bus_v={}, "
                           "current_bandwidth_rad_s={}, speed_bandwidth_rad_s={}, speed_loop_period_s={}, "
                           "position_bandwidth_rad_s={}, velocity_limit_rad_s={}, position_loop_period_s={}, "
                           "align_current_a={}, align_hold_s={}, align_tolerance_rad={})")
                .format(s.i_max_a, s.pwm_freq_hz, s.deadtime_ns, s.v_bus_v, s.current_bandwidth_rad_s,
                        s.speed_bandwidth_rad_s, s.speed_loop_period_s, s.position_bandwidth_rad_s,
                        s.velocity_limit_rad_s, s.position_loop_period_s, s.align_current_a, s.align_hold_s,
                        s.align_tolerance_rad);
        });

    py::class_<foc_motor_command_t>(m, "MotorCommand",
                                      "[en] One position move: target, per-move current ceiling, feedforward torque.\n"
                                      "[zh] 一次位置运动：目标、本次运动的电流上限、前馈转矩。")
        .def(py::init([](float position_rad, float max_current_a, float feedforward_current_a,
                         float max_velocity_rad_s, float max_accel_rad_s2) {
                 foc_motor_command_t command{};
                 command.position_rad = position_rad;
                 command.max_current_a = max_current_a;
                 command.feedforward_current_a = feedforward_current_a;
                 command.max_velocity_rad_s = max_velocity_rad_s;
                 command.max_accel_rad_s2 = max_accel_rad_s2;
                 return command;
             }),
             py::arg("position_rad") = 0.0F, py::arg("max_current_a") = 0.0F,
             py::arg("feedforward_current_a") = 0.0F, py::arg("max_velocity_rad_s") = 0.0F,
             py::arg("max_accel_rad_s2") = 0.0F)
        .def_readwrite("position_rad", &foc_motor_command_t::position_rad)
        .def_readwrite("max_current_a", &foc_motor_command_t::max_current_a)
        .def_readwrite("feedforward_current_a", &foc_motor_command_t::feedforward_current_a)
        .def_readwrite("max_velocity_rad_s", &foc_motor_command_t::max_velocity_rad_s)
        .def_readwrite("max_accel_rad_s2", &foc_motor_command_t::max_accel_rad_s2)
        .def("__repr__", [](const foc_motor_command_t &c) {
            return py::str("MotorCommand(position_rad={}, max_current_a={}, feedforward_current_a={})")
                .format(c.position_rad, c.max_current_a, c.feedforward_current_a);
        });

    py::class_<Motor>(m, "Motor",
                       "[en] One motor instance. Use it as a context manager: the handle is destroyed on\n"
                       "exit. Every failure raises FocError instead of returning a code.\n"
                       "[zh] 一个电机实例。建议用 with 语句：退出时销毁句柄。任何失败都抛 FocError，\n"
                       "而不是返回错误码。")
        .def(py::init<const foc_motor_params_t &>(), py::arg("params"))
        .def(py::init<const foc_motor_params_t &, const foc_motor_setup_t &>(), py::arg("params"),
             py::arg("setup"))
        .def("setup", &Motor::setup,
             "[en] The setup actually in force (defaults included). [zh] 实际生效的设置（含默认值）。")
        .def_static("from_parts", &make_motor, py::arg("rs") = 0.5F, py::arg("ld") = 4e-4F, py::arg("lq") = 4e-4F,
                    py::arg("kt") = 0.05F, py::arg("j") = 1e-5F, py::arg("b") = 1e-6F,
                    py::arg("pole_pairs") = static_cast<std::uint8_t>(7),
                    "[en] Build the params from numbers and create the motor in one call.\n"
                    "[zh] 用数值构造参数并一次创建电机。")
        .def("enable", &Motor::enable, "[en] Enable the output stage. [zh] 使能输出级。")
        .def("disable", &Motor::disable, "[en] Disable the output stage. [zh] 关闭输出级。")
        .def("clear_fault", &Motor::clear_fault,
             "[en] Clear a latched fault (FOC_ERR_STATE if the motor was not faulted).\n"
             "[zh] 清除锁存故障（不在故障态时抛 FOC_ERR_STATE）。")
        .def("set_speed", &Motor::set_speed, py::arg("rad_per_s"),
             "[en] Speed setpoint, mechanical rad/s. [zh] 速度给定，机械 rad/s。")
        .def("set_torque", &Motor::set_torque, py::arg("iq_a"),
             "[en] Torque setpoint as q-axis current, A. [zh] 转矩给定（q 轴电流），A。")
        .def("set_position", &Motor::set_position, py::arg("position_rad"),
             "[en] Position setpoint in the frame the sensor reports (i.e. after the alignment offset).\n"
             "[zh] 位置给定，坐标系与传感器汇报的一致（即已扣除对齐偏置）。")
        .def("set_command", &Motor::set_command, py::arg("command"),
             "[en] A full move: target, per-move current ceiling, feedforward torque.\n"
             "[zh] 一次完整运动：目标、本次电流上限、前馈转矩。")
        .def("status", &Motor::status,
             "[en] Mode, sticky error and the alignment result in one dict.\n"
             "[zh] 一次读回模式、粘滞错误与对齐结果。")
        .def("tick", &Motor::tick, py::arg("dt_s"),
             "[en] Advance the control loop by one step. [zh] 推进一个控制步。")
        .def("telemetry", &Motor::telemetry,
             "[en] Last step: {speed_mech_rad_s, id_a, iq_a}. [zh] 最近一步：{speed_mech_rad_s, id_a, iq_a}。")
        .def("close", &Motor::close, "[en] Destroy the handle now (idempotent). [zh] 立即销毁句柄（幂等）。")
        .def_property_readonly("alive", &Motor::alive,
                               "[en] False once the handle has been destroyed. [zh] 句柄销毁后为 False。")
        .def("__enter__", [](Motor &self) -> Motor & { return self; }, py::return_value_policy::reference_internal)
        .def("__exit__",
             [](Motor &self, const py::object &, const py::object &, const py::object &) { self.close(); })
        .def("__repr__", [](const Motor &self) { return py::str("Motor(alive={})").format(self.alive()); });

    // ------------------------------------------------------------------------------------------
    // ABI 0.1.1: the modules the C surface could not reach. Bound one for one, with no extra
    // Python-side logic -- a thin binding is the only kind that cannot drift from the C contract.
    // ABI 0.1.1：C 表面此前够不到的那些模块。一对一绑定，Python 侧不加任何额外逻辑——只有薄的
    // 绑定才不会与 C 契约漂移。
    // ------------------------------------------------------------------------------------------

    m.def(
        "axis_deadtime_v", &foc_axis_deadtime_v, py::arg("deadtime_s"), py::arg("switching_hz"),
        py::arg("v_bus_v"), py::arg("angle_rad"),
        "[en] Dead-time volts projected onto an axis; 0 on a non-finite argument.\n"
        "[zh] 死区伏秒投影到某轴上的电压；参数非有限返回 0。");
    m.def("board_item_name", &foc_board_item_name, py::arg("item"),
          "[en] Stable lowercase name of a board item. [zh] 检查项的稳定小写名。");
    m.def("board_cause_name", &foc_board_cause_name, py::arg("cause"),
          "[en] Stable lowercase name of a board cause. [zh] 成因的稳定小写名。");

    m.def(
        "field_weakening_worthwhile",
        [](const foc_motor_params_t &params, float i_max_a) {
            foc_fw_verdict_t out{};
            out.struct_size = static_cast<std::uint32_t>(sizeof(out));
            const foc_err_t rc = foc_field_weakening_worthwhile(&params, i_max_a, &out);
            if (rc != FOC_OK) {
                throw std::runtime_error("foc_field_weakening_worthwhile failed");
            }
            return py::make_tuple(out.i_char_a, out.ratio, out.speed_gain, out.worthwhile != 0);
        },
        py::arg("params"), py::arg("i_max_a"),
        "[en] (i_char_a, ratio, speed_gain, worthwhile) from nameplate numbers alone.\n"
        "[zh] 只用铭牌数字得到 (i_char_a, ratio, speed_gain, worthwhile)。");

    m.def(
        "mtpa_reference",
        [](const foc_motor_params_t &params, float i_max_a, float iq_command_a) {
            float id_a = 0.0F;
            float iq_a = 0.0F;
            const foc_err_t rc = foc_mtpa_reference(&params, i_max_a, iq_command_a, &id_a, &iq_a);
            if (rc != FOC_OK) {
                throw std::runtime_error("foc_mtpa_reference failed");
            }
            return py::make_tuple(id_a, iq_a);
        },
        py::arg("params"), py::arg("i_max_a"), py::arg("iq_command_a"),
        "[en] (id_a, iq_a) for maximum torque per amp. [zh] 每安培转矩最大的 (id_a, iq_a)。");

    py::class_<foc_board_measurements_t>(m, "BoardMeasurements",
                                         "[en] What a standstill self-test measures.\n"
                                         "[zh] 堵转自检测到的量。")
        .def(py::init([]() {
            foc_board_measurements_t out{};
            out.struct_size = static_cast<std::uint32_t>(sizeof(out));
            return out;
        }))
        .def_readwrite("sum_current_a", &foc_board_measurements_t::sum_current_a)
        .def_readwrite("ripple_measured_a", &foc_board_measurements_t::ripple_measured_a)
        .def_readwrite("ripple_expected_a", &foc_board_measurements_t::ripple_expected_a)
        .def_readwrite("deadtime_v", &foc_board_measurements_t::deadtime_v)
        .def_readwrite("deadtime_cmd_v", &foc_board_measurements_t::deadtime_cmd_v)
        .def_readwrite("encoder_offset_rad", &foc_board_measurements_t::encoder_offset_rad)
        .def_readwrite("rs_ohm", &foc_board_measurements_t::rs_ohm)
        .def_readwrite("rs_nameplate_ohm", &foc_board_measurements_t::rs_nameplate_ohm)
        .def_readwrite("psi_f_wb", &foc_board_measurements_t::psi_f_wb)
        .def_readwrite("psi_f_nameplate_wb", &foc_board_measurements_t::psi_f_nameplate_wb)
        .def_readwrite("observer_error_rad", &foc_board_measurements_t::observer_error_rad)
        .def("set_offset", [](foc_board_measurements_t &self, int phase, float value) {
            self.offset_a[phase] = value;
        })
        .def("set_response_sign", [](foc_board_measurements_t &self, int phase, float value) {
            self.response_sign[phase] = value;
        });

    py::class_<foc_board_limits_t>(m, "BoardLimits", "[en] The board-check tolerances.\n"
                                                     "[zh] 板级检查的容差。")
        .def(py::init([]() {
            foc_board_limits_t out{};
            out.struct_size = static_cast<std::uint32_t>(sizeof(out));
            out.sum_current_a = 0.05F;
            out.offset_a = 0.02F;
            out.ripple_ratio = 1.5F;
            out.deadtime_ratio = 0.25F;
            out.encoder_offset_rad = 0.02F;
            out.parameter_ratio = 0.10F;
            out.observer_error_rad = 0.10F;
            return out;
        }))
        .def_readwrite("sum_current_a", &foc_board_limits_t::sum_current_a)
        .def_readwrite("offset_a", &foc_board_limits_t::offset_a)
        .def_readwrite("ripple_ratio", &foc_board_limits_t::ripple_ratio)
        .def_readwrite("deadtime_ratio", &foc_board_limits_t::deadtime_ratio)
        .def_readwrite("encoder_offset_rad", &foc_board_limits_t::encoder_offset_rad)
        .def_readwrite("parameter_ratio", &foc_board_limits_t::parameter_ratio)
        .def_readwrite("observer_error_rad", &foc_board_limits_t::observer_error_rad);

    m.def(
        "board_check",
        [](const foc_board_measurements_t &m_in, const foc_board_limits_t &l_in) {
            foc_board_report_t report{};
            report.struct_size = static_cast<std::uint32_t>(sizeof(report));
            const foc_err_t rc = foc_board_check(&m_in, &l_in, &report);
            if (rc != FOC_OK) {
                throw std::runtime_error("foc_board_check failed");
            }
            py::list rows;
            for (int k = 0; k < report.count; ++k) {
                const foc_board_reading_t &r = report.items[k];
                rows.append(py::make_tuple(foc_board_item_name(r.item), r.value, r.limit,
                                           r.pass != 0, r.phase));
            }
            return py::make_tuple(rows, report.failures, foc_board_cause_name(report.cause),
                                  report.cause_phase);
        },
        py::arg("measurements"), py::arg("limits"),
        "[en] (rows, failures, cause_name, cause_phase); each row is (name, value, limit, pass, "
        "phase).\n"
        "[zh] (rows, failures, cause_name, cause_phase)；每行是 (名称, 值, 限值, 通过, 相)。");

    py::class_<Protector>(m, "Protector",
                          "[en] The supervisory protection layer, stepped once per control period.\n"
                          "[zh] 监督式保护层，每个控制周期推一步。")
        .def(py::init([]() {
            foc_protector_t handle = nullptr;
            const foc_err_t rc = foc_protector_create(nullptr, &handle);
            if (rc != FOC_OK) {
                throw std::runtime_error("foc_protector_create failed");
            }
            return Protector{handle};
        }))
        .def("step", &Protector::step, py::arg("i_magnitude_a"), py::arg("v_bus_v"),
             py::arg("temperature_c"), py::arg("omega_e_rad_s"), py::arg("dt_s"),
             py::arg("flux_error_ratio") = 0.0F, py::arg("flux_valid") = true,
             "[en] One period: returns (fault_name, i_limit_a, output_allowed, i2t_budget, "
             "latched).\n"
             "[zh] 一个周期：返回 (故障名, 电流限幅, 是否允许输出, I²t 预算, 是否锁存)。")
        .def("reset", &Protector::reset, py::arg("cold") = false,
             "[en] Clear the faults; cold also clears the I^2t memory.\n"
             "[zh] 清故障；cold 同时清 I²t 记忆。")
        .def("close", &Protector::close, "[en] Destroy the handle (idempotent). [zh] 销毁句柄（幂等）。")
        .def_property_readonly("alive", &Protector::alive,
                               "[en] False once destroyed. [zh] 销毁后为 False。")
        .def("__enter__", [](Protector &self) -> Protector & { return self; },
             py::return_value_policy::reference_internal)
        .def("__exit__",
             [](Protector &self, const py::object &, const py::object &, const py::object &) {
                 self.close();
             });
}
