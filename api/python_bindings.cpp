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

    void set_speed(double rad_per_s) { check(foc_set_speed_f64(handle_, rad_per_s), "foc_set_speed"); }

    void set_torque(double iq_a) { check(foc_set_torque_f64(handle_, iq_a), "foc_set_torque"); }

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
        .value("NOT_IMPL", FOC_ERR_NOT_IMPL);

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

    py::class_<Motor>(m, "Motor",
                       "[en] One motor instance. Use it as a context manager: the handle is destroyed on\n"
                       "exit. Every failure raises FocError instead of returning a code.\n"
                       "[zh] 一个电机实例。建议用 with 语句：退出时销毁句柄。任何失败都抛 FocError，\n"
                       "而不是返回错误码。")
        .def(py::init<const foc_motor_params_t &>(), py::arg("params"))
        .def_static("from_parts", &make_motor, py::arg("rs") = 0.5F, py::arg("ld") = 4e-4F, py::arg("lq") = 4e-4F,
                    py::arg("kt") = 0.05F, py::arg("j") = 1e-5F, py::arg("b") = 1e-6F,
                    py::arg("pole_pairs") = static_cast<std::uint8_t>(7),
                    "[en] Build the params from numbers and create the motor in one call.\n"
                    "[zh] 用数值构造参数并一次创建电机。")
        .def("enable", &Motor::enable, "[en] Enable the output stage. [zh] 使能输出级。")
        .def("disable", &Motor::disable, "[en] Disable the output stage. [zh] 关闭输出级。")
        .def("set_speed", &Motor::set_speed, py::arg("rad_per_s"),
             "[en] Speed setpoint, mechanical rad/s. [zh] 速度给定，机械 rad/s。")
        .def("set_torque", &Motor::set_torque, py::arg("iq_a"),
             "[en] Torque setpoint as q-axis current, A. [zh] 转矩给定（q 轴电流），A。")
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
}
