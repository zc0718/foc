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
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;



namespace {

/**
 * @brief The module-level `FocError` type, published to the translator once the module is initialised. 模块级的 `FocError` 类型；模块初始化时交给异常翻译器。
 */
PyObject *g_foc_error_type = nullptr;

/**
 *  [en] Raised when a C API call returns anything but FOC_OK; `code` is the raw foc_err_t.
 *  [zh] 任一 C API 调用返回非 FOC_OK 时抛出；`code` 是原始的 foc_err_t。
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


/**
 * @brief Turn a return code into a Python-visible exception; the C call site stays one line. 把返回码变成 Python 可见的异常；C 调用点在源码里仍然只有一行。
 */
auto check(foc_err_t code, const char *call) -> void {
    if (code == FOC_OK) {
        return;
    }
    throw FocError(code, std::string(call) + " failed: " + error_name(code));
}


/**
 *  [en] RAII owner of one `foc_motor_t`. The handle is destroyed exactly once: `close()` clears it, the destructor
 *       calls `close()`, and a destroyed instance cannot be used again.
 *  [zh] 一个 `foc_motor_t` 的 RAII 持有者。句柄只销毁一次：`close()` 清空它，析构函数调用 `close()`，已销毁的实例不能再
 *       用。
 */
/**
 * @brief [en] A Python object standing in for the plant: it receives the duties and supplies the measurements.
 * @brief [zh] 顶替被控对象的 Python 对象：接收占空比，提供测量值。
 * @details [en] Every trampoline catches what Python raises and latches it, because the control loop is `noexcept` and
 *               an exception crossing it would terminate the process. A latched failure stops further calls and is
 *               re-raised on the Python side by `tick()`, so the first place the author looks is the line that called
 *               it.
 * @details [zh] 每个 trampoline 都把 Python 抛出的东西捕获并锁存，因为控制环是 `noexcept`，异常穿过它会让进程终止。锁存
 *               的失败会停止后续调用，并由 `tick()` 在 Python 侧重抛，于是作者第一眼看到的就是调用它的那一行。
 * @since 0.1.2
 */
struct PyHal final {
    py::object write_duty;
    py::object read_current;
    py::object angle;
    py::object voltage;
    py::object velocity;
    py::object now_us;
    bool failed{false};
    std::string error;

    explicit PyHal(py::object plant) {
        for (const char *name : {"write_duty", "read_current", "angle_mech_rad", "voltage_v"}) {
            if (!py::hasattr(plant, name)) {
                throw py::attribute_error(std::string("the plant must provide ") + name + "()");
            }
        }
        write_duty = plant.attr("write_duty");
        read_current = plant.attr("read_current");
        angle = plant.attr("angle_mech_rad");
        voltage = plant.attr("voltage_v");
        velocity = py::getattr(plant, "velocity_rad_s", py::none());
        now_us = py::getattr(plant, "now_us", py::none());
    }

    void fail(const char *what) noexcept {
        if (!failed) {
            failed = true;
            error = what;
        }
    }

    [[nodiscard]] auto callbacks() -> foc_hal_callbacks_t {
        foc_hal_callbacks_t out{};
        out.struct_size = sizeof(foc_hal_callbacks_t);
        out.user = this;
        out.pwm_write_duty = &PyHal::on_write_duty;
        out.read_current = &PyHal::on_read_current;
        out.read_angle_mech_rad = &PyHal::on_read_angle;
        out.read_voltage_v = &PyHal::on_read_voltage;
        out.read_velocity_rad_s = velocity.is_none() ? nullptr : &PyHal::on_read_velocity;
        out.now_us = now_us.is_none() ? nullptr : &PyHal::on_now_us;
        return out;
    }

  private:
    static void on_write_duty(void *user, const float duty_uvw[3]) {
        auto *self = static_cast<PyHal *>(user);
        if (self->failed) {
            return;
        }
        try {
            self->write_duty(duty_uvw[0], duty_uvw[1], duty_uvw[2]);
        } catch (const py::error_already_set &e) {
            self->fail(e.what());
        } catch (const std::exception &e) {
            self->fail(e.what());
        }
    }

    static void on_read_current(void *user, float uvw_a[3]) {
        auto *self = static_cast<PyHal *>(user);
        if (self->failed) {
            uvw_a[0] = 0.0F;
            uvw_a[1] = 0.0F;
            uvw_a[2] = 0.0F;
            return;
        }
        try {
            const py::sequence values = self->read_current();
            uvw_a[0] = py::cast<float>(values[0]);
            uvw_a[1] = py::cast<float>(values[1]);
            uvw_a[2] = py::cast<float>(values[2]);
        } catch (const py::error_already_set &e) {
            self->fail(e.what());
        } catch (const std::exception &e) {
            self->fail(e.what());
        }
    }

    static auto on_read_angle(void *user) -> float {
        auto *self = static_cast<PyHal *>(user);
        if (self->failed) {
            return 0.0F;
        }
        try {
            return py::cast<float>(self->angle());
        } catch (const py::error_already_set &e) {
            self->fail(e.what());
        } catch (const std::exception &e) {
            self->fail(e.what());
        }
        return 0.0F;
    }

    static auto on_read_voltage(void *user) -> float {
        auto *self = static_cast<PyHal *>(user);
        if (self->failed) {
            return 0.0F;
        }
        try {
            return py::cast<float>(self->voltage());
        } catch (const py::error_already_set &e) {
            self->fail(e.what());
        } catch (const std::exception &e) {
            self->fail(e.what());
        }
        return 0.0F;
    }

    static auto on_read_velocity(void *user) -> float {
        auto *self = static_cast<PyHal *>(user);
        if (self->failed) {
            return 0.0F;
        }
        try {
            return py::cast<float>(self->velocity());
        } catch (const py::error_already_set &e) {
            self->fail(e.what());
        } catch (const std::exception &e) {
            self->fail(e.what());
        }
        return 0.0F;
    }

    static auto on_now_us(void *user) -> std::uint64_t {
        auto *self = static_cast<PyHal *>(user);
        if (self->failed) {
            return 0U;
        }
        try {
            return py::cast<std::uint64_t>(self->now_us());
        } catch (const py::error_already_set &e) {
            self->fail(e.what());
        } catch (const std::exception &e) {
            self->fail(e.what());
        }
        return 0U;
    }
};


/** [en] Forward: `Motor::set_trace` needs it, and `Trace` needs `Motor`'s handle. [zh] 前置声明：`Motor::set_trace` 需要它，
 * 而 `Trace` 需要 `Motor` 的句柄。 */
class Trace;

class Motor final {
public:
    explicit Motor(const foc_motor_params_t &params) {
        check(foc_motor_create(&params, &handle_), "foc_motor_create");
    }

    Motor(const foc_motor_params_t &params, const foc_motor_setup_t &setup) {
        check(foc_motor_create_ex(&params, &setup, &handle_), "foc_motor_create_ex");
    }

    /**
     * @brief [en] A motor whose plant is this Python object.
     * @brief [zh] 被控对象为这个 Python 对象的电机。
     */
    Motor(const foc_motor_params_t &params, py::object plant) { create_host(params, nullptr, std::move(plant)); }

    /**
     * @brief [en] The same, with an explicit setup.
     * @brief [zh] 同上，但显式给出设置。
     */
    Motor(const foc_motor_params_t &params, const foc_motor_setup_t &setup, py::object plant) {
        create_host(params, &setup, std::move(plant));
    }

    /**
     * @brief [en] The setup the library actually runs with, defaults included.
     * @brief [zh] 库实际生效的设置（含默认值）。
     */
    [[nodiscard]] auto setup() const -> foc_motor_setup_t {
        foc_motor_setup_t applied{};
        check(foc_motor_get_setup(handle_, &applied), "foc_motor_get_setup");
        return applied;
    }

    ~Motor() { close(); }

    Motor(const Motor &) = delete;
    auto operator=(const Motor &) -> Motor & = delete;

    Motor(Motor &&other) noexcept : handle_(other.handle_), plant_(std::move(other.plant_)) {
        other.handle_ = nullptr;
    }

    auto operator=(Motor &&other) noexcept -> Motor & {
        if (this != &other) {
            close();
            handle_ = other.handle_;
            plant_ = std::move(other.plant_);
            other.handle_ = nullptr;
        }
        return *this;
    }

    void enable() { check(foc_motor_enable(handle_), "foc_motor_enable"); }

    void disable() { check(foc_motor_disable(handle_), "foc_motor_disable"); }

    void clear_fault() { check(foc_motor_clear_fault(handle_), "foc_motor_clear_fault"); }

    /**
     * @brief [en] Hand the running estimate over to another observer mode, without rebuilding the motor.
     * @brief [zh] 把正在运行的估计交接给另一种观测器模式，不必重建电机。
     */
    void set_observer(foc_observer_t observer) {
        check(foc_motor_set_observer(handle_, observer), "foc_motor_set_observer");
    }

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
        // The observer's own answer, not a re-derivation: which mode is really running, whether it says it is producing
        // a measurement, and the signal it is measuring. `observer_locked` is the only channel through which a holding
        // observer differs from a stopped machine -- its angle and its speed look identical in both cases.
        // 观测器自己的回答，而不是重新推导：真正在跑哪个模式、它是否表示自己在产出测量、以及它在测的那个信号。
        // `observer_locked` 是一个正在保持的观测器与一台停住的机器之间唯一可区分的通道——两种情况下它的角度与转速看起来
        // 一模一样。
        out["observer"] = py::cast(static_cast<foc_observer_t>(raw.observer));
        out["observer_locked"] = (raw.observer_locked != 0);
        out["observer_error"] = raw.observer_error;
        return out;
    }

    /**
     * @brief [en] Attach a trace window (or detach with None). Defined after `Trace`, whose layout it needs.
     * @brief [zh] 挂上轨迹窗口（或用 None 拆除）。定义放在 `Trace` 之后，因为它需要后者的布局。
     */
    void set_trace(const py::object &trace);

    void tick(double dt_s) {
        check(foc_tick(handle_, static_cast<float>(dt_s)), "foc_tick");
        if ((plant_ != nullptr) && plant_->failed) {
            // The loop is `noexcept`: the exception was caught at the C++ boundary and is re-raised here, where the
            // author can still see which call caused it.
            // 控制环是 `noexcept`：异常在 C++ 边界被捕获，在这里重抛，作者仍能看到是哪一次调用引起的。
            throw std::runtime_error(std::string("the plant raised: ") + plant_->error);
        }
    }

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

    /**
     * @brief [en] The raw handle, for the module's own compound objects. Not bound to Python: the module's promise is
     *             that nobody outside it ever touches a `foc_motor_t`.
     * @brief [zh] 裸句柄，供模块自己的复合对象使用。不绑到 Python：模块的承诺就是外部任何东西都不碰 `foc_motor_t`。
     */
    [[nodiscard]] auto handle() const noexcept -> foc_motor_t { return handle_; }

private:
    /**
     * @brief [en] Present only for a host-fed motor: it owns the Python object the callbacks point at.
     * @brief [zh] 仅主控电机有：它拥有回调所指向的那个 Python 对象。
     */
    /**
     * @brief [en] The Python object holding the attached trace buffer, kept alive while attached.
     * @brief [zh] 持有已挂载轨迹缓冲区的 Python 对象；挂载期间保活。
     */
    py::object attached_trace_{};
    std::unique_ptr<PyHal> plant_{};
    foc_motor_t handle_{nullptr};

    void create_host(const foc_motor_params_t &params, const foc_motor_setup_t *setup, py::object plant) {
        plant_ = std::make_unique<PyHal>(std::move(plant));
        const foc_hal_callbacks_t callbacks = plant_->callbacks();
        check(foc_motor_create_host(&params, setup, &callbacks, &handle_), "foc_motor_create_host");
    }
};



/**
 * @brief [en] A window of recorded periods, owned by Python and handed to the motor.
 * @brief [zh] 一段被记录下来的周期窗口，由 Python 持有、交给电机。
 * @note [en] The storage is Python's on purpose: the C ABI takes a caller-owned array because a firmware author may
 *            want the trace in a linker section, and the natural Python equivalent is a buffer this object owns. It is
 *            attached by reference, so it must outlive the attachment -- `Motor.set_trace(None)` or letting the motor
 *            die first are both fine, and `attach` re-checks that the motor is alive.
 * @note [zh] 存储刻意归 Python：C ABI 之所以取调用方持有的数组，是因为固件作者可能想把轨迹放进链接段，而 Python 这边的
 *            自然对应就是本对象持有的缓冲区。它是按引用挂载的，因此必须活得比挂载更久——`Motor.set_trace(None)` 或先让电
 *            机销毁都可以；`attach` 会重新检查电机是否还活着。
 * @since 0.1.8
 */
class Trace final {
public:
    explicit Trace(std::size_t capacity) : samples_(capacity) {}

    [[nodiscard]] auto capacity() const -> std::size_t { return samples_.size(); }
    [[nodiscard]] auto data() -> foc_trace_sample_t * { return samples_.data(); }

    /**
     * @brief [en] The retained window as a list of dicts, oldest first.
     * @brief [zh] 保留窗口，以 dict 列表给出，最旧的在前面。
     */
    [[nodiscard]] auto samples(const Motor &motor) const -> py::list {
        py::list out;
        const std::uint32_t size = foc_motor_trace_size(motor.handle());
        for (std::uint32_t index = 0; index < size; ++index) {
            foc_trace_sample_t sample{};
            if (foc_motor_trace_read(motor.handle(), index, &sample) != FOC_OK) {
                throw std::runtime_error("foc_motor_trace_read failed");
            }
            py::dict row;
            row["t_us"] = sample.t_us;
            row["i_d"] = sample.i_d;
            row["i_q"] = sample.i_q;
            row["i_ref_d"] = sample.i_ref_d;
            row["i_ref_q"] = sample.i_ref_q;
            row["v_d"] = sample.v_d;
            row["v_q"] = sample.v_q;
            row["speed_mech"] = sample.speed_mech;
            row["mode"] = py::cast(static_cast<foc_motor_state_t>(sample.mode));
            row["status_byte"] = sample.status;
            out.append(row);
        }
        return out;
    }

private:
    std::vector<foc_trace_sample_t> samples_;
};

void Motor::set_trace(const py::object &trace) {
    if (trace.is_none()) {
        check(foc_motor_set_trace(handle_, nullptr, 0U), "foc_motor_set_trace");
        attached_trace_ = py::object();
        return;
    }
    Trace &owned = trace.cast<Trace &>();
    check(foc_motor_set_trace(handle_, owned.data(), static_cast<std::uint32_t>(owned.capacity())),
          "foc_motor_set_trace");
    attached_trace_ = trace;
}

/**
 *  [en] RAII owner of one `foc_protector_t`, same discipline as `Motor`: destroyed exactly once.
 *  [zh] 一个 `foc_protector_t` 的 RAII 持有者，纪律与 `Motor` 相同：只销毁一次。
 */
class Protector final {public:
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

    /**
     * @brief [en] One control period. The return is a tuple rather than a dict because the six values are positional in
     *             the C output struct and a dict would invite name drift.
     * @brief [zh] 一个控制周期。返回元组而非 dict，因为这六个数在 C 输出结构里是有位置的，dict 会招来名字漂移。
     */
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

    // The mode the motor is really running. `MotorSetup.observer` is the request; this is the answer, and the note on
    // `foc_observer_t` says why the two are kept apart: a selector that accepts a mode and quietly runs the default
    // makes the caller believe a capability it does not have.
    // 电机真正在跑的模式。`MotorSetup.observer` 是请求，这个是回答；`foc_observer_t` 上的注释说明了为什么两者要分开：
    // 一个接受某模式却悄悄跑默认值的选择器，会让调用方相信自己拥有一个并不存在的能力。
    py::enum_<foc_observer_t>(m, "Observer", "[en] The observer actually running.\n"
                                            "[zh] 真正在跑的观测器。")
        .value("BACK_EMF", FOC_OBSERVER_BACK_EMF)
        .value("NONLINEAR_FLUX", FOC_OBSERVER_NONLINEAR_FLUX)
        .value("HFI", FOC_OBSERVER_HFI)
        .value("BLENDED", FOC_OBSERVER_BLENDED);

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
                // The requested observer mode, as the enum rather than the raw integer it is stored in: an author sets
        // `setup.observer = foc.Observer.HFI`, and the library answers with the mode it actually runs. Before this the
        // field was reachable from C and not from Python, which is the same gap as the status report had.
        // 请求的观测器模式，以枚举而不是它所存储的裸整数给出：作者写 `setup.observer = foc.Observer.HFI`，而库用它真正
        // 在跑的模式回答。在此之前这个字段从 C 可达、从 Python 不可达，与状态报告曾经的缺口是同一个。
        .def_property(
            "observer",
            [](const foc_motor_setup_t &setup) { return static_cast<foc_observer_t>(setup.observer); },
            [](foc_motor_setup_t &setup, foc_observer_t observer) { setup.observer = static_cast<std::int32_t>(observer); })
        // The injection half of the setup, which was in the C struct and unreachable from Python: without these an author
        // could select `Observer.HFI` and then have no way to give it a carrier. `hfi_size()` answers what to put here.
        // 设置的注入那一半，本来在 C 结构里、从 Python 不可达：没有它们，作者能选 `Observer.HFI` 却没办法给它一个载波。
        // `hfi_size()` 回答这里该填什么。
        .def_readwrite("inject_v", &foc_motor_setup_t::inject_v)
        .def_readwrite("inject_hz", &foc_motor_setup_t::inject_hz)
        .def_readwrite("blend_start_rad_s", &foc_motor_setup_t::blend_start_rad_s)
        .def_readwrite("blend_end_rad_s", &foc_motor_setup_t::blend_end_rad_s)
        .def_readwrite("handover_lpf_hz", &foc_motor_setup_t::handover_lpf_hz)
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
        .def(py::init<const foc_motor_params_t &, py::object>(), py::arg("params"), py::arg("plant"),
             "[en] Drive this Python object as the plant: it supplies currents, angle and bus voltage, and receives the\n"
             "duties. Same control loop, measurements from outside the library.\n"
             "[zh] 把这个 Python 对象当作被控对象来驱动：它提供电流、角度与母线电压，接收占空比。同一个控制环，\n"
             "测量值来自库外。")
        .def(py::init<const foc_motor_params_t &, const foc_motor_setup_t &, py::object>(), py::arg("params"),
             py::arg("setup"), py::arg("plant"))
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
        // The runtime hand-over, which was reachable from C and not from Python. Its refusals are part of the
        // contract, not accidents: outside Run it answers STATE (a hand-over carries an estimate, and an unstepped
        // observer has none to carry), and it answers PARAM when the setup in force cannot support the requested
        // mode. Both arrive here as FocError rather than as a silently unchanged `status()["observer"]`.
        // 运行时交接，本来从 C 可达、从 Python 不可达。它的拒绝是契约的一部分而非意外：不在 Run 状态时答 STATE（交接
        // 携带一个估计，而未被推进的观测器没有估计可交），当生效的设置支撑不了所请求的模式时答 PARAM。这两者在这里都
        // 以 FocError 到达，而不是"`status()["observer"]` 静默不变"。
        .def("set_observer", &Motor::set_observer, py::arg("observer"),
             "[en] Switch the observer while the motor runs (STATE outside Run, PARAM if the setup cannot support it).\n"
             "[zh] 在电机运行时切换观测器（不在 Run 状态抛 STATE，设置支撑不了时抛 PARAM）。")
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
        // The trace is attached by reference to a buffer Python owns, because that is the shape the C ABI takes.
        // `None` detaches. The refcount is kept so the buffer cannot be collected while the loop is writing into it --
        // which would be a use-after-free that the C side has no way to notice.
        // 轨迹按引用挂到 Python 持有的缓冲区上，因为 C ABI 就是这个形状。`None` 拆挂。这里保住了引用计数，使缓冲区不会在
        // 控制环往里写的时候被回收——那是 C 侧根本无法察觉的 use-after-free。
        .def("set_trace", &Motor::set_trace, py::arg("trace"),
             "[en] Attach a Trace (or None to detach). The window is written by the loop itself, so it records what the\n"
             "controller believed, one sample per period.\n"
             "[zh] 挂上一个 Trace（或 None 拆除）。窗口由控制环自己写入，因此它记录的是控制器相信的东西，每周期一条。")
        .def("trace_size", [](const Motor &self) { return foc_motor_trace_size(self.handle()); },
             "[en] Samples currently retained. [zh] 当前保留的样本数。")
        .def("trace_dropped", [](const Motor &self) { return foc_motor_trace_dropped(self.handle()); },
             "[en] Samples overwritten because the window was full. [zh] 因窗口写满而被覆盖的样本数。")
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
    m.def("hfi_max_inject_hz", &foc_hfi_max_inject_hz, py::arg("pwm_freq_hz"),
          "[en] The highest injection carrier this control rate can resolve, in Hz. Ask it before choosing "
          "`MotorSetup.inject_hz`: past the ceiling the injection observer stops working silently.\n"
          "[zh] 该控制率能分辨的最高注入载波，Hz。在选 `MotorSetup.inject_hz` 之前问它：越过天花板后注入观测器会静默失效。");

    m.def(
        "hfi_size",
        [](const foc_motor_params_t &params, double pwm_freq_hz, double hf_current_a) {
            foc_hfi_sizing_t out{};
            out.struct_size = sizeof(out);
            if (foc_hfi_size(&params, static_cast<float>(pwm_freq_hz), static_cast<float>(hf_current_a), &out) !=
                FOC_OK) {
                throw std::runtime_error("foc_hfi_size rejected its arguments: the machine is not salient, or the rate "
                                         "or the current budget is not positive");
            }
            py::dict sized;
            sized["inject_hz"] = out.inject_hz;
            sized["inject_v"] = out.inject_v;
            sized["demod_lpf_hz"] = out.demod_lpf_hz;
            sized["speed_lpf_hz"] = out.speed_lpf_hz;
            sized["hf_current_a"] = out.hf_current_a;
            return sized;
        },
        py::arg("params"), py::arg("pwm_freq_hz") = 20000.0, py::arg("hf_current_a") = 0.5,
        "[en] Size an injection configuration from the machine and the control rate: the carrier the rate can resolve, "
        "the volts that draw the HF current you budget, and the two measured cutoffs. The result is a configuration the "
        "library accepts.\n"
        "[zh] 由机器与控制率定容一份注入配置：该控制率能分辨的载波、拉出你预算的高频电流所需的伏特、以及两个实测"
        "截止频率。结果是库会接受的配置。");

    // The measurement side of the same question `hfi_size` answers: the loop can now be asked what it did, in numbers a
    // tuning decision rests on, and the answer comes from the same C++ that the C ABI returns -- so a Python author
    // checking their own numpy fit against this one is comparing two implementations of one metric, not two metrics.
    // 与 `hfi_size` 回答同一个问题的测量那一侧：现在可以问环路它到底做了什么，答案是能据以整定的数，而且与 C ABI
    // 返回的来自同一份 C++——因此 Python 作者把自己的 numpy 拟合与它比对，比的是同一个指标的两份实现，而不是两个指标。
    m.def(
        "step_metrics",
        [](const std::vector<float> &y, double dt_s, double target, double band) {
            foc_step_metrics_t out{};
            out.struct_size = sizeof(out);
            const foc_err_t rc =
                ::foc_step_metrics(y.data(), static_cast<uint32_t>(y.size()), static_cast<float>(dt_s),
                                   static_cast<float>(target), static_cast<float>(band), &out);
            if (rc == FOC_ERR_PARAM) {
                throw std::runtime_error("foc_step_metrics rejected its arguments: fewer than three samples, a "
                                         "non-positive dt, a negative band, or a window that does not move");
            }
            py::dict measured;
            measured["status"] = rc;
            measured["bandwidth_rad_s"] = out.bandwidth_rad_s;
            measured["tau_s"] = out.tau_s;
            measured["settling_s"] = out.settling_s;
            measured["overshoot_fraction"] = out.overshoot_fraction;
            measured["tail_error_fraction"] = out.tail_error_fraction;
            return measured;
        },
        py::arg("y"), py::arg("dt_s"), py::arg("target"), py::arg("band"),
        "[en] Measure a recorded step response: fit a first-order decay and report the loop's bandwidth, time constant, "
        "settling time, overshoot and tail error. `status` is `ErrorCode.OK` when a decay was fitted, `STATE` when the "
        "window carried none (the settling numbers are still valid then), and arguments that cannot describe a step "
        "raise. The window must start at or before the step: `y[0]` is the pre-step value.\n"
        "[zh] 测量一段记录下来的阶跃响应：拟合一个一阶衰减，并给出环路的带宽、时间常数、稳定时间、超调与尾部误差。"
        "`status` 为 `ErrorCode.OK` 表示拟合到衰减，`STATE` 表示窗口里没有可用衰减（此时稳定时间等数仍然有效），"
        "无法描述阶跃的参数会抛异常。窗口必须从阶跃前或阶跃处开始：`y[0]` 是阶跃前的值。");

    m.def("board_item_name", &foc_board_item_name, py::arg("item"),
          "[en] Stable lowercase name of a board item. [zh] 检查项的稳定小写名。");    // The sweep's only moving part, next to the step-response metric: fit one tone at a known frequency out of a window.
    // A sweep is the instrument that notices a gain law scaled wrongly, because a magnitude at a known frequency moves
    // with the error instead of carrying a comparable bias of its own.
    // 扫频唯一的运动部件，与阶跃指标并列：从一个窗口里拟合出已知频率上的单音。扫频是能发现"增益律比例错"的仪器，
    // 因为已知频率上的幅值随误差移动，而不是自带与之相当的偏差。
    m.def(
        "sinusoid_fit",
        [](const std::vector<float> &y, double dt_s, double omega_rad_s) {
            foc_sinusoid_fit_t out{};
            out.struct_size = sizeof(out);
            const foc_err_t rc = ::foc_sinusoid_fit(y.data(), static_cast<uint32_t>(y.size()),
                                                    static_cast<float>(dt_s), static_cast<float>(omega_rad_s), &out);
            if (rc == FOC_ERR_PARAM) {
                throw std::runtime_error("foc_sinusoid_fit rejected its arguments: fewer than six samples, a "
                                         "non-positive dt or frequency, or a frequency at or above Nyquist");
            }
            py::dict fitted;
            fitted["status"] = rc;
            fitted["amplitude"] = out.amplitude;
            fitted["phase_rad"] = out.phase_rad;
            return fitted;
        },
        py::arg("y"), py::arg("dt_s"), py::arg("omega_rad_s"),
        "[en] Fit one tone at a known frequency: returns amplitude and phase of `A*cos(w*t + phase)`. `status` is "
        "`ErrorCode.OK` when the fit is conditioned and `STATE` when the window is shorter than one period.\n"
        "[zh] 在已知频率上拟合一个单音：返回 `A*cos(w*t + phase)` 的幅值与相位。`status` 为 `ErrorCode.OK` 表示拟合良态，"
        "`STATE` 表示窗口不足一个周期。");

    m.def("board_item_name", &foc_board_item_name, py::arg("item"),
          "[en] Stable lowercase name of a board item. [zh] 检查项的稳定小写名。");
    m.def("board_cause_name", &foc_board_cause_name, py::arg("cause"),
          "[en] Stable lowercase name of a board cause. [zh] 成因的稳定小写名。");    // The fault table, standalone. `Protector.step()` already returns a name, but only for the fault it just produced;
    // an author who holds a code -- a bit position in `active_mask`, a line in a log, a number from another surface --
    // had no way to ask what it is called. It takes a plain `int` for the same reason `board_item_name` above does:
    // the argument an author has is a bit index or a raw code, and refusing that would make the binding usable only by
    // callers who had already decoded the thing they are trying to decode.
    // 故障表，独立可调。`Protector.step()` 已经返回名字，但只对它刚产出的那一个故障；作者若手上有一个码——`active_mask`
    // 里的一个比特位、日志里的一行、别的表面给出的一个数——此前没办法问它叫什么。它取裸 `int`，理由与上面的
    // `board_item_name` 相同：作者手上的参数是一个比特下标或一个裸码，拒绝它会让这个绑定只能被"已经解码过目标"的调用方
    // 使用，而那正是他们想要的答案。
    m.def(
        "fault_name", [](std::int32_t fault) { return std::string(::foc_fault_name(fault)); }, py::arg("fault"),
        "[en] Stable lowercase name of an FOC_FAULT_* value; \"?\" outside the table. Takes the raw code, so a bit\n"
        "index from `active_mask` decodes directly.\n"
        "[zh] FOC_FAULT_* 取值的稳定小写名；表外取值返回 \"?\"。取裸码，因此 `active_mask` 的比特下标可直接解码。");
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

    // ABI 0.1.3: the gains and the identification primitives. Bound one for one, because the whole point of a tuning
    // tool is that it reproduces the shipped loop rather than re-deriving it.
    // ABI 0.1.3：增益与辨识原语。一对一绑定，因为整定工具的意义就在于复现出厂环路，而不是另推一遍。
    const auto gains_to_tuple = [](const foc_pi_gains_t &g) {
        return py::make_tuple(g.kp, g.ki, g.out_min, g.out_max, g.integral_min, g.integral_max);
    };
    const auto call_gains = [](const foc_motor_params_t &params, const py::object &setup,
                               foc_err_t (*query)(const foc_motor_params_t *, const foc_motor_setup_t *,
                                                  foc_pi_gains_t *)) {
        foc_pi_gains_t out{};
        out.struct_size = sizeof(out);
        foc_motor_setup_t requested{};
        const foc_motor_setup_t *raw = nullptr;
        if (!setup.is_none()) {
            requested = py::cast<foc_motor_setup_t>(setup);
            raw = &requested;
        }
        if (query(&params, raw, &out) != FOC_OK) {
            throw std::runtime_error("the gain query rejected its arguments");
        }
        return out;
    };

    m.def(
        "current_pi_gains",
        [call_gains, gains_to_tuple](const foc_motor_params_t &params, const py::object &setup) {
            return gains_to_tuple(call_gains(params, setup, &foc_current_pi_gains));
        },
        py::arg("params"), py::arg("setup") = py::none(),
        "[en] (kp, ki, out_min, out_max, integral_min, integral_max) of the current loop the library would run.\n"
        "[zh] 库会运行的电流环的 (kp, ki, out_min, out_max, integral_min, integral_max)。");
    m.def(
        "speed_pi_gains",
        [call_gains, gains_to_tuple](const foc_motor_params_t &params, const py::object &setup) {
            return gains_to_tuple(call_gains(params, setup, &foc_speed_pi_gains));
        },
        py::arg("params"), py::arg("setup") = py::none(),
        "[en] The same six numbers for the speed loop. [zh] 速度环的同样六个数值。");

    m.def(
        "rs_estimate",
        [](const foc_motor_params_t &params, const std::vector<float> &v_command,
           const std::vector<float> &i_current) {
            if (v_command.size() != i_current.size()) {
                throw py::value_error("v_command and i_current must have the same length");
            }
            foc_identify_result_t out{};
            out.struct_size = sizeof(out);
            if (foc_rs_estimate(&params, v_command.data(), i_current.data(), static_cast<std::uint32_t>(v_command.size()),
                                &out) != FOC_OK) {
                throw std::runtime_error("foc_rs_estimate failed");
            }
            return py::make_tuple(out.value, out.offset, out.residual, out.valid != 0, out.samples);
        },
        py::arg("params"), py::arg("v_command"), py::arg("i_current"),
        "[en] (rs, loss_v, residual_v, valid, samples) from a standstill sweep. Check `valid` before using `rs`.\n"
        "[zh] 堵转扫描得到的 (rs, loss_v, residual_v, valid, samples)。用 `rs` 之前先看 `valid`。");
    m.def(
        "flux_estimate",
        [](const foc_motor_params_t &params, const std::vector<float> &omega_e, const std::vector<float> &v_magnitude,
           const std::vector<float> &i_magnitude) {
            if ((omega_e.size() != v_magnitude.size()) || (omega_e.size() != i_magnitude.size())) {
                throw py::value_error("the three arrays must have the same length");
            }
            foc_identify_result_t out{};
            out.struct_size = sizeof(out);
            if (foc_flux_estimate(&params, omega_e.data(), v_magnitude.data(), i_magnitude.data(),
                                  static_cast<std::uint32_t>(omega_e.size()), &out) != FOC_OK) {
                throw std::runtime_error("foc_flux_estimate failed");
            }
            return py::make_tuple(out.value, out.offset, out.residual, out.valid != 0, out.samples);
        },
        py::arg("params"), py::arg("omega_e"), py::arg("v_magnitude"), py::arg("i_magnitude"),
        "[en] (psi_f, offset_v, residual_v, valid, samples) from a zero-current speed sweep.\n"
        "[zh] 零电流转速扫描得到的 (psi_f, offset_v, residual_v, valid, samples)。");

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

    py::class_<Trace>(m, "Trace",
                      "[en] A window of recorded control periods. Create it with a capacity, hand it to a motor, read it\n"
                      "back as a list of dicts.\n"
                      "[zh] 一段被记录下来的控制周期窗口。给定容量创建、交给电机、再以 dict 列表读回。")
        .def(py::init<std::size_t>(), py::arg("capacity"),
             "[en] Samples to retain; the storage is allocated once, here, and never grows.\n"
             "[zh] 保留的样本数；存储在此时一次分配，之后不再增长。")
        .def_property_readonly("capacity", &Trace::capacity,
                               "[en] Samples this window can hold. [zh] 这个窗口能装多少条样本。")
        .def("samples", &Trace::samples, py::arg("motor"),
             "[en] The retained window, oldest first: t_us, i_d, i_q, i_ref_d, i_ref_q, v_d, v_q, speed_mech, mode,\n"
             "status_byte.\n"
             "[zh] 保留窗口，最旧在前的 t_us、i_d、i_q、i_ref_d、i_ref_q、v_d、v_q、speed_mech、mode、status_byte。")
        .def("__len__", [](const Trace &self) { return self.capacity(); });

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
