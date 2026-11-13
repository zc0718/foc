Symbols and units
=================

Every symbol this library's documentation is allowed to use, with the four things that make a symbol usable:
what it means, its unit, the frame it lives in, and which direction is positive.

本库文档允许使用的全部符号，连同让一个符号可用的四件事：含义、单位、所在的参考系、哪个方向为正。

The page is generated from `.github/project/symbols.json` by `symbols_gate.py --render`; a symbol that is not
on it cannot appear in a docstring, because the gate reads the same file.
本页由 `symbols_gate.py --render` 从 `.github/project/symbols.json` 生成；不在表上的符号不能出现在 docstring 里，
因为门禁读的是同一个文件。

θ  ``theta_e``
    **Meaning.** electrical rotor angle: the rotor position in the rotating frame, i.e. the angle `park` is handed.
    电角度：旋转坐标系下的转子位置，也就是交给 `park` 的那个角。
    **Unit.** rad  **Frame.** electrical rotor (the dq frame rotates with it)
    **Positive.** not fixed by this library: it is whatever the caller's angle source counts up as positive. `park` applies the angle it is given and never re-references it.
    **Defined by.** ``include/foc/core/transforms.hpp`` (``auto park(Ab<T> ab, T theta_rad)``)

θ_mech  ``theta_mech``
    **Meaning.** mechanical rotor angle, the quantity a cogging harmonic is phased against.
    机械角，齿槽谐波的相位所对的量。
    **Unit.** rad  **Frame.** mechanical rotor
    **Positive.** same direction as `theta_e`, divided by `pole_pairs`.
    **Defined by.** ``include/foc/core/cogging_comp.hpp`` (``float phase_rad;``)

ω_m  ``omega_m``
    **Meaning.** mechanical angular speed of the shaft.
    轴上的机械角速度。
    **Unit.** rad/s  **Frame.** mechanical rotor
    **Positive.** positive = the direction the angle source counts up; a speed loop that needs the other direction inverts at its own input, never here.
    **Defined by.** ``include/foc/motor/motor.hpp`` (``speed_mech * static_cast<float>(config_.pole_pairs)``)

ω_e  ``omega_e``
    **Meaning.** electrical angular speed, `omega_e = omega_m * pole_pairs`. Entering this one where the mechanical one belongs is the whole class of error the table exists for.
    电角速度，`omega_e = omega_m * pole_pairs`。把电角速度用在需要机械角速度的地方，正是这张表存在的整类错误。
    **Unit.** rad/s  **Frame.** electrical rotor
    **Positive.** same sign as `omega_m`.
    **Defined by.** ``include/foc/motor/motor.hpp`` (``speed_mech * static_cast<float>(config_.pole_pairs)``)

p  ``pole_pairs``
    **Meaning.** number of pole pairs, `>= 1`; the ratio between electrical and mechanical quantities.
    极对数，`>= 1`；电与机械之间所有量的换算比。
    **Unit.** 1 (a count)  **Frame.** machine
    **Positive.** not applicable: it is a positive count.
    **Defined by.** ``include/foc/motor/motor_config.hpp`` (``std::uint8_t pole_pairs;``)

ψf  ``psi_f``
    **Meaning.** rotor (permanent-magnet) flux linkage. Not the torque constant: `psi_f = Kt / (1.5 * pole_pairs)`.
    转子（永磁）磁链。不是转矩常数：`psi_f = Kt / (1.5 * pole_pairs)`。
    **Unit.** Wb  **Frame.** rotor
    **Positive.** positive along the rotor's north axis (the d axis).
    **Defined by.** ``include/foc/core/machine_params.hpp`` (``float psi_f;``)

Kt  ``kt``
    **Meaning.** torque constant, the nameplate number, `psi_f * 1.5 * pole_pairs`.
    转矩常数，铭牌数字，`psi_f * 1.5 * pole_pairs`。
    **Unit.** N*m/A  **Frame.** machine
    **Positive.** positive for positive q-axis current.
    **Defined by.** ``include/foc/motor/motor_config.hpp`` (``float kt;``)

Ld  ``ld``
    **Meaning.** d-axis (rotor flux axis) inductance.
    d 轴（转子磁链轴）电感。
    **Unit.** H  **Frame.** dq, d axis
    **Positive.** not applicable: an inductance.
    **Defined by.** ``include/foc/motor/motor_config.hpp`` (``float ld;``)

Lq  ``lq``
    **Meaning.** q-axis (torque axis) inductance. A surface machine has `Lq == Ld`.
    q 轴（转矩轴）电感。表贴机 `Lq == Ld`。
    **Unit.** H  **Frame.** dq, q axis
    **Positive.** not applicable: an inductance.
    **Defined by.** ``include/foc/motor/motor_config.hpp`` (``float lq;``)

Rs  ``rs``
    **Meaning.** stator resistance, at whatever temperature the caller means -- the plant does not report temperature.
    定子电阻，取调用方所指温度下的值——被控对象不会报告温度。
    **Unit.** ohm  **Frame.** stator
    **Positive.** not applicable: a resistance.
    **Defined by.** ``include/foc/motor/motor_config.hpp`` (``float rs;``)

i_d  ``i_d``
    **Meaning.** d-axis (flux-producing) current.
    d 轴（产生磁链的）电流。
    **Unit.** A  **Frame.** dq, d axis
    **Positive.** positive into the machine, producing flux along the d axis; field weakening is the negative direction.
    **Defined by.** ``include/foc/core/types.hpp`` (``T d{};``)

i_q  ``i_q``
    **Meaning.** q-axis (torque-producing) current.
    q 轴（产生转矩的）电流。
    **Unit.** A  **Frame.** dq, q axis
    **Positive.** positive = motoring in the direction `omega_m` is positive.
    **Defined by.** ``include/foc/core/types.hpp`` (``T q{};``)

v_bus  ``v_bus``
    **Meaning.** DC link voltage. The current loop's output clamp is `+-v_bus/2` per axis, which is not the ceiling the modulator enforces on the pair: that one is the inscribed circle, `v_bus/sqrt(3)`.
    母线电压。电流环输出限幅是每轴 `±v_bus/2`，它不是调制器对矢量对施加的天花板：后者是内切圆 `v_bus/sqrt(3)`。
    **Unit.** V  **Frame.** DC link
    **Positive.** positive.
    **Defined by.** ``include/foc/motor/motor_config.hpp`` (``float bandwidth_rad_s, float v_bus``)

λ  ``lambda``
    **Meaning.** field-weakening closure ratio, `lambda = Ld*i_max/psi_f = 1/ratio`, i.e. the reciprocal of the `ratio` field the C API returns. Field weakening can hold the envelope only when `lambda >= 1`, which is the same statement as `i_char <= i_max`.
    弱磁闭合比，`lambda = Ld·i_max/psi_f = 1/ratio`，即 C API 返回的 `ratio` 字段的倒数。只有 `lambda >= 1` 时弱磁才守得住包络，这与 `i_char <= i_max` 是同一句话。
    **Unit.** 1 (dimensionless)  **Frame.** machine
    **Positive.** positive.
    **Defined by.** ``include/foc/core/field_weakening.hpp`` (``i_char <= i_max``)
    **Note.** Two spellings of one quantity live in the tree, and one of them is inverted. `lambda` (this table, `domain.json`) and `ratio` (`foc_fw_verdict_t.ratio`, `psi_f/Ld / i_max`) are reciprocals, so a threshold written as `lambda >= 1` and one written as `ratio <= 1` are the same rule -- but a reader who takes `ratio` for `lambda` reads the rule backwards. The table states the reciprocal relation rather than picking a winner, because both spellings are already in the public API.
    同一个量在树里有两个写法，而其中一个是倒数。`lambda`（本表、`domain.json`）与 `ratio`（`foc_fw_verdict_t.ratio`，`psi_f/Ld / i_max`）互为倒数，所以写成 `lambda >= 1` 和写成 `ratio <= 1` 是同一条规则——但把 `ratio` 当成 `lambda` 的读者会把规则读反。本表只陈述这个倒数关系，不裁定哪个写法胜出，因为两种写法都已经在公开 API 里。

i_char  ``i_char``
    **Meaning.** characteristic current, `i_char = psi_f/Ld`: the d-axis current that would null the rotor flux.
    特征电流，`i_char = ψf/Ld`：把转子磁链削到零所需的 d 轴电流。
    **Unit.** A  **Frame.** dq, d axis
    **Positive.** reported as a positive magnitude; the current that achieves it is negative.
    **Defined by.** ``include/foc/api/foc_types.h`` (``float i_char_a;``)

δ  ``delta`` *(reserved -- defined for readers, not used by the code yet)*
    **Meaning.** power angle (load angle) between the rotor flux and the stator flux linkage.
    功角：转子磁链与定子磁链之间的夹角。
    **Unit.** rad  **Frame.** rotor vs stator flux
    **Positive.** positive when the stator flux leads the rotor flux.

α, β  ``alpha_beta``
    **Meaning.** the stationary two-phase frame: `alpha` is the phase-u axis, `beta` is 90 deg ahead of it.
    静止两相坐标系：`alpha` 是 u 相轴，`beta` 领先它 90 度。
    **Unit.** 1 (a frame label, not a quantity)  **Frame.** stationary (Clarke output)
    **Positive.** not applicable: these name axes.
    **Defined by.** ``include/foc/core/transforms.hpp`` (``auto clarke(Uvw<T> uvw)``)

τ  ``tau``
    **Meaning.** a time constant, always the one named in the same sentence (observer pole, filter window, thermal lag).
    时间常数，始终是同句里点名的那个（观测器极点、滤波窗、热滞后）。
    **Unit.** s  **Frame.** the subsystem that owns it
    **Positive.** positive.
    **Defined by.** ``include/foc/core/thermal_limit.hpp`` (``τ = Rt·Ct``)

ω_h  ``omega_h``
    **Meaning.** HFI injection angular frequency: the carrier the high-frequency injection puts on the d axis. The reactance that sets the injected current is `omega_h * Ld`.
    高频注入的角频率：注入在 d 轴上的那个载波。决定注入电流的电抗是 `omega_h · Ld`。
    **Unit.** rad/s  **Frame.** electrical
    **Positive.** positive.
    **Defined by.** ``src/api/bridge.cpp`` (``ω_h·Ld``)

φ_inj  ``phi_inj``
    **Meaning.** injection phase, the phase at which a cogging harmonic is injected into the current command.
    注入相位，即把一个齿槽谐波注入电流指令时所用的相位。
    **Unit.** rad  **Frame.** the harmonic's own phase reference
    **Positive.** positive leads.
    **Defined by.** ``src/core/cogging_learn.cpp`` (``k·θ + φ_inj``)

θ_p  ``theta_p``
    **Meaning.** the angle of one phase current in the commissioning projection, `i_p` proportional to `cos(theta_p - angle)`.
    调试投影里某一相电流的相位角，`i_p` 正比于 `cos(θ_p − angle)`。
    **Unit.** rad  **Frame.** phase (stationary)
    **Positive.** positive leads.
    **Defined by.** ``include/foc/core/commissioning.hpp`` (``cos(θ_p − angle)``)

σ  ``sigma``
    **Meaning.** a standard deviation of a noise source, with the subscript naming which one: `sigma_theta` the angle, `sigma_v` the speed derived from it.
    某个噪声源的标准差，下标说明是哪一个：`σ_θ` 是角度，`σ_v` 是由它导出的转速。
    **Unit.** the unit of the subscribed quantity  **Frame.** n/a
    **Positive.** non-negative.
    **Defined by.** ``include/foc/core/speed_from_angle.hpp`` (``σ_v = σ_θ·√2/(window_ticks·dt)``)

η  ``eta_flux``
    **Meaning.** the nonlinear flux observer's correction vector: the part of the estimated linkage that is not the current's own flux.
    非线性磁链观测器的修正矢量：估计磁链里不属于电流自身磁链的那一部分。
    **Unit.** Wb  **Frame.** stationary (alpha-beta)
    **Positive.** a vector, no sign.
    **Defined by.** ``include/foc/core/observer/nonlinear_flux_observer.hpp`` (``(ψf² − |η|²)·η``)

ωL  ``omega_l``
    **Meaning.** an impedance term, `omega * L`, written as one token where the reactance is what the sentence is about. Not a symbol of its own and not a frequency.
    阻抗项 `ω·L`，在句子讲的就是电抗时写成一个记号。它不是独立符号，也不是频率。
    **Unit.** ohm  **Frame.** depends on which L the sentence names
    **Positive.** positive.
    **Defined by.** ``include/foc/core/deadtime.hpp`` (```ωL```)

ψ_ab  ``psi_ab``
    **Meaning.** stator flux linkage vector in the stationary frame -- the integrator state of a flux observer. Distinct from the rotor flux `ψf`, which is a constant of the machine.
    静止坐标系下的定子磁链矢量——磁链观测器的积分器状态。与机器常数转子磁链 `ψf` 不同。
    **Unit.** Wb  **Frame.** alpha-beta (stationary), as a vector
    **Positive.** positive along the winding's own flux direction; a vector, so it carries two components.
    **Defined by.** ``include/foc/core/observer/dual_inductance_observer.hpp`` (``Ab<float> psi_;``)
    **Note.** Registered when the bare `ψ` in this observer's prose turned out to be the last unregistered glyph: the sentence was naming the stator estimate while the table only knew the rotor constant `ψf`.
    登记它的原因：这个观测器的散文里那个裸 `ψ` 是最后一个未登记字形——句子说的是定子估计，而表里只有转子常数 `ψf`。

T_w  ``t_w``
    **Meaning.** winding temperature: what the derating limit is derived from. A drive with no temperature channel reports it as absent rather than as a number.
    绕组温度：降额限值的来源。没有温度通道的驱动把它报成缺席，而不是报成一个数。
    **Unit.** degC  **Frame.** machine
    **Positive.** positive above the ambient the record states.
    **Defined by.** ``include/foc/hal/temperature.hpp`` (``float celsius{0.0F};``)

B  ``beta_ntc``
    **Meaning.** the two-point model's exponent of an NTC, in kelvin: `1/T = 1/T0 + (1/B) ln(R/R0)`. Not a Steinhart-Hart coefficient set; the cost of that choice is stated on the header.
    NTC 两点模型的指数，单位开尔文： `1/T = 1/T0 + (1/B) ln(R/R0)`。不是 Steinhart-Hart 的系数组；这个选择的代价写在头文件里。
    **Unit.** K  **Frame.** sensor
    **Positive.** positive.
    **Defined by.** ``include/foc/core/thermistor.hpp`` (``float beta_k{3950.0F};``)

R_{ntc}  ``r_ntc``
    **Meaning.** the thermistor's own resistance in the divider at the temperature being read. The divider turns it into a code; a code at either rail means the sensor is gone, not that it is very hot or very cold.
    分压里热敏电阻在该温度下的阻值。分压把它变成一个码值；落在任一侧轨的码值说明传感器不在了，而不是它很热或很冷。
    **Unit.** ohm  **Frame.** sensor
    **Positive.** positive.
    **Defined by.** ``include/foc/core/thermistor.hpp`` (``auto resistance_ohm(float celsius)``)

\omega_{ref}  ``omega_ref``
    **Meaning.** the velocity a move trajectory commands this tick, rad/s: a reference, never a measurement.
    运动轨迹本拍指令的速度，rad/s：是参考，从不是测量。
    **Unit.** rad/s  **Frame.** mechanical
    **Positive.** positive in the direction the move is going.
    **Defined by.** ``include/foc/core/move_trajectory.hpp`` (``float velocity_rad_s;``)

\alpha_{ref}  ``alpha_ref``
    **Meaning.** how fast the commanded velocity may change, rad/s^2. It is the same number that appears inside the stopping-distance ceiling, which is why the ceiling is a quadratic root and not a ratio.
    指令速度允许的变化速率，rad/s^2。它同时出现在制动距离上限里，这正是那个上限是二次根而不是比值的原因。
    **Unit.** rad/s^2  **Frame.** mechanical
    **Positive.** magnitude; the sign of the move is carried by the reference.
    **Defined by.** ``include/foc/core/move_trajectory.hpp`` (``float accel_rad_s2;``)

R_{th}  ``rt_th``
    **Meaning.** thermal resistance from the winding to the ambient, K/W: the number that decides how many watts a temperature rise costs.
    绕组到环境的导热热阻，K/W：决定一摄氏度温升要付多少瓦的那个数。
    **Unit.** K/W  **Frame.** thermal
    **Positive.** positive.
    **Defined by.** ``include/foc/core/thermal_limit.hpp`` (``float rt_k_per_w{``)

\alpha_{th}  ``alpha_th``
    **Meaning.** the copper's temperature coefficient of resistance, per kelvin. It is what makes the settle a rational function rather than a product, and what bounds where the settle exists at all.
    铜的电阻温度系数，每开尔文。正是它让稳态温度成为有理式而不是一个乘积，也决定了稳态在什么范围内才存在。
    **Unit.** 1/K  **Frame.** thermal
    **Positive.** positive.
    **Defined by.** ``include/foc/core/thermal_limit.hpp`` (``float alpha_per_k{3.93e-3F};``)
