/* G6 - a *pure C11* consumer that validates the C API end to end: it compiles as C, links, and
 * checks the behaviour at run time. A C header that compiles but fails to link (undefined reference
 * to __cxa_* or to __gcov_*) is the one failure mode no C++ test can catch, which is why this file
 * exists separately from test/unit/test_api_c_smoke.cpp.
 * G6 - **纯 C11** 消费者，端到端校验 C API：按 C 编译、能链接、并在运行期断言行为。C 头"能编
 * 但不能链"（`undefined reference to __cxa_*` / `__gcov_*`）是 C++ 测试永远抓不到的那类故障，
 * 所以这个文件与 `test/unit/test_api_c_smoke.cpp` 分开存在。
 *
 * Nothing C++ is visible here: no C++ include, no template, no hand-written extern "C".
 * 这里看不到任何 C++：没有 C++ 头、没有模板、没有手写 extern "C"。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <foc/api/foc_api.h>

#define CHECK(expr)                                                                                                    \
    do {                                                                                                               \
        if (!(expr)) {                                                                                                 \
            (void)fprintf(stderr, "FAIL: %s (line %d)\n", #expr, __LINE__);                                            \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

int main(void) {
    int failures = 0;
    const foc_motor_params_t params = {0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U};
    foc_motor_t motor = NULL;

    CHECK(foc_motor_create(&params, &motor) == FOC_OK);
    CHECK(motor != NULL);
    CHECK(foc_motor_enable(motor) == FOC_OK);

    /* _Generic dispatch: float literal, double literal and int all reach the library. */
    CHECK(foc_set_speed_g(motor, 50.0F) == FOC_OK);
    CHECK(foc_set_speed_g(motor, 50.0) == FOC_OK);
    CHECK(foc_set_speed_g(motor, 50) == FOC_OK);
    CHECK(foc_set_torque_g(motor, 1.0F) == FOC_OK);
    CHECK(foc_set_torque_g(motor, 1.0) == FOC_OK);

    for (int i = 0; i < 10; ++i) {
        CHECK(foc_tick(motor, 1e-4F) == FOC_OK);
    }

    float speed = 0.0F;
    float id = 0.0F;
    float iq = 0.0F;
    CHECK(foc_get_state(motor, &speed, &id, &iq) == FOC_OK);
    CHECK(isfinite(speed) && isfinite(id) && isfinite(iq));

    /* The error contract, seen from C. */
    CHECK(foc_motor_enable(NULL) == FOC_ERR_PARAM);
    CHECK(foc_tick(motor, -1.0F) == FOC_ERR_PARAM);
    CHECK(foc_get_state(motor, NULL, NULL, NULL) == FOC_ERR_PARAM);
    CHECK(foc_set_torque_f64(motor, (double)NAN) == FOC_ERR_PARAM);

    /* The trace channel from a pure C caller: caller-owned storage, no allocation, and the two
     * half-attached shapes refused instead of guessed at. */
    static foc_trace_sample_t window[16];
    CHECK(foc_motor_trace_size(motor) == 0U); /* nothing attached yet */
    CHECK(foc_motor_set_trace(motor, NULL, 4U) == FOC_ERR_PARAM);
    CHECK(foc_motor_set_trace(motor, window, 0U) == FOC_ERR_PARAM);
    CHECK(foc_motor_set_trace(motor, window, 16U) == FOC_OK);
    for (int i = 0; i < 20; ++i) {
        CHECK(foc_tick(motor, 1e-4F) == FOC_OK);
    }
    CHECK(foc_motor_trace_size(motor) == 16U);
    CHECK(foc_motor_trace_dropped(motor) == 4U); /* 20 periods into a 16-deep ring */

    foc_trace_sample_t first = {0};
    foc_trace_sample_t last = {0};
    CHECK(foc_motor_trace_read(motor, 0U, &first) == FOC_OK);
    CHECK(foc_motor_trace_read(motor, 15U, &last) == FOC_OK);
    CHECK(isfinite(first.i_q) && isfinite(last.speed_mech));

    /* Out of range writes zeros over whatever was there: it does not read past the caller's array, and it does not
     * leave the previous contents behind for the caller to mistake for a sample. */
    foc_trace_sample_t beyond = {-1U, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, 9U, 9U, 9U};
    CHECK(foc_motor_trace_read(motor, 16U, &beyond) == FOC_OK);
    CHECK((beyond.t_us == 0U) && (beyond.i_q == 0.0F) && (beyond.mode == 0U));
    CHECK(foc_motor_trace_read(motor, 0U, NULL) == FOC_ERR_PARAM);
    CHECK(foc_motor_trace_reset(motor) == FOC_OK);
    CHECK(foc_motor_trace_size(motor) == 0U);
    CHECK(foc_motor_trace_dropped(motor) == 4U);           /* already lost is still on the record */
    CHECK(foc_motor_set_trace(motor, NULL, 0U) == FOC_OK); /* detach */
    CHECK(foc_motor_trace_size(motor) == 0U);

    CHECK(foc_motor_disable(motor) == FOC_OK);
    CHECK(foc_motor_destroy(motor) == FOC_OK);
    CHECK(foc_motor_destroy(motor) == FOC_ERR_PARAM); /* stale handle, not a crash */

    /* The optional setup: a pure C caller sizes the struct, sets what it cares about, and can read
     * back exactly what the library applied -- including the fields it left alone. */
    foc_motor_setup_t requested = {0};
    foc_motor_setup_t applied = {0};
    foc_motor_t tuned = NULL;

    requested.struct_size = (uint32_t)sizeof(foc_motor_setup_t);
    requested.i_max_a = 2.0F;
    requested.v_bus_v = 48.0F;

    CHECK(foc_motor_create_ex(&params, &requested, &tuned) == FOC_OK);
    CHECK(foc_motor_get_setup(tuned, &applied) == FOC_OK);
    CHECK(applied.struct_size == (uint32_t)sizeof(foc_motor_setup_t));
    CHECK(applied.i_max_a == 2.0F);
    CHECK(applied.v_bus_v == 48.0F);
    CHECK(applied.pwm_freq_hz == 20000.0F); /* untouched field keeps the library default */
    CHECK(foc_motor_get_setup(NULL, &applied) == FOC_ERR_PARAM);
    CHECK(foc_motor_destroy(tuned) == FOC_OK);

    /* Position moves and status: the C face reaches the same kernel the C++ face does, and the
     * `_Generic` macro keeps an integer literal from narrowing silently at the call site. */
    foc_motor_status_t status = {0};
    foc_motor_command_t command = {0};
    foc_motor_t mover = NULL;

    CHECK(foc_motor_create(&params, &mover) == FOC_OK);
    CHECK(foc_set_position_g(mover, 0.1) == FOC_OK);  /* routed to the double entry point */
    CHECK(foc_set_position_g(mover, 0.2F) == FOC_OK); /* routed to the float one */
    CHECK(foc_set_position(mover, 0.3F) == FOC_OK);

    command.position_rad = 0.4F;
    command.max_current_a = 1.0F;
    command.feedforward_current_a = 0.05F;
    command.max_velocity_rad_s = 2.0F; /* this move's own ceilings */
    command.max_accel_rad_s2 = 10.0F;
    CHECK(foc_set_command(mover, &command) == FOC_OK);
    CHECK(foc_set_command(mover, NULL) == FOC_ERR_PARAM);

    CHECK(foc_motor_get_status(mover, &status) == FOC_OK);
    CHECK(status.struct_size == (uint32_t)sizeof(foc_motor_status_t));
    CHECK(status.mode_state == (int32_t)FOC_MOTOR_IDLE);  /* created but not enabled yet */
    CHECK(status.align_state == (int32_t)FOC_ALIGN_IDLE); /* alignment was never requested */
    CHECK(status.error == (int32_t)FOC_OK);

    /* An alignment recipe is part of the setup; the sequence then runs from foc_tick(). */
    requested.align_current_a = 0.5F;
    requested.align_hold_s = 0.005F;
    requested.align_tolerance_rad = 0.05F;
    CHECK(foc_motor_create_ex(&params, &requested, &tuned) == FOC_OK);
    CHECK(foc_motor_enable(tuned) == FOC_OK);
    CHECK(foc_motor_get_status(tuned, &status) == FOC_OK);
    CHECK(status.mode_state == (int32_t)FOC_MOTOR_ALIGN);
    CHECK(foc_tick(tuned, 1.0e-4F) == FOC_OK);
    CHECK(foc_motor_destroy(tuned) == FOC_OK);

    CHECK(foc_motor_destroy(mover) == FOC_OK);

    /* ----------------------------------------------------------------------------------------
     * ABI 0.1.0: the modules a C consumer could not reach at all before.
     * ABI 0.1.0：在此之前 C 消费方完全够不到的那些模块。
     * -------------------------------------------------------------------------------------- */

    /* The per-phase / per-axis dead-time convention. A 1 us dead time at 20 kHz on 24 V is
     * 0.48 V per phase; on the d axis it is (4/3) of that, on q (2/sqrt(3)). Getting this
     * wrong is a 33 % error that reads like a dead-time fault on a good board, so the two
     * numbers are pinned here rather than trusted. */
    CHECK(fabsf(foc_axis_deadtime_v(1.0e-6F, 20000.0F, 24.0F, 0.0F) - 0.64F) < 0.01F);
    CHECK(fabsf(foc_axis_deadtime_v(1.0e-6F, 20000.0F, 24.0F, 3.14159265F) - 0.64F) < 0.01F);
    CHECK(fabsf(foc_axis_deadtime_v(1.0e-6F, 20000.0F, 24.0F, 1.57079632F) - 0.554F) < 0.01F);
    CHECK(foc_axis_deadtime_v(NAN, 20000.0F, 24.0F, 0.0F) == 0.0F);

    /* Field weakening is not worth it on this surface machine: i_char/i_max is 3.97. */
    foc_fw_verdict_t verdict;
    verdict.struct_size = (uint32_t)sizeof(verdict);
    CHECK(foc_field_weakening_worthwhile(&params, 3.0F, &verdict) == FOC_OK);
    CHECK(verdict.worthwhile == 0);
    CHECK(verdict.ratio > 3.0F);
    /* A wrong struct_size is rejected before anything is written. */
    verdict.struct_size = 4U;
    CHECK(foc_field_weakening_worthwhile(&params, 3.0F, &verdict) == FOC_ERR_PARAM);

    /* MTPA on a surface machine is exactly d = 0, not "close to zero". */
    float mtpa_id = 1.0F;
    float mtpa_iq = 0.0F;
    CHECK(foc_mtpa_reference(&params, 3.0F, 1.0F, &mtpa_id, &mtpa_iq) == FOC_OK);
    CHECK(mtpa_id == 0.0F);
    CHECK(fabsf(mtpa_iq - 1.0F) < 1e-6F);
    CHECK(foc_mtpa_reference(&params, 3.0F, 1.0F, NULL, &mtpa_iq) == FOC_ERR_PARAM);

    /* The board checklist: a good board passes, an inverted phase names the cause. */
    foc_board_measurements_t measured;
    foc_board_limits_t limits;
    foc_board_report_t report;
    memset(&measured, 0, sizeof(measured));
    memset(&limits, 0, sizeof(limits));
    memset(&report, 0, sizeof(report));
    measured.struct_size = (uint32_t)sizeof(measured);
    limits.struct_size = (uint32_t)sizeof(limits);
    report.struct_size = (uint32_t)sizeof(report);
    limits.sum_current_a = 0.05F;
    limits.offset_a = 0.02F;
    limits.ripple_ratio = 1.5F;
    limits.deadtime_ratio = 0.25F;
    limits.encoder_offset_rad = 0.02F;
    limits.parameter_ratio = 0.10F;
    limits.observer_error_rad = 0.10F;

    /* A +d drive puts `i_u` positive and `i_v`, `i_w` negative. Asserting all three positive is
     * a *fault*, and the report's `limit` column carries the expected sign, so the mistake is
     * visible rather than silent. */
    measured.response_sign[0] = 1.0F;
    measured.response_sign[1] = -1.0F;
    measured.response_sign[2] = -1.0F;
    measured.ripple_measured_a = 0.30F;
    measured.ripple_expected_a = 0.375F;
    measured.deadtime_v = 0.640F;
    measured.deadtime_cmd_v = 0.640F;
    measured.rs_ohm = 0.500F;
    measured.rs_nameplate_ohm = 0.500F;
    measured.psi_f_wb = 4.762e-3F;
    measured.psi_f_nameplate_wb = 4.762e-3F;
    measured.observer_error_rad = 0.04F;
    measured.encoder_offset_rad = 0.005F;

    CHECK(foc_board_check(&measured, &limits, &report) == FOC_OK);
    CHECK(report.count > 0);
    CHECK(report.failures == 0);
    CHECK(report.cause == FOC_BOARD_CAUSE_NONE);
    CHECK(foc_board_item_name(FOC_BOARD_ITEM_CURRENT_SUM) != NULL);
    CHECK(strcmp(foc_board_cause_name(FOC_BOARD_CAUSE_NONE), "none") == 0);
    CHECK(strcmp(foc_board_cause_name(99), "?") == 0);

    /* An inverted phase u: the three-wire sum goes negative and the cause names polarity. */
    measured.response_sign[0] = -1.0F;
    measured.sum_current_a = 0.4F;
    CHECK(foc_board_check(&measured, &limits, &report) == FOC_OK);
    CHECK(report.failures > 0);
    CHECK(report.cause == FOC_BOARD_CAUSE_POLARITY_REVERSED);

    /* The protector: create, over-current, latch, warm reset keeps the I^2t memory. */
    foc_protector_t protector = NULL;
    CHECK(foc_protector_create(NULL, &protector) == FOC_OK);
    CHECK(protector != NULL);

    foc_protection_input_t pin;
    foc_protection_output_t pout;
    memset(&pin, 0, sizeof(pin));
    memset(&pout, 0, sizeof(pout));
    pin.struct_size = (uint32_t)sizeof(pin);
    pout.struct_size = (uint32_t)sizeof(pout);
    pin.v_bus_v = 24.0F;
    pin.temperature_c = 25.0F;
    pin.flux_valid = 1;

    pin.i_magnitude_a = 0.5F;
    CHECK(foc_protector_step(protector, &pin, 5.0e-5F, &pout) == FOC_OK);
    CHECK(pout.fault == FOC_FAULT_NONE);
    CHECK(pout.output_allowed != 0);

    pin.i_magnitude_a = 5.0F;
    CHECK(foc_protector_step(protector, &pin, 5.0e-5F, &pout) == FOC_OK);
    CHECK(pout.fault == FOC_FAULT_OVER_CURRENT);
    CHECK(pout.output_allowed == 0);

    /* Over-current is latched: taking the current away does not clear it. */
    pin.i_magnitude_a = 0.1F;
    CHECK(foc_protector_step(protector, &pin, 5.0e-5F, &pout) == FOC_OK);
    CHECK(pout.fault == FOC_FAULT_OVER_CURRENT);
    CHECK(pout.latched_mask != 0U);

    CHECK(foc_protector_reset(protector, 0) == FOC_OK);
    CHECK(foc_protector_step(protector, &pin, 5.0e-5F, &pout) == FOC_OK);
    CHECK(pout.fault == FOC_FAULT_NONE);

    /* A *live* motor handle is not a protector handle: the two kinds of token carry different
     * magics, so this is a clean FOC_ERR_STATE instead of reading a Protector out of a Motor's
     * bytes. A stale handle would pass this by accident, so a fresh motor is created for it. */
    foc_motor_t live = NULL;
    CHECK(foc_motor_create(&params, &live) == FOC_OK);
    CHECK(foc_protector_step((foc_protector_t)live, &pin, 5.0e-5F, &pout) == FOC_ERR_STATE);
    CHECK(foc_protector_destroy((foc_protector_t)live) == FOC_ERR_STATE);
    CHECK(foc_motor_destroy(live) == FOC_OK);

    CHECK(foc_protector_destroy(protector) == FOC_OK);
    /* The handle is dead afterwards, even if the slot is reused. */
    CHECK(foc_protector_step(protector, &pin, 5.0e-5F, &pout) == FOC_ERR_STATE);

    /* The breakaway sequence, from C: what this file can assert is the refusal half, because every assertion
     * here has to hold whatever state the axis happens to be in -- `enable()` puts it in Align when the build
     * carries an aligner, and a test that assumed `Run` would be measuring the aligner instead. The positive path
     * (a rotor that really breaks loose at a named torque) is measured in `test_sil_stall_start_motor.cpp`.
     * 起步序列从 C 看：本文件能断言的是拒绝那一半，因为这里的每条断言都必须在轴处于任何状态时都成立——
     * 当构建带对齐器时 `enable()` 会把轴放进 Align，假设 `Run` 的用例测的就是对齐器了。正向路径
     * （转子真的在一个可命名的力矩上挣脱）由 `test_sil_stall_start_motor.cpp` 量化。 */
    {
        /* A **fresh** handle, and this is the whole point of the block: `motor` was destroyed at the top of this file,
         * so every `== FOC_ERR_PARAM` below would have passed for the wrong reason -- a stale handle answers PARAM to
         * everything, and the two positive assertions were the only ones that could tell. They did: this block was
         * written against `motor` and `c_api_validation` failed on exactly those two lines. A refusal that a dead
         * handle also produces is not evidence about the API.
         * 需要一个 **新句柄** ，而这正是这一段的意义：`motor` 在本文件开头就销毁了，于是下面每一行 `== FOC_ERR_PARAM`
         * 都会因为错误的原因通过——失效句柄对一切问题都答 PARAM，只有那两条正向断言能分辨。它们确实分辨出来了：这一段
         * 原先是用 `motor` 写的，`c_api_validation` 恰好在这两行上失败。一个失效句柄也会产生的拒绝，不是关于 API 的
         * 证据。 */
        foc_motor_t seq = NULL;
        /* Zero-initialised at the declaration, and that is not defensive tidiness: AppleClang's
         * -Wuninitialized-const-pointer rejects passing an uninitialised object's address even when the callee is
         * required to refuse it, and the first call below passes `&recipe` before the memset that used to follow. The
         * case being made -- "struct_size of zero is refused" -- is unchanged, since zero is exactly the value it is
         * asserting on. 在声明处零初始化，这不是防御性的整洁：AppleClang 的 -Wuninitialized-const-pointer 拒绝把未初始化
         * 对象的地址传出去，即使被调方必须拒绝它；而下面第一个调用在原先那次 memset 之前就传了 `&recipe`。要断言的
         * 「struct_size 为 0 被拒绝」一点没变，因为 0 正是它断言的值。 */
        foc_stall_start_config_t recipe = {0};
        foc_stall_start_status_t status = {0};
        CHECK(foc_motor_create(&params, &seq) == FOC_OK);
        CHECK(foc_motor_begin_stall_start(NULL, &recipe) == FOC_ERR_PARAM);
        CHECK(foc_motor_begin_stall_start(seq, NULL) == FOC_ERR_PARAM);
        (void)memset(&recipe, 0, sizeof(recipe));
        CHECK(foc_motor_begin_stall_start(seq, &recipe) == FOC_ERR_PARAM); /* struct_size not set */
        (void)memset(&status, 0, sizeof(status));
        CHECK(foc_motor_get_stall_start(seq, &status) == FOC_ERR_PARAM); /* struct_size not set */
        status.struct_size = (uint32_t)sizeof(status);
        CHECK(foc_motor_get_stall_start(seq, &status) == FOC_OK);
        CHECK(status.phase == FOC_STALL_IDLE);
        CHECK(status.torque_nm == 0.0F);
        CHECK(status.limited == 0);
        CHECK(foc_motor_cancel_stall_start(seq) == FOC_OK);
        CHECK(foc_motor_cancel_stall_start(NULL) == FOC_ERR_PARAM);
        CHECK(foc_motor_destroy(seq) == FOC_OK);
    }

    /*
     * The tuner's C surface: a session is a *procedure*, so the C side proves both halves -- that it refuses what it
     * cannot do, and that a window generated from a known machine comes back with that machine's ratios. The second
     * half needs no plant: `d(omega)/dt = a*iq - d*omega` is stepped in closed form, so the answer is checkable.
     */
    {
        foc_autotune_config_t tc;
        foc_autotune_t tune = NULL;
        foc_autotune_result_t tr;
        foc_autotune_t first = NULL;
        foc_autotune_t second = NULL;
        const float a_expected = 500.0F; /* kt/J */
        const float d_expected = 1.0F;   /* b/J  */
        const float step_s = 1.0e-3F;

        memset(&tc, 0, sizeof(tc));
        tc.struct_size = (uint32_t)sizeof(tc);
        tc.kt_nm_per_a = 0.05F;
        tc.j_expected_kg_m2 = 1.0e-4F;
        tc.b_expected_nm_s = 1.0e-4F;
        tc.bandwidth_rad_s = 200.0F;
        tc.i_max_a = 3.0F;
        tc.tolerance = 0.25F;
        tc.min_current_a = 0.05F;
        tc.max_residual = 0.35F;
        tc.min_samples = 100;

        /* --- the refusal half ------------------------------------------------------------------------- */
        CHECK(foc_autotune_create(NULL, &tune) == FOC_ERR_PARAM);
        CHECK(foc_autotune_create(&tc, NULL) == FOC_ERR_PARAM);
        {
            foc_autotune_config_t uncovered = tc;
            uncovered.struct_size = 0U; /* an older caller that never filled the field */
            CHECK(foc_autotune_create(&uncovered, &tune) == FOC_ERR_PARAM);
        }
        {
            foc_autotune_config_t hopeless = tc;
            hopeless.bandwidth_rad_s = 0.0F; /* no bandwidth, no gains: the library's own predicate refuses it */
            CHECK(foc_autotune_create(&hopeless, &tune) == FOC_ERR_PARAM);
        }
        /* Every slot, then one more: the third create has nowhere to live and says so. */
        CHECK(foc_autotune_create(&tc, &first) == FOC_OK);
        CHECK(foc_autotune_create(&tc, &second) == FOC_OK);
        CHECK(foc_autotune_create(&tc, &tune) == FOC_ERR_NO_RESOURCE);
        /* A stale handle is a state error, not a crash and not a silent success. */
        CHECK(foc_autotune_destroy(second) == FOC_OK);
        CHECK(foc_autotune_accumulate(second, step_s, 1.0F, 0.0F, 0) == FOC_ERR_STATE);
        CHECK(foc_autotune_destroy(first) == FOC_OK);

        /* --- the measurement half --------------------------------------------------------------------- */
        CHECK(foc_autotune_create(&tc, &tune) == FOC_OK);
        {
            float omega = 0.0F;
            int k;
            for (k = 0; k < 600; ++k) {
                /* A current that alternates so the fit has a slope to identify, and a speed that follows the model. */
                const float iq = ((k / 100) % 2 == 0) ? 1.0F : 1.5F;
                const float next = omega + ((a_expected * iq) - (d_expected * omega)) * step_s;
                CHECK(foc_autotune_accumulate(tune, step_s, iq, omega, 0) == FOC_OK);
                omega = next;
            }
        }
        memset(&tr, 0, sizeof(tr));
        tr.struct_size = (uint32_t)sizeof(tr);
        CHECK(foc_autotune_finish(tune, &tr) == FOC_OK);
        CHECK(tr.status == (int32_t)FOC_OK);
        CHECK(tr.valid == 1);
        CHECK(fabsf(tr.kt_over_j - a_expected) < 0.01F * a_expected);
        CHECK(fabsf(tr.b_over_j - d_expected) < 0.05F * d_expected);
        CHECK(fabsf(tr.j_kg_m2 - 1.0e-4F) < 0.01F * 1.0e-4F);
        /* kp = bandwidth/(kt/J): the ratio the fit produced, not the nameplate's. */
        CHECK(fabsf(tr.kp_a_per_rad_s - (200.0F / a_expected)) < 0.01F * (200.0F / a_expected));
        CHECK(tr.reason != NULL);
        CHECK(foc_autotune_reset(tune) == FOC_OK);
        CHECK(foc_autotune_destroy(tune) == FOC_OK);
        CHECK(foc_autotune_destroy(tune) == FOC_ERR_STATE); /* twice: the second is a stale token */
    }

    /* --- cogging learning: a procedure whose answer is a TABLE ---------------- */
    {
        foc_cogging_learn_config_t cc;
        foc_cogging_learn_t learn = NULL;
        foc_cogging_learn_t first = NULL;
        foc_cogging_learn_t second = NULL;
        foc_cogging_learn_result_t cr;
        foc_cogging_harmonic_t table[2];
        int32_t ready = -1;
        /* One harmonic a compensator could cancel: order 6, 4 mN*m at 0.3 rad, on a machine whose
         * kt_eff = 1.5*p*psi_f = 1.5*4*0.1 = 0.6 N*m/A. */
        const uint8_t order = 6U;
        const float amplitude_nm = 0.004F;
        const float phase_rad = 0.3F;
        const float kt_eff = 0.6F;
        const float omega = 5.0F; /* inside [0.5, 10] */
        const int periods = 6000; /* 3 revolutions at 2000 samples per revolution */

        memset(&cc, 0, sizeof(cc));
        cc.struct_size = (uint32_t)sizeof(cc);
        cc.psi_f = 0.1F;
        cc.ld = 1.0e-3F;
        cc.lq = 1.0e-3F;
        cc.pole_pairs = 4U;
        cc.orders[0] = order;
        cc.count = 1U;
        cc.revolutions = 2;
        cc.min_speed_rad_s = 0.5F;
        cc.max_speed_rad_s = 10.0F;
        cc.speed_tolerance = 0.10F;

        /* --- the refusal half ------------------------------------------------------ */
        CHECK(foc_cogging_learn_create(NULL, &learn) == FOC_ERR_PARAM);
        CHECK(foc_cogging_learn_create(&cc, NULL) == FOC_ERR_PARAM);
        {
            foc_cogging_learn_config_t uncovered = cc;
            uncovered.struct_size = 0U;
            CHECK(foc_cogging_learn_create(&uncovered, &learn) == FOC_ERR_PARAM);
        }
        {
            foc_cogging_learn_config_t empty = cc;
            empty.count = 0U; /* no orders asked for: nothing to learn */
            CHECK(foc_cogging_learn_create(&empty, &learn) == FOC_ERR_PARAM);
        }
        {
            foc_cogging_learn_config_t dc = cc;
            dc.orders[0] = 0U; /* order 0 is a load, not cogging */
            CHECK(foc_cogging_learn_create(&dc, &learn) == FOC_ERR_PARAM);
        }
        {
            foc_cogging_learn_config_t inverted = cc;
            inverted.max_speed_rad_s = inverted.min_speed_rad_s; /* an empty band */
            CHECK(foc_cogging_learn_create(&inverted, &learn) == FOC_ERR_PARAM);
        }
        /* Every slot, then one more. */
        CHECK(foc_cogging_learn_create(&cc, &first) == FOC_OK);
        CHECK(foc_cogging_learn_create(&cc, &second) == FOC_OK);
        CHECK(foc_cogging_learn_create(&cc, &learn) == FOC_ERR_NO_RESOURCE);
        CHECK(foc_cogging_learn_destroy(second) == FOC_OK);
        /* A stale handle is a state error on every entry, not a crash. The result struct is filled in first:
         * `struct_size` is the caller's promise about its own header, and an unwritten one is a caller bug that
         * answers FOC_ERR_PARAM no matter what the handle is. */
        memset(&cr, 0, sizeof(cr));
        cr.struct_size = (uint32_t)sizeof(cr);
        CHECK(foc_cogging_learn_accumulate(second, 0.0F, 0.0F, 0.0F, omega, 0) == FOC_ERR_STATE);
        CHECK(foc_cogging_learn_ready(second, &ready) == FOC_ERR_STATE);
        CHECK(foc_cogging_learn_finish(second, table, 2, &cr) == FOC_ERR_STATE);
        {
            /* The other way round: an uncovered struct_size is a param error even on a live handle. */
            foc_cogging_learn_result_t uncovered;
            memset(&uncovered, 0, sizeof(uncovered));
            uncovered.struct_size = 0U;
            CHECK(foc_cogging_learn_finish(first, table, 2, &uncovered) == FOC_ERR_PARAM);
        }
        CHECK(foc_cogging_learn_reset(second) == FOC_ERR_STATE);
        CHECK(foc_cogging_learn_destroy(second) == FOC_ERR_STATE);

        /* A report-only call on an empty window: the report comes back complete and says the window is
         * the problem, not the (absent) buffer. */
        CHECK(foc_cogging_learn_finish(first, NULL, 0, &cr) == FOC_ERR_STATE);
        CHECK(cr.status == (int32_t)FOC_ERR_STATE);
        CHECK(cr.count == 0);
        CHECK(cr.required == 1);
        CHECK(cr.capacity == 0);
        /* A sample that is not a number is not taken, and says so. */
        CHECK(foc_cogging_learn_accumulate(first, 0.0F, (float)NAN, 0.0F, omega, 0) == FOC_ERR_PARAM);
        CHECK(foc_cogging_learn_ready(first, &ready) == FOC_OK);
        CHECK(ready == 0); /* the NaN did not grow the window */
        CHECK(foc_cogging_learn_destroy(first) == FOC_OK);

        /* --- the measurement half -------------------------------------------------- */
        CHECK(foc_cogging_learn_create(&cc, &learn) == FOC_OK);
        {
            int k;
            for (k = 0; k < periods; ++k) {
                const float theta = (6.28318530718F * 3.0F) * ((float)k / (float)periods);
                /* The loop supplies the current that produces -T_cog, so the q-axis current runs a
                 * half period behind the torque and scaled by kt_eff. */
                const float iq = (amplitude_nm / kt_eff) * sinf((((float)order) * theta) + phase_rad + 3.14159265358979323846F);
                CHECK(foc_cogging_learn_accumulate(learn, theta, iq, 0.0F, omega, 0) == FOC_OK);
            }
        }
        CHECK(foc_cogging_learn_ready(learn, &ready) == FOC_OK);
        CHECK(ready == 1);
        /* Report only: the window is good, and `count == 0` says "nothing was written", not "refused". */
        memset(&cr, 0, sizeof(cr));
        cr.struct_size = (uint32_t)sizeof(cr);
        CHECK(foc_cogging_learn_finish(learn, NULL, 0, &cr) == FOC_OK);
        CHECK(cr.status == (int32_t)FOC_OK);
        CHECK(cr.count == 0);
        CHECK(cr.required == 1);
        CHECK(cr.capacity == 0);
        CHECK(cr.revolutions >= 2.0F);
        CHECK(cr.speed_min_rad_s > 4.9F && cr.speed_max_rad_s < 5.1F);
        /* A buffer too small is the CALLER's problem, and the window is still reported honestly. */
        CHECK(foc_cogging_learn_finish(learn, table, 0, &cr) == FOC_ERR_PARAM);
        CHECK(cr.count == 0);
        CHECK(cr.required == 1);
        CHECK(cr.capacity == 0);
        /* The table itself: the learned harmonic must come back as the torque that was injected, with the
         * compensator's sign, i.e. a positive amplitude at the phase it was injected with. */
        memset(table, 0, sizeof(table));
        CHECK(foc_cogging_learn_finish(learn, table, 2, &cr) == FOC_OK);
        CHECK(cr.status == (int32_t)FOC_OK);
        CHECK(cr.count == 1);
        CHECK(cr.required == 1);
        CHECK(cr.capacity == 2);
        CHECK(table[0].order == (uint32_t)order);
        CHECK(fabsf(table[0].amplitude_nm - amplitude_nm) < 0.01F * amplitude_nm);
        CHECK(fabsf(table[0].phase_rad - phase_rad) < 0.01F);
        /* A second window answers the same question, so a re-learn is not a new answer. */
        CHECK(foc_cogging_learn_reset(learn) == FOC_OK);
        CHECK(foc_cogging_learn_ready(learn, &ready) == FOC_OK);
        CHECK(ready == 0);
        CHECK(foc_cogging_learn_destroy(learn) == FOC_OK);
        CHECK(foc_cogging_learn_destroy(learn) == FOC_ERR_STATE); /* twice */
    }

    /* --- The machine-readable command surface, exercised as a pure C link. ---
     * Ten registers, framed, sent through a loopback that is nothing but a memcpy, decoded again: the point is not that
     * a codec round-trips its own output (it does) but that a C host can reach all of it without a single C++ symbol.
     * --- 机器可读的命令面，作为纯 C 链接来走一遍。十个寄存器、封装成帧、经一个只是 memcpy 的环回、再解回来：要点不是
     * 编解码器能还原自己的输出（那当然可以），而是 C 宿主不碰任何 C++ 符号就能走到全部这些。 --- */
    {
        const uint32_t count = foc_reg_count();
        CHECK(count == 10U);

        uint8_t payload[10U * 4U];
        uint32_t written = 0U;
        uint32_t ok_reads = 0U;
        uint32_t index;
        for (index = 0U; index < count; ++index) {
            uint32_t word = 0U;
            const foc_err_t got = foc_reg_read(motor, index, &word);
            CHECK((got == FOC_OK) || (got == FOC_ERR_STATE));
            if (got == FOC_OK) {
                ++ok_reads;
            }
            /* Little-endian by construction, not by trusting the host: this is the wire order the decoder reassembles. */
            /* 按构造用小端，而不是相信主机字节序：这就是解码方重新拼装的线上顺序。 */
            payload[written + 0U] = (uint8_t)(word & 0xFFU);
            payload[written + 1U] = (uint8_t)((word >> 8U) & 0xFFU);
            payload[written + 2U] = (uint8_t)((word >> 16U) & 0xFFU);
            payload[written + 3U] = (uint8_t)((word >> 24U) & 0xFFU);
            written += 4U;
        }
        /* At least one register has something to say about a motor that was created and enabled; the rest are allowed
         * to answer STATE, because a register whose source has not run yet is a state and not a bug.
         * 至少有一项寄存器对一台已创建并使能的电机有话可说；其余允许答 STATE，因为来源还没跑过的寄存器是一种状态，
         * 不是缺陷。 */
        CHECK(ok_reads > 0U);

        /* Out of range is a caller bug, a NULL receiver likewise; neither touches the word. */
        /* 越界是调用方的错误，接收者为 NULL 同理；两者都不碰那个字。 */
        {
            uint32_t guard = 0xDEADBEEFU;
            CHECK(foc_reg_read(motor, count, &guard) == FOC_ERR_PARAM);
            CHECK(guard == 0xDEADBEEFU);
            CHECK(foc_reg_read(motor, 0U, NULL) == FOC_ERR_PARAM);
        }

        uint8_t wire[64];
        const uint32_t frame_size = foc_frame_encode(7U, 0x42U, payload, (uint8_t)written, wire, (uint32_t)sizeof wire);
        CHECK(frame_size == (uint32_t)(written + 6U));

        /* A buffer one byte too small writes nothing at all, so a caller that hands the answer to write() cannot send
         * half a frame. */
        /* 小一个字节的缓冲区一个字节都不写，于是把答案交给 write() 的调用方不可能发出半帧。 */
        {
            uint8_t small[8];
            CHECK(foc_frame_encode(7U, 0x42U, payload, (uint8_t)written, small, (uint32_t)sizeof small) == 0U);
        }

        foc_frame_t decoded;
        (void)memset(&decoded, 0, sizeof decoded);
        CHECK(foc_frame_decode(wire, frame_size, &decoded) == FOC_FRAME_OK);
        CHECK(decoded.seq == 7U);
        CHECK(decoded.type == 0x42U);
        CHECK(decoded.length == (uint8_t)written);
        CHECK(decoded.payload != NULL);
        for (index = 0U; index < written; ++index) {
            CHECK(decoded.payload[index] == payload[index]);
        }

        /* Every single-byte change is refused, and the receiver is left exactly as the caller had it -- so a host that
         * keeps the last good frame keeps a good frame, not a two-thirds-written one. */
        /* 任何单字节改动都被拒绝，且接收者保持调用方留下时的样子——于是保留上一帧的宿主保留的是一帧好帧，而不是写了三分
         * 之二的帧。 */
        {
            uint32_t byte;
            for (byte = 0U; byte < frame_size; ++byte) {
                uint8_t tampered[64];
                (void)memcpy(tampered, wire, frame_size);
                tampered[byte] = (uint8_t)(tampered[byte] ^ 0x01U);
                static const uint8_t sentinel_byte = 0U;
                foc_frame_t keep;
                (void)memset(&keep, 0x5A, sizeof keep);
                keep.payload = &sentinel_byte;
                const foc_frame_status_t status = foc_frame_decode(tampered, frame_size, &keep);
                CHECK(status != FOC_FRAME_OK);
                CHECK(keep.seq == 0x5AU);
                CHECK(keep.type == 0x5AU);
                CHECK(keep.length == 0x5AU);
                CHECK(keep.payload == &sentinel_byte);
            }
        }

        /* And the three shapes that are not a frame at all are named, not folded into one error. */
        /* 而三种"根本不是帧"的形状各有名字，不并成一个错误。 */
        CHECK(foc_frame_decode(wire, 3U, &decoded) == FOC_FRAME_TOO_SHORT);
        {
            uint8_t wrong_start[8];
            (void)memcpy(wrong_start, wire, 8U);
            wrong_start[0] = 0x00U;
            CHECK(foc_frame_decode(wrong_start, 8U, &decoded) == FOC_FRAME_BAD_START);
            CHECK(foc_frame_decode(wire, 8U, &decoded) == FOC_FRAME_BAD_LENGTH);
        }
    }

    /* The drive tick's order, from C: one call decides, and the caller is not asked to act on a verdict.
     * Two motors, because the two answers worth proving are opposite: a bus inside its band keeps the loop running,
     * and a bus outside it stops the loop on the tick that saw it.
     * 驱动器拍的顺序，从 C 出发：一次调用就做出决定，而调用方不被要求去处置判决。两台电机，因为值得证明的两个回答相反：
     * 母线在区间内时环继续跑，母线在区间外时在看见它的那一拍就停。
     */
    {
        foc_supervisor_config_t config;
        (void)memset(&config, 0, sizeof(config));
        config.struct_size = (uint32_t)sizeof(config);
        config.protection.struct_size = (uint32_t)sizeof(config.protection);
        config.protection.i_hard_a = 4.0F;
        config.protection.i_max_a = 3.0F;
        config.protection.i2t_a2s = 1.0F;
        config.protection.i2t_cooling_s = 10.0F;
        config.protection.v_max_v = 30.0F;
        config.protection.v_min_v = 18.0F;
        config.protection.t_trip_c = 110.0F;
        config.protection.t_derate_start_c = 60.0F;
        config.protection.t_derate_end_c = 80.0F;
        config.protection.stall_speed_rad_s = 1.0F;
        config.protection.stall_current_a = 2.0F;
        config.protection.stall_time_s = 1.0F;
        config.protection.sync_error_ratio = 0.5F;
        config.protection.hysteresis_ratio = 0.05F;
        config.chopper.struct_size = (uint32_t)sizeof(config.chopper);
        config.chopper.v_on_v = 28.0F;
        config.chopper.v_off_v = 26.0F;
        config.chopper.resistance_ohm = 2.2F;
        config.chopper.duty_max = 0.95F;
        config.chopper.min_on_s = 1e-4F;
        config.chopper.min_off_s = 1e-4F;
        config.pole_pairs = 7.0F;

        /* A band that cannot be a band is refused, and no handle is minted. */
        {
            foc_supervisor_config_t bad = config;
            foc_supervisor_t none = NULL;
            bad.protection.v_min_v = 40.0F; /* above its own v_max_v */
            CHECK(foc_supervisor_create(motor, &bad, &none) == FOC_ERR_PARAM);
            CHECK(none == NULL);
        }

        /* Inside the band: the arm appears in `resolved.v_bus_v` (24 V by default) and the stub DC link reports it,
         * so the band is opened down to 0 V to keep the loop alive whatever that setup says.
         * 区间内：母线电压来自 `resolved.v_bus_v`（默认 24 V），由桩直流母线报出来；所以把区间下限开到 0 V，
         * 不管那份 setup 说什么，环都活在里面。 */
        {
            foc_supervisor_config_t inside = config;
            foc_motor_t second_motor = NULL;
            foc_supervisor_t supervisor = NULL;
            foc_supervisor_status_t status;

            inside.protection.v_min_v = 0.0F;
            CHECK(foc_motor_create(&params, &second_motor) == FOC_OK);
            CHECK(foc_supervisor_create(second_motor, &inside, &supervisor) == FOC_OK);
            CHECK(supervisor != NULL);
            {
                foc_supervisor_t another = NULL;
                CHECK(foc_supervisor_create(second_motor, &inside, &another) == FOC_ERR_STATE);
                CHECK(another == NULL);
            }
            CHECK(foc_supervisor_enable(supervisor) == FOC_OK);
            for (int i = 0; i < 10; ++i) {
                CHECK(foc_supervisor_step(supervisor, 1e-4F) == FOC_OK);
            }
            (void)memset(&status, 0, sizeof(status));
            status.struct_size = (uint32_t)sizeof(status);
            CHECK(foc_supervisor_status(supervisor, &status) == FOC_OK);
            CHECK(status.tick == 10U);
            CHECK(status.fault == FOC_FAULT_NONE);
            CHECK(status.output_allowed == 1);
            CHECK(foc_supervisor_disable(supervisor) == FOC_OK);
            CHECK(foc_supervisor_destroy(supervisor) == FOC_OK);
            /* The handle is dead afterwards: stale is a state error, not undefined behaviour. */
            CHECK(foc_supervisor_step(supervisor, 1e-4F) == FOC_ERR_STATE);
            CHECK(foc_supervisor_destroy(supervisor) == FOC_ERR_STATE);
            CHECK(foc_motor_destroy(second_motor) == FOC_OK);
        }

        /* Outside the band: the same 24 V bus against a 30 V floor, so the first tick stops the loop and says why.
         * The band is moved rather than the bus, because the bus on this bench is what the motor's setup resolved and a
         * fixture that lied about it would be testing its own arithmetic.
         * 区间外：同一条 24 V 母线对上 30 V 下限，于是第一拍就停掉环并说明原因。动的是区间而不是母线：
         * 这台架上的母线就是电机 setup 解出来的那个值，一个去改口它的夹具只是在测自己的算术。 */
        {
            foc_supervisor_config_t outside = config;
            foc_motor_t third_motor = NULL;
            foc_supervisor_t supervisor = NULL;
            foc_supervisor_status_t status;
            foc_motor_status_t motor_status;

            outside.protection.v_max_v = 40.0F;
            outside.protection.v_min_v = 30.0F;
            CHECK(foc_motor_create(&params, &third_motor) == FOC_OK);
            CHECK(foc_supervisor_create(third_motor, &outside, &supervisor) == FOC_OK);
            CHECK(foc_supervisor_enable(supervisor) == FOC_OK);
            CHECK(foc_supervisor_step(supervisor, 1e-4F) == FOC_ERR_FAULT);

            (void)memset(&status, 0, sizeof(status));
            status.struct_size = (uint32_t)sizeof(status);
            CHECK(foc_supervisor_status(supervisor, &status) == FOC_OK);
            CHECK(status.tick == 1U);
            CHECK(status.fault == FOC_FAULT_UNDER_VOLTAGE);
            CHECK(status.output_allowed == 0);
            /* The motor was stopped by the supervisor, not by the caller noticing. */
            (void)memset(&motor_status, 0, sizeof(motor_status));
            motor_status.struct_size = (uint32_t)sizeof(motor_status);
            CHECK(foc_motor_get_status(third_motor, &motor_status) == FOC_OK);
            CHECK(motor_status.mode_state == FOC_MOTOR_FAULT);

            CHECK(foc_supervisor_clear(supervisor) == FOC_OK);
            CHECK(foc_supervisor_destroy(supervisor) == FOC_OK);
            CHECK(foc_motor_destroy(third_motor) == FOC_OK);
        }
    }

    if (failures != 0) {
        (void)fprintf(stderr, "C11 consumer: %d failure(s)\n", failures);
        return 1;
    }
    (void)printf("C11 consumer: the C API compiles, links and runs (validation passed)\n");
    return 0;
}
