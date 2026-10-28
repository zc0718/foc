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
    CHECK(foc_set_position_g(mover, 0.1) == FOC_OK);       /* routed to the double entry point */
    CHECK(foc_set_position_g(mover, 0.2F) == FOC_OK);      /* routed to the float one */
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
    CHECK(status.mode_state == (int32_t)FOC_MOTOR_IDLE); /* created but not enabled yet */
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
     * ABI 0.1.1: the modules a C consumer could not reach at all before.
     * ABI 0.1.1：在此之前 C 消费方完全够不到的那些模块。
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

    if (failures != 0) {
        (void)fprintf(stderr, "C11 consumer: %d failure(s)\n", failures);
        return 1;
    }
    (void)printf("C11 consumer: the C API compiles, links and runs (validation passed)\n");
    return 0;
}
