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

    if (failures != 0) {
        (void)fprintf(stderr, "C11 consumer: %d failure(s)\n", failures);
        return 1;
    }
    (void)printf("C11 consumer: the C API compiles, links and runs (validation passed)\n");
    return 0;
}
