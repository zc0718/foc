// The controller as an FMI 2.0 Co-Simulation FMU.
//
// This file contains no control logic and must not grow any: it translates between the FMI calling
// convention and `foc_hal_callbacks_t`, which is the same hand-over the Python binding uses. If a
// behaviour exists here and nowhere else, it is a behaviour nobody can test without a master.
// 本文件不含任何控制逻辑，也不该长出控制逻辑：它只在 FMI 调用约定与 `foc_hal_callbacks_t` 之间翻译，
// 而后者正是 Python 绑定用的同一个交接面。如果某个行为只在这里存在，那它就是一个离开主控就无法测试的
// 行为。

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fmi2Functions.h"

#include <foc/api/foc_api.h>

enum {
    kIqCommandA = 1,
    kEnable = 2,
    kVbusV = 3,
    kIuA = 4,
    kIvA = 5,
    kIwA = 6,
    kAngleMechRad = 7,
    kSpeedMechRadS = 8,
    kDutyU = 9,
    kDutyV = 10,
    kDutyW = 11,
    kState = 12,
    kError = 13,
    kIdA = 14,
    kIqA = 15,
    kSpeedEstRadS = 16
};

typedef struct {
    fmi2String instanceName;
    fmi2CallbackFunctions callbacks;
    fmi2Boolean loggingOn;

    foc_motor_t motor;
    foc_hal_callbacks_t hal;
    foc_motor_params_t params;

    // Inputs: written by the master, read by the HAL callbacks.
    fmi2Real iq_command_a;
    fmi2Boolean enable;
    fmi2Real v_bus_v;
    fmi2Real i_u_a;
    fmi2Real i_v_a;
    fmi2Real i_w_a;
    fmi2Real angle_mech_rad;
    fmi2Real speed_mech_rad_s;

    // Outputs.
    fmi2Real duty[3];
    fmi2Integer state;
    fmi2Integer error;
    fmi2Real id_a;
    fmi2Real iq_a;
    fmi2Real speed_est_rad_s;

    fmi2Boolean enabled;
} Instance;


// ---------------------------------------------------------------- HAL callbacks
static void on_write_duty(void *user, const float duty_uvw[3]) {
    Instance *s = (Instance *)user;
    s->duty[0] = duty_uvw[0];
    s->duty[1] = duty_uvw[1];
    s->duty[2] = duty_uvw[2];
}

static void on_read_current(void *user, float uvw_a[3]) {
    Instance *s = (Instance *)user;
    uvw_a[0] = (float)s->i_u_a;
    uvw_a[1] = (float)s->i_v_a;
    uvw_a[2] = (float)s->i_w_a;
}

static float on_read_angle(void *user) { return (float)((Instance *)user)->angle_mech_rad; }

static float on_read_voltage(void *user) { return (float)((Instance *)user)->v_bus_v; }

static float on_read_velocity(void *user) { return (float)((Instance *)user)->speed_mech_rad_s; }


// ---------------------------------------------------------------- FMI plumbing
#define INSTANCE(component) ((Instance *)(component))

const char *fmi2GetTypesPlatform(void) { return fmi2TypesPlatform; }

const char *fmi2GetVersion(void) { return fmi2Version; }

fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t n, const fmi2String categories[]) {
    (void)n;
    (void)categories;
    INSTANCE(c)->loggingOn = loggingOn;
    return fmi2OK;
}

static void default_params(foc_motor_params_t *params) {
    memset(params, 0, sizeof(*params));
    params->rs = 0.5F;
    params->ld = 4.0e-4F;
    params->lq = 4.0e-4F;
    params->kt = 0.05F;
    params->j = 1.0e-5F;
    params->b = 1.0e-6F;
    params->pole_pairs = 7U;
}

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                              fmi2String fmuResourceLocation, const fmi2CallbackFunctions *callbacks,
                              fmi2Boolean visible, fmi2Boolean loggingOn) {
    (void)fmuGUID;
    (void)fmuResourceLocation;
    (void)visible;

    if (callbacks == NULL || fmuType != fmi2CoSimulation) {
        return NULL;
    }

    Instance *s = (Instance *)calloc(1, sizeof(Instance));
    if (s == NULL) {
        return NULL;
    }
    s->instanceName = instanceName;
    s->callbacks = *callbacks;
    s->loggingOn = loggingOn;
    s->v_bus_v = 24.0;

    s->hal.struct_size = (uint32_t)sizeof(foc_hal_callbacks_t);
    s->hal.user = s;
    s->hal.pwm_write_duty = on_write_duty;
    s->hal.read_current = on_read_current;
    s->hal.read_angle_mech_rad = on_read_angle;
    s->hal.read_voltage_v = on_read_voltage;
    s->hal.read_velocity_rad_s = on_read_velocity;
    return s;
}

void fmi2FreeInstance(fmi2Component c) {
    Instance *s = INSTANCE(c);
    if (s == NULL) {
        return;
    }
    if (s->motor != NULL) {
        (void)foc_motor_destroy(s->motor);
    }
    free(s);
}

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime,
                               fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
    (void)c;
    (void)toleranceDefined;
    (void)tolerance;
    (void)startTime;
    (void)stopTimeDefined;
    (void)stopTime;
    return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) { return c == NULL ? fmi2Error : fmi2OK; }

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    Instance *s = INSTANCE(c);
    if (s == NULL) {
        return fmi2Error;
    }
    // The motor is created here rather than in Instantiate: the master may set inputs (bus voltage, machine
    // parameters) between the two, and creating it earlier would bake in values the master then changes.
    // 电机在此时而不是 Instantiate 时创建：主控可能在这两步之间设置输入（母线电压、机器参数），更早创建会把
    // 主控随后修改的值烤进去。
    if (s->motor == NULL) {
        default_params(&s->params);
        if (foc_motor_create_host(&s->params, NULL, &s->hal, &s->motor) != FOC_OK) {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component c) { return c == NULL ? fmi2Error : fmi2OK; }

fmi2Status fmi2Reset(fmi2Component c) {
    Instance *s = INSTANCE(c);
    if (s == NULL) {
        return fmi2Error;
    }
    s->enabled = fmi2False;
    s->state = 0;
    s->error = 0;
    return fmi2OK;
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize,
                      fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    (void)currentCommunicationPoint;
    (void)noSetFMUStatePriorToCurrentPoint;
    Instance *s = INSTANCE(c);
    if (s == NULL || s->motor == NULL) {
        return fmi2Error;
    }

    // Enable is edge-triggered because that is what the C ABI offers; a level-triggered call every step would
    // restart the aligner. The edge is tracked here, and only here, because no other caller has a master that
    // can toggle it mid-run.
    // enable 是边沿触发，因为 C ABI 提供的就是这个语义；每步都按电平调用会重启对齐器。边沿只在这里跟踪，因为
    // 没有别的调用方的主控会在运行中切换它。
    if (s->enable != 0 && !s->enabled) {
        (void)foc_motor_enable(s->motor);
        s->enabled = fmi2True;
    } else if (s->enable == 0 && s->enabled) {
        (void)foc_motor_disable(s->motor);
        s->enabled = fmi2False;
    }

    (void)foc_set_torque(s->motor, s->iq_command_a);
    const foc_err_t rc = foc_tick(s->motor, (float)communicationStepSize);

    foc_motor_status_t status;
    memset(&status, 0, sizeof(status));
    status.struct_size = (uint32_t)sizeof(status);
    (void)foc_motor_get_status(s->motor, &status);

    float speed = 0.0F;
    float id = 0.0F;
    float iq = 0.0F;
    (void)foc_get_state(s->motor, &speed, &id, &iq);

    s->state = (fmi2Integer)status.mode_state;
    s->error = (fmi2Integer)((rc == FOC_OK) ? status.error : (int32_t)rc);
    s->id_a = (fmi2Real)id;
    s->iq_a = (fmi2Real)iq;
    s->speed_est_rad_s = (fmi2Real)speed;
    return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component c) { return c == NULL ? fmi2Error : fmi2Discard; }

fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status *value) {
    (void)c;
    (void)s;
    (void)value;
    return fmi2Discard;
}

fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real *value) {
    (void)c;
    (void)s;
    (void)value;
    return fmi2Discard;
}

fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer *value) {
    (void)c;
    (void)s;
    (void)value;
    return fmi2Discard;
}

fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean *value) {
    (void)c;
    (void)s;
    (void)value;
    return fmi2Discard;
}

fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String *value) {
    (void)c;
    (void)s;
    (void)value;
    return fmi2Discard;
}

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
    Instance *s = INSTANCE(c);
    if (s == NULL) {
        return fmi2Error;
    }
    for (size_t i = 0; i < nvr; ++i) {
        switch (vr[i]) {
        case kIqCommandA:
            s->iq_command_a = value[i];
            break;
        case kVbusV:
            s->v_bus_v = value[i];
            break;
        case kIuA:
            s->i_u_a = value[i];
            break;
        case kIvA:
            s->i_v_a = value[i];
            break;
        case kIwA:
            s->i_w_a = value[i];
            break;
        case kAngleMechRad:
            s->angle_mech_rad = value[i];
            break;
        case kSpeedMechRadS:
            s->speed_mech_rad_s = value[i];
            break;
        default:
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
    Instance *s = INSTANCE(c);
    if (s == NULL) {
        return fmi2Error;
    }
    for (size_t i = 0; i < nvr; ++i) {
        switch (vr[i]) {
        case kDutyU:
            value[i] = s->duty[0];
            break;
        case kDutyV:
            value[i] = s->duty[1];
            break;
        case kDutyW:
            value[i] = s->duty[2];
            break;
        case kIdA:
            value[i] = s->id_a;
            break;
        case kIqA:
            value[i] = s->iq_a;
            break;
        case kSpeedEstRadS:
            value[i] = s->speed_est_rad_s;
            break;
        default:
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {
    (void)c;
    (void)vr;
    (void)nvr;
    (void)value;
    return fmi2Error;
}

fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {
    Instance *s = INSTANCE(c);
    if (s == NULL) {
        return fmi2Error;
    }
    for (size_t i = 0; i < nvr; ++i) {
        switch (vr[i]) {
        case kState:
            value[i] = s->state;
            break;
        case kError:
            value[i] = s->error;
            break;
        default:
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {
    Instance *s = INSTANCE(c);
    if (s == NULL) {
        return fmi2Error;
    }
    for (size_t i = 0; i < nvr; ++i) {
        if (vr[i] == kEnable) {
            s->enable = value[i];
        } else {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {
    (void)c;
    (void)vr;
    (void)nvr;
    (void)value;
    return fmi2Error;
}

fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) {
    (void)c;
    (void)vr;
    (void)nvr;
    (void)value;
    return fmi2Error;
}

fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[]) {
    (void)c;
    (void)vr;
    (void)nvr;
    (void)value;
    return fmi2Error;
}


// Optional functions, present because masters look the symbols up unconditionally even when the model
// declares `canGetAndSetFMUstate="false"`. Each answers `fmi2Error`: the honest answer, since we do not
// implement state save/restore and pretending otherwise would let a master restore a half-state.
// 可选函数之所以存在，是因为即使模型声明 `canGetAndSetFMUstate="false"`，主控也会无条件查找这些符号。
// 它们一律答 `fmi2Error`：我们不实现状态存取，假装实现会让主控恢复出一个半状态。
fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate *state) {
    (void)c;
    (void)state;
    return fmi2Error;
}

fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate state) {
    (void)c;
    (void)state;
    return fmi2Error;
}

fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate *state) {
    (void)c;
    (void)state;
    return fmi2Error;
}

fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate state, size_t *size) {
    (void)c;
    (void)state;
    (void)size;
    return fmi2Error;
}

fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate state, fmi2Byte serializedState[], size_t size) {
    (void)c;
    (void)state;
    (void)serializedState;
    (void)size;
    return fmi2Error;
}

fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte serializedState[], size_t size,
                                   fmi2FMUstate *state) {
    (void)c;
    (void)serializedState;
    (void)size;
    (void)state;
    return fmi2Error;
}

fmi2Status fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference unknown[], size_t nUnknown,
                                        const fmi2ValueReference known[], size_t nKnown, const fmi2Real dvKnown[],
                                        fmi2Real dvUnknown[]) {
    (void)c;
    (void)unknown;
    (void)nUnknown;
    (void)known;
    (void)nKnown;
    (void)dvKnown;
    (void)dvUnknown;
    return fmi2Error;
}

fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                        const fmi2Integer order[], fmi2Real value[]) {
    (void)c;
    (void)vr;
    (void)nvr;
    (void)order;
    (void)value;
    return fmi2Error;
}

fmi2Status fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                       const fmi2Integer order[], const fmi2Real value[]) {
    (void)c;
    (void)vr;
    (void)nvr;
    (void)order;
    (void)value;
    return fmi2Error;
}
