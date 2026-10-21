/* Test-only host shim. The original flux.c/flux.h snapshots are unmodified. */
#ifndef MATLAB_OBSERVER_TEST_HEADLINE_H
#define MATLAB_OBSERVER_TEST_HEADLINE_H
#include <stdint.h>
#include <math.h>
#define PI 3.14159265358979323846f
typedef struct { float PhaseU_Curr, PhaseV_Curr, PhaseW_Curr;
                 float Svpwm_Km_BackwS, BUS_Voltage; } TestVoltCurr;
typedef struct { float Alpha, Beta; } TestClarke;
typedef struct { float Ds; } TestPark;
typedef struct { float Ualpha, Ubeta; } TestSvpwm;
typedef struct { int16_t IQAngle; float SpeedRPM; } TestMotor;
extern TestVoltCurr Volt_CurrPara;
extern TestClarke CLARKE_PCurr;
extern TestPark PARK_PCurr;
extern TestSvpwm Svpwm_dq;
extern TestMotor motor;
float Limit_Sat(float x, float maximum, float minimum);
#define FOC_VOLTAGE_PU_MAX 0.95f
#include "PI_Cale.h"
#endif
