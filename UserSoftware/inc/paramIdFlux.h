/** @file paramIdFlux.h @brief MODE6 空载拖转与封波反电动势测量。 */
#ifndef PARAM_ID_FLUX_H
#define PARAM_ID_FLUX_H
#include <stdint.h>

void ParamIdFluxStart(float biasVoltagePu);
uint8_t ParamIdFluxRun(void); /* 0=继续，1=结束；错误和候选保留在 pid 中 */
#endif
