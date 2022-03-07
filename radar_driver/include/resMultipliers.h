#ifndef RES_MULTIPLIERS_H
#define RES_MULTIPLIERS_H

/* General Use Includes */
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <stdint.h>

typedef struct resMultipliersRDI
{
    float f_RangeRes      = 0.004577776;
    float f_VrelRadRes    = 0.004577776;
    float f_AzAng0Res     = 0.0000958767;
    float f_AzAng1Res     = 0.0000958767;
    float f_ElAngRes      = 0.0000958767;
    float f_RCS0Res       = 0.003051851;
    float f_RCS1Res       = 0.003051851;
    float f_Prob0Res      = 0.003937008;
    float f_Prob1Res      = 0.003937008;
    float f_RangeVarRes   = 0.000152593;
    float f_VrelRadVarRes = 0.000152593;
    float f_AzAngVar0Res  = 0.0000152593;
    float f_AzAngVar1Res  = 0.0000152593;
    float f_ElAngVarRes   = 0.0000152593;
    float f_SNRRes        = 0.1;
    float VambigRes       = 0.0030519;
    float CenterFreqRes   = 0.05;
} resMultipliersRDI_t;

typedef struct resMultipliersSS
{
    float surfaceDampingRes    = 9.3132e-09;
    float MaxRangeNearRes      = 0.1;
    float MaxRangeFarRes       = 0.1;
} resMultipliersSS_t;

#endif /* RES_MULTIPLIERS_H */
