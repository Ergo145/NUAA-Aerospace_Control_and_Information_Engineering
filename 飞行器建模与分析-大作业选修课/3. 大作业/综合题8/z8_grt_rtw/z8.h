/*
 * z8.h
 *
 * Code generation for model "z8".
 *
 * Model version              : 1.7
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Tue Oct 14 20:30:14 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_z8_h_
#define RTW_HEADER_z8_h_
#ifndef z8_COMMON_INCLUDES_
#define z8_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* z8_COMMON_INCLUDES_ */

#include "z8_types.h"
#include <float.h>
#include <string.h>
#include <stddef.h>
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Constant[12];                 /* '<Root>/Constant' */
  real_T Integrator;                   /* '<S6>/Integrator' */
  real_T Integrator1;                  /* '<S6>/Integrator1' */
  real_T Integrator2;                  /* '<S6>/Integrator2' */
  real_T Integrator_i;                 /* '<S9>/Integrator' */
  real_T Integrator1_h;                /* '<S9>/Integrator1' */
  real_T Integrator2_l;                /* '<S9>/Integrator2' */
  real_T Integrator_c;                 /* '<S7>/Integrator' */
  real_T Integrator1_o;                /* '<S7>/Integrator1' */
  real_T Integrator2_d;                /* '<S7>/Integrator2' */
  real_T Integrator2_a;                /* '<S8>/Integrator2' */
  real_T Integrator1_n;                /* '<S8>/Integrator1' */
  real_T Integrator3;                  /* '<S8>/Integrator3' */
  real_T Ix;                           /* '<S10>/飞行器本体参数3' */
  real_T Iy;                           /* '<S10>/飞行器本体参数3' */
  real_T Iz;                           /* '<S10>/飞行器本体参数3' */
  real_T dV;                           /* '<S9>/速度方程组' */
  real_T dchi;                         /* '<S9>/速度方程组' */
  real_T dgamma;                       /* '<S9>/速度方程组' */
  real_T dp;                           /* '<S8>/角速率方程组' */
  real_T dq;                           /* '<S8>/角速率方程组' */
  real_T dr;                           /* '<S8>/角速率方程组' */
  real_T dsigma;                       /* '<S12>/气流姿态角方程组1' */
  real_T dalpha;                       /* '<S12>/气流姿态角方程组' */
  real_T dbeta;                        /* '<S12>/气流姿态角方程组' */
  real_T dR;                           /* '<S6>/位置方程组' */
  real_T dtau;                         /* '<S6>/位置方程组' */
  real_T ddelta;                       /* '<S6>/位置方程组' */
} B_z8_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  int_T Integrator_IWORK;              /* '<S6>/Integrator' */
  int_T Integrator1_IWORK;             /* '<S6>/Integrator1' */
  int_T Integrator2_IWORK;             /* '<S6>/Integrator2' */
  int_T Integrator_IWORK_k;            /* '<S9>/Integrator' */
  int_T Integrator1_IWORK_i;           /* '<S9>/Integrator1' */
  int_T Integrator2_IWORK_f;           /* '<S9>/Integrator2' */
  int_T Integrator_IWORK_l;            /* '<S7>/Integrator' */
  int_T Integrator1_IWORK_e;           /* '<S7>/Integrator1' */
  int_T Integrator2_IWORK_fw;          /* '<S7>/Integrator2' */
  int_T Integrator2_IWORK_p;           /* '<S8>/Integrator2' */
  int_T Integrator1_IWORK_l;           /* '<S8>/Integrator1' */
  int_T Integrator3_IWORK;             /* '<S8>/Integrator3' */
} DW_z8_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S6>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<S6>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S6>/Integrator2' */
  real_T Integrator_CSTATE_a;          /* '<S9>/Integrator' */
  real_T Integrator1_CSTATE_n;         /* '<S9>/Integrator1' */
  real_T Integrator2_CSTATE_p;         /* '<S9>/Integrator2' */
  real_T Integrator_CSTATE_i;          /* '<S7>/Integrator' */
  real_T Integrator1_CSTATE_e;         /* '<S7>/Integrator1' */
  real_T Integrator2_CSTATE_g;         /* '<S7>/Integrator2' */
  real_T Integrator2_CSTATE_f;         /* '<S8>/Integrator2' */
  real_T Integrator1_CSTATE_d;         /* '<S8>/Integrator1' */
  real_T Integrator3_CSTATE;           /* '<S8>/Integrator3' */
} X_z8_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S6>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<S6>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S6>/Integrator2' */
  real_T Integrator_CSTATE_a;          /* '<S9>/Integrator' */
  real_T Integrator1_CSTATE_n;         /* '<S9>/Integrator1' */
  real_T Integrator2_CSTATE_p;         /* '<S9>/Integrator2' */
  real_T Integrator_CSTATE_i;          /* '<S7>/Integrator' */
  real_T Integrator1_CSTATE_e;         /* '<S7>/Integrator1' */
  real_T Integrator2_CSTATE_g;         /* '<S7>/Integrator2' */
  real_T Integrator2_CSTATE_f;         /* '<S8>/Integrator2' */
  real_T Integrator1_CSTATE_d;         /* '<S8>/Integrator1' */
  real_T Integrator3_CSTATE;           /* '<S8>/Integrator3' */
} XDot_z8_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S6>/Integrator' */
  boolean_T Integrator1_CSTATE;        /* '<S6>/Integrator1' */
  boolean_T Integrator2_CSTATE;        /* '<S6>/Integrator2' */
  boolean_T Integrator_CSTATE_a;       /* '<S9>/Integrator' */
  boolean_T Integrator1_CSTATE_n;      /* '<S9>/Integrator1' */
  boolean_T Integrator2_CSTATE_p;      /* '<S9>/Integrator2' */
  boolean_T Integrator_CSTATE_i;       /* '<S7>/Integrator' */
  boolean_T Integrator1_CSTATE_e;      /* '<S7>/Integrator1' */
  boolean_T Integrator2_CSTATE_g;      /* '<S7>/Integrator2' */
  boolean_T Integrator2_CSTATE_f;      /* '<S8>/Integrator2' */
  boolean_T Integrator1_CSTATE_d;      /* '<S8>/Integrator1' */
  boolean_T Integrator3_CSTATE;        /* '<S8>/Integrator3' */
} XDis_z8_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters (default storage) */
struct P_z8_T_ {
  real_T Subsystem_C_C;                /* Mask Parameter: Subsystem_C_C
                                        * Referenced by: '<S1>/Constant35'
                                        */
  real_T Subsystem_C_D;                /* Mask Parameter: Subsystem_C_D
                                        * Referenced by: '<S1>/Constant34'
                                        */
  real_T Subsystem_C_I;                /* Mask Parameter: Subsystem_C_I
                                        * Referenced by: '<S1>/Constant31'
                                        */
  real_T Subsystem_C_L;                /* Mask Parameter: Subsystem_C_L
                                        * Referenced by: '<S1>/Constant36'
                                        */
  real_T Subsystem_C_m;                /* Mask Parameter: Subsystem_C_m
                                        * Referenced by: '<S1>/Constant32'
                                        */
  real_T Subsystem_C_n;                /* Mask Parameter: Subsystem_C_n
                                        * Referenced by: '<S1>/Constant33'
                                        */
  real_T Subsystem_S;                  /* Mask Parameter: Subsystem_S
                                        * Referenced by: '<S1>/Constant26'
                                        */
  real_T Subsystem_X_cg;               /* Mask Parameter: Subsystem_X_cg
                                        * Referenced by: '<S1>/Constant29'
                                        */
  real_T Subsystem_b;                  /* Mask Parameter: Subsystem_b
                                        * Referenced by: '<S1>/Constant28'
                                        */
  real_T Subsystem_c;                  /* Mask Parameter: Subsystem_c
                                        * Referenced by: '<S1>/Constant27'
                                        */
  real_T Subsystem_g;                  /* Mask Parameter: Subsystem_g
                                        * Referenced by: '<S1>/Constant18'
                                        */
  real_T Subsystem_m;                  /* Mask Parameter: Subsystem_m
                                        * Referenced by:
                                        *   '<S1>/Constant14'
                                        *   '<S1>/Constant37'
                                        */
  real_T Subsystem_rou;                /* Mask Parameter: Subsystem_rou
                                        * Referenced by: '<S1>/Constant30'
                                        */
  real_T Subsystem_w_E;                /* Mask Parameter: Subsystem_w_E
                                        * Referenced by:
                                        *   '<S1>/Constant19'
                                        *   '<S1>/Constant22'
                                        */
  real_T Constant_Value[12];
                        /* Expression: [6411000;118.8;32;3000;180;0;2;3;2;0;0;0]
                         * Referenced by: '<Root>/Constant'
                         */
};

/* Real-time Model Data Structure */
struct tag_RTM_z8_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_z8_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_z8_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[12];
  real_T odeF[3][12];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    time_T tStart;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_z8_T z8_P;

/* Block signals (default storage) */
extern B_z8_T z8_B;

/* Continuous states (default storage) */
extern X_z8_T z8_X;

/* Disabled states (default storage) */
extern XDis_z8_T z8_XDis;

/* Block states (default storage) */
extern DW_z8_T z8_DW;

/* Model entry point functions */
extern void z8_initialize(void);
extern void z8_step(void);
extern void z8_terminate(void);

/* Real-time Model object */
extern RT_MODEL_z8_T *const z8_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'z8'
 * '<S1>'   : 'z8/Subsystem'
 * '<S2>'   : 'z8/Subsystem1'
 * '<S3>'   : 'z8/Subsystem2'
 * '<S4>'   : 'z8/Subsystem3'
 * '<S5>'   : 'z8/Subsystem4'
 * '<S6>'   : 'z8/Subsystem/位置方程组1'
 * '<S7>'   : 'z8/Subsystem/气流姿态角方程组1'
 * '<S8>'   : 'z8/Subsystem/角速率方程组1'
 * '<S9>'   : 'z8/Subsystem/速度方程组1'
 * '<S10>'  : 'z8/Subsystem/飞行器自身参数'
 * '<S11>'  : 'z8/Subsystem/位置方程组1/位置方程组'
 * '<S12>'  : 'z8/Subsystem/气流姿态角方程组1/气流姿态角方程组'
 * '<S13>'  : 'z8/Subsystem/气流姿态角方程组1/气流姿态角方程组/气流姿态角方程组'
 * '<S14>'  : 'z8/Subsystem/气流姿态角方程组1/气流姿态角方程组/气流姿态角方程组1'
 * '<S15>'  : 'z8/Subsystem/角速率方程组1/角速率方程组'
 * '<S16>'  : 'z8/Subsystem/速度方程组1/速度方程组'
 * '<S17>'  : 'z8/Subsystem/飞行器自身参数/飞行器本体参数1'
 * '<S18>'  : 'z8/Subsystem/飞行器自身参数/飞行器本体参数2'
 * '<S19>'  : 'z8/Subsystem/飞行器自身参数/飞行器本体参数3'
 */
#endif                                 /* RTW_HEADER_z8_h_ */
