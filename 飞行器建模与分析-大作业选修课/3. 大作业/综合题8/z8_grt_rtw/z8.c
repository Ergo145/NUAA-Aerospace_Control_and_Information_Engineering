/*
 * z8.c
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

#include "z8.h"
#include <string.h>
#include <math.h>
#include "rtwtypes.h"
#include "z8_private.h"
#include "rt_nonfinite.h"

/* Block signals (default storage) */
B_z8_T z8_B;

/* Continuous states */
X_z8_T z8_X;

/* Disabled State Vector */
XDis_z8_T z8_XDis;

/* Block states (default storage) */
DW_z8_T z8_DW;

/* Real-time model */
static RT_MODEL_z8_T z8_M_;
RT_MODEL_z8_T *const z8_M = &z8_M_;

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 12;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  z8_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  z8_step();
  z8_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  z8_step();
  z8_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void z8_step(void)
{
  real_T dR_tmp;
  real_T dV_tmp;
  real_T dV_tmp_0;
  real_T dV_tmp_1;
  real_T dV_tmp_2;
  real_T dchi_tmp;
  real_T dchi_tmp_0;
  real_T ddelta_tmp;
  real_T dtau_tmp;
  real_T dtau_tmp_0;
  real_T dtau_tmp_1;
  real_T dtau_tmp_2;
  real_T dtau_tmp_3;
  real_T q_guji;
  real_T q_guji_tmp;
  real_T rtb_C;
  real_T rtb_D;
  real_T rtb_L;
  if (rtmIsMajorTimeStep(z8_M)) {
    /* set solver stop time */
    if (!(z8_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&z8_M->solverInfo, ((z8_M->Timing.clockTickH0 + 1) *
        z8_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&z8_M->solverInfo, ((z8_M->Timing.clockTick0 + 1) *
        z8_M->Timing.stepSize0 + z8_M->Timing.clockTickH0 *
        z8_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(z8_M)) {
    z8_M->Timing.t[0] = rtsiGetT(&z8_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(z8_M)) {
    /* Constant: '<Root>/Constant' */
    memcpy(&z8_B.Constant[0], &z8_P.Constant_Value[0], 12U * sizeof(real_T));
  }

  /* Integrator: '<S6>/Integrator' */
  if (z8_DW.Integrator_IWORK != 0) {
    z8_X.Integrator_CSTATE = z8_B.Constant[0];
  }

  /* Integrator: '<S6>/Integrator' */
  z8_B.Integrator = z8_X.Integrator_CSTATE;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S6>/Integrator1' */
  if (z8_DW.Integrator1_IWORK != 0) {
    z8_X.Integrator1_CSTATE = z8_B.Constant[1];
  }

  /* Integrator: '<S6>/Integrator1' */
  z8_B.Integrator1 = z8_X.Integrator1_CSTATE;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S6>/Integrator2' */
  if (z8_DW.Integrator2_IWORK != 0) {
    z8_X.Integrator2_CSTATE = z8_B.Constant[2];
  }

  /* Integrator: '<S6>/Integrator2' */
  z8_B.Integrator2 = z8_X.Integrator2_CSTATE;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S9>/Integrator' */
  if (z8_DW.Integrator_IWORK_k != 0) {
    z8_X.Integrator_CSTATE_a = z8_B.Constant[3];
  }

  /* Integrator: '<S9>/Integrator' */
  z8_B.Integrator_i = z8_X.Integrator_CSTATE_a;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S9>/Integrator1' */
  if (z8_DW.Integrator1_IWORK_i != 0) {
    z8_X.Integrator1_CSTATE_n = z8_B.Constant[4];
  }

  /* Integrator: '<S9>/Integrator1' */
  z8_B.Integrator1_h = z8_X.Integrator1_CSTATE_n;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S9>/Integrator2' */
  if (z8_DW.Integrator2_IWORK_f != 0) {
    z8_X.Integrator2_CSTATE_p = z8_B.Constant[5];
  }

  /* Integrator: '<S9>/Integrator2' */
  z8_B.Integrator2_l = z8_X.Integrator2_CSTATE_p;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S7>/Integrator' */
  if (z8_DW.Integrator_IWORK_l != 0) {
    z8_X.Integrator_CSTATE_i = z8_B.Constant[6];
  }

  /* Integrator: '<S7>/Integrator' */
  z8_B.Integrator_c = z8_X.Integrator_CSTATE_i;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S7>/Integrator1' */
  if (z8_DW.Integrator1_IWORK_e != 0) {
    z8_X.Integrator1_CSTATE_e = z8_B.Constant[7];
  }

  /* Integrator: '<S7>/Integrator1' */
  z8_B.Integrator1_o = z8_X.Integrator1_CSTATE_e;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S7>/Integrator2' */
  if (z8_DW.Integrator2_IWORK_fw != 0) {
    z8_X.Integrator2_CSTATE_g = z8_B.Constant[8];
  }

  /* Integrator: '<S7>/Integrator2' */
  z8_B.Integrator2_d = z8_X.Integrator2_CSTATE_g;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S8>/Integrator2' */
  if (z8_DW.Integrator2_IWORK_p != 0) {
    z8_X.Integrator2_CSTATE_f = z8_B.Constant[9];
  }

  /* Integrator: '<S8>/Integrator2' */
  z8_B.Integrator2_a = z8_X.Integrator2_CSTATE_f;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S8>/Integrator1' */
  if (z8_DW.Integrator1_IWORK_l != 0) {
    z8_X.Integrator1_CSTATE_d = z8_B.Constant[10];
  }

  /* Integrator: '<S8>/Integrator1' */
  z8_B.Integrator1_n = z8_X.Integrator1_CSTATE_d;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* Integrator: '<S8>/Integrator3' */
  if (z8_DW.Integrator3_IWORK != 0) {
    z8_X.Integrator3_CSTATE = z8_B.Constant[11];
  }

  /* Integrator: '<S8>/Integrator3' */
  z8_B.Integrator3 = z8_X.Integrator3_CSTATE;
  if (rtmIsMajorTimeStep(z8_M)) {
  }

  /* MATLAB Function: '<S6>/位置方程组' incorporates:
   *  MATLAB Function: '<S12>/气流姿态角方程组'
   *  MATLAB Function: '<S12>/气流姿态角方程组1'
   *  MATLAB Function: '<S9>/速度方程组'
   */
  dR_tmp = sin(z8_B.Integrator2_l);
  z8_B.dR = z8_B.Integrator_i * dR_tmp;
  dtau_tmp = cos(z8_B.Integrator2);
  dtau_tmp_0 = cos(z8_B.Integrator2_l);
  dtau_tmp_1 = sin(z8_B.Integrator1_h);
  dtau_tmp_2 = z8_B.Integrator_i * dtau_tmp_0;
  dtau_tmp_3 = dtau_tmp_2 * dtau_tmp_1;
  z8_B.dtau = dtau_tmp_3 / (z8_B.Integrator * dtau_tmp);
  ddelta_tmp = cos(z8_B.Integrator1_h);
  z8_B.ddelta = dtau_tmp_2 * ddelta_tmp / z8_B.Integrator;

  /* MATLAB Function: '<S10>/飞行器本体参数1' incorporates:
   *  Constant: '<S1>/Constant26'
   *  Constant: '<S1>/Constant30'
   *  Constant: '<S1>/Constant34'
   *  Constant: '<S1>/Constant35'
   *  Constant: '<S1>/Constant36'
   *  MATLAB Function: '<S10>/飞行器本体参数2'
   */
  q_guji_tmp = 0.5 * z8_P.Subsystem_rou * (z8_B.Integrator_i * z8_B.Integrator_i);
  rtb_D = z8_P.Subsystem_C_D * q_guji_tmp * z8_P.Subsystem_S;
  rtb_C = z8_P.Subsystem_C_C * q_guji_tmp * z8_P.Subsystem_S;
  rtb_L = z8_P.Subsystem_C_L * q_guji_tmp * z8_P.Subsystem_S;

  /* MATLAB Function: '<S9>/速度方程组' incorporates:
   *  Constant: '<S1>/Constant14'
   *  Constant: '<S1>/Constant18'
   *  Constant: '<S1>/Constant19'
   *  MATLAB Function: '<S12>/气流姿态角方程组'
   *  MATLAB Function: '<S12>/气流姿态角方程组1'
   */
  dV_tmp = sin(z8_B.Integrator2);
  dV_tmp_0 = z8_P.Subsystem_w_E * z8_P.Subsystem_w_E * z8_B.Integrator;
  dV_tmp_1 = dV_tmp_0 * dtau_tmp;
  dV_tmp_2 = dtau_tmp_0 * dV_tmp;
  z8_B.dV = dV_tmp_1 / (dR_tmp * dtau_tmp - dV_tmp_2 * ddelta_tmp) + (-rtb_D /
    z8_P.Subsystem_m - z8_P.Subsystem_g * dR_tmp);
  q_guji = cos(z8_B.Integrator2_d);
  dchi_tmp = sin(z8_B.Integrator2_d);
  dchi_tmp_0 = z8_P.Subsystem_m * z8_B.Integrator_i;
  z8_B.dchi = ((-(rtb_C * dchi_tmp - rtb_L * q_guji) / (dchi_tmp_0 * dtau_tmp_0)
                + dtau_tmp_3 * tan(z8_B.Integrator2) / z8_B.Integrator) +
               (dV_tmp - dtau_tmp * tan(z8_B.Integrator2_l) * ddelta_tmp) * (2.0
    * z8_P.Subsystem_w_E)) + dV_tmp_0 * dV_tmp * dtau_tmp * dtau_tmp_1 /
    dtau_tmp_2;
  dtau_tmp_2 = dV_tmp * dR_tmp;
  z8_B.dgamma = (((rtb_C * q_guji + rtb_L * dchi_tmp) / dchi_tmp_0 +
                  (z8_B.Integrator_i / z8_B.Integrator - z8_P.Subsystem_g /
                   z8_B.Integrator_i) * dtau_tmp_0) + 2.0 * z8_P.Subsystem_w_E *
                 dtau_tmp * dtau_tmp_1) + (dtau_tmp_2 * ddelta_tmp + dtau_tmp *
    dtau_tmp_0) * dV_tmp_1 / z8_B.Integrator_i;

  /* MATLAB Function: '<S12>/气流姿态角方程组' incorporates:
   *  Constant: '<S1>/Constant22'
   *  MATLAB Function: '<S10>/飞行器本体参数2'
   *  MATLAB Function: '<S12>/气流姿态角方程组1'
   */
  dV_tmp = cos(z8_B.Integrator1_o);
  dV_tmp_1 = sin(z8_B.Integrator_c);
  dchi_tmp_0 = cos(z8_B.Integrator_c);
  dtau_tmp_3 = z8_B.dtau + z8_P.Subsystem_w_E;
  dV_tmp_0 = (z8_B.dgamma - z8_B.ddelta * ddelta_tmp) - dtau_tmp_3 * dtau_tmp *
    dtau_tmp_1;
  dtau_tmp_1 *= z8_B.ddelta;
  ddelta_tmp *= dtau_tmp;
  dtau_tmp = (ddelta_tmp * dR_tmp - dV_tmp_2) * dtau_tmp_3 + (z8_B.dchi *
    dtau_tmp_0 - dtau_tmp_1 * dR_tmp);
  dV_tmp_2 = z8_B.Integrator3 * dV_tmp_1;
  z8_B.dalpha = ((z8_B.Integrator1_n - (z8_B.Integrator2_a * dchi_tmp_0 +
    dV_tmp_2) * tan(z8_B.Integrator1_o)) + dchi_tmp / dV_tmp * dtau_tmp) -
    q_guji / dV_tmp * dV_tmp_0;
  z8_B.dbeta = ((z8_B.Integrator2_a * dV_tmp_1 - z8_B.Integrator3 * dchi_tmp_0)
                + dV_tmp_0 * dchi_tmp) + dtau_tmp * q_guji;

  /* MATLAB Function: '<S12>/气流姿态角方程组1' */
  dtau_tmp = sin(z8_B.Integrator1_o);
  z8_B.dsigma = (((((-z8_B.Integrator2_a * dchi_tmp_0 * dV_tmp -
                     z8_B.Integrator1_n * dtau_tmp) - dV_tmp_2 * dV_tmp) +
                   z8_B.dalpha * dtau_tmp) + z8_B.dchi * dR_tmp) - dtau_tmp_1 *
                 dtau_tmp_0) + (ddelta_tmp * dtau_tmp_0 + dtau_tmp_2) *
    dtau_tmp_3;
  if (rtmIsMajorTimeStep(z8_M)) {
    /* MATLAB Function: '<S10>/飞行器本体参数3' incorporates:
     *  Constant: '<S1>/Constant37'
     */
    dR_tmp = z8_P.Subsystem_m * z8_P.Subsystem_m;
    z8_B.Ix = (dR_tmp * -7.1E-5 + 19.91 * z8_P.Subsystem_m) - 59340.0;
    dR_tmp = (dR_tmp * -0.000803 + 219.74 * z8_P.Subsystem_m) - 1.69E+6;
    z8_B.Iy = dR_tmp;
    z8_B.Iz = dR_tmp;
  }

  /* MATLAB Function: '<S8>/角速率方程组' incorporates:
   *  Constant: '<S1>/Constant26'
   *  Constant: '<S1>/Constant27'
   *  Constant: '<S1>/Constant28'
   *  Constant: '<S1>/Constant29'
   *  Constant: '<S1>/Constant31'
   *  Constant: '<S1>/Constant32'
   *  Constant: '<S1>/Constant33'
   *  MATLAB Function: '<S10>/飞行器本体参数2'
   */
  z8_B.dp = (z8_B.Iy - z8_B.Iz) * z8_B.Integrator1_n * z8_B.Integrator3 /
    z8_B.Ix + z8_P.Subsystem_C_I * q_guji_tmp * z8_P.Subsystem_S *
    z8_P.Subsystem_b / z8_B.Ix;
  dR_tmp = (z8_B.Iz - z8_B.Ix) * z8_B.Integrator2_a;
  z8_B.dq = (z8_P.Subsystem_C_m * q_guji_tmp * z8_P.Subsystem_S *
             z8_P.Subsystem_c + (rtb_D * dV_tmp_1 + rtb_L * dchi_tmp_0) *
             z8_P.Subsystem_X_cg) / z8_B.Iy + dR_tmp * z8_B.Integrator3 /
    z8_B.Iy;
  z8_B.dr = (z8_P.Subsystem_C_n * q_guji_tmp * z8_P.Subsystem_S *
             z8_P.Subsystem_b + z8_P.Subsystem_X_cg * rtb_C * dV_tmp) / z8_B.Iz
    + dR_tmp * z8_B.Integrator1_n / z8_B.Iz;
  if (rtmIsMajorTimeStep(z8_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(z8_M->rtwLogInfo, (z8_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(z8_M)) {
    /* Update for Integrator: '<S6>/Integrator' */
    z8_DW.Integrator_IWORK = 0;

    /* Update for Integrator: '<S6>/Integrator1' */
    z8_DW.Integrator1_IWORK = 0;

    /* Update for Integrator: '<S6>/Integrator2' */
    z8_DW.Integrator2_IWORK = 0;

    /* Update for Integrator: '<S9>/Integrator' */
    z8_DW.Integrator_IWORK_k = 0;

    /* Update for Integrator: '<S9>/Integrator1' */
    z8_DW.Integrator1_IWORK_i = 0;

    /* Update for Integrator: '<S9>/Integrator2' */
    z8_DW.Integrator2_IWORK_f = 0;

    /* Update for Integrator: '<S7>/Integrator' */
    z8_DW.Integrator_IWORK_l = 0;

    /* Update for Integrator: '<S7>/Integrator1' */
    z8_DW.Integrator1_IWORK_e = 0;

    /* Update for Integrator: '<S7>/Integrator2' */
    z8_DW.Integrator2_IWORK_fw = 0;

    /* Update for Integrator: '<S8>/Integrator2' */
    z8_DW.Integrator2_IWORK_p = 0;

    /* Update for Integrator: '<S8>/Integrator1' */
    z8_DW.Integrator1_IWORK_l = 0;

    /* Update for Integrator: '<S8>/Integrator3' */
    z8_DW.Integrator3_IWORK = 0;
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(z8_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(z8_M)!=-1) &&
          !((rtmGetTFinal(z8_M)-(((z8_M->Timing.clockTick1+
               z8_M->Timing.clockTickH1* 4294967296.0)) * 0.01)) >
            (((z8_M->Timing.clockTick1+z8_M->Timing.clockTickH1* 4294967296.0)) *
             0.01) * (DBL_EPSILON))) {
        rtmSetErrorStatus(z8_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&z8_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++z8_M->Timing.clockTick0)) {
      ++z8_M->Timing.clockTickH0;
    }

    z8_M->Timing.t[0] = rtsiGetSolverStopTime(&z8_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      z8_M->Timing.clockTick1++;
      if (!z8_M->Timing.clockTick1) {
        z8_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void z8_derivatives(void)
{
  XDot_z8_T *_rtXdot;
  _rtXdot = ((XDot_z8_T *) z8_M->derivs);

  /* Derivatives for Integrator: '<S6>/Integrator' */
  _rtXdot->Integrator_CSTATE = z8_B.dR;

  /* Derivatives for Integrator: '<S6>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = z8_B.dtau;

  /* Derivatives for Integrator: '<S6>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = z8_B.ddelta;

  /* Derivatives for Integrator: '<S9>/Integrator' */
  _rtXdot->Integrator_CSTATE_a = z8_B.dV;

  /* Derivatives for Integrator: '<S9>/Integrator1' */
  _rtXdot->Integrator1_CSTATE_n = z8_B.dchi;

  /* Derivatives for Integrator: '<S9>/Integrator2' */
  _rtXdot->Integrator2_CSTATE_p = z8_B.dgamma;

  /* Derivatives for Integrator: '<S7>/Integrator' */
  _rtXdot->Integrator_CSTATE_i = z8_B.dalpha;

  /* Derivatives for Integrator: '<S7>/Integrator1' */
  _rtXdot->Integrator1_CSTATE_e = z8_B.dbeta;

  /* Derivatives for Integrator: '<S7>/Integrator2' */
  _rtXdot->Integrator2_CSTATE_g = z8_B.dsigma;

  /* Derivatives for Integrator: '<S8>/Integrator2' */
  _rtXdot->Integrator2_CSTATE_f = z8_B.dp;

  /* Derivatives for Integrator: '<S8>/Integrator1' */
  _rtXdot->Integrator1_CSTATE_d = z8_B.dq;

  /* Derivatives for Integrator: '<S8>/Integrator3' */
  _rtXdot->Integrator3_CSTATE = z8_B.dr;
}

/* Model initialize function */
void z8_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)z8_M, 0,
                sizeof(RT_MODEL_z8_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&z8_M->solverInfo, &z8_M->Timing.simTimeStep);
    rtsiSetTPtr(&z8_M->solverInfo, &rtmGetTPtr(z8_M));
    rtsiSetStepSizePtr(&z8_M->solverInfo, &z8_M->Timing.stepSize0);
    rtsiSetdXPtr(&z8_M->solverInfo, &z8_M->derivs);
    rtsiSetContStatesPtr(&z8_M->solverInfo, (real_T **) &z8_M->contStates);
    rtsiSetNumContStatesPtr(&z8_M->solverInfo, &z8_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&z8_M->solverInfo,
      &z8_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&z8_M->solverInfo,
      &z8_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&z8_M->solverInfo,
      &z8_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&z8_M->solverInfo, (boolean_T**)
      &z8_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&z8_M->solverInfo, (&rtmGetErrorStatus(z8_M)));
    rtsiSetRTModelPtr(&z8_M->solverInfo, z8_M);
  }

  rtsiSetSimTimeStep(&z8_M->solverInfo, MAJOR_TIME_STEP);
  z8_M->intgData.y = z8_M->odeY;
  z8_M->intgData.f[0] = z8_M->odeF[0];
  z8_M->intgData.f[1] = z8_M->odeF[1];
  z8_M->intgData.f[2] = z8_M->odeF[2];
  z8_M->contStates = ((X_z8_T *) &z8_X);
  z8_M->contStateDisabled = ((XDis_z8_T *) &z8_XDis);
  z8_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&z8_M->solverInfo, (void *)&z8_M->intgData);
  rtsiSetIsMinorTimeStepWithModeChange(&z8_M->solverInfo, false);
  rtsiSetSolverName(&z8_M->solverInfo,"ode3");
  rtmSetTPtr(z8_M, &z8_M->Timing.tArray[0]);
  rtmSetTFinal(z8_M, 10.0);
  z8_M->Timing.stepSize0 = 0.01;
  rtmSetFirstInitCond(z8_M, 1);

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    z8_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(z8_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(z8_M->rtwLogInfo, (NULL));
    rtliSetLogT(z8_M->rtwLogInfo, "tout");
    rtliSetLogX(z8_M->rtwLogInfo, "");
    rtliSetLogXFinal(z8_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(z8_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(z8_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(z8_M->rtwLogInfo, 0);
    rtliSetLogDecimation(z8_M->rtwLogInfo, 1);
    rtliSetLogY(z8_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(z8_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(z8_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &z8_B), 0,
                sizeof(B_z8_T));

  /* states (continuous) */
  {
    (void) memset((void *)&z8_X, 0,
                  sizeof(X_z8_T));
  }

  /* disabled states */
  {
    (void) memset((void *)&z8_XDis, 0,
                  sizeof(XDis_z8_T));
  }

  /* states (dwork) */
  (void) memset((void *)&z8_DW, 0,
                sizeof(DW_z8_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(z8_M->rtwLogInfo, 0.0, rtmGetTFinal(z8_M),
    z8_M->Timing.stepSize0, (&rtmGetErrorStatus(z8_M)));

  /* Start for Constant: '<Root>/Constant' */
  memcpy(&z8_B.Constant[0], &z8_P.Constant_Value[0], 12U * sizeof(real_T));

  /* InitializeConditions for Integrator: '<S6>/Integrator' incorporates:
   *  Integrator: '<S6>/Integrator1'
   */
  if (rtmIsFirstInitCond(z8_M)) {
    z8_X.Integrator_CSTATE = 6.411E+6;
    z8_X.Integrator1_CSTATE = 118.8;
  }

  z8_DW.Integrator_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S6>/Integrator' */

  /* InitializeConditions for Integrator: '<S6>/Integrator1' */
  z8_DW.Integrator1_IWORK = 1;

  /* InitializeConditions for Integrator: '<S6>/Integrator2' incorporates:
   *  Integrator: '<S9>/Integrator'
   */
  if (rtmIsFirstInitCond(z8_M)) {
    z8_X.Integrator2_CSTATE = 32.0;
    z8_X.Integrator_CSTATE_a = 3000.0;
  }

  z8_DW.Integrator2_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S6>/Integrator2' */

  /* InitializeConditions for Integrator: '<S9>/Integrator' */
  z8_DW.Integrator_IWORK_k = 1;

  /* InitializeConditions for Integrator: '<S9>/Integrator1' incorporates:
   *  Integrator: '<S9>/Integrator2'
   */
  if (rtmIsFirstInitCond(z8_M)) {
    z8_X.Integrator1_CSTATE_n = 180.0;
    z8_X.Integrator2_CSTATE_p = 0.0;
  }

  z8_DW.Integrator1_IWORK_i = 1;

  /* End of InitializeConditions for Integrator: '<S9>/Integrator1' */

  /* InitializeConditions for Integrator: '<S9>/Integrator2' */
  z8_DW.Integrator2_IWORK_f = 1;

  /* InitializeConditions for Integrator: '<S7>/Integrator' incorporates:
   *  Integrator: '<S7>/Integrator1'
   */
  if (rtmIsFirstInitCond(z8_M)) {
    z8_X.Integrator_CSTATE_i = 2.0;
    z8_X.Integrator1_CSTATE_e = 3.0;
  }

  z8_DW.Integrator_IWORK_l = 1;

  /* End of InitializeConditions for Integrator: '<S7>/Integrator' */

  /* InitializeConditions for Integrator: '<S7>/Integrator1' */
  z8_DW.Integrator1_IWORK_e = 1;

  /* InitializeConditions for Integrator: '<S7>/Integrator2' incorporates:
   *  Integrator: '<S8>/Integrator2'
   */
  if (rtmIsFirstInitCond(z8_M)) {
    z8_X.Integrator2_CSTATE_g = 2.0;
    z8_X.Integrator2_CSTATE_f = 0.0;
  }

  z8_DW.Integrator2_IWORK_fw = 1;

  /* End of InitializeConditions for Integrator: '<S7>/Integrator2' */

  /* InitializeConditions for Integrator: '<S8>/Integrator2' */
  z8_DW.Integrator2_IWORK_p = 1;

  /* InitializeConditions for Integrator: '<S8>/Integrator1' incorporates:
   *  Integrator: '<S8>/Integrator3'
   */
  if (rtmIsFirstInitCond(z8_M)) {
    z8_X.Integrator1_CSTATE_d = 0.0;
    z8_X.Integrator3_CSTATE = 0.0;
  }

  z8_DW.Integrator1_IWORK_l = 1;

  /* End of InitializeConditions for Integrator: '<S8>/Integrator1' */

  /* InitializeConditions for Integrator: '<S8>/Integrator3' */
  z8_DW.Integrator3_IWORK = 1;

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(z8_M)) {
    rtmSetFirstInitCond(z8_M, 0);
  }
}

/* Model terminate function */
void z8_terminate(void)
{
  /* (no terminate code required) */
}
