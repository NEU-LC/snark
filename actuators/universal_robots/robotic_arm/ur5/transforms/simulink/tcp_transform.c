/*
 * File: tcp_transform.c
 *
 * Code generated for Simulink model 'tcp_transform'.
 *
 * Model version                  : 1.9
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Mon Aug 04 14:27:15 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "tcp_transform.h"
#include "tcp_transform_private.h"

/* External inputs (root inport signals with auto storage) */
ExtU_tcp_transform_T tcp_transform_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_tcp_transform_T tcp_transform_Y;

/* Real-time model */
RT_MODEL_tcp_transform_T tcp_transform_M_;
RT_MODEL_tcp_transform_T *const tcp_transform_M = &tcp_transform_M_;

/* Forward declaration for local functions */
static void tcp_transform_cosd(real_T *x);
static void tcp_transform_sind(real_T *x);
real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T u1_0;
  if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf(u1)))))
  {
    y = (rtNaN);
  } else {
    if (u1 < 0.0) {
      u1_0 = ceil(u1);
    } else {
      u1_0 = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != u1_0)) {
      u1_0 = u0 / u1;
      if (fabs(u1_0 - rt_roundd_snf(u1_0)) <= DBL_EPSILON * fabs(u1_0)) {
        y = 0.0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S1>/getToolPosition' */
static void tcp_transform_cosd(real_T *x)
{
  int8_T n;
  real_T b_x;
  real_T absx;
  if (!((!rtIsInf(*x)) && (!rtIsNaN(*x)))) {
    *x = (rtNaN);
  } else {
    b_x = rt_remd_snf(*x, 360.0);
    absx = fabs(b_x);
    if (absx > 180.0) {
      if (b_x > 0.0) {
        b_x -= 360.0;
      } else {
        b_x += 360.0;
      }

      absx = fabs(b_x);
    }

    if (absx <= 45.0) {
      b_x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (b_x > 0.0) {
        b_x = (b_x - 90.0) * 0.017453292519943295;
        n = 1;
      } else {
        b_x = (b_x + 90.0) * 0.017453292519943295;
        n = -1;
      }
    } else if (b_x > 0.0) {
      b_x = (b_x - 180.0) * 0.017453292519943295;
      n = 2;
    } else {
      b_x = (b_x + 180.0) * 0.017453292519943295;
      n = -2;
    }

    if (n == 0) {
      *x = cos(b_x);
    } else if (n == 1) {
      *x = -sin(b_x);
    } else if (n == -1) {
      *x = sin(b_x);
    } else {
      *x = -cos(b_x);
    }
  }
}

/* Function for MATLAB Function: '<S1>/getToolPosition' */
static void tcp_transform_sind(real_T *x)
{
  int8_T n;
  real_T b_x;
  real_T absx;
  if (!((!rtIsInf(*x)) && (!rtIsNaN(*x)))) {
    b_x = (rtNaN);
  } else {
    b_x = rt_remd_snf(*x, 360.0);
    absx = fabs(b_x);
    if (absx > 180.0) {
      if (b_x > 0.0) {
        b_x -= 360.0;
      } else {
        b_x += 360.0;
      }

      absx = fabs(b_x);
    }

    if (absx <= 45.0) {
      b_x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (b_x > 0.0) {
        b_x = (b_x - 90.0) * 0.017453292519943295;
        n = 1;
      } else {
        b_x = (b_x + 90.0) * 0.017453292519943295;
        n = -1;
      }
    } else if (b_x > 0.0) {
      b_x = (b_x - 180.0) * 0.017453292519943295;
      n = 2;
    } else {
      b_x = (b_x + 180.0) * 0.017453292519943295;
      n = -2;
    }

    if (n == 0) {
      b_x = sin(b_x);
    } else if (n == 1) {
      b_x = cos(b_x);
    } else if (n == -1) {
      b_x = -cos(b_x);
    } else {
      b_x = -sin(b_x);
    }
  }

  *x = b_x;
}

/* Model step function */
void tcp_transform_step(void)
{
  real_T total_transform[16];
  real_T laser_position[4];
  real_T i;
  real_T j;
  real_T k;
  real_T m;
  real_T n;
  real_T o;
  real_T p;
  real_T q;
  real_T s;
  real_T t;
  real_T u;
  real_T v;
  real_T w;
  real_T y;
  real_T ab;
  real_T bb;
  real_T db;
  real_T eb;
  real_T fb;
  real_T hb;
  real_T ib;
  real_T jb;
  real_T i_0[16];
  real_T m_0[16];
  real_T i_1[16];
  real_T s_0[16];
  int32_T i_2;
  real_T y_0[16];
  real_T db_0[16];
  real_T hb_0[16];
  int32_T i_3;
  real_T rtb_TmpSignalConversionAtSFun_0;
  real_T rtb_TmpSignalConversionAtSFun_1;
  real_T rtb_TmpSignalConversionAtSFun_2;
  real_T rtb_TmpSignalConversionAtSFun_3;
  real_T rtb_TmpSignalConversionAtSFun_4;
  real_T rtb_TmpSignalConversionAtSFun_5;

  /* SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  Inport: '<Root>/Joint1'
   *  Inport: '<Root>/Joint2'
   *  Inport: '<Root>/Joint3'
   *  Inport: '<Root>/Joint4'
   *  Inport: '<Root>/Joint5'
   *  Inport: '<Root>/Joint6'
   *  MATLAB Function: '<S1>/getToolPosition'
   */
  rtb_TmpSignalConversionAtSFun_0 = tcp_transform_U.Joint1;
  rtb_TmpSignalConversionAtSFun_1 = tcp_transform_U.Joint2;
  rtb_TmpSignalConversionAtSFun_2 = tcp_transform_U.Joint3;
  rtb_TmpSignalConversionAtSFun_3 = tcp_transform_U.Joint4;
  rtb_TmpSignalConversionAtSFun_4 = tcp_transform_U.Joint5;
  rtb_TmpSignalConversionAtSFun_5 = tcp_transform_U.Joint6;

  /* MATLAB Function: '<S1>/getToolPosition' incorporates:
   *  Inport: '<Root>/Joint1'
   *  SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'tcp_transform/getToolPosition': '<S2>:1' */
  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* '<S2>:1:5' */
  /* '<S2>:1:6' */
  /* '<S2>:1:7' */
  /* '<S2>:1:8' */
  /* '<S2>:1:9' */
  /* '<S2>:1:10' */
  /* '<S2>:1:12' */
  i = tcp_transform_U.Joint1;
  tcp_transform_cosd(&i);
  j = rtb_TmpSignalConversionAtSFun_0;
  tcp_transform_sind(&j);
  k = rtb_TmpSignalConversionAtSFun_0;
  tcp_transform_sind(&k);
  tcp_transform_cosd(&rtb_TmpSignalConversionAtSFun_0);
  m = rtb_TmpSignalConversionAtSFun_1;
  tcp_transform_cosd(&m);
  n = rtb_TmpSignalConversionAtSFun_1;
  tcp_transform_sind(&n);
  o = rtb_TmpSignalConversionAtSFun_1;
  tcp_transform_cosd(&o);
  p = rtb_TmpSignalConversionAtSFun_1;
  tcp_transform_sind(&p);
  q = rtb_TmpSignalConversionAtSFun_1;
  tcp_transform_cosd(&q);
  tcp_transform_sind(&rtb_TmpSignalConversionAtSFun_1);
  s = rtb_TmpSignalConversionAtSFun_2;
  tcp_transform_cosd(&s);
  t = rtb_TmpSignalConversionAtSFun_2;
  tcp_transform_sind(&t);
  u = rtb_TmpSignalConversionAtSFun_2;
  tcp_transform_cosd(&u);
  v = rtb_TmpSignalConversionAtSFun_2;
  tcp_transform_sind(&v);
  w = rtb_TmpSignalConversionAtSFun_2;
  tcp_transform_cosd(&w);
  tcp_transform_sind(&rtb_TmpSignalConversionAtSFun_2);
  y = rtb_TmpSignalConversionAtSFun_3;
  tcp_transform_cosd(&y);
  ab = rtb_TmpSignalConversionAtSFun_3;
  tcp_transform_sind(&ab);
  bb = rtb_TmpSignalConversionAtSFun_3;
  tcp_transform_sind(&bb);
  tcp_transform_cosd(&rtb_TmpSignalConversionAtSFun_3);
  db = rtb_TmpSignalConversionAtSFun_4;
  tcp_transform_cosd(&db);
  eb = rtb_TmpSignalConversionAtSFun_4;
  tcp_transform_sind(&eb);
  fb = rtb_TmpSignalConversionAtSFun_4;
  tcp_transform_sind(&fb);
  tcp_transform_cosd(&rtb_TmpSignalConversionAtSFun_4);
  hb = rtb_TmpSignalConversionAtSFun_5;
  tcp_transform_cosd(&hb);
  ib = rtb_TmpSignalConversionAtSFun_5;
  tcp_transform_sind(&ib);
  jb = rtb_TmpSignalConversionAtSFun_5;
  tcp_transform_sind(&jb);
  tcp_transform_cosd(&rtb_TmpSignalConversionAtSFun_5);
  i_0[0] = i;
  i_0[4] = 0.0;
  i_0[8] = j;
  i_0[12] = 0.0;
  i_0[1] = k;
  i_0[5] = 0.0;
  i_0[9] = -rtb_TmpSignalConversionAtSFun_0;
  i_0[13] = 0.0;
  i_0[2] = 0.0;
  i_0[6] = 1.0;
  i_0[10] = 0.0;
  i_0[14] = 0.089;
  i_0[3] = 0.0;
  i_0[7] = 0.0;
  i_0[11] = 0.0;
  i_0[15] = 1.0;
  m_0[0] = m;
  m_0[4] = -n;
  m_0[8] = 0.0;
  m_0[12] = -0.425 * o;
  m_0[1] = p;
  m_0[5] = q;
  m_0[9] = 0.0;
  m_0[13] = -0.425 * rtb_TmpSignalConversionAtSFun_1;
  m_0[2] = 0.0;
  m_0[6] = 0.0;
  m_0[10] = 1.0;
  m_0[14] = 0.0;
  m_0[3] = 0.0;
  m_0[7] = 0.0;
  m_0[11] = 0.0;
  m_0[15] = 1.0;
  for (i_2 = 0; i_2 < 4; i_2++) {
    for (i_3 = 0; i_3 < 4; i_3++) {
      i_1[i_2 + (i_3 << 2)] = 0.0;
      i_1[i_2 + (i_3 << 2)] += m_0[i_3 << 2] * i_0[i_2];
      i_1[i_2 + (i_3 << 2)] += m_0[(i_3 << 2) + 1] * i_0[i_2 + 4];
      i_1[i_2 + (i_3 << 2)] += m_0[(i_3 << 2) + 2] * i_0[i_2 + 8];
      i_1[i_2 + (i_3 << 2)] += m_0[(i_3 << 2) + 3] * i_0[i_2 + 12];
    }
  }

  s_0[0] = s;
  s_0[4] = -t;
  s_0[8] = 0.0;
  s_0[12] = -0.392 * u;
  s_0[1] = v;
  s_0[5] = w;
  s_0[9] = 0.0;
  s_0[13] = -0.392 * rtb_TmpSignalConversionAtSFun_2;
  s_0[2] = 0.0;
  s_0[6] = 0.0;
  s_0[10] = 1.0;
  s_0[14] = 0.0;
  s_0[3] = 0.0;
  s_0[7] = 0.0;
  s_0[11] = 0.0;
  s_0[15] = 1.0;
  for (i_2 = 0; i_2 < 4; i_2++) {
    for (i_3 = 0; i_3 < 4; i_3++) {
      i_0[i_2 + (i_3 << 2)] = 0.0;
      i_0[i_2 + (i_3 << 2)] += s_0[i_3 << 2] * i_1[i_2];
      i_0[i_2 + (i_3 << 2)] += s_0[(i_3 << 2) + 1] * i_1[i_2 + 4];
      i_0[i_2 + (i_3 << 2)] += s_0[(i_3 << 2) + 2] * i_1[i_2 + 8];
      i_0[i_2 + (i_3 << 2)] += s_0[(i_3 << 2) + 3] * i_1[i_2 + 12];
    }
  }

  y_0[0] = y;
  y_0[4] = 0.0;
  y_0[8] = ab;
  y_0[12] = 0.0;
  y_0[1] = bb;
  y_0[5] = 0.0;
  y_0[9] = -rtb_TmpSignalConversionAtSFun_3;
  y_0[13] = 0.0;
  y_0[2] = 0.0;
  y_0[6] = 1.0;
  y_0[10] = 0.0;
  y_0[14] = 0.109;
  y_0[3] = 0.0;
  y_0[7] = 0.0;
  y_0[11] = 0.0;
  y_0[15] = 1.0;
  for (i_2 = 0; i_2 < 4; i_2++) {
    for (i_3 = 0; i_3 < 4; i_3++) {
      i_1[i_2 + (i_3 << 2)] = 0.0;
      i_1[i_2 + (i_3 << 2)] += y_0[i_3 << 2] * i_0[i_2];
      i_1[i_2 + (i_3 << 2)] += y_0[(i_3 << 2) + 1] * i_0[i_2 + 4];
      i_1[i_2 + (i_3 << 2)] += y_0[(i_3 << 2) + 2] * i_0[i_2 + 8];
      i_1[i_2 + (i_3 << 2)] += y_0[(i_3 << 2) + 3] * i_0[i_2 + 12];
    }
  }

  db_0[0] = db;
  db_0[4] = 0.0;
  db_0[8] = -eb;
  db_0[12] = 0.0;
  db_0[1] = fb;
  db_0[5] = 0.0;
  db_0[9] = rtb_TmpSignalConversionAtSFun_4;
  db_0[13] = 0.0;
  db_0[2] = 0.0;
  db_0[6] = -1.0;
  db_0[10] = 0.0;
  db_0[14] = 0.095;
  db_0[3] = 0.0;
  db_0[7] = 0.0;
  db_0[11] = 0.0;
  db_0[15] = 1.0;
  for (i_2 = 0; i_2 < 4; i_2++) {
    for (i_3 = 0; i_3 < 4; i_3++) {
      i_0[i_2 + (i_3 << 2)] = 0.0;
      i_0[i_2 + (i_3 << 2)] += db_0[i_3 << 2] * i_1[i_2];
      i_0[i_2 + (i_3 << 2)] += db_0[(i_3 << 2) + 1] * i_1[i_2 + 4];
      i_0[i_2 + (i_3 << 2)] += db_0[(i_3 << 2) + 2] * i_1[i_2 + 8];
      i_0[i_2 + (i_3 << 2)] += db_0[(i_3 << 2) + 3] * i_1[i_2 + 12];
    }
  }

  hb_0[0] = hb;
  hb_0[4] = -ib;
  hb_0[8] = 0.0;
  hb_0[12] = 0.0;
  hb_0[1] = jb;
  hb_0[5] = rtb_TmpSignalConversionAtSFun_5;
  hb_0[9] = 0.0;
  hb_0[13] = 0.0;
  hb_0[2] = 0.0;
  hb_0[6] = 0.0;
  hb_0[10] = 1.0;
  hb_0[14] = 0.082;
  hb_0[3] = 0.0;
  hb_0[7] = 0.0;
  hb_0[11] = 0.0;
  hb_0[15] = 1.0;
  for (i_2 = 0; i_2 < 4; i_2++) {
    for (i_3 = 0; i_3 < 4; i_3++) {
      total_transform[i_2 + (i_3 << 2)] = 0.0;
      total_transform[i_2 + (i_3 << 2)] += hb_0[i_3 << 2] * i_0[i_2];
      total_transform[i_2 + (i_3 << 2)] += hb_0[(i_3 << 2) + 1] * i_0[i_2 + 4];
      total_transform[i_2 + (i_3 << 2)] += hb_0[(i_3 << 2) + 2] * i_0[i_2 + 8];
      total_transform[i_2 + (i_3 << 2)] += hb_0[(i_3 << 2) + 3] * i_0[i_2 + 12];
    }
  }

  /* laser_offset = -0.110; %negative = upwards */
  /* laser_position = total_transform*[0;laser_offset;0;1]; */
  /* '<S2>:1:17' */
  for (i_2 = 0; i_2 < 4; i_2++) {
    rtb_TmpSignalConversionAtSFun_0 = total_transform[i_2 + 12] +
      (total_transform[i_2 + 8] * 0.0 + (total_transform[i_2 + 4] * 0.0 +
        total_transform[i_2] * 0.0));
    laser_position[i_2] = rtb_TmpSignalConversionAtSFun_0;
  }

  /* Outport: '<Root>/x' incorporates:
   *  MATLAB Function: '<S1>/getToolPosition'
   */
  /* end effector position */
  /* '<S2>:1:20' */
  /* '<S2>:1:21' */
  /* '<S2>:1:22' */
  /* co-ordinate system rotational transformation */
  /* '<S2>:1:25' */
  /* '<S2>:1:27' */
  /* '<S2>:1:28' */
  /* '<S2>:1:29' */
  tcp_transform_Y.x = laser_position[0];

  /* Outport: '<Root>/y' incorporates:
   *  MATLAB Function: '<S1>/getToolPosition'
   */
  tcp_transform_Y.y = laser_position[1];

  /* Outport: '<Root>/z' incorporates:
   *  MATLAB Function: '<S1>/getToolPosition'
   */
  tcp_transform_Y.z = laser_position[2];

  /* Outport: '<Root>/Pan' incorporates:
   *  MATLAB Function: '<S1>/getToolPosition'
   */
  tcp_transform_Y.Pan = 57.295779513082323 * asin(total_transform[9]);

  /* Outport: '<Root>/Tilt' incorporates:
   *  MATLAB Function: '<S1>/getToolPosition'
   */
  tcp_transform_Y.Tilt = atan(total_transform[10] / total_transform[8]) *
    57.295779513082323;

  /* Outport: '<Root>/Roll' incorporates:
   *  MATLAB Function: '<S1>/getToolPosition'
   */
  tcp_transform_Y.Roll = 0.0;
}

/* Model initialize function */
void tcp_transform_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(tcp_transform_M, (NULL));

  /* external inputs */
  (void) memset((void *)&tcp_transform_U, 0,
                sizeof(ExtU_tcp_transform_T));

  /* external outputs */
  (void) memset((void *)&tcp_transform_Y, 0,
                sizeof(ExtY_tcp_transform_T));
}

/* Model terminate function */
void tcp_transform_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
