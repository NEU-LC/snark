/*
 * File: tcp_transform.c
 *
 * Code generated for Simulink model 'tcp_transform'.
 *
 * Model version                  : 1.22
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Mon Sep 22 14:50:15 2014
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

/* Function for MATLAB Function: '<S1>/getToolLaser' */
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

/* Function for MATLAB Function: '<S1>/getToolLaser' */
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

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void tcp_transform_step(void)
{
  real_T total_transform[16];
  real_T tcp_position[4];
  real_T laser_orientation[9];
  real_T yaw;
  static const int8_T b[9] = { 0, 0, 1, 1, 0, 0, 0, 1, 0 };

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
  real_T yaw_0[16];
  real_T m_0[16];
  real_T yaw_1[16];
  real_T s_0[16];
  int32_T i;
  real_T y_0[16];
  real_T db_0[16];
  real_T hb_0[16];
  real_T total_transform_0[9];
  int32_T i_0;
  real_T joint_angles_degrees_idx_0;
  real_T joint_angles_degrees_idx_1;
  real_T joint_angles_degrees_idx_2;
  real_T joint_angles_degrees_idx_3;
  real_T joint_angles_degrees_idx_4;
  real_T joint_angles_degrees_idx_5;

  /* MATLAB Function: '<S1>/getToolLaser' incorporates:
   *  Inport: '<Root>/Joint1'
   *  Inport: '<Root>/Joint2'
   *  Inport: '<Root>/Joint3'
   *  Inport: '<Root>/Joint4'
   *  Inport: '<Root>/Joint5'
   *  Inport: '<Root>/Joint6'
   *  SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'tcp_transform/getToolLaser': '<S2>:1' */
  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* '<S2>:1:5' */
  joint_angles_degrees_idx_0 = tcp_transform_U.Joint1 * 57.295779513082323;
  joint_angles_degrees_idx_1 = tcp_transform_U.Joint2 * 57.295779513082323;
  joint_angles_degrees_idx_2 = tcp_transform_U.Joint3 * 57.295779513082323;
  joint_angles_degrees_idx_3 = tcp_transform_U.Joint4 * 57.295779513082323;
  joint_angles_degrees_idx_4 = tcp_transform_U.Joint5 * 57.295779513082323;
  joint_angles_degrees_idx_5 = tcp_transform_U.Joint6 * 57.295779513082323;

  /* convert to degrees */
  /* '<S2>:1:6' */
  /* '<S2>:1:7' */
  /* '<S2>:1:8' */
  /* '<S2>:1:9' */
  /* '<S2>:1:10' */
  /* '<S2>:1:11' */
  /* '<S2>:1:13' */
  yaw = joint_angles_degrees_idx_0;
  tcp_transform_cosd(&yaw);
  j = joint_angles_degrees_idx_0;
  tcp_transform_sind(&j);
  k = joint_angles_degrees_idx_0;
  tcp_transform_sind(&k);
  tcp_transform_cosd(&joint_angles_degrees_idx_0);
  m = joint_angles_degrees_idx_1;
  tcp_transform_cosd(&m);
  n = joint_angles_degrees_idx_1;
  tcp_transform_sind(&n);
  o = joint_angles_degrees_idx_1;
  tcp_transform_cosd(&o);
  p = joint_angles_degrees_idx_1;
  tcp_transform_sind(&p);
  q = joint_angles_degrees_idx_1;
  tcp_transform_cosd(&q);
  tcp_transform_sind(&joint_angles_degrees_idx_1);
  s = joint_angles_degrees_idx_2;
  tcp_transform_cosd(&s);
  t = joint_angles_degrees_idx_2;
  tcp_transform_sind(&t);
  u = joint_angles_degrees_idx_2;
  tcp_transform_cosd(&u);
  v = joint_angles_degrees_idx_2;
  tcp_transform_sind(&v);
  w = joint_angles_degrees_idx_2;
  tcp_transform_cosd(&w);
  tcp_transform_sind(&joint_angles_degrees_idx_2);
  y = joint_angles_degrees_idx_3;
  tcp_transform_cosd(&y);
  ab = joint_angles_degrees_idx_3;
  tcp_transform_sind(&ab);
  bb = joint_angles_degrees_idx_3;
  tcp_transform_sind(&bb);
  tcp_transform_cosd(&joint_angles_degrees_idx_3);
  db = joint_angles_degrees_idx_4;
  tcp_transform_cosd(&db);
  eb = joint_angles_degrees_idx_4;
  tcp_transform_sind(&eb);
  fb = joint_angles_degrees_idx_4;
  tcp_transform_sind(&fb);
  tcp_transform_cosd(&joint_angles_degrees_idx_4);
  hb = joint_angles_degrees_idx_5;
  tcp_transform_cosd(&hb);
  ib = joint_angles_degrees_idx_5;
  tcp_transform_sind(&ib);
  jb = joint_angles_degrees_idx_5;
  tcp_transform_sind(&jb);
  tcp_transform_cosd(&joint_angles_degrees_idx_5);
  yaw_0[0] = yaw;
  yaw_0[4] = 0.0;
  yaw_0[8] = j;
  yaw_0[12] = 0.0;
  yaw_0[1] = k;
  yaw_0[5] = 0.0;
  yaw_0[9] = -joint_angles_degrees_idx_0;
  yaw_0[13] = 0.0;
  yaw_0[2] = 0.0;
  yaw_0[6] = 1.0;
  yaw_0[10] = 0.0;
  yaw_0[14] = 0.089;
  yaw_0[3] = 0.0;
  yaw_0[7] = 0.0;
  yaw_0[11] = 0.0;
  yaw_0[15] = 1.0;
  m_0[0] = m;
  m_0[4] = -n;
  m_0[8] = 0.0;
  m_0[12] = -0.425 * o;
  m_0[1] = p;
  m_0[5] = q;
  m_0[9] = 0.0;
  m_0[13] = -0.425 * joint_angles_degrees_idx_1;
  m_0[2] = 0.0;
  m_0[6] = 0.0;
  m_0[10] = 1.0;
  m_0[14] = 0.0;
  m_0[3] = 0.0;
  m_0[7] = 0.0;
  m_0[11] = 0.0;
  m_0[15] = 1.0;
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      yaw_1[i + (i_0 << 2)] = 0.0;
      yaw_1[i + (i_0 << 2)] += m_0[i_0 << 2] * yaw_0[i];
      yaw_1[i + (i_0 << 2)] += m_0[(i_0 << 2) + 1] * yaw_0[i + 4];
      yaw_1[i + (i_0 << 2)] += m_0[(i_0 << 2) + 2] * yaw_0[i + 8];
      yaw_1[i + (i_0 << 2)] += m_0[(i_0 << 2) + 3] * yaw_0[i + 12];
    }
  }

  s_0[0] = s;
  s_0[4] = -t;
  s_0[8] = 0.0;
  s_0[12] = -0.392 * u;
  s_0[1] = v;
  s_0[5] = w;
  s_0[9] = 0.0;
  s_0[13] = -0.392 * joint_angles_degrees_idx_2;
  s_0[2] = 0.0;
  s_0[6] = 0.0;
  s_0[10] = 1.0;
  s_0[14] = 0.0;
  s_0[3] = 0.0;
  s_0[7] = 0.0;
  s_0[11] = 0.0;
  s_0[15] = 1.0;
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      yaw_0[i + (i_0 << 2)] = 0.0;
      yaw_0[i + (i_0 << 2)] += s_0[i_0 << 2] * yaw_1[i];
      yaw_0[i + (i_0 << 2)] += s_0[(i_0 << 2) + 1] * yaw_1[i + 4];
      yaw_0[i + (i_0 << 2)] += s_0[(i_0 << 2) + 2] * yaw_1[i + 8];
      yaw_0[i + (i_0 << 2)] += s_0[(i_0 << 2) + 3] * yaw_1[i + 12];
    }
  }

  y_0[0] = y;
  y_0[4] = 0.0;
  y_0[8] = ab;
  y_0[12] = 0.0;
  y_0[1] = bb;
  y_0[5] = 0.0;
  y_0[9] = -joint_angles_degrees_idx_3;
  y_0[13] = 0.0;
  y_0[2] = 0.0;
  y_0[6] = 1.0;
  y_0[10] = 0.0;
  y_0[14] = 0.109;
  y_0[3] = 0.0;
  y_0[7] = 0.0;
  y_0[11] = 0.0;
  y_0[15] = 1.0;
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      yaw_1[i + (i_0 << 2)] = 0.0;
      yaw_1[i + (i_0 << 2)] += y_0[i_0 << 2] * yaw_0[i];
      yaw_1[i + (i_0 << 2)] += y_0[(i_0 << 2) + 1] * yaw_0[i + 4];
      yaw_1[i + (i_0 << 2)] += y_0[(i_0 << 2) + 2] * yaw_0[i + 8];
      yaw_1[i + (i_0 << 2)] += y_0[(i_0 << 2) + 3] * yaw_0[i + 12];
    }
  }

  db_0[0] = db;
  db_0[4] = 0.0;
  db_0[8] = -eb;
  db_0[12] = 0.0;
  db_0[1] = fb;
  db_0[5] = 0.0;
  db_0[9] = joint_angles_degrees_idx_4;
  db_0[13] = 0.0;
  db_0[2] = 0.0;
  db_0[6] = -1.0;
  db_0[10] = 0.0;
  db_0[14] = 0.095;
  db_0[3] = 0.0;
  db_0[7] = 0.0;
  db_0[11] = 0.0;
  db_0[15] = 1.0;
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      yaw_0[i + (i_0 << 2)] = 0.0;
      yaw_0[i + (i_0 << 2)] += db_0[i_0 << 2] * yaw_1[i];
      yaw_0[i + (i_0 << 2)] += db_0[(i_0 << 2) + 1] * yaw_1[i + 4];
      yaw_0[i + (i_0 << 2)] += db_0[(i_0 << 2) + 2] * yaw_1[i + 8];
      yaw_0[i + (i_0 << 2)] += db_0[(i_0 << 2) + 3] * yaw_1[i + 12];
    }
  }

  hb_0[0] = hb;
  hb_0[4] = -ib;
  hb_0[8] = 0.0;
  hb_0[12] = 0.0;
  hb_0[1] = jb;
  hb_0[5] = joint_angles_degrees_idx_5;
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
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      total_transform[i + (i_0 << 2)] = 0.0;
      total_transform[i + (i_0 << 2)] += hb_0[i_0 << 2] * yaw_0[i];
      total_transform[i + (i_0 << 2)] += hb_0[(i_0 << 2) + 1] * yaw_0[i + 4];
      total_transform[i + (i_0 << 2)] += hb_0[(i_0 << 2) + 2] * yaw_0[i + 8];
      total_transform[i + (i_0 << 2)] += hb_0[(i_0 << 2) + 3] * yaw_0[i + 12];
    }
  }

  /* '<S2>:1:15' */
  for (i = 0; i < 4; i++) {
    joint_angles_degrees_idx_0 = total_transform[i + 12] + (total_transform[i +
      8] * 0.0 + (total_transform[i + 4] * 0.0 + total_transform[i] * 0.0));
    tcp_position[i] = joint_angles_degrees_idx_0;
  }

  /* Outport: '<Root>/x' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  /* '<S2>:1:17' */
  tcp_transform_Y.x = tcp_position[0];

  /* Outport: '<Root>/y' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  /* '<S2>:1:18' */
  tcp_transform_Y.y = tcp_position[1];

  /* Outport: '<Root>/z' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  /* '<S2>:1:19' */
  tcp_transform_Y.z = tcp_position[2];

  /* MATLAB Function: '<S1>/getToolLaser' */
  /* negative = downwards */
  /* '<S2>:1:23' */
  for (i = 0; i < 4; i++) {
    joint_angles_degrees_idx_0 = total_transform[i + 12] + (total_transform[i +
      8] * 0.0 + (total_transform[i + 4] * 0.11 + total_transform[i] * 0.0));
    tcp_position[i] = joint_angles_degrees_idx_0;
  }

  /* '<S2>:1:25' */
  /* '<S2>:1:26' */
  /* '<S2>:1:27' */
  /* tool_orientation = [total_transform(1,1),total_transform(1,2),total_transform(1,3); total_transform(2,1),total_transform(2,2),total_transform(2,3); total_transform(3,1),total_transform(3,2),total_transform(3,3)]; */
  /* co-ordinate system rotational transformation */
  /* '<S2>:1:32' */
  /* pan = asind(laser_orientation(2,3)); */
  /* tilt = atand(laser_orientation(3,3)/laser_orientation(1,3)); */
  /* roll = 0; */
  /* default_pos_transform = [0 1 0;0 0 1;1 0 0]; %giraffe position laser orientation transposed */
  /* laser_orientation = laser_orientation*default_pos_transform; %finds transformation between new position and default giraffe position */
  /* roll = atand(laser_orientation(3,2)/laser_orientation(3,3)); */
  /* tilt = -atand(-laser_orientation(3,1)/sqrt((laser_orientation(3,2)^2) + (laser_orientation(3,3)^2))); %negative sign reverse direction */
  /* pan = atand(laser_orientation(2,1)/laser_orientation(1,1)); */
  /* pan = asin(tool_orientation(2,3)); */
  /* tilt = atan(tool_orientation(3,3)/tool_orientation(1,3)); */
  /* roll = 0; */
  /* default_pos_transform = [0 1 0;0 0 1;1 0 0]; %giraffe position laser orientation transposed */
  /* laser_orientation = default_pos_transform*laser_orientation; %finds transformation between new position and default giraffe position */
  /* Rx = atand(laser_orientation(3,2)/laser_orientation(3,3)); %roll about the end effector x-axis */
  /* Ry = atand(-laser_orientation(3,1)/sqrt((laser_orientation(3,2)^2) + (laser_orientation(3,3)^2))); %pitch about the end effector y-axis */
  /* Rz = atand(laser_orientation(2,1)/laser_orientation(1,1)); %yaw about the end effector z-axis */
  /* '<S2>:1:57' */
  total_transform_0[0] = total_transform[0];
  total_transform_0[3] = total_transform[4];
  total_transform_0[6] = total_transform[8];
  total_transform_0[1] = total_transform[1];
  total_transform_0[4] = total_transform[5];
  total_transform_0[7] = total_transform[9];
  total_transform_0[2] = total_transform[2];
  total_transform_0[5] = total_transform[6];
  total_transform_0[8] = total_transform[10];
  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      laser_orientation[i + 3 * i_0] = 0.0;
      laser_orientation[i + 3 * i_0] += (real_T)b[3 * i_0] * total_transform_0[i];
      laser_orientation[i + 3 * i_0] += (real_T)b[3 * i_0 + 1] *
        total_transform_0[i + 3];
      laser_orientation[i + 3 * i_0] += (real_T)b[3 * i_0 + 2] *
        total_transform_0[i + 6];
    }
  }

  /* '<S2>:1:59' */
  if ((laser_orientation[2] == 1.0) || (laser_orientation[2] == -1.0)) {
    /* '<S2>:1:60' */
    /* '<S2>:1:61' */
    k = 0.0;

    /* '<S2>:1:62' */
    yaw = 57.295779513082323 * rt_atan2d_snf(laser_orientation[7],
      laser_orientation[6]);
  } else {
    /* '<S2>:1:64' */
    k = 57.295779513082323 * rt_atan2d_snf(laser_orientation[5],
      laser_orientation[8]);

    /* '<S2>:1:65' */
    yaw = 57.295779513082323 * rt_atan2d_snf(laser_orientation[1],
      laser_orientation[0]);
  }

  /* '<S2>:1:69' */
  k *= 0.017453292519943295;

  /* '<S2>:1:70' */
  yaw *= 0.017453292519943295;

  /* '<S2>:1:71' */
  j = 57.295779513082323 * asin(-laser_orientation[2]) * 0.017453292519943295;

  /* Outport: '<Root>/Pan' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  /* '<S2>:1:73' */
  /* '<S2>:1:74' */
  /* '<S2>:1:75' */
  tcp_transform_Y.Pan = yaw;

  /* Outport: '<Root>/Tilt' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.Tilt = j;

  /* Outport: '<Root>/Roll' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.Roll = k;

  /* Outport: '<Root>/x_laser' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.x_laser = tcp_position[0];

  /* Outport: '<Root>/y_laser' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.y_laser = tcp_position[1];

  /* Outport: '<Root>/z_laser' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.z_laser = tcp_position[2];

  /* Outport: '<Root>/pan_laser' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.pan_laser = yaw;

  /* Outport: '<Root>/tilt_laser' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.tilt_laser = j;

  /* Outport: '<Root>/roll_laser' incorporates:
   *  MATLAB Function: '<S1>/getToolLaser'
   */
  tcp_transform_Y.roll_laser = k;
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
