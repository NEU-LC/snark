/*
 * File: Arm_controller_v2.c
 *
 * Code generated for Simulink model 'Arm_controller_v2'.
 *
 * Model version                  : 1.87
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Tue Sep 16 11:48:38 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Arm_controller_v2.h"
#include "Arm_controller_v2_private.h"

/* Block states (auto storage) */
DW_Arm_controller_v2_T Arm_controller_v2_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_Arm_controller_v2_T Arm_controller_v2_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_Arm_controller_v2_T Arm_controller_v2_Y;

/* Real-time model */
RT_MODEL_Arm_controller_v2_T Arm_controller_v2_M_;
RT_MODEL_Arm_controller_v2_T *const Arm_controller_v2_M = &Arm_controller_v2_M_;

/* Forward declaration for local functions */
static void Arm_controller_v2_cosd_g(real_T *x);
static void Arm_controller_v2_sind_j(real_T *x);
static real_T Arm_controller_v2_norm_g(const real_T x[3]);
static real_T Arm_controller_v2_dot_p(const real_T a[3], const real_T b[3]);
static real_T Arm_contr_DistBetween2Segment_m(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3]);
static void Arm_controller_v2_cosd(real_T *x);
static void Arm_controller_v2_sind(real_T *x);
static real_T Arm_controller_v2_norm(const real_T x[3]);
static real_T Arm_controller_v2_dot(const real_T a[3], const real_T b[3]);
static real_T Arm_control_DistBetween2Segment(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3]);
static real_T Arm_contro_check_self_collision(const real_T joint_angles_degrees
  [6]);
static void Arm_controller_v2_cosd_b(real_T *x);
static void Arm_controller_v2_sind_c(real_T *x);
static real_T Arm_controller_v2_norm_c(const real_T x[3]);
static real_T Arm_controller_v2_dot_e(const real_T a[3], const real_T b[3]);
static real_T Arm_contr_DistBetween2Segment_g(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3]);
static real_T Arm_controller__check_collision(const real_T joint_angles_degrees
  [6]);
static void Arm_controller_v2_getIK(real_T x_pos, real_T y_pos, real_T z_pos,
  real_T *num_sols, real_T q_sols[48]);
static void Arm_controller_v2_cosd_m(real_T *x);
static void Arm_controller_v2_sind_f(real_T *x);
static real_T Arm_controller_v2_norm_m(const real_T x[3]);
static real_T Arm_controller_v2_dot_n(const real_T a[3], const real_T b[3]);
static real_T Arm_contr_DistBetween2Segment_a(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3]);
static real_T Arm_controlle_check_collision_k(const real_T joint_angles_degrees
  [6]);
static void Arm_controller_v2_cosd_e(real_T *x);
static void Arm_controller_v2_sind_p(real_T *x);
static real_T Arm_controller_v2_norm_e(const real_T x[3]);
static real_T Arm_controller_v2_dot_b(const real_T a[3], const real_T b[3]);
static real_T Arm_contr_DistBetween2Segment_o(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3]);
static real_T Arm_controlle_check_collision_f(const real_T joint_angles_degrees
  [6]);
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

/* Function for MATLAB Function: '<S1>/move_cam' */
static void Arm_controller_v2_cosd_g(real_T *x)
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

/* Function for MATLAB Function: '<S1>/move_cam' */
static void Arm_controller_v2_sind_j(real_T *x)
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

/* Function for MATLAB Function: '<S1>/move_cam' */
static real_T Arm_controller_v2_norm_g(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 2.2250738585072014E-308;
  absxk = fabs(x[0]);
  if (absxk > 2.2250738585072014E-308) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S1>/move_cam' */
static real_T Arm_controller_v2_dot_p(const real_T a[3], const real_T b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

/* Function for MATLAB Function: '<S1>/move_cam' */
static real_T Arm_contr_DistBetween2Segment_m(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3])
{
  real_T distance;
  real_T u[3];
  real_T v[3];
  real_T w[3];
  real_T a;
  real_T b;
  real_T c;
  real_T d;
  real_T e;
  real_T D;
  real_T tD;
  real_T sN;
  real_T tN;
  real_T w_0[3];

  /*  Computes the minimum distance between two line segments. Code */
  /*  is adapted for Matlab from Dan Sunday's Geometry Algorithms originally */
  /*  written in C++ */
  /*  http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment */
  /*  Usage: Input the start and end x,y,z coordinates for two line segments.  */
  /*  p1, p2 are [x,y,z] coordinates of first line segment and p3,p4 are for */
  /*  second line segment.  */
  /*  Output: scalar minimum distance between the two segments. */
  /*   Example: */
  /* 	P1 = [0 0 0];     P2 = [1 0 0]; */
  /*    P3 = [0 1 0];     P4 = [1 1 0]; */
  /* 	dist = DistBetween2Segment(P1, P2, P3, P4) */
  /*    dist = */
  /*  */
  /*     1 */
  /*   */
  u[0] = p1[0] - p2[0];
  u[1] = p1[1] - p2[1];
  u[2] = p1[2] - p2[2];
  v[0] = p3[0] - p4[0];
  v[1] = p3[1] - p4[1];
  v[2] = p3[2] - p4[2];
  w[0] = p2[0] - p4[0];
  w[1] = p2[1] - p4[1];
  w[2] = p2[2] - p4[2];
  a = Arm_controller_v2_dot_p(u, u);
  b = Arm_controller_v2_dot_p(u, v);
  c = Arm_controller_v2_dot_p(v, v);
  d = Arm_controller_v2_dot_p(u, w);
  e = Arm_controller_v2_dot_p(v, w);
  D = a * c - b * b;
  tD = D;

  /*  compute the line parameters of the two closest points */
  if (D < 1.0E-8) {
    /*  the lines are almost parallel */
    sN = 0.0;

    /*  force using point P0 on segment S1 */
    D = 1.0;

    /*  to prevent possible division by 0.0 later */
    tN = e;
    tD = c;
  } else {
    /*  get the closest points on the infinite lines */
    sN = b * e - c * d;
    tN = a * e - b * d;
    if (sN < 0.0) {
      /*  sc < 0 => the s=0 edge is visible        */
      sN = 0.0;
      tN = e;
      tD = c;
    } else {
      if (sN > D) {
        /*  sc > 1 => the s=1 edge is visible */
        sN = D;
        tN = e + b;
        tD = c;
      }
    }
  }

  if (tN < 0.0) {
    /*  tc < 0 => the t=0 edge is visible */
    tN = 0.0;

    /*  recompute sc for this edge */
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = D;
    } else {
      sN = -d;
      D = a;
    }
  } else {
    if (tN > tD) {
      /*  tc > 1 => the t=1 edge is visible */
      tN = tD;

      /*  recompute sc for this edge */
      if (-d + b < 0.0) {
        sN = 0.0;
      } else if (-d + b > a) {
        sN = D;
      } else {
        sN = -d + b;
        D = a;
      }
    }
  }

  /*  finally do the division to get sc and tc */
  if (fabs(sN) < 1.0E-8) {
    b = 0.0;
  } else {
    b = sN / D;
  }

  if (fabs(tN) < 1.0E-8) {
    a = 0.0;
  } else {
    a = tN / tD;
  }

  /*  get the difference of the two closest points */
  /*  = S1(sc) - S2(tc) */
  w_0[0] = (b * u[0] + w[0]) - a * v[0];
  w_0[1] = (b * u[1] + w[1]) - a * v[1];
  w_0[2] = (b * u[2] + w[2]) - a * v[2];
  distance = Arm_controller_v2_norm_g(w_0);

  /* outV = dP; */
  /* varargout(1) = {outV};      % vector connecting the closest points */
  /* varargout(2) = {p2+sc*u};   % Closest point on object 1  */
  /* varargout(3) = {p4+tc*v};   % Closest point on object 2 */
  return distance;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void Arm_controller_v2_cosd(real_T *x)
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

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void Arm_controller_v2_sind(real_T *x)
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

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T Arm_controller_v2_norm(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 2.2250738585072014E-308;
  absxk = fabs(x[0]);
  if (absxk > 2.2250738585072014E-308) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T Arm_controller_v2_dot(const real_T a[3], const real_T b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T Arm_control_DistBetween2Segment(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3])
{
  real_T distance;
  real_T u[3];
  real_T v[3];
  real_T w[3];
  real_T a;
  real_T b;
  real_T c;
  real_T d;
  real_T e;
  real_T D;
  real_T tD;
  real_T sN;
  real_T tN;
  real_T w_0[3];

  /*  Computes the minimum distance between two line segments. Code */
  /*  is adapted for Matlab from Dan Sunday's Geometry Algorithms originally */
  /*  written in C++ */
  /*  http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment */
  /*  Usage: Input the start and end x,y,z coordinates for two line segments.  */
  /*  p1, p2 are [x,y,z] coordinates of first line segment and p3,p4 are for */
  /*  second line segment.  */
  /*  Output: scalar minimum distance between the two segments. */
  /*   Example: */
  /* 	P1 = [0 0 0];     P2 = [1 0 0]; */
  /*    P3 = [0 1 0];     P4 = [1 1 0]; */
  /* 	dist = DistBetween2Segment(P1, P2, P3, P4) */
  /*    dist = */
  /*  */
  /*     1 */
  /*   */
  u[0] = p1[0] - p2[0];
  u[1] = p1[1] - p2[1];
  u[2] = p1[2] - p2[2];
  v[0] = p3[0] - p4[0];
  v[1] = p3[1] - p4[1];
  v[2] = p3[2] - p4[2];
  w[0] = p2[0] - p4[0];
  w[1] = p2[1] - p4[1];
  w[2] = p2[2] - p4[2];
  a = Arm_controller_v2_dot(u, u);
  b = Arm_controller_v2_dot(u, v);
  c = Arm_controller_v2_dot(v, v);
  d = Arm_controller_v2_dot(u, w);
  e = Arm_controller_v2_dot(v, w);
  D = a * c - b * b;
  tD = D;

  /*  compute the line parameters of the two closest points */
  if (D < 1.0E-8) {
    /*  the lines are almost parallel */
    sN = 0.0;

    /*  force using point P0 on segment S1 */
    D = 1.0;

    /*  to prevent possible division by 0.0 later */
    tN = e;
    tD = c;
  } else {
    /*  get the closest points on the infinite lines */
    sN = b * e - c * d;
    tN = a * e - b * d;
    if (sN < 0.0) {
      /*  sc < 0 => the s=0 edge is visible        */
      sN = 0.0;
      tN = e;
      tD = c;
    } else {
      if (sN > D) {
        /*  sc > 1 => the s=1 edge is visible */
        sN = D;
        tN = e + b;
        tD = c;
      }
    }
  }

  if (tN < 0.0) {
    /*  tc < 0 => the t=0 edge is visible */
    tN = 0.0;

    /*  recompute sc for this edge */
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = D;
    } else {
      sN = -d;
      D = a;
    }
  } else {
    if (tN > tD) {
      /*  tc > 1 => the t=1 edge is visible */
      tN = tD;

      /*  recompute sc for this edge */
      if (-d + b < 0.0) {
        sN = 0.0;
      } else if (-d + b > a) {
        sN = D;
      } else {
        sN = -d + b;
        D = a;
      }
    }
  }

  /*  finally do the division to get sc and tc */
  if (fabs(sN) < 1.0E-8) {
    b = 0.0;
  } else {
    b = sN / D;
  }

  if (fabs(tN) < 1.0E-8) {
    a = 0.0;
  } else {
    a = tN / tD;
  }

  /*  get the difference of the two closest points */
  /*  = S1(sc) - S2(tc) */
  w_0[0] = (b * u[0] + w[0]) - a * v[0];
  w_0[1] = (b * u[1] + w[1]) - a * v[1];
  w_0[2] = (b * u[2] + w[2]) - a * v[2];
  distance = Arm_controller_v2_norm(w_0);

  /* outV = dP; */
  /* varargout(1) = {outV};      % vector connecting the closest points */
  /* varargout(2) = {p2+sc*u};   % Closest point on object 1  */
  /* varargout(3) = {p4+tc*v};   % Closest point on object 2 */
  return distance;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T Arm_contro_check_self_collision(const real_T joint_angles_degrees
  [6])
{
  real_T flag;
  real_T T1[16];
  real_T T2[16];
  real_T T3[16];
  real_T T4[16];
  real_T T5[16];
  real_T total_transform[16];
  real_T shoulder_start[4];
  real_T shoulder_orig[4];
  real_T elbow_orig[4];
  real_T wrist_2[4];
  real_T end_effector_orig[4];
  real_T wrist_back[3];
  real_T p3offset[3];
  real_T p1offset[3];
  real_T pwirestart[3];
  real_T i;
  real_T k;
  real_T elbow_orig_0[3];
  real_T shoulder_orig_0[3];
  real_T p2[3];
  real_T elbow_orig_1[3];
  real_T shoulder_orig_1[3];
  real_T p1[3];
  real_T elbow_orig_2[3];
  real_T shoulder_orig_2[3];
  real_T elbow_orig_3[3];
  real_T shoulder_orig_3[3];
  real_T wrist_2_0[3];
  real_T wrist_2_1[3];
  real_T p2_0[3];
  real_T T1_0[16];
  int32_T i_0;
  real_T T1_1[16];
  real_T i_1[16];
  int32_T i_2;
  real_T p1_idx_0;
  real_T p1_idx_1;
  real_T p1_idx_2;
  real_T wrist_vect_idx_0;
  real_T wrist_vect_idx_1;
  real_T wrist_vect_idx_2;

  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  i = joint_angles_degrees[0];
  Arm_controller_v2_cosd(&i);
  wrist_vect_idx_2 = joint_angles_degrees[0];
  Arm_controller_v2_sind(&wrist_vect_idx_2);
  k = joint_angles_degrees[0];
  Arm_controller_v2_sind(&k);
  p1_idx_0 = joint_angles_degrees[0];
  Arm_controller_v2_cosd(&p1_idx_0);
  T1[0] = i;
  T1[4] = 0.0;
  T1[8] = wrist_vect_idx_2;
  T1[12] = 0.0;
  T1[1] = k;
  T1[5] = 0.0;
  T1[9] = -p1_idx_0;
  T1[13] = 0.0;
  T1[2] = 0.0;
  T1[6] = 1.0;
  T1[10] = 0.0;
  T1[14] = 0.089;
  T1[3] = 0.0;
  T1[7] = 0.0;
  T1[11] = 0.0;
  T1[15] = 1.0;
  i = joint_angles_degrees[1];
  Arm_controller_v2_cosd(&i);
  wrist_vect_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind(&wrist_vect_idx_2);
  k = joint_angles_degrees[1];
  Arm_controller_v2_cosd(&k);
  p1_idx_0 = joint_angles_degrees[1];
  Arm_controller_v2_sind(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[1];
  Arm_controller_v2_cosd(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind(&p1_idx_2);
  T2[0] = i;
  T2[4] = -wrist_vect_idx_2;
  T2[8] = 0.0;
  T2[12] = -0.425 * k;
  T2[1] = p1_idx_0;
  T2[5] = p1_idx_1;
  T2[9] = 0.0;
  T2[13] = -0.425 * p1_idx_2;
  T2[2] = 0.0;
  T2[6] = 0.0;
  T2[10] = 1.0;
  T2[14] = 0.0;
  T2[3] = 0.0;
  T2[7] = 0.0;
  T2[11] = 0.0;
  T2[15] = 1.0;
  i = joint_angles_degrees[2];
  Arm_controller_v2_cosd(&i);
  wrist_vect_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind(&wrist_vect_idx_2);
  k = joint_angles_degrees[2];
  Arm_controller_v2_cosd(&k);
  p1_idx_0 = joint_angles_degrees[2];
  Arm_controller_v2_sind(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[2];
  Arm_controller_v2_cosd(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind(&p1_idx_2);
  T3[0] = i;
  T3[4] = -wrist_vect_idx_2;
  T3[8] = 0.0;
  T3[12] = -0.392 * k;
  T3[1] = p1_idx_0;
  T3[5] = p1_idx_1;
  T3[9] = 0.0;
  T3[13] = -0.392 * p1_idx_2;
  T3[2] = 0.0;
  T3[6] = 0.0;
  T3[10] = 1.0;
  T3[14] = 0.0;
  T3[3] = 0.0;
  T3[7] = 0.0;
  T3[11] = 0.0;
  T3[15] = 1.0;
  i = joint_angles_degrees[3];
  Arm_controller_v2_cosd(&i);
  wrist_vect_idx_2 = joint_angles_degrees[3];
  Arm_controller_v2_sind(&wrist_vect_idx_2);
  k = joint_angles_degrees[3];
  Arm_controller_v2_sind(&k);
  p1_idx_0 = joint_angles_degrees[3];
  Arm_controller_v2_cosd(&p1_idx_0);
  T4[0] = i;
  T4[4] = 0.0;
  T4[8] = wrist_vect_idx_2;
  T4[12] = 0.0;
  T4[1] = k;
  T4[5] = 0.0;
  T4[9] = -p1_idx_0;
  T4[13] = 0.0;
  T4[2] = 0.0;
  T4[6] = 1.0;
  T4[10] = 0.0;
  T4[14] = 0.109;
  T4[3] = 0.0;
  T4[7] = 0.0;
  T4[11] = 0.0;
  T4[15] = 1.0;
  i = joint_angles_degrees[4];
  Arm_controller_v2_cosd(&i);
  wrist_vect_idx_2 = joint_angles_degrees[4];
  Arm_controller_v2_sind(&wrist_vect_idx_2);
  k = joint_angles_degrees[4];
  Arm_controller_v2_sind(&k);
  p1_idx_0 = joint_angles_degrees[4];
  Arm_controller_v2_cosd(&p1_idx_0);
  T5[0] = i;
  T5[4] = 0.0;
  T5[8] = -wrist_vect_idx_2;
  T5[12] = 0.0;
  T5[1] = k;
  T5[5] = 0.0;
  T5[9] = p1_idx_0;
  T5[13] = 0.0;
  T5[2] = 0.0;
  T5[6] = -1.0;
  T5[10] = 0.0;
  T5[14] = 0.095;
  T5[3] = 0.0;
  T5[7] = 0.0;
  T5[11] = 0.0;
  T5[15] = 1.0;
  i = joint_angles_degrees[5];
  Arm_controller_v2_cosd(&i);
  wrist_vect_idx_2 = joint_angles_degrees[5];
  Arm_controller_v2_sind(&wrist_vect_idx_2);
  k = joint_angles_degrees[5];
  Arm_controller_v2_sind(&k);
  p1_idx_0 = joint_angles_degrees[5];
  Arm_controller_v2_cosd(&p1_idx_0);
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_0[i_0 + (i_2 << 2)] = 0.0;
      T1_0[i_0 + (i_2 << 2)] += T2[i_2 << 2] * T1[i_0];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 1] * T1[i_0 + 4];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 2] * T1[i_0 + 8];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 3] * T1[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_1[i_0 + (i_2 << 2)] = 0.0;
      T1_1[i_0 + (i_2 << 2)] += T3[i_2 << 2] * T1_0[i_0];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 1] * T1_0[i_0 + 4];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 2] * T1_0[i_0 + 8];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 3] * T1_0[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_0[i_0 + (i_2 << 2)] = 0.0;
      T1_0[i_0 + (i_2 << 2)] += T4[i_2 << 2] * T1_1[i_0];
      T1_0[i_0 + (i_2 << 2)] += T4[(i_2 << 2) + 1] * T1_1[i_0 + 4];
      T1_0[i_0 + (i_2 << 2)] += T4[(i_2 << 2) + 2] * T1_1[i_0 + 8];
      T1_0[i_0 + (i_2 << 2)] += T4[(i_2 << 2) + 3] * T1_1[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_1[i_0 + (i_2 << 2)] = 0.0;
      T1_1[i_0 + (i_2 << 2)] += T5[i_2 << 2] * T1_0[i_0];
      T1_1[i_0 + (i_2 << 2)] += T5[(i_2 << 2) + 1] * T1_0[i_0 + 4];
      T1_1[i_0 + (i_2 << 2)] += T5[(i_2 << 2) + 2] * T1_0[i_0 + 8];
      T1_1[i_0 + (i_2 << 2)] += T5[(i_2 << 2) + 3] * T1_0[i_0 + 12];
    }
  }

  i_1[0] = i;
  i_1[4] = -wrist_vect_idx_2;
  i_1[8] = 0.0;
  i_1[12] = 0.0;
  i_1[1] = k;
  i_1[5] = p1_idx_0;
  i_1[9] = 0.0;
  i_1[13] = 0.0;
  i_1[2] = 0.0;
  i_1[6] = 0.0;
  i_1[10] = 1.0;
  i_1[14] = 0.082;
  i_1[3] = 0.0;
  i_1[7] = 0.0;
  i_1[11] = 0.0;
  i_1[15] = 1.0;
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      total_transform[i_0 + (i_2 << 2)] = 0.0;
      total_transform[i_0 + (i_2 << 2)] += i_1[i_2 << 2] * T1_1[i_0];
      total_transform[i_0 + (i_2 << 2)] += i_1[(i_2 << 2) + 1] * T1_1[i_0 + 4];
      total_transform[i_0 + (i_2 << 2)] += i_1[(i_2 << 2) + 2] * T1_1[i_0 + 8];
      total_transform[i_0 + (i_2 << 2)] += i_1[(i_2 << 2) + 3] * T1_1[i_0 + 12];
    }
  }

  /* arm_orig = [0;0;0;1]; */
  for (i_0 = 0; i_0 < 4; i_0++) {
    wrist_vect_idx_2 = T1[i_0 + 12] + (T1[i_0 + 8] * 0.0 + (T1[i_0 + 4] * 0.0 +
      T1[i_0] * 0.0));
    shoulder_start[i_0] = wrist_vect_idx_2;
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_0[i_0 + (i_2 << 2)] = 0.0;
      T1_0[i_0 + (i_2 << 2)] += T2[i_2 << 2] * T1[i_0];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 1] * T1[i_0 + 4];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 2] * T1[i_0 + 8];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 3] * T1[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    wrist_vect_idx_2 = T1_0[i_0 + 12] + (T1_0[i_0 + 8] * 0.0 + (T1_0[i_0 + 4] *
      0.0 + T1_0[i_0] * 0.0));
    shoulder_orig[i_0] = wrist_vect_idx_2;
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_0[i_0 + (i_2 << 2)] = 0.0;
      T1_0[i_0 + (i_2 << 2)] += T2[i_2 << 2] * T1[i_0];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 1] * T1[i_0 + 4];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 2] * T1[i_0 + 8];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 3] * T1[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_1[i_0 + (i_2 << 2)] = 0.0;
      T1_1[i_0 + (i_2 << 2)] += T3[i_2 << 2] * T1_0[i_0];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 1] * T1_0[i_0 + 4];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 2] * T1_0[i_0 + 8];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 3] * T1_0[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    wrist_vect_idx_2 = T1_1[i_0 + 12] + (T1_1[i_0 + 8] * 0.0 + (T1_1[i_0 + 4] *
      0.0 + T1_1[i_0] * 0.0));
    elbow_orig[i_0] = wrist_vect_idx_2;
  }

  /* wrist_1 = T1*T2*T3*T4*[0;0;0;1]; */
  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_0[i_0 + (i_2 << 2)] = 0.0;
      T1_0[i_0 + (i_2 << 2)] += T2[i_2 << 2] * T1[i_0];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 1] * T1[i_0 + 4];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 2] * T1[i_0 + 8];
      T1_0[i_0 + (i_2 << 2)] += T2[(i_2 << 2) + 3] * T1[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_1[i_0 + (i_2 << 2)] = 0.0;
      T1_1[i_0 + (i_2 << 2)] += T3[i_2 << 2] * T1_0[i_0];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 1] * T1_0[i_0 + 4];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 2] * T1_0[i_0 + 8];
      T1_1[i_0 + (i_2 << 2)] += T3[(i_2 << 2) + 3] * T1_0[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_0[i_0 + (i_2 << 2)] = 0.0;
      T1_0[i_0 + (i_2 << 2)] += T4[i_2 << 2] * T1_1[i_0];
      T1_0[i_0 + (i_2 << 2)] += T4[(i_2 << 2) + 1] * T1_1[i_0 + 4];
      T1_0[i_0 + (i_2 << 2)] += T4[(i_2 << 2) + 2] * T1_1[i_0 + 8];
      T1_0[i_0 + (i_2 << 2)] += T4[(i_2 << 2) + 3] * T1_1[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    for (i_2 = 0; i_2 < 4; i_2++) {
      T1_1[i_0 + (i_2 << 2)] = 0.0;
      T1_1[i_0 + (i_2 << 2)] += T5[i_2 << 2] * T1_0[i_0];
      T1_1[i_0 + (i_2 << 2)] += T5[(i_2 << 2) + 1] * T1_0[i_0 + 4];
      T1_1[i_0 + (i_2 << 2)] += T5[(i_2 << 2) + 2] * T1_0[i_0 + 8];
      T1_1[i_0 + (i_2 << 2)] += T5[(i_2 << 2) + 3] * T1_0[i_0 + 12];
    }
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    k = T1_1[i_0 + 12] + (T1_1[i_0 + 8] * 0.0 + (T1_1[i_0 + 4] * 0.0 + T1_1[i_0]
      * 0.0));
    wrist_2[i_0] = k;
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    wrist_vect_idx_2 = total_transform[i_0 + 12] + (total_transform[i_0 + 8] *
      0.0 + (total_transform[i_0 + 4] * 0.0 + total_transform[i_0] * 0.0));
    end_effector_orig[i_0] = wrist_vect_idx_2;
  }

  /* find distance between end effector and elbow */
  /* end effector */
  p1_idx_0 = wrist_2[0];
  p1_idx_1 = wrist_2[1];
  p1_idx_2 = wrist_2[2];

  /* wrist */
  p2_0[0] = end_effector_orig[0] - wrist_2[0];
  p2_0[1] = end_effector_orig[1] - wrist_2[1];
  p2_0[2] = end_effector_orig[2] - wrist_2[2];
  i = Arm_controller_v2_norm(p2_0);
  wrist_vect_idx_0 = (end_effector_orig[0] - wrist_2[0]) / i;
  wrist_vect_idx_1 = (end_effector_orig[1] - wrist_2[1]) / i;
  wrist_vect_idx_2 = (end_effector_orig[2] - wrist_2[2]) / i;

  /* normalised vector */
  wrist_back[0] = wrist_2[0] - 0.06 * wrist_vect_idx_0;
  wrist_back[1] = wrist_2[1] - 0.06 * wrist_vect_idx_1;
  wrist_back[2] = wrist_2[2] - 0.06 * wrist_vect_idx_2;

  /* same line as */
  /* point representing camera */
  /* point representing end of wire connector */
  /* top of camera mount */
  /* p2offset = total_transform*[0;-0.072;0;1];%if end effector is rotated by 0 */
  for (i_0 = 0; i_0 < 4; i_0++) {
    k = total_transform[i_0 + 12] + (total_transform[i_0 + 8] * 0.0 +
      (total_transform[i_0 + 4] * 0.072 + total_transform[i_0] * 0.0));
    wrist_2[i_0] = k;
  }

  /* if end effector is rotated by 180 */
  p3offset[0] = 0.09 * wrist_vect_idx_0 + wrist_2[0];
  p3offset[1] = 0.09 * wrist_vect_idx_1 + wrist_2[1];
  p3offset[2] = 0.09 * wrist_vect_idx_2 + wrist_2[2];
  p1offset[0] = wrist_2[0] - 0.2 * wrist_vect_idx_0;
  p1offset[1] = wrist_2[1] - 0.2 * wrist_vect_idx_1;
  p1offset[2] = wrist_2[2] - 0.2 * wrist_vect_idx_2;

  /* end of wire */
  pwirestart[0] = wrist_2[0] - 0.082 * wrist_vect_idx_0;
  pwirestart[1] = wrist_2[1] - 0.082 * wrist_vect_idx_1;
  pwirestart[2] = wrist_2[2] - 0.082 * wrist_vect_idx_2;

  /* start of wire */
  /* collision with shoulder. Need to offset points by a certain point */
  /* shoulder CS */
  /* end of shoulder */
  /* start of shoulder */
  /* actual shouler */
  for (i_0 = 0; i_0 < 4; i_0++) {
    k = T1[i_0 + 12] + (T1[i_0 + 8] * 0.135 + (T1[i_0 + 4] * 0.0 + T1[i_0] * 0.0));
    wrist_2[i_0] = k;
  }

  /* bottom of shoulder */
  /* top of shoulder */
  /* checking self collision */
  /* if (dist_mount < 0.095 || dist_wire < 0.054 || dist_shoulder < 0.09) */
  elbow_orig_0[0] = elbow_orig[0];
  elbow_orig_0[1] = elbow_orig[1];
  elbow_orig_0[2] = elbow_orig[2];
  shoulder_orig_0[0] = shoulder_orig[0];
  shoulder_orig_0[1] = shoulder_orig[1];
  shoulder_orig_0[2] = shoulder_orig[2];
  p2[0] = 0.09 * wrist_vect_idx_0 + end_effector_orig[0];
  p2[1] = 0.09 * wrist_vect_idx_1 + end_effector_orig[1];
  p2[2] = 0.09 * wrist_vect_idx_2 + end_effector_orig[2];
  elbow_orig_1[0] = elbow_orig[0];
  elbow_orig_1[1] = elbow_orig[1];
  elbow_orig_1[2] = elbow_orig[2];
  shoulder_orig_1[0] = shoulder_orig[0];
  shoulder_orig_1[1] = shoulder_orig[1];
  shoulder_orig_1[2] = shoulder_orig[2];
  p1[0] = p1_idx_0 - 0.114 * wrist_vect_idx_0;
  p1[1] = p1_idx_1 - 0.114 * wrist_vect_idx_1;
  p1[2] = p1_idx_2 - 0.114 * wrist_vect_idx_2;
  elbow_orig_2[0] = elbow_orig[0];
  elbow_orig_2[1] = elbow_orig[1];
  elbow_orig_2[2] = elbow_orig[2];
  shoulder_orig_2[0] = shoulder_orig[0];
  shoulder_orig_2[1] = shoulder_orig[1];
  shoulder_orig_2[2] = shoulder_orig[2];
  elbow_orig_3[0] = elbow_orig[0];
  elbow_orig_3[1] = elbow_orig[1];
  elbow_orig_3[2] = elbow_orig[2];
  shoulder_orig_3[0] = shoulder_orig[0];
  shoulder_orig_3[1] = shoulder_orig[1];
  shoulder_orig_3[2] = shoulder_orig[2];
  wrist_2_0[0] = (shoulder_orig[0] - shoulder_start[0]) + wrist_2[0];
  wrist_2_0[1] = (shoulder_orig[1] - shoulder_start[1]) + wrist_2[1];
  wrist_2_0[2] = (shoulder_orig[2] - shoulder_start[2]) + wrist_2[2];
  wrist_2_1[0] = wrist_2[0];
  wrist_2_1[1] = wrist_2[1];
  wrist_2_1[2] = wrist_2[2];
  if ((Arm_control_DistBetween2Segment(p3offset, pwirestart, elbow_orig_0,
        shoulder_orig_0) < 0.1) || (Arm_control_DistBetween2Segment(p2,
        wrist_back, elbow_orig_1, shoulder_orig_1) < 0.1) ||
      (Arm_control_DistBetween2Segment(wrist_back, p1, elbow_orig_2,
        shoulder_orig_2) < 0.054) || (Arm_control_DistBetween2Segment(pwirestart,
        p1offset, elbow_orig_3, shoulder_orig_3) < 0.054) ||
      (Arm_control_DistBetween2Segment(p3offset, p1offset, wrist_2_0, wrist_2_1)
       < 0.09)) {
    flag = 1.0;

    /* there is a collision */
  } else {
    flag = 0.0;

    /* there is no collision */
  }

  return flag;
}

/* Function for MATLAB Function: '<S1>/move_cam2' */
static void Arm_controller_v2_cosd_b(real_T *x)
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

/* Function for MATLAB Function: '<S1>/move_cam2' */
static void Arm_controller_v2_sind_c(real_T *x)
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

/* Function for MATLAB Function: '<S1>/move_cam2' */
static real_T Arm_controller_v2_norm_c(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 2.2250738585072014E-308;
  absxk = fabs(x[0]);
  if (absxk > 2.2250738585072014E-308) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S1>/move_cam2' */
static real_T Arm_controller_v2_dot_e(const real_T a[3], const real_T b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

/* Function for MATLAB Function: '<S1>/move_cam2' */
static real_T Arm_contr_DistBetween2Segment_g(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3])
{
  real_T distance;
  real_T u[3];
  real_T v[3];
  real_T w[3];
  real_T a;
  real_T b;
  real_T c;
  real_T d;
  real_T e;
  real_T D;
  real_T tD;
  real_T sN;
  real_T tN;
  real_T w_0[3];

  /*  Computes the minimum distance between two line segments. Code */
  /*  is adapted for Matlab from Dan Sunday's Geometry Algorithms originally */
  /*  written in C++ */
  /*  http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment */
  /*  Usage: Input the start and end x,y,z coordinates for two line segments.  */
  /*  p1, p2 are [x,y,z] coordinates of first line segment and p3,p4 are for */
  /*  second line segment.  */
  /*  Output: scalar minimum distance between the two segments. */
  /*   Example: */
  /* 	P1 = [0 0 0];     P2 = [1 0 0]; */
  /*    P3 = [0 1 0];     P4 = [1 1 0]; */
  /* 	dist = DistBetween2Segment(P1, P2, P3, P4) */
  /*    dist = */
  /*  */
  /*     1 */
  /*   */
  u[0] = p1[0] - p2[0];
  u[1] = p1[1] - p2[1];
  u[2] = p1[2] - p2[2];
  v[0] = p3[0] - p4[0];
  v[1] = p3[1] - p4[1];
  v[2] = p3[2] - p4[2];
  w[0] = p2[0] - p4[0];
  w[1] = p2[1] - p4[1];
  w[2] = p2[2] - p4[2];
  a = Arm_controller_v2_dot_e(u, u);
  b = Arm_controller_v2_dot_e(u, v);
  c = Arm_controller_v2_dot_e(v, v);
  d = Arm_controller_v2_dot_e(u, w);
  e = Arm_controller_v2_dot_e(v, w);
  D = a * c - b * b;
  tD = D;

  /*  compute the line parameters of the two closest points */
  if (D < 1.0E-8) {
    /*  the lines are almost parallel */
    sN = 0.0;

    /*  force using point P0 on segment S1 */
    D = 1.0;

    /*  to prevent possible division by 0.0 later */
    tN = e;
    tD = c;
  } else {
    /*  get the closest points on the infinite lines */
    sN = b * e - c * d;
    tN = a * e - b * d;
    if (sN < 0.0) {
      /*  sc < 0 => the s=0 edge is visible        */
      sN = 0.0;
      tN = e;
      tD = c;
    } else {
      if (sN > D) {
        /*  sc > 1 => the s=1 edge is visible */
        sN = D;
        tN = e + b;
        tD = c;
      }
    }
  }

  if (tN < 0.0) {
    /*  tc < 0 => the t=0 edge is visible */
    tN = 0.0;

    /*  recompute sc for this edge */
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = D;
    } else {
      sN = -d;
      D = a;
    }
  } else {
    if (tN > tD) {
      /*  tc > 1 => the t=1 edge is visible */
      tN = tD;

      /*  recompute sc for this edge */
      if (-d + b < 0.0) {
        sN = 0.0;
      } else if (-d + b > a) {
        sN = D;
      } else {
        sN = -d + b;
        D = a;
      }
    }
  }

  /*  finally do the division to get sc and tc */
  if (fabs(sN) < 1.0E-8) {
    b = 0.0;
  } else {
    b = sN / D;
  }

  if (fabs(tN) < 1.0E-8) {
    a = 0.0;
  } else {
    a = tN / tD;
  }

  /*  get the difference of the two closest points */
  /*  = S1(sc) - S2(tc) */
  w_0[0] = (b * u[0] + w[0]) - a * v[0];
  w_0[1] = (b * u[1] + w[1]) - a * v[1];
  w_0[2] = (b * u[2] + w[2]) - a * v[2];
  distance = Arm_controller_v2_norm_c(w_0);

  /* outV = dP; */
  /* varargout(1) = {outV};      % vector connecting the closest points */
  /* varargout(2) = {p2+sc*u};   % Closest point on object 1  */
  /* varargout(3) = {p4+tc*v};   % Closest point on object 2 */
  return distance;
}

/* Function for MATLAB Function: '<S1>/move_cam2' */
static real_T Arm_controller__check_collision(const real_T joint_angles_degrees
  [6])
{
  real_T flag;
  real_T T1[16];
  real_T T2[16];
  real_T T3[16];
  real_T T4[16];
  real_T T5[16];
  real_T total_transform[16];
  real_T shoulder_start[4];
  real_T shoulder_orig[4];
  real_T elbow_orig[4];
  real_T wrist_2[4];
  real_T end_effector_orig[4];
  real_T p2[3];
  real_T wrist_back[3];
  real_T p3offset[3];
  real_T p1offset[3];
  real_T pwirestart[3];
  real_T dist_mount;
  real_T shoulder_points[129];
  real_T elbow_points[120];
  real_T laser_points[87];
  real_T fullarm[336];
  int32_T x;
  real_T l;
  boolean_T exitg1;
  real_T elbow_orig_0[3];
  real_T shoulder_orig_0[3];
  real_T p1[3];
  real_T elbow_orig_1[3];
  real_T shoulder_orig_1[3];
  real_T elbow_orig_2[3];
  real_T shoulder_orig_2[3];
  real_T wrist_2_0[3];
  real_T p2_0[3];
  real_T elbow_orig_3[3];
  real_T shoulder_orig_3[3];
  real_T p2_1[3];
  real_T shoulder[3];
  real_T elbow[3];
  real_T laser[3];
  real_T T1_0[16];
  real_T T1_1[16];
  real_T dist_mount_0[16];
  int32_T i;
  real_T p1_idx_0;
  real_T p1_idx_1;
  real_T p1_idx_2;
  real_T wrist_vect_idx_0;
  real_T wrist_vect_idx_1;
  real_T wrist_vect_idx_2;

  /* UNTITLED2 Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* create sample of points */
  /* update with new self collision function */
  /* DH matrices */
  dist_mount = joint_angles_degrees[0];
  Arm_controller_v2_cosd_b(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[0];
  Arm_controller_v2_sind_c(&wrist_vect_idx_2);
  l = joint_angles_degrees[0];
  Arm_controller_v2_sind_c(&l);
  p1_idx_0 = joint_angles_degrees[0];
  Arm_controller_v2_cosd_b(&p1_idx_0);
  T1[0] = dist_mount;
  T1[4] = 0.0;
  T1[8] = wrist_vect_idx_2;
  T1[12] = 0.0;
  T1[1] = l;
  T1[5] = 0.0;
  T1[9] = -p1_idx_0;
  T1[13] = 0.0;
  T1[2] = 0.0;
  T1[6] = 1.0;
  T1[10] = 0.0;
  T1[14] = 0.089;
  T1[3] = 0.0;
  T1[7] = 0.0;
  T1[11] = 0.0;
  T1[15] = 1.0;
  dist_mount = joint_angles_degrees[1];
  Arm_controller_v2_cosd_b(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind_c(&wrist_vect_idx_2);
  l = joint_angles_degrees[1];
  Arm_controller_v2_cosd_b(&l);
  p1_idx_0 = joint_angles_degrees[1];
  Arm_controller_v2_sind_c(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[1];
  Arm_controller_v2_cosd_b(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind_c(&p1_idx_2);
  T2[0] = dist_mount;
  T2[4] = -wrist_vect_idx_2;
  T2[8] = 0.0;
  T2[12] = -0.425 * l;
  T2[1] = p1_idx_0;
  T2[5] = p1_idx_1;
  T2[9] = 0.0;
  T2[13] = -0.425 * p1_idx_2;
  T2[2] = 0.0;
  T2[6] = 0.0;
  T2[10] = 1.0;
  T2[14] = 0.0;
  T2[3] = 0.0;
  T2[7] = 0.0;
  T2[11] = 0.0;
  T2[15] = 1.0;
  dist_mount = joint_angles_degrees[2];
  Arm_controller_v2_cosd_b(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind_c(&wrist_vect_idx_2);
  l = joint_angles_degrees[2];
  Arm_controller_v2_cosd_b(&l);
  p1_idx_0 = joint_angles_degrees[2];
  Arm_controller_v2_sind_c(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[2];
  Arm_controller_v2_cosd_b(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind_c(&p1_idx_2);
  T3[0] = dist_mount;
  T3[4] = -wrist_vect_idx_2;
  T3[8] = 0.0;
  T3[12] = -0.392 * l;
  T3[1] = p1_idx_0;
  T3[5] = p1_idx_1;
  T3[9] = 0.0;
  T3[13] = -0.392 * p1_idx_2;
  T3[2] = 0.0;
  T3[6] = 0.0;
  T3[10] = 1.0;
  T3[14] = 0.0;
  T3[3] = 0.0;
  T3[7] = 0.0;
  T3[11] = 0.0;
  T3[15] = 1.0;
  dist_mount = joint_angles_degrees[3];
  Arm_controller_v2_cosd_b(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[3];
  Arm_controller_v2_sind_c(&wrist_vect_idx_2);
  l = joint_angles_degrees[3];
  Arm_controller_v2_sind_c(&l);
  p1_idx_0 = joint_angles_degrees[3];
  Arm_controller_v2_cosd_b(&p1_idx_0);
  T4[0] = dist_mount;
  T4[4] = 0.0;
  T4[8] = wrist_vect_idx_2;
  T4[12] = 0.0;
  T4[1] = l;
  T4[5] = 0.0;
  T4[9] = -p1_idx_0;
  T4[13] = 0.0;
  T4[2] = 0.0;
  T4[6] = 1.0;
  T4[10] = 0.0;
  T4[14] = 0.109;
  T4[3] = 0.0;
  T4[7] = 0.0;
  T4[11] = 0.0;
  T4[15] = 1.0;
  dist_mount = joint_angles_degrees[4];
  Arm_controller_v2_cosd_b(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[4];
  Arm_controller_v2_sind_c(&wrist_vect_idx_2);
  l = joint_angles_degrees[4];
  Arm_controller_v2_sind_c(&l);
  p1_idx_0 = joint_angles_degrees[4];
  Arm_controller_v2_cosd_b(&p1_idx_0);
  T5[0] = dist_mount;
  T5[4] = 0.0;
  T5[8] = -wrist_vect_idx_2;
  T5[12] = 0.0;
  T5[1] = l;
  T5[5] = 0.0;
  T5[9] = p1_idx_0;
  T5[13] = 0.0;
  T5[2] = 0.0;
  T5[6] = -1.0;
  T5[10] = 0.0;
  T5[14] = 0.095;
  T5[3] = 0.0;
  T5[7] = 0.0;
  T5[11] = 0.0;
  T5[15] = 1.0;
  dist_mount = joint_angles_degrees[5];
  Arm_controller_v2_cosd_b(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[5];
  Arm_controller_v2_sind_c(&wrist_vect_idx_2);
  l = joint_angles_degrees[5];
  Arm_controller_v2_sind_c(&l);
  p1_idx_0 = joint_angles_degrees[5];
  Arm_controller_v2_cosd_b(&p1_idx_0);
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T4[i << 2] * T1_1[x];
      T1_0[x + (i << 2)] += T4[(i << 2) + 1] * T1_1[x + 4];
      T1_0[x + (i << 2)] += T4[(i << 2) + 2] * T1_1[x + 8];
      T1_0[x + (i << 2)] += T4[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T5[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T5[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T5[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T5[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  dist_mount_0[0] = dist_mount;
  dist_mount_0[4] = -wrist_vect_idx_2;
  dist_mount_0[8] = 0.0;
  dist_mount_0[12] = 0.0;
  dist_mount_0[1] = l;
  dist_mount_0[5] = p1_idx_0;
  dist_mount_0[9] = 0.0;
  dist_mount_0[13] = 0.0;
  dist_mount_0[2] = 0.0;
  dist_mount_0[6] = 0.0;
  dist_mount_0[10] = 1.0;
  dist_mount_0[14] = 0.082;
  dist_mount_0[3] = 0.0;
  dist_mount_0[7] = 0.0;
  dist_mount_0[11] = 0.0;
  dist_mount_0[15] = 1.0;
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      total_transform[x + (i << 2)] = 0.0;
      total_transform[x + (i << 2)] += dist_mount_0[i << 2] * T1_1[x];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 1] * T1_1[x + 4];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 2] * T1_1[x + 8];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  /* joint origin co-ordinates */
  /* arm_orig = [0;0;0;1]; */
  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1[x + 12] + (T1[x + 8] * 0.0 + (T1[x + 4] * 0.0 + T1[x] *
      0.0));
    shoulder_start[x] = wrist_vect_idx_2;
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1_0[x + 12] + (T1_0[x + 8] * 0.0 + (T1_0[x + 4] * 0.0 +
      T1_0[x] * 0.0));
    shoulder_orig[x] = wrist_vect_idx_2;
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1_1[x + 12] + (T1_1[x + 8] * 0.0 + (T1_1[x + 4] * 0.0 +
      T1_1[x] * 0.0));
    elbow_orig[x] = wrist_vect_idx_2;
  }

  /* wrist_1 = T1*T2*T3*T4*[0;0;0;1]; */
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T4[i << 2] * T1_1[x];
      T1_0[x + (i << 2)] += T4[(i << 2) + 1] * T1_1[x + 4];
      T1_0[x + (i << 2)] += T4[(i << 2) + 2] * T1_1[x + 8];
      T1_0[x + (i << 2)] += T4[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T5[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T5[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T5[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T5[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    l = T1_1[x + 12] + (T1_1[x + 8] * 0.0 + (T1_1[x + 4] * 0.0 + T1_1[x] * 0.0));
    wrist_2[x] = l;
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = total_transform[x + 12] + (total_transform[x + 8] * 0.0 +
      (total_transform[x + 4] * 0.0 + total_transform[x] * 0.0));
    end_effector_orig[x] = wrist_vect_idx_2;
  }

  /* %checking self collision%%%% */
  /* find distance between end effector and elbow */
  /* end effector */
  p1_idx_0 = wrist_2[0];
  p1_idx_1 = wrist_2[1];
  p1_idx_2 = wrist_2[2];

  /* wrist */
  p2_1[0] = end_effector_orig[0] - wrist_2[0];
  p2_1[1] = end_effector_orig[1] - wrist_2[1];
  p2_1[2] = end_effector_orig[2] - wrist_2[2];
  dist_mount = Arm_controller_v2_norm_c(p2_1);
  wrist_vect_idx_0 = (end_effector_orig[0] - wrist_2[0]) / dist_mount;
  wrist_vect_idx_1 = (end_effector_orig[1] - wrist_2[1]) / dist_mount;
  wrist_vect_idx_2 = (end_effector_orig[2] - wrist_2[2]) / dist_mount;

  /* normalised vector */
  /* wrist_front = p2; */
  wrist_back[0] = wrist_2[0] - 0.06 * wrist_vect_idx_0;
  wrist_back[1] = wrist_2[1] - 0.06 * wrist_vect_idx_1;
  wrist_back[2] = wrist_2[2] - 0.06 * wrist_vect_idx_2;

  /* same line as */
  /* point representing camera */
  /* point representing end of wire connector */
  /* top of camera mount */
  /* p2offset = total_transform*[0;-0.072;0;1];%if end effector is rotated by 0 */
  for (x = 0; x < 4; x++) {
    l = total_transform[x + 12] + (total_transform[x + 8] * 0.0 +
      (total_transform[x + 4] * 0.072 + total_transform[x] * 0.0));
    wrist_2[x] = l;
  }

  /* if end effector is rotated by 180 */
  p3offset[0] = 0.09 * wrist_vect_idx_0 + wrist_2[0];
  p3offset[1] = 0.09 * wrist_vect_idx_1 + wrist_2[1];
  p3offset[2] = 0.09 * wrist_vect_idx_2 + wrist_2[2];
  p1offset[0] = wrist_2[0] - 0.2 * wrist_vect_idx_0;
  p1offset[1] = wrist_2[1] - 0.2 * wrist_vect_idx_1;
  p1offset[2] = wrist_2[2] - 0.2 * wrist_vect_idx_2;

  /* end of wire */
  pwirestart[0] = wrist_2[0] - 0.082 * wrist_vect_idx_0;
  pwirestart[1] = wrist_2[1] - 0.082 * wrist_vect_idx_1;
  pwirestart[2] = wrist_2[2] - 0.082 * wrist_vect_idx_2;

  /* start of wire */
  p2_0[0] = 0.09 * wrist_vect_idx_0 + end_effector_orig[0];
  p2_0[1] = 0.09 * wrist_vect_idx_1 + end_effector_orig[1];
  p2_0[2] = 0.09 * wrist_vect_idx_2 + end_effector_orig[2];
  elbow_orig_3[0] = elbow_orig[0];
  elbow_orig_3[1] = elbow_orig[1];
  elbow_orig_3[2] = elbow_orig[2];
  shoulder_orig_3[0] = shoulder_orig[0];
  shoulder_orig_3[1] = shoulder_orig[1];
  shoulder_orig_3[2] = shoulder_orig[2];

  /* collision with shoulder. Need to offset points by a certain point */
  /* shoulder CS */
  /* end of shoulder */
  /* start of shoulder */
  /* actual shouler */
  for (x = 0; x < 4; x++) {
    l = T1[x + 12] + (T1[x + 8] * 0.135 + (T1[x + 4] * 0.0 + T1[x] * 0.0));
    wrist_2[x] = l;
  }

  /* bottom of shoulder */
  p2[0] = (shoulder_orig[0] - shoulder_start[0]) + wrist_2[0];
  p2[1] = (shoulder_orig[1] - shoulder_start[1]) + wrist_2[1];
  p2[2] = (shoulder_orig[2] - shoulder_start[2]) + wrist_2[2];

  /* top of shoulder */
  /* checking self collision */
  elbow_orig_0[0] = elbow_orig[0];
  elbow_orig_0[1] = elbow_orig[1];
  elbow_orig_0[2] = elbow_orig[2];
  shoulder_orig_0[0] = shoulder_orig[0];
  shoulder_orig_0[1] = shoulder_orig[1];
  shoulder_orig_0[2] = shoulder_orig[2];
  p1[0] = p1_idx_0 - 0.114 * wrist_vect_idx_0;
  p1[1] = p1_idx_1 - 0.114 * wrist_vect_idx_1;
  p1[2] = p1_idx_2 - 0.114 * wrist_vect_idx_2;
  elbow_orig_1[0] = elbow_orig[0];
  elbow_orig_1[1] = elbow_orig[1];
  elbow_orig_1[2] = elbow_orig[2];
  shoulder_orig_1[0] = shoulder_orig[0];
  shoulder_orig_1[1] = shoulder_orig[1];
  shoulder_orig_1[2] = shoulder_orig[2];
  elbow_orig_2[0] = elbow_orig[0];
  elbow_orig_2[1] = elbow_orig[1];
  elbow_orig_2[2] = elbow_orig[2];
  shoulder_orig_2[0] = shoulder_orig[0];
  shoulder_orig_2[1] = shoulder_orig[1];
  shoulder_orig_2[2] = shoulder_orig[2];
  wrist_2_0[0] = wrist_2[0];
  wrist_2_0[1] = wrist_2[1];
  wrist_2_0[2] = wrist_2[2];
  if ((Arm_contr_DistBetween2Segment_g(p3offset, pwirestart, elbow_orig_0,
        shoulder_orig_0) < 0.1) || (Arm_contr_DistBetween2Segment_g(p2_0,
        wrist_back, elbow_orig_3, shoulder_orig_3) < 0.1) ||
      (Arm_contr_DistBetween2Segment_g(wrist_back, p1, elbow_orig_1,
        shoulder_orig_1) < 0.054) || (Arm_contr_DistBetween2Segment_g(pwirestart,
        p1offset, elbow_orig_2, shoulder_orig_2) < 0.054) ||
      (Arm_contr_DistBetween2Segment_g(p3offset, p1offset, p2, wrist_2_0) < 0.09))
  {
    flag = 1.0;

    /* there is a collision */
  } else {
    flag = 0.0;

    /* there is no collision */
  }

  /* %%%%%%%%end of self collision%%%% */
  /* %%creating sample of points across the arm%%%% */
  /* reference co-ordinates of 3 main linkages */
  /* start and end co-ordinates for shoulder */
  /* %creating a set of points which represent the arm */
  for (x = 0; x < 43; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    shoulder[0] = p2[0] - wrist_2[0];
    shoulder[1] = p2[1] - wrist_2[1];
    shoulder[2] = p2[2] - wrist_2[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_c(shoulder);
    shoulder_points[x] = (p2[0] - wrist_2[0]) * dist_mount / wrist_vect_idx_2 +
      wrist_2[0];
    shoulder_points[x + 43] = (p2[1] - wrist_2[1]) * dist_mount /
      wrist_vect_idx_2 + wrist_2[1];
    shoulder_points[x + 86] = (p2[2] - wrist_2[2]) * dist_mount /
      wrist_vect_idx_2 + wrist_2[2];
  }

  for (x = 0; x < 40; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    elbow[0] = shoulder_orig[0] - elbow_orig[0];
    elbow[1] = shoulder_orig[1] - elbow_orig[1];
    elbow[2] = shoulder_orig[2] - elbow_orig[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_c(elbow);
    elbow_points[x] = (shoulder_orig[0] - elbow_orig[0]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[0];
    elbow_points[x + 40] = (shoulder_orig[1] - elbow_orig[1]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[1];
    elbow_points[x + 80] = (shoulder_orig[2] - elbow_orig[2]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[2];
  }

  for (x = 0; x < 29; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    laser[0] = p1offset[0] - p3offset[0];
    laser[1] = p1offset[1] - p3offset[1];
    laser[2] = p1offset[2] - p3offset[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_c(laser);
    laser_points[x] = (p1offset[0] - p3offset[0]) * dist_mount /
      wrist_vect_idx_2 + p3offset[0];
    laser_points[x + 29] = (p1offset[1] - p3offset[1]) * dist_mount /
      wrist_vect_idx_2 + p3offset[1];
    laser_points[x + 58] = (p1offset[2] - p3offset[2]) * dist_mount /
      wrist_vect_idx_2 + p3offset[2];
  }

  /* %combining the vectors into one */
  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x], &shoulder_points[43 * x], 43U * sizeof(real_T));
  }

  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x + 43], &elbow_points[40 * x], 40U * sizeof(real_T));
  }

  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x + 83], &laser_points[29 * x], 29U * sizeof(real_T));
  }

  /* [m, n] = size(fullarm); */
  /* %defining obstacles%% */
  /* %radius of shoulder */
  /* %left wheel%% */
  /* x, y, z co-ordinate of left wheel top right corner closest to base */
  /* if (  */
  /* %right wheel%% */
  /* x, y, z co-ordinate of right wheel top left corner closest to base */
  /* %body%% */
  /* top right corner */
  /* exagerrated to make sure arm never goes in that region */
  /* %pan tilt%% */
  /* %ground%% use laser scanner */
  /* % */
  /* disp(m); */
  x = 0;
  exitg1 = false;
  while ((!exitg1) && (x < 112)) {
    /* disp([x, y, z]); */
    /* disp(y); */
    /* disp(z); */
    if (fullarm[224 + x] < -0.25) {
      /* dont move */
      /* give collision flag */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.05) && (fullarm[x] < 0.21000000000000002) &&
               (fullarm[112 + x] > 0.15000000000000002) && (fullarm[112 + x] <
                0.37) && (fullarm[224 + x] < 0.135) && (fullarm[224 + x] >
                -0.31499999999999995)) {
      /* %the point is inside left wheel */
      /* %collision */
      /* disp('left wheel collision'); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.090000000000000011) && (fullarm[x] <
                0.21000000000000002) && (fullarm[112 + x] > -0.37) && (fullarm
                [112 + x] < -0.15000000000000002) && (fullarm[224 + x] < 0.135) &&
               (fullarm[224 + x] > -0.31499999999999995)) {
      /* %right wheel */
      /* disp('right wheel collision'); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -1.6400000000000001) && (fullarm[x] <
                -0.039999999999999994) && (fullarm[112 + x] > -0.25) &&
               (fullarm[112 + x] < 0.25) && (fullarm[224 + x] < 0.25) &&
               (fullarm[224 + x] > -0.25)) {
      /* %body */
      /* disp('body collision'); */
      /* disp([i x y z]); */
      /* disp([body_x_b, body_x_f]); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.14) && (fullarm[x] < 0.15000000000000002) &&
               (fullarm[112 + x] > 0.03) && (fullarm[112 + x] < 0.29) &&
               (fullarm[224 + x] < 0.0) && (fullarm[224 + x] > -0.25)) {
      /* %pantilt */
      /* disp('camera collision'); */
      flag = 1.0;
      exitg1 = true;
    } else {
      /* no collision */
      /* disp('no collision'); */
      flag = 0.0;
      x++;
    }
  }

  return flag;
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

/* Function for MATLAB Function: '<S1>/move_effector' */
static void Arm_controller_v2_getIK(real_T x_pos, real_T y_pos, real_T z_pos,
  real_T *num_sols, real_T q_sols[48])
{
  real_T q1[2];
  real_T A;
  real_T B;
  real_T R;
  real_T arccos;
  real_T q5[4];
  real_T s1;
  real_T q6;
  real_T c6;
  real_T x04x;
  real_T x04y;
  real_T p13x;
  int32_T i;
  boolean_T guard1;
  real_T q2_idx_0;
  real_T q2_idx_1;

  /* UNTITLED2 Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* transformation matrix so that end effector faces forward */
  /* ZERO_THRESH = 0.00000001; */
  /* DH parameters */
  /* variables for IK */
  memset(&q_sols[0], 0, 48U * sizeof(real_T));
  q1[0] = 0.0;
  q1[1] = 0.0;
  *num_sols = 0.0;

  /* q1 */
  /* %%%%% */
  R = (-0.0 - (-(-y_pos))) * (-0.0 - (-(-y_pos))) + (0.0823 - (-(-x_pos))) *
    (0.0823 - (-(-x_pos)));
  if (fabs(-0.0 - (-(-y_pos))) < 0.0001) {
    if (fabs(0.10915 - fabs(0.0823 - (-(-x_pos)))) < 0.0001) {
      if (0.0823 - (-(-x_pos)) < 0.0) {
        R = 1.0;
      } else if (0.0823 - (-(-x_pos)) > 0.0) {
        R = -1.0;
      } else if (0.0823 - (-(-x_pos)) == 0.0) {
        R = -0.0;
      } else {
        R = -(0.0823 - (-(-x_pos)));
      }
    } else {
      R = -0.10915 / (0.0823 - (-(-x_pos)));
    }

    R = asin(R);
    if (fabs(R) < 0.0001) {
      R = 0.0;
    }

    if (R < 0.0) {
      q1[0] = R + 6.2831853071795862;
    } else {
      q1[0] = R;
    }

    q1[1] = 3.1415926535897931 - R;
  } else if (fabs(0.0823 - (-(-x_pos))) < 0.0001) {
    if (fabs(0.10915 - fabs(-0.0 - (-(-y_pos)))) < 0.0001) {
      if (-0.0 - (-(-y_pos)) < 0.0) {
        R = -1.0;
      } else if (-0.0 - (-(-y_pos)) > 0.0) {
        R = 1.0;
      } else if (-0.0 - (-(-y_pos)) == 0.0) {
        R = 0.0;
      } else {
        R = -0.0 - (-(-y_pos));
      }
    } else {
      R = 0.10915 / (-0.0 - (-(-y_pos)));
    }

    arccos = acos(R);
    q1[0] = arccos;
    q1[1] = 6.2831853071795862 - arccos;
  } else if (!(0.0119137225 > R)) {
    arccos = acos(0.10915 / sqrt(R));
    R = rt_atan2d_snf(-(0.0823 - (-(-x_pos))), -0.0 - (-(-y_pos)));
    A = arccos + R;
    R += -arccos;
    if (fabs(A) < 0.0001) {
      A = 0.0;
    }

    if (fabs(R) < 0.0001) {
      R = 0.0;
    }

    if (A >= 0.0) {
      q1[0] = A;
    } else {
      q1[0] = 6.2831853071795862 + A;
    }

    if (R >= 0.0) {
      q1[1] = R;
    } else {
      q1[1] = 6.2831853071795862 + R;
    }
  } else {
    /* disp('no solution'); */
    /* return num_sols; */
  }

  /* %%%%% */
  /* q5 */
  /* %%% */
  R = (-(-x_pos) * sin(q1[0]) - -(-y_pos) * cos(q1[0])) - 0.10915;
  if (fabs(fabs(R) - 0.0823) < 0.0001) {
    if (R < 0.0) {
      R = -1.0;
    } else if (R > 0.0) {
      R = 1.0;
    } else {
      if (R == 0.0) {
        R = 0.0;
      }
    }
  } else {
    R /= 0.0823;
  }

  arccos = acos(R);
  q5[0] = arccos;
  q5[2] = 6.2831853071795862 - arccos;
  R = (-(-x_pos) * sin(q1[1]) - -(-y_pos) * cos(q1[1])) - 0.10915;
  if (fabs(fabs(R) - 0.0823) < 0.0001) {
    if (R < 0.0) {
      R = -1.0;
    } else if (R > 0.0) {
      R = 1.0;
    } else {
      if (R == 0.0) {
        R = 0.0;
      }
    }
  } else {
    R /= 0.0823;
  }

  arccos = acos(R);
  q5[1] = arccos;
  q5[3] = 6.2831853071795862 - arccos;

  /* to convert */
  for (i = 0; i < 2; i++) {
    B = cos(q1[i]);
    s1 = sin(q1[i]);
    R = cos(q5[i]);
    A = sin(q5[i]);

    /* ////////////////////////////// wrist 3 joint (q6) ////////////////////////////// */
    if (fabs(A) < 0.0001) {
      /* q6 = q6_des; */
      q6 = 3.1415926535897931;
    } else {
      if (A < 0.0) {
        c6 = -1.0;
      } else if (A > 0.0) {
        c6 = 1.0;
      } else if (A == 0.0) {
        c6 = 0.0;
      } else {
        c6 = A;
      }

      if (A < 0.0) {
        p13x = -1.0;
      } else if (A > 0.0) {
        p13x = 1.0;
      } else if (A == 0.0) {
        p13x = 0.0;
      } else {
        p13x = A;
      }

      q6 = rt_atan2d_snf(-(0.0 * s1 - 0.0 * B) * c6, (0.0 * s1 - B) * p13x);
      if (fabs(q6) < 0.0001) {
        q6 = 0.0;
      }

      if (q6 < 0.0) {
        q6 += 6.2831853071795862;
      }
    }

    /* //////////////////////////////////////////////////////////////////////////////// */
    /* ///////////////////////////// RRR joints (q2,q3,q4) //////////////////////////// */
    c6 = cos(q6);
    arccos = sin(q6);
    x04x = (-0.0 * s1 + B) * -A - ((0.0 * B + 0.0 * s1) * arccos - (0.0 * B + s1)
      * c6) * R;
    x04y = (-0.0 * c6 - arccos) * R - 0.0 * A;
    p13x = ((((0.0 * B + s1) * arccos + (0.0 * B + 0.0 * s1) * c6) * 0.09465 - (
              -0.0 * s1 + B) * 0.0823) + -(-x_pos) * B) + -(-y_pos) * s1;
    s1 = (-0.0 * arccos + c6) * 0.09465 + (z_pos - 0.089159);
    R = (((p13x * p13x + s1 * s1) - 0.18062499999999998) - 0.15386006249999998) /
      0.3334125;

    /* %%%% */
    guard1 = false;
    if (fabs(fabs(R) - 1.0) < 0.0001) {
      if (R < 0.0) {
        R = -1.0;
      } else if (R > 0.0) {
        R = 1.0;
      } else {
        if (R == 0.0) {
          R = 0.0;
        }
      }

      guard1 = true;
    } else if (!(fabs(R) > 1.0)) {
      guard1 = true;
    } else {
      /* // TODO NO SOLUTION */
    }

    if (guard1) {
      arccos = acos(R);
      c6 = 0.3334125 * R + 0.3344850625;
      A = -0.39225 * R + -0.425;
      B = -0.39225 * sin(arccos);
      q2_idx_0 = rt_atan2d_snf((A * s1 - B * p13x) / c6, (A * p13x + B * s1) /
        c6);
      q2_idx_1 = rt_atan2d_snf((A * s1 + B * p13x) / c6, (A * p13x - B * s1) /
        c6);
      R = cos(q2_idx_0 + arccos);
      A = sin(q2_idx_0 + arccos);
      B = cos((6.2831853071795862 - arccos) + q2_idx_1);
      s1 = sin((6.2831853071795862 - arccos) + q2_idx_1);
      p13x = rt_atan2d_snf(R * x04y - A * x04x, x04x * R + x04y * A);
      c6 = rt_atan2d_snf(B * x04y - s1 * x04x, x04x * B + x04y * s1);

      /* //////////////////////////////////////////////////////////////////////////////// */
      if (fabs(q2_idx_0) < 0.0001) {
        q2_idx_0 = 0.0;
      } else {
        if (q2_idx_0 < 0.0) {
          q2_idx_0 += 6.2831853071795862;
        }
      }

      if (fabs(p13x) < 0.0001) {
        p13x = 0.0;
      } else {
        if (p13x < 0.0) {
          p13x += 6.2831853071795862;
        }
      }

      q_sols[(int32_T)(*num_sols + 1.0) - 1] = q1[i];
      q_sols[(int32_T)(*num_sols + 1.0) + 7] = q2_idx_0;
      q_sols[(int32_T)(*num_sols + 1.0) + 15] = arccos;
      q_sols[(int32_T)(*num_sols + 1.0) + 23] = p13x;
      q_sols[(int32_T)(*num_sols + 1.0) + 31] = q5[i];
      q_sols[(int32_T)(*num_sols + 1.0) + 39] = q6;
      (*num_sols)++;
      p13x = c6;
      q2_idx_0 = q2_idx_1;
      if (fabs(q2_idx_1) < 0.0001) {
        q2_idx_0 = 0.0;
      } else {
        if (q2_idx_1 < 0.0) {
          q2_idx_0 = q2_idx_1 + 6.2831853071795862;
        }
      }

      if (fabs(c6) < 0.0001) {
        p13x = 0.0;
      } else {
        if (c6 < 0.0) {
          p13x = c6 + 6.2831853071795862;
        }
      }

      q_sols[(int32_T)(*num_sols + 1.0) - 1] = q1[i];
      q_sols[(int32_T)(*num_sols + 1.0) + 7] = q2_idx_0;
      q_sols[(int32_T)(*num_sols + 1.0) + 15] = 6.2831853071795862 - arccos;
      q_sols[(int32_T)(*num_sols + 1.0) + 23] = p13x;
      q_sols[(int32_T)(*num_sols + 1.0) + 31] = q5[i];
      q_sols[(int32_T)(*num_sols + 1.0) + 39] = q6;
      (*num_sols)++;
    }

    B = cos(q1[i]);
    s1 = sin(q1[i]);
    R = cos(q5[i + 2]);
    A = sin(q5[i + 2]);

    /* ////////////////////////////// wrist 3 joint (q6) ////////////////////////////// */
    if (fabs(A) < 0.0001) {
      /* q6 = q6_des; */
      q6 = 3.1415926535897931;
    } else {
      if (A < 0.0) {
        c6 = -1.0;
      } else if (A > 0.0) {
        c6 = 1.0;
      } else if (A == 0.0) {
        c6 = 0.0;
      } else {
        c6 = A;
      }

      if (A < 0.0) {
        p13x = -1.0;
      } else if (A > 0.0) {
        p13x = 1.0;
      } else if (A == 0.0) {
        p13x = 0.0;
      } else {
        p13x = A;
      }

      q6 = rt_atan2d_snf(-(0.0 * s1 - 0.0 * B) * c6, (0.0 * s1 - B) * p13x);
      if (fabs(q6) < 0.0001) {
        q6 = 0.0;
      }

      if (q6 < 0.0) {
        q6 += 6.2831853071795862;
      }
    }

    /* //////////////////////////////////////////////////////////////////////////////// */
    /* ///////////////////////////// RRR joints (q2,q3,q4) //////////////////////////// */
    c6 = cos(q6);
    arccos = sin(q6);
    x04x = (-0.0 * s1 + B) * -A - ((0.0 * B + 0.0 * s1) * arccos - (0.0 * B + s1)
      * c6) * R;
    x04y = (-0.0 * c6 - arccos) * R - 0.0 * A;
    p13x = ((((0.0 * B + s1) * arccos + (0.0 * B + 0.0 * s1) * c6) * 0.09465 - (
              -0.0 * s1 + B) * 0.0823) + -(-x_pos) * B) + -(-y_pos) * s1;
    s1 = (-0.0 * arccos + c6) * 0.09465 + (z_pos - 0.089159);
    R = (((p13x * p13x + s1 * s1) - 0.18062499999999998) - 0.15386006249999998) /
      0.3334125;

    /* %%%% */
    guard1 = false;
    if (fabs(fabs(R) - 1.0) < 0.0001) {
      if (R < 0.0) {
        R = -1.0;
      } else if (R > 0.0) {
        R = 1.0;
      } else {
        if (R == 0.0) {
          R = 0.0;
        }
      }

      guard1 = true;
    } else if (!(fabs(R) > 1.0)) {
      guard1 = true;
    } else {
      /* // TODO NO SOLUTION */
    }

    if (guard1) {
      arccos = acos(R);
      c6 = 0.3334125 * R + 0.3344850625;
      A = -0.39225 * R + -0.425;
      B = -0.39225 * sin(arccos);
      q2_idx_0 = rt_atan2d_snf((A * s1 - B * p13x) / c6, (A * p13x + B * s1) /
        c6);
      q2_idx_1 = rt_atan2d_snf((A * s1 + B * p13x) / c6, (A * p13x - B * s1) /
        c6);
      R = cos(q2_idx_0 + arccos);
      A = sin(q2_idx_0 + arccos);
      B = cos((6.2831853071795862 - arccos) + q2_idx_1);
      s1 = sin((6.2831853071795862 - arccos) + q2_idx_1);
      p13x = rt_atan2d_snf(R * x04y - A * x04x, x04x * R + x04y * A);
      c6 = rt_atan2d_snf(B * x04y - s1 * x04x, x04x * B + x04y * s1);

      /* //////////////////////////////////////////////////////////////////////////////// */
      if (fabs(q2_idx_0) < 0.0001) {
        q2_idx_0 = 0.0;
      } else {
        if (q2_idx_0 < 0.0) {
          q2_idx_0 += 6.2831853071795862;
        }
      }

      if (fabs(p13x) < 0.0001) {
        p13x = 0.0;
      } else {
        if (p13x < 0.0) {
          p13x += 6.2831853071795862;
        }
      }

      q_sols[(int32_T)(*num_sols + 1.0) - 1] = q1[i];
      q_sols[(int32_T)(*num_sols + 1.0) + 7] = q2_idx_0;
      q_sols[(int32_T)(*num_sols + 1.0) + 15] = arccos;
      q_sols[(int32_T)(*num_sols + 1.0) + 23] = p13x;
      q_sols[(int32_T)(*num_sols + 1.0) + 31] = q5[i + 2];
      q_sols[(int32_T)(*num_sols + 1.0) + 39] = q6;
      (*num_sols)++;
      p13x = c6;
      q2_idx_0 = q2_idx_1;
      if (fabs(q2_idx_1) < 0.0001) {
        q2_idx_0 = 0.0;
      } else {
        if (q2_idx_1 < 0.0) {
          q2_idx_0 = q2_idx_1 + 6.2831853071795862;
        }
      }

      if (fabs(c6) < 0.0001) {
        p13x = 0.0;
      } else {
        if (c6 < 0.0) {
          p13x = c6 + 6.2831853071795862;
        }
      }

      q_sols[(int32_T)(*num_sols + 1.0) - 1] = q1[i];
      q_sols[(int32_T)(*num_sols + 1.0) + 7] = q2_idx_0;
      q_sols[(int32_T)(*num_sols + 1.0) + 15] = 6.2831853071795862 - arccos;
      q_sols[(int32_T)(*num_sols + 1.0) + 23] = p13x;
      q_sols[(int32_T)(*num_sols + 1.0) + 31] = q5[i + 2];
      q_sols[(int32_T)(*num_sols + 1.0) + 39] = q6;
      (*num_sols)++;
    }
  }
}

/* Function for MATLAB Function: '<S1>/move_effector' */
static void Arm_controller_v2_cosd_m(real_T *x)
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

/* Function for MATLAB Function: '<S1>/move_effector' */
static void Arm_controller_v2_sind_f(real_T *x)
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

/* Function for MATLAB Function: '<S1>/move_effector' */
static real_T Arm_controller_v2_norm_m(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 2.2250738585072014E-308;
  absxk = fabs(x[0]);
  if (absxk > 2.2250738585072014E-308) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S1>/move_effector' */
static real_T Arm_controller_v2_dot_n(const real_T a[3], const real_T b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

/* Function for MATLAB Function: '<S1>/move_effector' */
static real_T Arm_contr_DistBetween2Segment_a(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3])
{
  real_T distance;
  real_T u[3];
  real_T v[3];
  real_T w[3];
  real_T a;
  real_T b;
  real_T c;
  real_T d;
  real_T e;
  real_T D;
  real_T tD;
  real_T sN;
  real_T tN;
  real_T w_0[3];

  /*  Computes the minimum distance between two line segments. Code */
  /*  is adapted for Matlab from Dan Sunday's Geometry Algorithms originally */
  /*  written in C++ */
  /*  http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment */
  /*  Usage: Input the start and end x,y,z coordinates for two line segments.  */
  /*  p1, p2 are [x,y,z] coordinates of first line segment and p3,p4 are for */
  /*  second line segment.  */
  /*  Output: scalar minimum distance between the two segments. */
  /*   Example: */
  /* 	P1 = [0 0 0];     P2 = [1 0 0]; */
  /*    P3 = [0 1 0];     P4 = [1 1 0]; */
  /* 	dist = DistBetween2Segment(P1, P2, P3, P4) */
  /*    dist = */
  /*  */
  /*     1 */
  /*   */
  u[0] = p1[0] - p2[0];
  u[1] = p1[1] - p2[1];
  u[2] = p1[2] - p2[2];
  v[0] = p3[0] - p4[0];
  v[1] = p3[1] - p4[1];
  v[2] = p3[2] - p4[2];
  w[0] = p2[0] - p4[0];
  w[1] = p2[1] - p4[1];
  w[2] = p2[2] - p4[2];
  a = Arm_controller_v2_dot_n(u, u);
  b = Arm_controller_v2_dot_n(u, v);
  c = Arm_controller_v2_dot_n(v, v);
  d = Arm_controller_v2_dot_n(u, w);
  e = Arm_controller_v2_dot_n(v, w);
  D = a * c - b * b;
  tD = D;

  /*  compute the line parameters of the two closest points */
  if (D < 1.0E-8) {
    /*  the lines are almost parallel */
    sN = 0.0;

    /*  force using point P0 on segment S1 */
    D = 1.0;

    /*  to prevent possible division by 0.0 later */
    tN = e;
    tD = c;
  } else {
    /*  get the closest points on the infinite lines */
    sN = b * e - c * d;
    tN = a * e - b * d;
    if (sN < 0.0) {
      /*  sc < 0 => the s=0 edge is visible        */
      sN = 0.0;
      tN = e;
      tD = c;
    } else {
      if (sN > D) {
        /*  sc > 1 => the s=1 edge is visible */
        sN = D;
        tN = e + b;
        tD = c;
      }
    }
  }

  if (tN < 0.0) {
    /*  tc < 0 => the t=0 edge is visible */
    tN = 0.0;

    /*  recompute sc for this edge */
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = D;
    } else {
      sN = -d;
      D = a;
    }
  } else {
    if (tN > tD) {
      /*  tc > 1 => the t=1 edge is visible */
      tN = tD;

      /*  recompute sc for this edge */
      if (-d + b < 0.0) {
        sN = 0.0;
      } else if (-d + b > a) {
        sN = D;
      } else {
        sN = -d + b;
        D = a;
      }
    }
  }

  /*  finally do the division to get sc and tc */
  if (fabs(sN) < 1.0E-8) {
    b = 0.0;
  } else {
    b = sN / D;
  }

  if (fabs(tN) < 1.0E-8) {
    a = 0.0;
  } else {
    a = tN / tD;
  }

  /*  get the difference of the two closest points */
  /*  = S1(sc) - S2(tc) */
  w_0[0] = (b * u[0] + w[0]) - a * v[0];
  w_0[1] = (b * u[1] + w[1]) - a * v[1];
  w_0[2] = (b * u[2] + w[2]) - a * v[2];
  distance = Arm_controller_v2_norm_m(w_0);

  /* outV = dP; */
  /* varargout(1) = {outV};      % vector connecting the closest points */
  /* varargout(2) = {p2+sc*u};   % Closest point on object 1  */
  /* varargout(3) = {p4+tc*v};   % Closest point on object 2 */
  return distance;
}

/* Function for MATLAB Function: '<S1>/move_effector' */
static real_T Arm_controlle_check_collision_k(const real_T joint_angles_degrees
  [6])
{
  real_T flag;
  real_T T1[16];
  real_T T2[16];
  real_T T3[16];
  real_T T4[16];
  real_T T5[16];
  real_T total_transform[16];
  real_T shoulder_start[4];
  real_T shoulder_orig[4];
  real_T elbow_orig[4];
  real_T wrist_2[4];
  real_T end_effector_orig[4];
  real_T p2[3];
  real_T wrist_back[3];
  real_T p3offset[3];
  real_T p1offset[3];
  real_T pwirestart[3];
  real_T dist_mount;
  real_T shoulder_points[129];
  real_T elbow_points[120];
  real_T laser_points[87];
  real_T fullarm[336];
  int32_T x;
  real_T l;
  boolean_T exitg1;
  real_T elbow_orig_0[3];
  real_T shoulder_orig_0[3];
  real_T p1[3];
  real_T elbow_orig_1[3];
  real_T shoulder_orig_1[3];
  real_T elbow_orig_2[3];
  real_T shoulder_orig_2[3];
  real_T wrist_2_0[3];
  real_T p2_0[3];
  real_T elbow_orig_3[3];
  real_T shoulder_orig_3[3];
  real_T p2_1[3];
  real_T shoulder[3];
  real_T elbow[3];
  real_T laser[3];
  real_T T1_0[16];
  real_T T1_1[16];
  real_T dist_mount_0[16];
  int32_T i;
  real_T p1_idx_0;
  real_T p1_idx_1;
  real_T p1_idx_2;
  real_T wrist_vect_idx_0;
  real_T wrist_vect_idx_1;
  real_T wrist_vect_idx_2;

  /* UNTITLED2 Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* create sample of points */
  /* update with new self collision function */
  /* DH matrices */
  dist_mount = joint_angles_degrees[0];
  Arm_controller_v2_cosd_m(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[0];
  Arm_controller_v2_sind_f(&wrist_vect_idx_2);
  l = joint_angles_degrees[0];
  Arm_controller_v2_sind_f(&l);
  p1_idx_0 = joint_angles_degrees[0];
  Arm_controller_v2_cosd_m(&p1_idx_0);
  T1[0] = dist_mount;
  T1[4] = 0.0;
  T1[8] = wrist_vect_idx_2;
  T1[12] = 0.0;
  T1[1] = l;
  T1[5] = 0.0;
  T1[9] = -p1_idx_0;
  T1[13] = 0.0;
  T1[2] = 0.0;
  T1[6] = 1.0;
  T1[10] = 0.0;
  T1[14] = 0.089;
  T1[3] = 0.0;
  T1[7] = 0.0;
  T1[11] = 0.0;
  T1[15] = 1.0;
  dist_mount = joint_angles_degrees[1];
  Arm_controller_v2_cosd_m(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind_f(&wrist_vect_idx_2);
  l = joint_angles_degrees[1];
  Arm_controller_v2_cosd_m(&l);
  p1_idx_0 = joint_angles_degrees[1];
  Arm_controller_v2_sind_f(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[1];
  Arm_controller_v2_cosd_m(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind_f(&p1_idx_2);
  T2[0] = dist_mount;
  T2[4] = -wrist_vect_idx_2;
  T2[8] = 0.0;
  T2[12] = -0.425 * l;
  T2[1] = p1_idx_0;
  T2[5] = p1_idx_1;
  T2[9] = 0.0;
  T2[13] = -0.425 * p1_idx_2;
  T2[2] = 0.0;
  T2[6] = 0.0;
  T2[10] = 1.0;
  T2[14] = 0.0;
  T2[3] = 0.0;
  T2[7] = 0.0;
  T2[11] = 0.0;
  T2[15] = 1.0;
  dist_mount = joint_angles_degrees[2];
  Arm_controller_v2_cosd_m(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind_f(&wrist_vect_idx_2);
  l = joint_angles_degrees[2];
  Arm_controller_v2_cosd_m(&l);
  p1_idx_0 = joint_angles_degrees[2];
  Arm_controller_v2_sind_f(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[2];
  Arm_controller_v2_cosd_m(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind_f(&p1_idx_2);
  T3[0] = dist_mount;
  T3[4] = -wrist_vect_idx_2;
  T3[8] = 0.0;
  T3[12] = -0.392 * l;
  T3[1] = p1_idx_0;
  T3[5] = p1_idx_1;
  T3[9] = 0.0;
  T3[13] = -0.392 * p1_idx_2;
  T3[2] = 0.0;
  T3[6] = 0.0;
  T3[10] = 1.0;
  T3[14] = 0.0;
  T3[3] = 0.0;
  T3[7] = 0.0;
  T3[11] = 0.0;
  T3[15] = 1.0;
  dist_mount = joint_angles_degrees[3];
  Arm_controller_v2_cosd_m(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[3];
  Arm_controller_v2_sind_f(&wrist_vect_idx_2);
  l = joint_angles_degrees[3];
  Arm_controller_v2_sind_f(&l);
  p1_idx_0 = joint_angles_degrees[3];
  Arm_controller_v2_cosd_m(&p1_idx_0);
  T4[0] = dist_mount;
  T4[4] = 0.0;
  T4[8] = wrist_vect_idx_2;
  T4[12] = 0.0;
  T4[1] = l;
  T4[5] = 0.0;
  T4[9] = -p1_idx_0;
  T4[13] = 0.0;
  T4[2] = 0.0;
  T4[6] = 1.0;
  T4[10] = 0.0;
  T4[14] = 0.109;
  T4[3] = 0.0;
  T4[7] = 0.0;
  T4[11] = 0.0;
  T4[15] = 1.0;
  dist_mount = joint_angles_degrees[4];
  Arm_controller_v2_cosd_m(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[4];
  Arm_controller_v2_sind_f(&wrist_vect_idx_2);
  l = joint_angles_degrees[4];
  Arm_controller_v2_sind_f(&l);
  p1_idx_0 = joint_angles_degrees[4];
  Arm_controller_v2_cosd_m(&p1_idx_0);
  T5[0] = dist_mount;
  T5[4] = 0.0;
  T5[8] = -wrist_vect_idx_2;
  T5[12] = 0.0;
  T5[1] = l;
  T5[5] = 0.0;
  T5[9] = p1_idx_0;
  T5[13] = 0.0;
  T5[2] = 0.0;
  T5[6] = -1.0;
  T5[10] = 0.0;
  T5[14] = 0.095;
  T5[3] = 0.0;
  T5[7] = 0.0;
  T5[11] = 0.0;
  T5[15] = 1.0;
  dist_mount = joint_angles_degrees[5];
  Arm_controller_v2_cosd_m(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[5];
  Arm_controller_v2_sind_f(&wrist_vect_idx_2);
  l = joint_angles_degrees[5];
  Arm_controller_v2_sind_f(&l);
  p1_idx_0 = joint_angles_degrees[5];
  Arm_controller_v2_cosd_m(&p1_idx_0);
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T4[i << 2] * T1_1[x];
      T1_0[x + (i << 2)] += T4[(i << 2) + 1] * T1_1[x + 4];
      T1_0[x + (i << 2)] += T4[(i << 2) + 2] * T1_1[x + 8];
      T1_0[x + (i << 2)] += T4[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T5[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T5[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T5[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T5[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  dist_mount_0[0] = dist_mount;
  dist_mount_0[4] = -wrist_vect_idx_2;
  dist_mount_0[8] = 0.0;
  dist_mount_0[12] = 0.0;
  dist_mount_0[1] = l;
  dist_mount_0[5] = p1_idx_0;
  dist_mount_0[9] = 0.0;
  dist_mount_0[13] = 0.0;
  dist_mount_0[2] = 0.0;
  dist_mount_0[6] = 0.0;
  dist_mount_0[10] = 1.0;
  dist_mount_0[14] = 0.082;
  dist_mount_0[3] = 0.0;
  dist_mount_0[7] = 0.0;
  dist_mount_0[11] = 0.0;
  dist_mount_0[15] = 1.0;
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      total_transform[x + (i << 2)] = 0.0;
      total_transform[x + (i << 2)] += dist_mount_0[i << 2] * T1_1[x];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 1] * T1_1[x + 4];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 2] * T1_1[x + 8];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  /* joint origin co-ordinates */
  /* arm_orig = [0;0;0;1]; */
  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1[x + 12] + (T1[x + 8] * 0.0 + (T1[x + 4] * 0.0 + T1[x] *
      0.0));
    shoulder_start[x] = wrist_vect_idx_2;
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1_0[x + 12] + (T1_0[x + 8] * 0.0 + (T1_0[x + 4] * 0.0 +
      T1_0[x] * 0.0));
    shoulder_orig[x] = wrist_vect_idx_2;
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1_1[x + 12] + (T1_1[x + 8] * 0.0 + (T1_1[x + 4] * 0.0 +
      T1_1[x] * 0.0));
    elbow_orig[x] = wrist_vect_idx_2;
  }

  /* wrist_1 = T1*T2*T3*T4*[0;0;0;1]; */
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T4[i << 2] * T1_1[x];
      T1_0[x + (i << 2)] += T4[(i << 2) + 1] * T1_1[x + 4];
      T1_0[x + (i << 2)] += T4[(i << 2) + 2] * T1_1[x + 8];
      T1_0[x + (i << 2)] += T4[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T5[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T5[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T5[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T5[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    l = T1_1[x + 12] + (T1_1[x + 8] * 0.0 + (T1_1[x + 4] * 0.0 + T1_1[x] * 0.0));
    wrist_2[x] = l;
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = total_transform[x + 12] + (total_transform[x + 8] * 0.0 +
      (total_transform[x + 4] * 0.0 + total_transform[x] * 0.0));
    end_effector_orig[x] = wrist_vect_idx_2;
  }

  /* %checking self collision%%%% */
  /* find distance between end effector and elbow */
  /* end effector */
  p1_idx_0 = wrist_2[0];
  p1_idx_1 = wrist_2[1];
  p1_idx_2 = wrist_2[2];

  /* wrist */
  p2_1[0] = end_effector_orig[0] - wrist_2[0];
  p2_1[1] = end_effector_orig[1] - wrist_2[1];
  p2_1[2] = end_effector_orig[2] - wrist_2[2];
  dist_mount = Arm_controller_v2_norm_m(p2_1);
  wrist_vect_idx_0 = (end_effector_orig[0] - wrist_2[0]) / dist_mount;
  wrist_vect_idx_1 = (end_effector_orig[1] - wrist_2[1]) / dist_mount;
  wrist_vect_idx_2 = (end_effector_orig[2] - wrist_2[2]) / dist_mount;

  /* normalised vector */
  /* wrist_front = p2; */
  wrist_back[0] = wrist_2[0] - 0.06 * wrist_vect_idx_0;
  wrist_back[1] = wrist_2[1] - 0.06 * wrist_vect_idx_1;
  wrist_back[2] = wrist_2[2] - 0.06 * wrist_vect_idx_2;

  /* same line as */
  /* point representing camera */
  /* point representing end of wire connector */
  /* top of camera mount */
  /* p2offset = total_transform*[0;-0.072;0;1];%if end effector is rotated by 0 */
  for (x = 0; x < 4; x++) {
    l = total_transform[x + 12] + (total_transform[x + 8] * 0.0 +
      (total_transform[x + 4] * 0.072 + total_transform[x] * 0.0));
    wrist_2[x] = l;
  }

  /* if end effector is rotated by 180 */
  p3offset[0] = 0.09 * wrist_vect_idx_0 + wrist_2[0];
  p3offset[1] = 0.09 * wrist_vect_idx_1 + wrist_2[1];
  p3offset[2] = 0.09 * wrist_vect_idx_2 + wrist_2[2];
  p1offset[0] = wrist_2[0] - 0.2 * wrist_vect_idx_0;
  p1offset[1] = wrist_2[1] - 0.2 * wrist_vect_idx_1;
  p1offset[2] = wrist_2[2] - 0.2 * wrist_vect_idx_2;

  /* end of wire */
  pwirestart[0] = wrist_2[0] - 0.082 * wrist_vect_idx_0;
  pwirestart[1] = wrist_2[1] - 0.082 * wrist_vect_idx_1;
  pwirestart[2] = wrist_2[2] - 0.082 * wrist_vect_idx_2;

  /* start of wire */
  p2_0[0] = 0.09 * wrist_vect_idx_0 + end_effector_orig[0];
  p2_0[1] = 0.09 * wrist_vect_idx_1 + end_effector_orig[1];
  p2_0[2] = 0.09 * wrist_vect_idx_2 + end_effector_orig[2];
  elbow_orig_3[0] = elbow_orig[0];
  elbow_orig_3[1] = elbow_orig[1];
  elbow_orig_3[2] = elbow_orig[2];
  shoulder_orig_3[0] = shoulder_orig[0];
  shoulder_orig_3[1] = shoulder_orig[1];
  shoulder_orig_3[2] = shoulder_orig[2];

  /* collision with shoulder. Need to offset points by a certain point */
  /* shoulder CS */
  /* end of shoulder */
  /* start of shoulder */
  /* actual shouler */
  for (x = 0; x < 4; x++) {
    l = T1[x + 12] + (T1[x + 8] * 0.135 + (T1[x + 4] * 0.0 + T1[x] * 0.0));
    wrist_2[x] = l;
  }

  /* bottom of shoulder */
  p2[0] = (shoulder_orig[0] - shoulder_start[0]) + wrist_2[0];
  p2[1] = (shoulder_orig[1] - shoulder_start[1]) + wrist_2[1];
  p2[2] = (shoulder_orig[2] - shoulder_start[2]) + wrist_2[2];

  /* top of shoulder */
  /* checking self collision */
  elbow_orig_0[0] = elbow_orig[0];
  elbow_orig_0[1] = elbow_orig[1];
  elbow_orig_0[2] = elbow_orig[2];
  shoulder_orig_0[0] = shoulder_orig[0];
  shoulder_orig_0[1] = shoulder_orig[1];
  shoulder_orig_0[2] = shoulder_orig[2];
  p1[0] = p1_idx_0 - 0.114 * wrist_vect_idx_0;
  p1[1] = p1_idx_1 - 0.114 * wrist_vect_idx_1;
  p1[2] = p1_idx_2 - 0.114 * wrist_vect_idx_2;
  elbow_orig_1[0] = elbow_orig[0];
  elbow_orig_1[1] = elbow_orig[1];
  elbow_orig_1[2] = elbow_orig[2];
  shoulder_orig_1[0] = shoulder_orig[0];
  shoulder_orig_1[1] = shoulder_orig[1];
  shoulder_orig_1[2] = shoulder_orig[2];
  elbow_orig_2[0] = elbow_orig[0];
  elbow_orig_2[1] = elbow_orig[1];
  elbow_orig_2[2] = elbow_orig[2];
  shoulder_orig_2[0] = shoulder_orig[0];
  shoulder_orig_2[1] = shoulder_orig[1];
  shoulder_orig_2[2] = shoulder_orig[2];
  wrist_2_0[0] = wrist_2[0];
  wrist_2_0[1] = wrist_2[1];
  wrist_2_0[2] = wrist_2[2];
  if ((Arm_contr_DistBetween2Segment_a(p3offset, pwirestart, elbow_orig_0,
        shoulder_orig_0) < 0.1) || (Arm_contr_DistBetween2Segment_a(p2_0,
        wrist_back, elbow_orig_3, shoulder_orig_3) < 0.1) ||
      (Arm_contr_DistBetween2Segment_a(wrist_back, p1, elbow_orig_1,
        shoulder_orig_1) < 0.054) || (Arm_contr_DistBetween2Segment_a(pwirestart,
        p1offset, elbow_orig_2, shoulder_orig_2) < 0.054) ||
      (Arm_contr_DistBetween2Segment_a(p3offset, p1offset, p2, wrist_2_0) < 0.09))
  {
    flag = 1.0;

    /* there is a collision */
  } else {
    flag = 0.0;

    /* there is no collision */
  }

  /* %%%%%%%%end of self collision%%%% */
  /* %%creating sample of points across the arm%%%% */
  /* reference co-ordinates of 3 main linkages */
  /* start and end co-ordinates for shoulder */
  /* %creating a set of points which represent the arm */
  for (x = 0; x < 43; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    shoulder[0] = p2[0] - wrist_2[0];
    shoulder[1] = p2[1] - wrist_2[1];
    shoulder[2] = p2[2] - wrist_2[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_m(shoulder);
    shoulder_points[x] = (p2[0] - wrist_2[0]) * dist_mount / wrist_vect_idx_2 +
      wrist_2[0];
    shoulder_points[x + 43] = (p2[1] - wrist_2[1]) * dist_mount /
      wrist_vect_idx_2 + wrist_2[1];
    shoulder_points[x + 86] = (p2[2] - wrist_2[2]) * dist_mount /
      wrist_vect_idx_2 + wrist_2[2];
  }

  for (x = 0; x < 40; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    elbow[0] = shoulder_orig[0] - elbow_orig[0];
    elbow[1] = shoulder_orig[1] - elbow_orig[1];
    elbow[2] = shoulder_orig[2] - elbow_orig[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_m(elbow);
    elbow_points[x] = (shoulder_orig[0] - elbow_orig[0]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[0];
    elbow_points[x + 40] = (shoulder_orig[1] - elbow_orig[1]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[1];
    elbow_points[x + 80] = (shoulder_orig[2] - elbow_orig[2]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[2];
  }

  for (x = 0; x < 29; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    laser[0] = p1offset[0] - p3offset[0];
    laser[1] = p1offset[1] - p3offset[1];
    laser[2] = p1offset[2] - p3offset[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_m(laser);
    laser_points[x] = (p1offset[0] - p3offset[0]) * dist_mount /
      wrist_vect_idx_2 + p3offset[0];
    laser_points[x + 29] = (p1offset[1] - p3offset[1]) * dist_mount /
      wrist_vect_idx_2 + p3offset[1];
    laser_points[x + 58] = (p1offset[2] - p3offset[2]) * dist_mount /
      wrist_vect_idx_2 + p3offset[2];
  }

  /* %combining the vectors into one */
  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x], &shoulder_points[43 * x], 43U * sizeof(real_T));
  }

  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x + 43], &elbow_points[40 * x], 40U * sizeof(real_T));
  }

  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x + 83], &laser_points[29 * x], 29U * sizeof(real_T));
  }

  /* [m, n] = size(fullarm); */
  /* %defining obstacles%% */
  /* %radius of shoulder */
  /* %left wheel%% */
  /* x, y, z co-ordinate of left wheel top right corner closest to base */
  /* if (  */
  /* %right wheel%% */
  /* x, y, z co-ordinate of right wheel top left corner closest to base */
  /* %body%% */
  /* top right corner */
  /* exagerrated to make sure arm never goes in that region */
  /* %pan tilt%% */
  /* %ground%% use laser scanner */
  /* % */
  /* disp(m); */
  x = 0;
  exitg1 = false;
  while ((!exitg1) && (x < 112)) {
    /* disp([x, y, z]); */
    /* disp(y); */
    /* disp(z); */
    if (fullarm[224 + x] < -0.25) {
      /* dont move */
      /* give collision flag */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.05) && (fullarm[x] < 0.21000000000000002) &&
               (fullarm[112 + x] > 0.15000000000000002) && (fullarm[112 + x] <
                0.37) && (fullarm[224 + x] < 0.135) && (fullarm[224 + x] >
                -0.31499999999999995)) {
      /* %the point is inside left wheel */
      /* %collision */
      /* disp('left wheel collision'); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.090000000000000011) && (fullarm[x] <
                0.21000000000000002) && (fullarm[112 + x] > -0.37) && (fullarm
                [112 + x] < -0.15000000000000002) && (fullarm[224 + x] < 0.135) &&
               (fullarm[224 + x] > -0.31499999999999995)) {
      /* %right wheel */
      /* disp('right wheel collision'); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -1.6400000000000001) && (fullarm[x] <
                -0.039999999999999994) && (fullarm[112 + x] > -0.25) &&
               (fullarm[112 + x] < 0.25) && (fullarm[224 + x] < 0.25) &&
               (fullarm[224 + x] > -0.25)) {
      /* %body */
      /* disp('body collision'); */
      /* disp([i x y z]); */
      /* disp([body_x_b, body_x_f]); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.14) && (fullarm[x] < 0.15000000000000002) &&
               (fullarm[112 + x] > 0.03) && (fullarm[112 + x] < 0.29) &&
               (fullarm[224 + x] < 0.0) && (fullarm[224 + x] > -0.25)) {
      /* %pantilt */
      /* disp('camera collision'); */
      flag = 1.0;
      exitg1 = true;
    } else {
      /* no collision */
      /* disp('no collision'); */
      flag = 0.0;
      x++;
    }
  }

  return flag;
}

/* Function for MATLAB Function: '<S1>/set_pos2' */
static void Arm_controller_v2_cosd_e(real_T *x)
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

/* Function for MATLAB Function: '<S1>/set_pos2' */
static void Arm_controller_v2_sind_p(real_T *x)
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

/* Function for MATLAB Function: '<S1>/set_pos2' */
static real_T Arm_controller_v2_norm_e(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 2.2250738585072014E-308;
  absxk = fabs(x[0]);
  if (absxk > 2.2250738585072014E-308) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S1>/set_pos2' */
static real_T Arm_controller_v2_dot_b(const real_T a[3], const real_T b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

/* Function for MATLAB Function: '<S1>/set_pos2' */
static real_T Arm_contr_DistBetween2Segment_o(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3])
{
  real_T distance;
  real_T u[3];
  real_T v[3];
  real_T w[3];
  real_T a;
  real_T b;
  real_T c;
  real_T d;
  real_T e;
  real_T D;
  real_T tD;
  real_T sN;
  real_T tN;
  real_T w_0[3];

  /*  Computes the minimum distance between two line segments. Code */
  /*  is adapted for Matlab from Dan Sunday's Geometry Algorithms originally */
  /*  written in C++ */
  /*  http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment */
  /*  Usage: Input the start and end x,y,z coordinates for two line segments.  */
  /*  p1, p2 are [x,y,z] coordinates of first line segment and p3,p4 are for */
  /*  second line segment.  */
  /*  Output: scalar minimum distance between the two segments. */
  /*   Example: */
  /* 	P1 = [0 0 0];     P2 = [1 0 0]; */
  /*    P3 = [0 1 0];     P4 = [1 1 0]; */
  /* 	dist = DistBetween2Segment(P1, P2, P3, P4) */
  /*    dist = */
  /*  */
  /*     1 */
  /*   */
  u[0] = p1[0] - p2[0];
  u[1] = p1[1] - p2[1];
  u[2] = p1[2] - p2[2];
  v[0] = p3[0] - p4[0];
  v[1] = p3[1] - p4[1];
  v[2] = p3[2] - p4[2];
  w[0] = p2[0] - p4[0];
  w[1] = p2[1] - p4[1];
  w[2] = p2[2] - p4[2];
  a = Arm_controller_v2_dot_b(u, u);
  b = Arm_controller_v2_dot_b(u, v);
  c = Arm_controller_v2_dot_b(v, v);
  d = Arm_controller_v2_dot_b(u, w);
  e = Arm_controller_v2_dot_b(v, w);
  D = a * c - b * b;
  tD = D;

  /*  compute the line parameters of the two closest points */
  if (D < 1.0E-8) {
    /*  the lines are almost parallel */
    sN = 0.0;

    /*  force using point P0 on segment S1 */
    D = 1.0;

    /*  to prevent possible division by 0.0 later */
    tN = e;
    tD = c;
  } else {
    /*  get the closest points on the infinite lines */
    sN = b * e - c * d;
    tN = a * e - b * d;
    if (sN < 0.0) {
      /*  sc < 0 => the s=0 edge is visible        */
      sN = 0.0;
      tN = e;
      tD = c;
    } else {
      if (sN > D) {
        /*  sc > 1 => the s=1 edge is visible */
        sN = D;
        tN = e + b;
        tD = c;
      }
    }
  }

  if (tN < 0.0) {
    /*  tc < 0 => the t=0 edge is visible */
    tN = 0.0;

    /*  recompute sc for this edge */
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = D;
    } else {
      sN = -d;
      D = a;
    }
  } else {
    if (tN > tD) {
      /*  tc > 1 => the t=1 edge is visible */
      tN = tD;

      /*  recompute sc for this edge */
      if (-d + b < 0.0) {
        sN = 0.0;
      } else if (-d + b > a) {
        sN = D;
      } else {
        sN = -d + b;
        D = a;
      }
    }
  }

  /*  finally do the division to get sc and tc */
  if (fabs(sN) < 1.0E-8) {
    b = 0.0;
  } else {
    b = sN / D;
  }

  if (fabs(tN) < 1.0E-8) {
    a = 0.0;
  } else {
    a = tN / tD;
  }

  /*  get the difference of the two closest points */
  /*  = S1(sc) - S2(tc) */
  w_0[0] = (b * u[0] + w[0]) - a * v[0];
  w_0[1] = (b * u[1] + w[1]) - a * v[1];
  w_0[2] = (b * u[2] + w[2]) - a * v[2];
  distance = Arm_controller_v2_norm_e(w_0);

  /* outV = dP; */
  /* varargout(1) = {outV};      % vector connecting the closest points */
  /* varargout(2) = {p2+sc*u};   % Closest point on object 1  */
  /* varargout(3) = {p4+tc*v};   % Closest point on object 2 */
  return distance;
}

/* Function for MATLAB Function: '<S1>/set_pos2' */
static real_T Arm_controlle_check_collision_f(const real_T joint_angles_degrees
  [6])
{
  real_T flag;
  real_T T1[16];
  real_T T2[16];
  real_T T3[16];
  real_T T4[16];
  real_T T5[16];
  real_T total_transform[16];
  real_T shoulder_start[4];
  real_T shoulder_orig[4];
  real_T elbow_orig[4];
  real_T wrist_2[4];
  real_T end_effector_orig[4];
  real_T p2[3];
  real_T wrist_back[3];
  real_T p3offset[3];
  real_T p1offset[3];
  real_T pwirestart[3];
  real_T dist_mount;
  real_T shoulder_points[129];
  real_T elbow_points[120];
  real_T laser_points[87];
  real_T fullarm[336];
  int32_T x;
  real_T l;
  boolean_T exitg1;
  real_T elbow_orig_0[3];
  real_T shoulder_orig_0[3];
  real_T p1[3];
  real_T elbow_orig_1[3];
  real_T shoulder_orig_1[3];
  real_T elbow_orig_2[3];
  real_T shoulder_orig_2[3];
  real_T wrist_2_0[3];
  real_T p2_0[3];
  real_T elbow_orig_3[3];
  real_T shoulder_orig_3[3];
  real_T p2_1[3];
  real_T shoulder[3];
  real_T elbow[3];
  real_T laser[3];
  real_T T1_0[16];
  real_T T1_1[16];
  real_T dist_mount_0[16];
  int32_T i;
  real_T p1_idx_0;
  real_T p1_idx_1;
  real_T p1_idx_2;
  real_T wrist_vect_idx_0;
  real_T wrist_vect_idx_1;
  real_T wrist_vect_idx_2;

  /* UNTITLED2 Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* create sample of points */
  /* update with new self collision function */
  /* DH matrices */
  dist_mount = joint_angles_degrees[0];
  Arm_controller_v2_cosd_e(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[0];
  Arm_controller_v2_sind_p(&wrist_vect_idx_2);
  l = joint_angles_degrees[0];
  Arm_controller_v2_sind_p(&l);
  p1_idx_0 = joint_angles_degrees[0];
  Arm_controller_v2_cosd_e(&p1_idx_0);
  T1[0] = dist_mount;
  T1[4] = 0.0;
  T1[8] = wrist_vect_idx_2;
  T1[12] = 0.0;
  T1[1] = l;
  T1[5] = 0.0;
  T1[9] = -p1_idx_0;
  T1[13] = 0.0;
  T1[2] = 0.0;
  T1[6] = 1.0;
  T1[10] = 0.0;
  T1[14] = 0.089;
  T1[3] = 0.0;
  T1[7] = 0.0;
  T1[11] = 0.0;
  T1[15] = 1.0;
  dist_mount = joint_angles_degrees[1];
  Arm_controller_v2_cosd_e(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind_p(&wrist_vect_idx_2);
  l = joint_angles_degrees[1];
  Arm_controller_v2_cosd_e(&l);
  p1_idx_0 = joint_angles_degrees[1];
  Arm_controller_v2_sind_p(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[1];
  Arm_controller_v2_cosd_e(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[1];
  Arm_controller_v2_sind_p(&p1_idx_2);
  T2[0] = dist_mount;
  T2[4] = -wrist_vect_idx_2;
  T2[8] = 0.0;
  T2[12] = -0.425 * l;
  T2[1] = p1_idx_0;
  T2[5] = p1_idx_1;
  T2[9] = 0.0;
  T2[13] = -0.425 * p1_idx_2;
  T2[2] = 0.0;
  T2[6] = 0.0;
  T2[10] = 1.0;
  T2[14] = 0.0;
  T2[3] = 0.0;
  T2[7] = 0.0;
  T2[11] = 0.0;
  T2[15] = 1.0;
  dist_mount = joint_angles_degrees[2];
  Arm_controller_v2_cosd_e(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind_p(&wrist_vect_idx_2);
  l = joint_angles_degrees[2];
  Arm_controller_v2_cosd_e(&l);
  p1_idx_0 = joint_angles_degrees[2];
  Arm_controller_v2_sind_p(&p1_idx_0);
  p1_idx_1 = joint_angles_degrees[2];
  Arm_controller_v2_cosd_e(&p1_idx_1);
  p1_idx_2 = joint_angles_degrees[2];
  Arm_controller_v2_sind_p(&p1_idx_2);
  T3[0] = dist_mount;
  T3[4] = -wrist_vect_idx_2;
  T3[8] = 0.0;
  T3[12] = -0.392 * l;
  T3[1] = p1_idx_0;
  T3[5] = p1_idx_1;
  T3[9] = 0.0;
  T3[13] = -0.392 * p1_idx_2;
  T3[2] = 0.0;
  T3[6] = 0.0;
  T3[10] = 1.0;
  T3[14] = 0.0;
  T3[3] = 0.0;
  T3[7] = 0.0;
  T3[11] = 0.0;
  T3[15] = 1.0;
  dist_mount = joint_angles_degrees[3];
  Arm_controller_v2_cosd_e(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[3];
  Arm_controller_v2_sind_p(&wrist_vect_idx_2);
  l = joint_angles_degrees[3];
  Arm_controller_v2_sind_p(&l);
  p1_idx_0 = joint_angles_degrees[3];
  Arm_controller_v2_cosd_e(&p1_idx_0);
  T4[0] = dist_mount;
  T4[4] = 0.0;
  T4[8] = wrist_vect_idx_2;
  T4[12] = 0.0;
  T4[1] = l;
  T4[5] = 0.0;
  T4[9] = -p1_idx_0;
  T4[13] = 0.0;
  T4[2] = 0.0;
  T4[6] = 1.0;
  T4[10] = 0.0;
  T4[14] = 0.109;
  T4[3] = 0.0;
  T4[7] = 0.0;
  T4[11] = 0.0;
  T4[15] = 1.0;
  dist_mount = joint_angles_degrees[4];
  Arm_controller_v2_cosd_e(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[4];
  Arm_controller_v2_sind_p(&wrist_vect_idx_2);
  l = joint_angles_degrees[4];
  Arm_controller_v2_sind_p(&l);
  p1_idx_0 = joint_angles_degrees[4];
  Arm_controller_v2_cosd_e(&p1_idx_0);
  T5[0] = dist_mount;
  T5[4] = 0.0;
  T5[8] = -wrist_vect_idx_2;
  T5[12] = 0.0;
  T5[1] = l;
  T5[5] = 0.0;
  T5[9] = p1_idx_0;
  T5[13] = 0.0;
  T5[2] = 0.0;
  T5[6] = -1.0;
  T5[10] = 0.0;
  T5[14] = 0.095;
  T5[3] = 0.0;
  T5[7] = 0.0;
  T5[11] = 0.0;
  T5[15] = 1.0;
  dist_mount = joint_angles_degrees[5];
  Arm_controller_v2_cosd_e(&dist_mount);
  wrist_vect_idx_2 = joint_angles_degrees[5];
  Arm_controller_v2_sind_p(&wrist_vect_idx_2);
  l = joint_angles_degrees[5];
  Arm_controller_v2_sind_p(&l);
  p1_idx_0 = joint_angles_degrees[5];
  Arm_controller_v2_cosd_e(&p1_idx_0);
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T4[i << 2] * T1_1[x];
      T1_0[x + (i << 2)] += T4[(i << 2) + 1] * T1_1[x + 4];
      T1_0[x + (i << 2)] += T4[(i << 2) + 2] * T1_1[x + 8];
      T1_0[x + (i << 2)] += T4[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T5[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T5[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T5[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T5[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  dist_mount_0[0] = dist_mount;
  dist_mount_0[4] = -wrist_vect_idx_2;
  dist_mount_0[8] = 0.0;
  dist_mount_0[12] = 0.0;
  dist_mount_0[1] = l;
  dist_mount_0[5] = p1_idx_0;
  dist_mount_0[9] = 0.0;
  dist_mount_0[13] = 0.0;
  dist_mount_0[2] = 0.0;
  dist_mount_0[6] = 0.0;
  dist_mount_0[10] = 1.0;
  dist_mount_0[14] = 0.082;
  dist_mount_0[3] = 0.0;
  dist_mount_0[7] = 0.0;
  dist_mount_0[11] = 0.0;
  dist_mount_0[15] = 1.0;
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      total_transform[x + (i << 2)] = 0.0;
      total_transform[x + (i << 2)] += dist_mount_0[i << 2] * T1_1[x];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 1] * T1_1[x + 4];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 2] * T1_1[x + 8];
      total_transform[x + (i << 2)] += dist_mount_0[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  /* joint origin co-ordinates */
  /* arm_orig = [0;0;0;1]; */
  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1[x + 12] + (T1[x + 8] * 0.0 + (T1[x + 4] * 0.0 + T1[x] *
      0.0));
    shoulder_start[x] = wrist_vect_idx_2;
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1_0[x + 12] + (T1_0[x + 8] * 0.0 + (T1_0[x + 4] * 0.0 +
      T1_0[x] * 0.0));
    shoulder_orig[x] = wrist_vect_idx_2;
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = T1_1[x + 12] + (T1_1[x + 8] * 0.0 + (T1_1[x + 4] * 0.0 +
      T1_1[x] * 0.0));
    elbow_orig[x] = wrist_vect_idx_2;
  }

  /* wrist_1 = T1*T2*T3*T4*[0;0;0;1]; */
  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T2[i << 2] * T1[x];
      T1_0[x + (i << 2)] += T2[(i << 2) + 1] * T1[x + 4];
      T1_0[x + (i << 2)] += T2[(i << 2) + 2] * T1[x + 8];
      T1_0[x + (i << 2)] += T2[(i << 2) + 3] * T1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T3[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T3[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T3[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T3[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_0[x + (i << 2)] = 0.0;
      T1_0[x + (i << 2)] += T4[i << 2] * T1_1[x];
      T1_0[x + (i << 2)] += T4[(i << 2) + 1] * T1_1[x + 4];
      T1_0[x + (i << 2)] += T4[(i << 2) + 2] * T1_1[x + 8];
      T1_0[x + (i << 2)] += T4[(i << 2) + 3] * T1_1[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    for (i = 0; i < 4; i++) {
      T1_1[x + (i << 2)] = 0.0;
      T1_1[x + (i << 2)] += T5[i << 2] * T1_0[x];
      T1_1[x + (i << 2)] += T5[(i << 2) + 1] * T1_0[x + 4];
      T1_1[x + (i << 2)] += T5[(i << 2) + 2] * T1_0[x + 8];
      T1_1[x + (i << 2)] += T5[(i << 2) + 3] * T1_0[x + 12];
    }
  }

  for (x = 0; x < 4; x++) {
    l = T1_1[x + 12] + (T1_1[x + 8] * 0.0 + (T1_1[x + 4] * 0.0 + T1_1[x] * 0.0));
    wrist_2[x] = l;
  }

  for (x = 0; x < 4; x++) {
    wrist_vect_idx_2 = total_transform[x + 12] + (total_transform[x + 8] * 0.0 +
      (total_transform[x + 4] * 0.0 + total_transform[x] * 0.0));
    end_effector_orig[x] = wrist_vect_idx_2;
  }

  /* %checking self collision%%%% */
  /* find distance between end effector and elbow */
  /* end effector */
  p1_idx_0 = wrist_2[0];
  p1_idx_1 = wrist_2[1];
  p1_idx_2 = wrist_2[2];

  /* wrist */
  p2_1[0] = end_effector_orig[0] - wrist_2[0];
  p2_1[1] = end_effector_orig[1] - wrist_2[1];
  p2_1[2] = end_effector_orig[2] - wrist_2[2];
  dist_mount = Arm_controller_v2_norm_e(p2_1);
  wrist_vect_idx_0 = (end_effector_orig[0] - wrist_2[0]) / dist_mount;
  wrist_vect_idx_1 = (end_effector_orig[1] - wrist_2[1]) / dist_mount;
  wrist_vect_idx_2 = (end_effector_orig[2] - wrist_2[2]) / dist_mount;

  /* normalised vector */
  /* wrist_front = p2; */
  wrist_back[0] = wrist_2[0] - 0.06 * wrist_vect_idx_0;
  wrist_back[1] = wrist_2[1] - 0.06 * wrist_vect_idx_1;
  wrist_back[2] = wrist_2[2] - 0.06 * wrist_vect_idx_2;

  /* same line as */
  /* point representing camera */
  /* point representing end of wire connector */
  /* top of camera mount */
  /* p2offset = total_transform*[0;-0.072;0;1];%if end effector is rotated by 0 */
  for (x = 0; x < 4; x++) {
    l = total_transform[x + 12] + (total_transform[x + 8] * 0.0 +
      (total_transform[x + 4] * 0.072 + total_transform[x] * 0.0));
    wrist_2[x] = l;
  }

  /* if end effector is rotated by 180 */
  p3offset[0] = 0.09 * wrist_vect_idx_0 + wrist_2[0];
  p3offset[1] = 0.09 * wrist_vect_idx_1 + wrist_2[1];
  p3offset[2] = 0.09 * wrist_vect_idx_2 + wrist_2[2];
  p1offset[0] = wrist_2[0] - 0.2 * wrist_vect_idx_0;
  p1offset[1] = wrist_2[1] - 0.2 * wrist_vect_idx_1;
  p1offset[2] = wrist_2[2] - 0.2 * wrist_vect_idx_2;

  /* end of wire */
  pwirestart[0] = wrist_2[0] - 0.082 * wrist_vect_idx_0;
  pwirestart[1] = wrist_2[1] - 0.082 * wrist_vect_idx_1;
  pwirestart[2] = wrist_2[2] - 0.082 * wrist_vect_idx_2;

  /* start of wire */
  p2_0[0] = 0.09 * wrist_vect_idx_0 + end_effector_orig[0];
  p2_0[1] = 0.09 * wrist_vect_idx_1 + end_effector_orig[1];
  p2_0[2] = 0.09 * wrist_vect_idx_2 + end_effector_orig[2];
  elbow_orig_3[0] = elbow_orig[0];
  elbow_orig_3[1] = elbow_orig[1];
  elbow_orig_3[2] = elbow_orig[2];
  shoulder_orig_3[0] = shoulder_orig[0];
  shoulder_orig_3[1] = shoulder_orig[1];
  shoulder_orig_3[2] = shoulder_orig[2];

  /* collision with shoulder. Need to offset points by a certain point */
  /* shoulder CS */
  /* end of shoulder */
  /* start of shoulder */
  /* actual shouler */
  for (x = 0; x < 4; x++) {
    l = T1[x + 12] + (T1[x + 8] * 0.135 + (T1[x + 4] * 0.0 + T1[x] * 0.0));
    wrist_2[x] = l;
  }

  /* bottom of shoulder */
  p2[0] = (shoulder_orig[0] - shoulder_start[0]) + wrist_2[0];
  p2[1] = (shoulder_orig[1] - shoulder_start[1]) + wrist_2[1];
  p2[2] = (shoulder_orig[2] - shoulder_start[2]) + wrist_2[2];

  /* top of shoulder */
  /* checking self collision */
  elbow_orig_0[0] = elbow_orig[0];
  elbow_orig_0[1] = elbow_orig[1];
  elbow_orig_0[2] = elbow_orig[2];
  shoulder_orig_0[0] = shoulder_orig[0];
  shoulder_orig_0[1] = shoulder_orig[1];
  shoulder_orig_0[2] = shoulder_orig[2];
  p1[0] = p1_idx_0 - 0.114 * wrist_vect_idx_0;
  p1[1] = p1_idx_1 - 0.114 * wrist_vect_idx_1;
  p1[2] = p1_idx_2 - 0.114 * wrist_vect_idx_2;
  elbow_orig_1[0] = elbow_orig[0];
  elbow_orig_1[1] = elbow_orig[1];
  elbow_orig_1[2] = elbow_orig[2];
  shoulder_orig_1[0] = shoulder_orig[0];
  shoulder_orig_1[1] = shoulder_orig[1];
  shoulder_orig_1[2] = shoulder_orig[2];
  elbow_orig_2[0] = elbow_orig[0];
  elbow_orig_2[1] = elbow_orig[1];
  elbow_orig_2[2] = elbow_orig[2];
  shoulder_orig_2[0] = shoulder_orig[0];
  shoulder_orig_2[1] = shoulder_orig[1];
  shoulder_orig_2[2] = shoulder_orig[2];
  wrist_2_0[0] = wrist_2[0];
  wrist_2_0[1] = wrist_2[1];
  wrist_2_0[2] = wrist_2[2];
  if ((Arm_contr_DistBetween2Segment_o(p3offset, pwirestart, elbow_orig_0,
        shoulder_orig_0) < 0.1) || (Arm_contr_DistBetween2Segment_o(p2_0,
        wrist_back, elbow_orig_3, shoulder_orig_3) < 0.1) ||
      (Arm_contr_DistBetween2Segment_o(wrist_back, p1, elbow_orig_1,
        shoulder_orig_1) < 0.054) || (Arm_contr_DistBetween2Segment_o(pwirestart,
        p1offset, elbow_orig_2, shoulder_orig_2) < 0.054) ||
      (Arm_contr_DistBetween2Segment_o(p3offset, p1offset, p2, wrist_2_0) < 0.09))
  {
    flag = 1.0;

    /* there is a collision */
  } else {
    flag = 0.0;

    /* there is no collision */
  }

  /* %%%%%%%%end of self collision%%%% */
  /* %%creating sample of points across the arm%%%% */
  /* reference co-ordinates of 3 main linkages */
  /* start and end co-ordinates for shoulder */
  /* %creating a set of points which represent the arm */
  for (x = 0; x < 43; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    shoulder[0] = p2[0] - wrist_2[0];
    shoulder[1] = p2[1] - wrist_2[1];
    shoulder[2] = p2[2] - wrist_2[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_e(shoulder);
    shoulder_points[x] = (p2[0] - wrist_2[0]) * dist_mount / wrist_vect_idx_2 +
      wrist_2[0];
    shoulder_points[x + 43] = (p2[1] - wrist_2[1]) * dist_mount /
      wrist_vect_idx_2 + wrist_2[1];
    shoulder_points[x + 86] = (p2[2] - wrist_2[2]) * dist_mount /
      wrist_vect_idx_2 + wrist_2[2];
  }

  for (x = 0; x < 40; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    elbow[0] = shoulder_orig[0] - elbow_orig[0];
    elbow[1] = shoulder_orig[1] - elbow_orig[1];
    elbow[2] = shoulder_orig[2] - elbow_orig[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_e(elbow);
    elbow_points[x] = (shoulder_orig[0] - elbow_orig[0]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[0];
    elbow_points[x + 40] = (shoulder_orig[1] - elbow_orig[1]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[1];
    elbow_points[x + 80] = (shoulder_orig[2] - elbow_orig[2]) * dist_mount /
      wrist_vect_idx_2 + elbow_orig[2];
  }

  for (x = 0; x < 29; x++) {
    dist_mount = (1.0 + (real_T)x) * 0.01;
    laser[0] = p1offset[0] - p3offset[0];
    laser[1] = p1offset[1] - p3offset[1];
    laser[2] = p1offset[2] - p3offset[2];
    wrist_vect_idx_2 = Arm_controller_v2_norm_e(laser);
    laser_points[x] = (p1offset[0] - p3offset[0]) * dist_mount /
      wrist_vect_idx_2 + p3offset[0];
    laser_points[x + 29] = (p1offset[1] - p3offset[1]) * dist_mount /
      wrist_vect_idx_2 + p3offset[1];
    laser_points[x + 58] = (p1offset[2] - p3offset[2]) * dist_mount /
      wrist_vect_idx_2 + p3offset[2];
  }

  /* %combining the vectors into one */
  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x], &shoulder_points[43 * x], 43U * sizeof(real_T));
  }

  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x + 43], &elbow_points[40 * x], 40U * sizeof(real_T));
  }

  for (x = 0; x < 3; x++) {
    memcpy(&fullarm[112 * x + 83], &laser_points[29 * x], 29U * sizeof(real_T));
  }

  /* [m, n] = size(fullarm); */
  /* %defining obstacles%% */
  /* %radius of shoulder */
  /* %left wheel%% */
  /* x, y, z co-ordinate of left wheel top right corner closest to base */
  /* if (  */
  /* %right wheel%% */
  /* x, y, z co-ordinate of right wheel top left corner closest to base */
  /* %body%% */
  /* top right corner */
  /* exagerrated to make sure arm never goes in that region */
  /* %pan tilt%% */
  /* %ground%% use laser scanner */
  /* % */
  /* disp(m); */
  x = 0;
  exitg1 = false;
  while ((!exitg1) && (x < 112)) {
    /* disp([x, y, z]); */
    /* disp(y); */
    /* disp(z); */
    if (fullarm[224 + x] < -0.25) {
      /* dont move */
      /* give collision flag */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.05) && (fullarm[x] < 0.21000000000000002) &&
               (fullarm[112 + x] > 0.15000000000000002) && (fullarm[112 + x] <
                0.37) && (fullarm[224 + x] < 0.135) && (fullarm[224 + x] >
                -0.31499999999999995)) {
      /* %the point is inside left wheel */
      /* %collision */
      /* disp('left wheel collision'); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.090000000000000011) && (fullarm[x] <
                0.21000000000000002) && (fullarm[112 + x] > -0.37) && (fullarm
                [112 + x] < -0.15000000000000002) && (fullarm[224 + x] < 0.135) &&
               (fullarm[224 + x] > -0.31499999999999995)) {
      /* %right wheel */
      /* disp('right wheel collision'); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -1.6400000000000001) && (fullarm[x] <
                -0.039999999999999994) && (fullarm[112 + x] > -0.25) &&
               (fullarm[112 + x] < 0.25) && (fullarm[224 + x] < 0.25) &&
               (fullarm[224 + x] > -0.25)) {
      /* %body */
      /* disp('body collision'); */
      /* disp([i x y z]); */
      /* disp([body_x_b, body_x_f]); */
      flag = 1.0;
      exitg1 = true;
    } else if ((fullarm[x] > -0.14) && (fullarm[x] < 0.15000000000000002) &&
               (fullarm[112 + x] > 0.03) && (fullarm[112 + x] < 0.29) &&
               (fullarm[224 + x] < 0.0) && (fullarm[224 + x] > -0.25)) {
      /* %pantilt */
      /* disp('camera collision'); */
      flag = 1.0;
      exitg1 = true;
    } else {
      /* no collision */
      /* disp('no collision'); */
      flag = 0.0;
      x++;
    }
  }

  return flag;
}

/* Model step function */
void Arm_controller_v2_step(void)
{
  int32_T statusflag_movecam;
  real_T j4;
  real_T j3;
  real_T joint_angles_degrees[6];
  int32_T collision_flag;
  real_T T1[16];
  real_T T2[16];
  real_T T3[16];
  real_T T4[16];
  real_T T5[16];
  real_T total_transform[16];
  real_T shoulder_start[4];
  real_T shoulder_orig[4];
  real_T elbow_orig[4];
  real_T wrist_2[4];
  real_T end_effector_orig[4];
  real_T wrist_back[3];
  real_T p3offset[3];
  real_T p1offset[3];
  real_T pwirestart[3];
  real_T j;
  real_T p;
  real_T q;
  real_T r;
  real_T s;
  real_T t;
  real_T u;
  real_T v;
  real_T w;
  real_T x;
  real_T y;
  real_T ab;
  real_T bb;
  real_T cb;
  real_T db;
  real_T eb;
  real_T fb;
  real_T gb;
  real_T hb;
  real_T ib;
  real_T jb;
  static const int16_T b[6] = { 0, -90, 0, -90, 90, 180 };

  int32_T status_scan;
  real_T scan_bottompos1[6];
  real_T scan_toppos1[6];
  int32_T status_movecam2;
  int32_T status_flag;
  int32_T waypoint_number;
  real_T q_sols[48];
  int8_T checksol[8];
  int32_T current_config;
  static const real_T b_0[6] = { 180.0, -90.0, 0.0, -90.0, 90.0, 180.0 };

  static const real_T c[6] = { 180.0, -90.0, 0.0, -90.0, -90.0, 180.0 };

  static const real_T d[6] = { 0.0, -90.0, 0.0, -90.0, -90.0, 180.0 };

  static const real_T e[6] = { 0.0, -90.0, 0.0, -90.0, 90.0, 180.0 };

  int32_T waypoint_number_0;
  static const real_T b_1[6] = { 0.0, -90.0, 0.0, -90.0, -90.0, 180.0 };

  static const real_T c_0[6] = { 0.0, -90.0, 0.0, -90.0, 90.0, 180.0 };

  real_T rtb_Converttodegrees[6];
  real_T rtb_writehome;
  real_T rtb_MultiportSwitch2[6];
  int32_T rtb_portoutput;
  int32_T rtb_statusflag_setpos;
  int32_T rtb_statusflag_movej;
  real_T rtb_TmpSignalConversionAtSFunct[6];
  int8_T rtb_enableflags[8];
  real_T rtb_outputs[6];
  real_T rtb_wp4_h[6];
  real_T rtb_wp5_o[6];
  real_T rtb_jointangles_n[6];
  real_T rtb_jointangles[6];
  real_T rtb_jointangles_l[6];
  real_T rtb_scan_startpos[6];
  real_T rtb_scan_bottompos[6];
  real_T rtb_scan_toppos[6];
  real_T rtb_jointangles_a[6];
  real_T rtb_wp1_k[6];
  real_T rtb_wp2_o[6];
  real_T rtb_wp3_cw[6];
  real_T rtb_wp4_m2[6];
  real_T rtb_wp5_m[6];
  real_T rtb_wp5[6];
  real_T rtb_wp4[6];
  int32_T i;
  real_T elbow_orig_0[3];
  real_T shoulder_orig_0[3];
  real_T p2[3];
  real_T elbow_orig_1[3];
  real_T shoulder_orig_1[3];
  real_T p1[3];
  real_T elbow_orig_2[3];
  real_T shoulder_orig_2[3];
  real_T elbow_orig_3[3];
  real_T shoulder_orig_3[3];
  real_T wrist_2_0[3];
  real_T wrist_2_1[3];
  real_T p2_0[3];
  real_T T1_0[16];
  real_T T1_1[16];
  real_T hb_0[16];
  real_T rtb_writehome_0[16];
  real_T l[16];
  real_T r_0[16];
  real_T x_0[16];
  real_T cb_0[16];
  real_T gb_0[16];
  real_T p1_idx_0;
  real_T p1_idx_1;
  real_T p1_idx_2;
  real_T wrist_vect_idx_0;
  real_T wrist_vect_idx_1;
  real_T wrist_vect_idx_2;

  /* MATLAB Function: '<S1>/functionselector' incorporates:
   *  Inport: '<Root>/Input_1'
   *  Inport: '<Root>/Input_2'
   *  Inport: '<Root>/Input_3'
   *  Inport: '<Root>/Input_4'
   *  Inport: '<Root>/Input_5'
   *  Inport: '<Root>/Input_6'
   *  Inport: '<Root>/motion_primitive'
   *  SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport2'
   */
  /* MATLAB Function 'Arm_controller_v2/functionselector': '<S3>:1' */
  /* '<S3>:1:4' */
  for (i = 0; i < 8; i++) {
    rtb_enableflags[i] = 0;
  }

  /* '<S3>:1:5' */
  for (i = 0; i < 6; i++) {
    rtb_outputs[i] = 0.0;
  }

  /* '<S3>:1:7' */
  rtb_portoutput = 2;
  if (Arm_controller_v2_U.motion_primitive == 1.0) {
    /* '<S3>:1:9' */
    /* activate move_cam and disable home position. Can expand concept to */
    /* other functions */
    /* '<S3>:1:12' */
    rtb_enableflags[0] = 1;

    /* transfer the inputs received to the inputs of the move_cam function */
    /* '<S3>:1:15' */
    rtb_outputs[0] = Arm_controller_v2_U.Input_1;

    /* '<S3>:1:16' */
    rtb_outputs[1] = Arm_controller_v2_U.Input_2;

    /* '<S3>:1:17' */
    rtb_outputs[2] = Arm_controller_v2_U.Input_3;

    /* '<S3>:1:18' */
    rtb_portoutput = 1;
  } else if (Arm_controller_v2_U.motion_primitive == 2.0) {
    /* '<S3>:1:20' */
    /* activate move to home/giraffe position */
    /* '<S3>:1:22' */
    rtb_enableflags[1] = 1;

    /* '<S3>:1:24' */
    rtb_outputs[0] = Arm_controller_v2_U.Input_1;

    /* '<S3>:1:25' */
  } else if (Arm_controller_v2_U.motion_primitive == 3.0) {
    /* '<S3>:1:27' */
    /* '<S3>:1:28' */
    rtb_enableflags[2] = 1;

    /* set home position */
  } else if (Arm_controller_v2_U.motion_primitive == 4.0) {
    /* '<S3>:1:31' */
    /* movej function. full control of arm motion.  */
    /* '<S3>:1:33' */
    rtb_enableflags[3] = 1;

    /* '<S3>:1:34' */
    rtb_outputs[0] = Arm_controller_v2_U.Input_1;

    /* '<S3>:1:35' */
    rtb_outputs[1] = Arm_controller_v2_U.Input_2;

    /* '<S3>:1:36' */
    rtb_outputs[2] = Arm_controller_v2_U.Input_3;

    /* '<S3>:1:37' */
    rtb_outputs[3] = Arm_controller_v2_U.Input_4;

    /* '<S3>:1:38' */
    rtb_outputs[4] = Arm_controller_v2_U.Input_5;

    /* '<S3>:1:39' */
    rtb_outputs[5] = Arm_controller_v2_U.Input_6;

    /* '<S3>:1:40' */
    rtb_portoutput = 3;
  } else if (Arm_controller_v2_U.motion_primitive == 5.0) {
    /* '<S3>:1:42' */
    /* scan function */
    /* '<S3>:1:43' */
    rtb_enableflags[4] = 1;

    /* '<S3>:1:44' */
    rtb_outputs[0] = Arm_controller_v2_U.Input_1;

    /* '<S3>:1:45' */
    rtb_portoutput = 4;
  } else if (Arm_controller_v2_U.motion_primitive == 6.0) {
    /* '<S3>:1:47' */
    /* move_cam function */
    /* '<S3>:1:48' */
    rtb_enableflags[5] = 1;

    /* '<S3>:1:49' */
    rtb_outputs[0] = Arm_controller_v2_U.Input_1;

    /* '<S3>:1:50' */
    rtb_outputs[1] = Arm_controller_v2_U.Input_2;

    /* '<S3>:1:51' */
    rtb_portoutput = 5;
  } else if (Arm_controller_v2_U.motion_primitive == 7.0) {
    /* '<S3>:1:53' */
    /* move_effector function */
    /* '<S3>:1:54' */
    rtb_enableflags[6] = 1;

    /* '<S3>:1:55' */
    rtb_outputs[0] = Arm_controller_v2_U.Input_1;

    /* '<S3>:1:56' */
    rtb_outputs[1] = Arm_controller_v2_U.Input_2;

    /* '<S3>:1:57' */
    rtb_outputs[2] = Arm_controller_v2_U.Input_3;

    /* '<S3>:1:58' */
    rtb_portoutput = 6;
  } else if (Arm_controller_v2_U.motion_primitive == 8.0) {
    /* '<S3>:1:60' */
    /* setpos2 function */
    /* '<S3>:1:61' */
    rtb_enableflags[7] = 1;

    /* '<S3>:1:62' */
    rtb_outputs[0] = Arm_controller_v2_U.Input_1;

    /* '<S3>:1:63' */
    rtb_portoutput = 7;
  } else {
    /* reject input and dont do anything */
    /* '<S3>:1:66' */
    rtb_portoutput = 1;
  }

  /* End of MATLAB Function: '<S1>/functionselector' */

  /* Gain: '<S1>/Convert to degrees' incorporates:
   *  Inport: '<Root>/Joint1'
   *  Inport: '<Root>/Joint2'
   *  Inport: '<Root>/Joint3'
   *  Inport: '<Root>/Joint4'
   *  Inport: '<Root>/Joint5'
   *  Inport: '<Root>/Joint6'
   */
  rtb_Converttodegrees[0] = Arm_controller_v2_P.Converttodegrees_Gain *
    Arm_controller_v2_U.Joint1;
  rtb_Converttodegrees[1] = Arm_controller_v2_P.Converttodegrees_Gain *
    Arm_controller_v2_U.Joint2;
  rtb_Converttodegrees[2] = Arm_controller_v2_P.Converttodegrees_Gain *
    Arm_controller_v2_U.Joint3;
  rtb_Converttodegrees[3] = Arm_controller_v2_P.Converttodegrees_Gain *
    Arm_controller_v2_U.Joint4;
  rtb_Converttodegrees[4] = Arm_controller_v2_P.Converttodegrees_Gain *
    Arm_controller_v2_U.Joint5;
  rtb_Converttodegrees[5] = Arm_controller_v2_P.Converttodegrees_Gain *
    Arm_controller_v2_U.Joint6;

  /* MATLAB Function: '<S1>/set_home' */
  /* MATLAB Function 'Arm_controller_v2/set_home': '<S9>:1' */
  /* '<S9>:1:3' */
  if (rtb_enableflags[2] == 1) {
    /* '<S9>:1:4' */
    /* '<S9>:1:5' */
    rtb_writehome = 1.0;
  } else {
    /* '<S9>:1:7' */
    rtb_writehome = 2.0;
  }

  for (i = 0; i < 6; i++) {
    /* MultiPortSwitch: '<S1>/Multiport Switch2' incorporates:
     *  MATLAB Function: '<S1>/set_home'
     *  Memory: '<S1>/Memory1'
     */
    if (rtb_writehome == 1.0) {
      wrist_vect_idx_2 = rtb_Converttodegrees[i];
    } else {
      wrist_vect_idx_2 = Arm_controller_v2_DW.Memory1_PreviousInput[i];
    }

    /* DataStoreWrite: '<S1>/Data Store Write' */
    Arm_controller_v2_DW.home_position[i] = wrist_vect_idx_2;

    /* MATLAB Function: '<S1>/move_cam' */
    rtb_jointangles_n[i] = 0.0;

    /* MultiPortSwitch: '<S1>/Multiport Switch2' */
    rtb_MultiportSwitch2[i] = wrist_vect_idx_2;
  }

  /* MATLAB Function: '<S1>/move_cam' */
  rtb_writehome = rtb_outputs[2];

  /* MATLAB Function 'Arm_controller_v2/move_cam': '<S4>:1' */
  /* '<S4>:1:4' */
  /* j1 and j2 will ultimately use the home position of the robot as reference */
  /* j1 = -37.93; %this angle orients the base towards the user */
  /* j2 = abs(-173.74 + 90);  */
  /* '<S4>:1:8' */
  /* this angle orients the base towards the user */
  /* '<S4>:1:9' */
  if (rtb_enableflags[0] == 1) {
    /* '<S4>:1:11' */
    /* max_height = 0.392 + 0.425*cosd(j2); %max possible height of the end effector */
    /* if height > max_height, %catch invalid height */
    /* disp('The specified height is larger than the maximum possible height for this configuration. Using a height of: ') */
    /* disp(max_height) */
    /* statusflag = -2; %out of target height */
    /* height = max_height; */
    /* end */
    if (rtb_outputs[2] == 1.0) {
      /* '<S4>:1:20' */
      /* they want the giraffe position */
      /* '<S4>:1:24' */
      rtb_jointangles_n[0] = 0.0;
      rtb_jointangles_n[1] = -90.0;
      rtb_jointangles_n[2] = 0.0;
      rtb_jointangles_n[3] = rtb_outputs[1] - 90.0;
      rtb_jointangles_n[4] = 90.0 + rtb_outputs[0];
      rtb_jointangles_n[5] = 180.0;

      /* '<S4>:1:25' */
      statusflag_movecam = 1;
    } else {
      /* they want standard height control */
      /* '<S4>:1:28' */
      /* max possible height of the end effector */
      if (rtb_outputs[2] > 0.8105432950301884) {
        /* '<S4>:1:29' */
        /* catch invalid height */
        /* '<S4>:1:30' */
        rtb_writehome = 0.8105432950301884;
      } else {
        if (rtb_outputs[2] < 0.13) {
          /* '<S4>:1:31' */
          /* '<S4>:1:32' */
          rtb_writehome = 0.13;
        }
      }

      /* j3 = 90 - j2 + asind((height - 0.425*cosd(j2))/0.392);  %calculates the elbow joint angle needed to reach the required height. works properly */
      /* j4 = tilt_angle - j2 - j3 + 180; %joint angle to apply to achieve target tilt position. works properly */
      /* j5 = pan_angle; %assumption. good assumption */
      /* j6 = 0; %not important at the moment */
      if (rtb_writehome < 0.41854329503018839) {
        /* '<S4>:1:40' */
        /* '<S4>:1:41' */
        /* '<S4>:1:42' */
        /* '<S4>:1:43' */
        j3 = -(asin((0.41854329503018839 - rtb_writehome) / 0.392) *
               57.295779513082323 + 100.0);
      } else {
        /* '<S4>:1:45' */
        /* '<S4>:1:46' */
        /* '<S4>:1:47' */
        j3 = -((180.0 - asin((rtb_writehome - 0.41854329503018839) / 0.392) *
                57.295779513082323) - 80.0);
      }

      /* with arm pitching forward */
      /* j4 = j2-j3-90-tilt_angle-90; */
      /* '<S4>:1:52' */
      j4 = (((80.0 - j3) - 90.0) + rtb_outputs[1]) - 90.0;

      /* with arm pitching back */
      /* j4 = j2 - j3 - 90 - tilt_angle; */
      /* assumption */
      /* '<S4>:1:57' */
      /* very basic check to see if the robot will collide with itself */
      /* if (j4 > 205)||((180-j3) > 155)||((90+j5) > 140), %found these upper limits by observation */
      /* disp('Movement will cause collision') */
      /* statusflag_movecam = -1; */
      /* return; */
      /* end */
      /* %self collision checking %%%%%% */
      /* '<S4>:1:67' */
      /* '<S4>:1:68' */
      /* UNTITLED Summary of this function goes here */
      /*    Detailed explanation goes here */
      wrist_vect_idx_2 = -0.0;
      Arm_controller_v2_cosd_g(&wrist_vect_idx_2);
      j = -0.0;
      Arm_controller_v2_sind_j(&j);
      p1_idx_0 = -0.0;
      Arm_controller_v2_sind_j(&p1_idx_0);
      p1_idx_1 = -0.0;
      Arm_controller_v2_cosd_g(&p1_idx_1);
      T1[0] = wrist_vect_idx_2;
      T1[4] = 0.0;
      T1[8] = j;
      T1[12] = 0.0;
      T1[1] = p1_idx_0;
      T1[5] = 0.0;
      T1[9] = -p1_idx_1;
      T1[13] = 0.0;
      T1[2] = 0.0;
      T1[6] = 1.0;
      T1[10] = 0.0;
      T1[14] = 0.089;
      T1[3] = 0.0;
      T1[7] = 0.0;
      T1[11] = 0.0;
      T1[15] = 1.0;
      p1_idx_2 = -80.0;
      Arm_controller_v2_cosd_g(&p1_idx_2);
      wrist_vect_idx_0 = -80.0;
      Arm_controller_v2_sind_j(&wrist_vect_idx_0);
      wrist_vect_idx_1 = -80.0;
      Arm_controller_v2_cosd_g(&wrist_vect_idx_1);
      p = -80.0;
      Arm_controller_v2_sind_j(&p);
      q = -80.0;
      Arm_controller_v2_cosd_g(&q);
      r = -80.0;
      Arm_controller_v2_sind_j(&r);
      T2[0] = p1_idx_2;
      T2[4] = -wrist_vect_idx_0;
      T2[8] = 0.0;
      T2[12] = -0.425 * wrist_vect_idx_1;
      T2[1] = p;
      T2[5] = q;
      T2[9] = 0.0;
      T2[13] = -0.425 * r;
      T2[2] = 0.0;
      T2[6] = 0.0;
      T2[10] = 1.0;
      T2[14] = 0.0;
      T2[3] = 0.0;
      T2[7] = 0.0;
      T2[11] = 0.0;
      T2[15] = 1.0;
      s = j3;
      Arm_controller_v2_cosd_g(&s);
      t = j3;
      Arm_controller_v2_sind_j(&t);
      u = j3;
      Arm_controller_v2_cosd_g(&u);
      v = j3;
      Arm_controller_v2_sind_j(&v);
      w = j3;
      Arm_controller_v2_cosd_g(&w);
      x = j3;
      Arm_controller_v2_sind_j(&x);
      T3[0] = s;
      T3[4] = -t;
      T3[8] = 0.0;
      T3[12] = -0.392 * u;
      T3[1] = v;
      T3[5] = w;
      T3[9] = 0.0;
      T3[13] = -0.392 * x;
      T3[2] = 0.0;
      T3[6] = 0.0;
      T3[10] = 1.0;
      T3[14] = 0.0;
      T3[3] = 0.0;
      T3[7] = 0.0;
      T3[11] = 0.0;
      T3[15] = 1.0;
      y = j4;
      Arm_controller_v2_cosd_g(&y);
      ab = j4;
      Arm_controller_v2_sind_j(&ab);
      bb = j4;
      Arm_controller_v2_sind_j(&bb);
      cb = j4;
      Arm_controller_v2_cosd_g(&cb);
      T4[0] = y;
      T4[4] = 0.0;
      T4[8] = ab;
      T4[12] = 0.0;
      T4[1] = bb;
      T4[5] = 0.0;
      T4[9] = -cb;
      T4[13] = 0.0;
      T4[2] = 0.0;
      T4[6] = 1.0;
      T4[10] = 0.0;
      T4[14] = 0.109;
      T4[3] = 0.0;
      T4[7] = 0.0;
      T4[11] = 0.0;
      T4[15] = 1.0;
      db = 90.0 + rtb_outputs[0];
      Arm_controller_v2_cosd_g(&db);
      eb = 90.0 + rtb_outputs[0];
      Arm_controller_v2_sind_j(&eb);
      fb = 90.0 + rtb_outputs[0];
      Arm_controller_v2_sind_j(&fb);
      gb = 90.0 + rtb_outputs[0];
      Arm_controller_v2_cosd_g(&gb);
      T5[0] = db;
      T5[4] = 0.0;
      T5[8] = -eb;
      T5[12] = 0.0;
      T5[1] = fb;
      T5[5] = 0.0;
      T5[9] = gb;
      T5[13] = 0.0;
      T5[2] = 0.0;
      T5[6] = -1.0;
      T5[10] = 0.0;
      T5[14] = 0.095;
      T5[3] = 0.0;
      T5[7] = 0.0;
      T5[11] = 0.0;
      T5[15] = 1.0;
      hb = 180.0;
      Arm_controller_v2_cosd_g(&hb);
      ib = 180.0;
      Arm_controller_v2_sind_j(&ib);
      jb = 180.0;
      Arm_controller_v2_sind_j(&jb);
      rtb_writehome = 180.0;
      Arm_controller_v2_cosd_g(&rtb_writehome);
      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_0[current_config + (i << 2)] = 0.0;
          T1_0[current_config + (i << 2)] += T2[i << 2] * T1[current_config];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 1] *
            T1[current_config + 4];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 2] *
            T1[current_config + 8];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 3] *
            T1[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_1[current_config + (i << 2)] = 0.0;
          T1_1[current_config + (i << 2)] += T3[i << 2] * T1_0[current_config];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 1] *
            T1_0[current_config + 4];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 2] *
            T1_0[current_config + 8];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 3] *
            T1_0[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_0[current_config + (i << 2)] = 0.0;
          T1_0[current_config + (i << 2)] += T4[i << 2] * T1_1[current_config];
          T1_0[current_config + (i << 2)] += T4[(i << 2) + 1] *
            T1_1[current_config + 4];
          T1_0[current_config + (i << 2)] += T4[(i << 2) + 2] *
            T1_1[current_config + 8];
          T1_0[current_config + (i << 2)] += T4[(i << 2) + 3] *
            T1_1[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_1[current_config + (i << 2)] = 0.0;
          T1_1[current_config + (i << 2)] += T5[i << 2] * T1_0[current_config];
          T1_1[current_config + (i << 2)] += T5[(i << 2) + 1] *
            T1_0[current_config + 4];
          T1_1[current_config + (i << 2)] += T5[(i << 2) + 2] *
            T1_0[current_config + 8];
          T1_1[current_config + (i << 2)] += T5[(i << 2) + 3] *
            T1_0[current_config + 12];
        }
      }

      hb_0[0] = hb;
      hb_0[4] = -ib;
      hb_0[8] = 0.0;
      hb_0[12] = 0.0;
      hb_0[1] = jb;
      hb_0[5] = rtb_writehome;
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
      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          total_transform[current_config + (i << 2)] = 0.0;
          total_transform[current_config + (i << 2)] += hb_0[i << 2] *
            T1_1[current_config];
          total_transform[current_config + (i << 2)] += hb_0[(i << 2) + 1] *
            T1_1[current_config + 4];
          total_transform[current_config + (i << 2)] += hb_0[(i << 2) + 2] *
            T1_1[current_config + 8];
          total_transform[current_config + (i << 2)] += hb_0[(i << 2) + 3] *
            T1_1[current_config + 12];
        }
      }

      /* arm_orig = [0;0;0;1]; */
      for (current_config = 0; current_config < 4; current_config++) {
        wrist_vect_idx_2 = T1[current_config + 12] + (T1[current_config + 8] *
          0.0 + (T1[current_config + 4] * 0.0 + T1[current_config] * 0.0));
        shoulder_start[current_config] = wrist_vect_idx_2;
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_0[current_config + (i << 2)] = 0.0;
          T1_0[current_config + (i << 2)] += T2[i << 2] * T1[current_config];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 1] *
            T1[current_config + 4];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 2] *
            T1[current_config + 8];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 3] *
            T1[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        wrist_vect_idx_2 = T1_0[current_config + 12] + (T1_0[current_config + 8]
          * 0.0 + (T1_0[current_config + 4] * 0.0 + T1_0[current_config] * 0.0));
        shoulder_orig[current_config] = wrist_vect_idx_2;
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_0[current_config + (i << 2)] = 0.0;
          T1_0[current_config + (i << 2)] += T2[i << 2] * T1[current_config];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 1] *
            T1[current_config + 4];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 2] *
            T1[current_config + 8];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 3] *
            T1[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_1[current_config + (i << 2)] = 0.0;
          T1_1[current_config + (i << 2)] += T3[i << 2] * T1_0[current_config];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 1] *
            T1_0[current_config + 4];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 2] *
            T1_0[current_config + 8];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 3] *
            T1_0[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        wrist_vect_idx_2 = T1_1[current_config + 12] + (T1_1[current_config + 8]
          * 0.0 + (T1_1[current_config + 4] * 0.0 + T1_1[current_config] * 0.0));
        elbow_orig[current_config] = wrist_vect_idx_2;
      }

      /* wrist_1 = T1*T2*T3*T4*[0;0;0;1]; */
      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_0[current_config + (i << 2)] = 0.0;
          T1_0[current_config + (i << 2)] += T2[i << 2] * T1[current_config];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 1] *
            T1[current_config + 4];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 2] *
            T1[current_config + 8];
          T1_0[current_config + (i << 2)] += T2[(i << 2) + 3] *
            T1[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_1[current_config + (i << 2)] = 0.0;
          T1_1[current_config + (i << 2)] += T3[i << 2] * T1_0[current_config];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 1] *
            T1_0[current_config + 4];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 2] *
            T1_0[current_config + 8];
          T1_1[current_config + (i << 2)] += T3[(i << 2) + 3] *
            T1_0[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_0[current_config + (i << 2)] = 0.0;
          T1_0[current_config + (i << 2)] += T4[i << 2] * T1_1[current_config];
          T1_0[current_config + (i << 2)] += T4[(i << 2) + 1] *
            T1_1[current_config + 4];
          T1_0[current_config + (i << 2)] += T4[(i << 2) + 2] *
            T1_1[current_config + 8];
          T1_0[current_config + (i << 2)] += T4[(i << 2) + 3] *
            T1_1[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        for (i = 0; i < 4; i++) {
          T1_1[current_config + (i << 2)] = 0.0;
          T1_1[current_config + (i << 2)] += T5[i << 2] * T1_0[current_config];
          T1_1[current_config + (i << 2)] += T5[(i << 2) + 1] *
            T1_0[current_config + 4];
          T1_1[current_config + (i << 2)] += T5[(i << 2) + 2] *
            T1_0[current_config + 8];
          T1_1[current_config + (i << 2)] += T5[(i << 2) + 3] *
            T1_0[current_config + 12];
        }
      }

      for (current_config = 0; current_config < 4; current_config++) {
        j = T1_1[current_config + 12] + (T1_1[current_config + 8] * 0.0 +
          (T1_1[current_config + 4] * 0.0 + T1_1[current_config] * 0.0));
        wrist_2[current_config] = j;
      }

      for (current_config = 0; current_config < 4; current_config++) {
        wrist_vect_idx_2 = total_transform[current_config + 12] +
          (total_transform[current_config + 8] * 0.0 +
           (total_transform[current_config + 4] * 0.0 +
            total_transform[current_config] * 0.0));
        end_effector_orig[current_config] = wrist_vect_idx_2;
      }

      /* find distance between end effector and elbow */
      /* end effector */
      p1_idx_0 = wrist_2[0];
      p1_idx_1 = wrist_2[1];
      p1_idx_2 = wrist_2[2];

      /* wrist */
      p2_0[0] = end_effector_orig[0] - wrist_2[0];
      p2_0[1] = end_effector_orig[1] - wrist_2[1];
      p2_0[2] = end_effector_orig[2] - wrist_2[2];
      rtb_writehome = Arm_controller_v2_norm_g(p2_0);
      wrist_vect_idx_0 = (end_effector_orig[0] - wrist_2[0]) / rtb_writehome;
      wrist_vect_idx_1 = (end_effector_orig[1] - wrist_2[1]) / rtb_writehome;
      wrist_vect_idx_2 = (end_effector_orig[2] - wrist_2[2]) / rtb_writehome;

      /* normalised vector */
      wrist_back[0] = wrist_2[0] - 0.06 * wrist_vect_idx_0;
      wrist_back[1] = wrist_2[1] - 0.06 * wrist_vect_idx_1;
      wrist_back[2] = wrist_2[2] - 0.06 * wrist_vect_idx_2;

      /* same line as */
      /* point representing camera */
      /* point representing end of wire connector */
      /* top of camera mount */
      /* p2offset = total_transform*[0;-0.072;0;1];%if end effector is rotated by 0 */
      for (current_config = 0; current_config < 4; current_config++) {
        j = total_transform[current_config + 12] +
          (total_transform[current_config + 8] * 0.0 +
           (total_transform[current_config + 4] * 0.072 +
            total_transform[current_config] * 0.0));
        wrist_2[current_config] = j;
      }

      /* if end effector is rotated by 180 */
      p3offset[0] = 0.09 * wrist_vect_idx_0 + wrist_2[0];
      p3offset[1] = 0.09 * wrist_vect_idx_1 + wrist_2[1];
      p3offset[2] = 0.09 * wrist_vect_idx_2 + wrist_2[2];
      p1offset[0] = wrist_2[0] - 0.2 * wrist_vect_idx_0;
      p1offset[1] = wrist_2[1] - 0.2 * wrist_vect_idx_1;
      p1offset[2] = wrist_2[2] - 0.2 * wrist_vect_idx_2;

      /* end of wire */
      pwirestart[0] = wrist_2[0] - 0.082 * wrist_vect_idx_0;
      pwirestart[1] = wrist_2[1] - 0.082 * wrist_vect_idx_1;
      pwirestart[2] = wrist_2[2] - 0.082 * wrist_vect_idx_2;

      /* start of wire */
      /* collision with shoulder. Need to offset points by a certain point */
      /* shoulder CS */
      /* end of shoulder */
      /* start of shoulder */
      /* actual shouler */
      for (current_config = 0; current_config < 4; current_config++) {
        j = T1[current_config + 12] + (T1[current_config + 8] * 0.135 +
          (T1[current_config + 4] * 0.0 + T1[current_config] * 0.0));
        wrist_2[current_config] = j;
      }

      /* bottom of shoulder */
      /* top of shoulder */
      /* checking self collision */
      /* if (dist_mount < 0.095 || dist_wire < 0.054 || dist_shoulder < 0.09) */
      elbow_orig_0[0] = elbow_orig[0];
      elbow_orig_0[1] = elbow_orig[1];
      elbow_orig_0[2] = elbow_orig[2];
      shoulder_orig_0[0] = shoulder_orig[0];
      shoulder_orig_0[1] = shoulder_orig[1];
      shoulder_orig_0[2] = shoulder_orig[2];
      p2[0] = 0.09 * wrist_vect_idx_0 + end_effector_orig[0];
      p2[1] = 0.09 * wrist_vect_idx_1 + end_effector_orig[1];
      p2[2] = 0.09 * wrist_vect_idx_2 + end_effector_orig[2];
      elbow_orig_1[0] = elbow_orig[0];
      elbow_orig_1[1] = elbow_orig[1];
      elbow_orig_1[2] = elbow_orig[2];
      shoulder_orig_1[0] = shoulder_orig[0];
      shoulder_orig_1[1] = shoulder_orig[1];
      shoulder_orig_1[2] = shoulder_orig[2];
      p1[0] = p1_idx_0 - 0.114 * wrist_vect_idx_0;
      p1[1] = p1_idx_1 - 0.114 * wrist_vect_idx_1;
      p1[2] = p1_idx_2 - 0.114 * wrist_vect_idx_2;
      elbow_orig_2[0] = elbow_orig[0];
      elbow_orig_2[1] = elbow_orig[1];
      elbow_orig_2[2] = elbow_orig[2];
      shoulder_orig_2[0] = shoulder_orig[0];
      shoulder_orig_2[1] = shoulder_orig[1];
      shoulder_orig_2[2] = shoulder_orig[2];
      elbow_orig_3[0] = elbow_orig[0];
      elbow_orig_3[1] = elbow_orig[1];
      elbow_orig_3[2] = elbow_orig[2];
      shoulder_orig_3[0] = shoulder_orig[0];
      shoulder_orig_3[1] = shoulder_orig[1];
      shoulder_orig_3[2] = shoulder_orig[2];
      wrist_2_0[0] = (shoulder_orig[0] - shoulder_start[0]) + wrist_2[0];
      wrist_2_0[1] = (shoulder_orig[1] - shoulder_start[1]) + wrist_2[1];
      wrist_2_0[2] = (shoulder_orig[2] - shoulder_start[2]) + wrist_2[2];
      wrist_2_1[0] = wrist_2[0];
      wrist_2_1[1] = wrist_2[1];
      wrist_2_1[2] = wrist_2[2];
      if ((Arm_contr_DistBetween2Segment_m(p3offset, pwirestart, elbow_orig_0,
            shoulder_orig_0) < 0.1) || (Arm_contr_DistBetween2Segment_m(p2,
            wrist_back, elbow_orig_1, shoulder_orig_1) < 0.1) ||
          (Arm_contr_DistBetween2Segment_m(wrist_back, p1, elbow_orig_2,
            shoulder_orig_2) < 0.054) || (Arm_contr_DistBetween2Segment_m
           (pwirestart, p1offset, elbow_orig_3, shoulder_orig_3) < 0.054) ||
          (Arm_contr_DistBetween2Segment_m(p3offset, p1offset, wrist_2_0,
            wrist_2_1) < 0.09)) {
        collision_flag = 1;

        /* there is a collision */
      } else {
        collision_flag = 0;

        /* there is no collision */
      }

      /* checking self collision */
      /* if (dist_mount < 0.095 || dist_wire < 0.054 || dist_shoulder < 0.09) */
      if (collision_flag == 1) {
        /* '<S4>:1:72' */
        /* '<S4>:1:73' */
        statusflag_movecam = -1;
      } else {
        /* '<S4>:1:76' */
        rtb_jointangles_n[0] = -0.0;
        rtb_jointangles_n[1] = -80.0;
        rtb_jointangles_n[2] = j3;
        rtb_jointangles_n[3] = j4;
        rtb_jointangles_n[4] = 90.0 + rtb_outputs[0];
        rtb_jointangles_n[5] = 180.0;

        /* '<S4>:1:77' */
        statusflag_movecam = 1;

        /* %%%%%%% */
        /* jointangles = [-37.93 -173.74 180-j3 -j4 90+j5 0]; %adjusts the calculated joint angles so it matches the current robot configuration */
      }
    }
  } else {
    /* '<S4>:1:90' */
    statusflag_movecam = 0;

    /* no movement carried out */
    /* '<S4>:1:91' */
    /* interpreted as doing nothing */
  }

  /* MATLAB Function: '<S1>/setpos' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read3'
   */
  /* MATLAB Function 'Arm_controller_v2/setpos': '<S11>:1' */
  if (rtb_enableflags[1] == 1) {
    /* '<S11>:1:5' */
    if (rtb_outputs[0] == 1.0) {
      /* '<S11>:1:6' */
      /* replace 'home' with a integer */
      /* move to the defined home position */
      /* '<S11>:1:8' */
      rtb_jointangles[0] = 57.295779513082323 *
        Arm_controller_v2_DW.home_position[0];
      rtb_jointangles[1] = 57.295779513082323 *
        Arm_controller_v2_DW.home_position[1];
      rtb_jointangles[2] = 57.295779513082323 *
        Arm_controller_v2_DW.home_position[2];
      rtb_jointangles[3] = 57.295779513082323 *
        Arm_controller_v2_DW.home_position[3];
      rtb_jointangles[4] = 57.295779513082323 *
        Arm_controller_v2_DW.home_position[4];
      rtb_jointangles[5] = 57.295779513082323 *
        Arm_controller_v2_DW.home_position[5];

      /* jointangles = [0 0 0 0 0 0]; %need to read some sort of internal memory */
      /* '<S11>:1:10' */
      rtb_statusflag_setpos = 1;
    } else if (rtb_outputs[0] == 2.0) {
      /* '<S11>:1:11' */
      /* move to giraffe position- replace with integer */
      /* '<S11>:1:12' */
      for (current_config = 0; current_config < 6; current_config++) {
        rtb_jointangles[current_config] = b[current_config];
      }

      /* '<S11>:1:13' */
      rtb_statusflag_setpos = 1;
    } else {
      /* output error message */
      /* '<S11>:1:16' */
      rtb_statusflag_setpos = -3;

      /* invalid argument */
      /* '<S11>:1:17' */
      for (i = 0; i < 6; i++) {
        rtb_jointangles[i] = 0.0;
      }

      /* interpreted at do nothing */
    }
  } else {
    /* '<S11>:1:20' */
    rtb_statusflag_setpos = 0;

    /* function not executed */
    /* '<S11>:1:21' */
    for (i = 0; i < 6; i++) {
      rtb_jointangles[i] = 0.0;
    }
  }

  /* End of MATLAB Function: '<S1>/setpos' */

  /* MATLAB Function: '<S1>/move_joints' */
  /* MATLAB Function 'Arm_controller_v2/move_joints': '<S7>:1' */
  /* need to implement some collision checking */
  /* '<S7>:1:4' */
  for (i = 0; i < 6; i++) {
    rtb_jointangles_l[i] = 0.0;
  }

  if (rtb_enableflags[3] == 1) {
    /* '<S7>:1:6' */
    /* check collisions */
    /* '<S7>:1:8' */
    rtb_statusflag_movej = 1;

    /* '<S7>:1:9' */
    rtb_jointangles_l[0] = rtb_outputs[0];

    /* '<S7>:1:10' */
    rtb_jointangles_l[1] = rtb_outputs[1];

    /* '<S7>:1:11' */
    rtb_jointangles_l[2] = rtb_outputs[2];

    /* '<S7>:1:12' */
    rtb_jointangles_l[3] = rtb_outputs[3];

    /* '<S7>:1:13' */
    rtb_jointangles_l[4] = rtb_outputs[4];

    /* '<S7>:1:14' */
    rtb_jointangles_l[5] = rtb_outputs[5];
  } else {
    /* '<S7>:1:16' */
    rtb_statusflag_movej = 0;
  }

  /* End of MATLAB Function: '<S1>/move_joints' */

  /* MATLAB Function: '<S1>/MATLAB Function' */
  /* MATLAB Function 'Arm_controller_v2/MATLAB Function': '<S2>:1' */
  /* #calculates joint angles needed to scan] */
  /* '<S2>:1:3' */
  /* '<S2>:1:4' */
  /* '<S2>:1:5' */
  for (i = 0; i < 6; i++) {
    rtb_scan_startpos[i] = 0.0;
    rtb_scan_bottompos[i] = 0.0;
    rtb_scan_toppos[i] = 0.0;
  }

  /* output_num = 0; */
  if (rtb_enableflags[4] == 1) {
    /* '<S2>:1:9' */
    /* '<S2>:1:10' */
    joint_angles_degrees[0] = rtb_Converttodegrees[0];
    joint_angles_degrees[1] = rtb_Converttodegrees[1];
    joint_angles_degrees[2] = rtb_Converttodegrees[2];
    joint_angles_degrees[3] = rtb_Converttodegrees[3];
    joint_angles_degrees[4] = rtb_Converttodegrees[4];
    joint_angles_degrees[5] = 180.0;

    /* '<S2>:1:11' */
    scan_bottompos1[0] = rtb_Converttodegrees[0];
    scan_bottompos1[1] = rtb_Converttodegrees[1];
    scan_bottompos1[2] = rtb_Converttodegrees[2];
    scan_bottompos1[3] = rtb_Converttodegrees[3] - 0.5 * rtb_outputs[0];
    scan_bottompos1[4] = rtb_Converttodegrees[4];
    scan_bottompos1[5] = 180.0;

    /* '<S2>:1:12' */
    scan_toppos1[0] = rtb_Converttodegrees[0];
    scan_toppos1[1] = rtb_Converttodegrees[1];
    scan_toppos1[2] = rtb_Converttodegrees[2];
    scan_toppos1[3] = 0.5 * rtb_outputs[0] + rtb_Converttodegrees[3];
    scan_toppos1[4] = rtb_Converttodegrees[4];
    scan_toppos1[5] = 180.0;

    /* '<S2>:1:14' */
    rtb_writehome = Arm_contro_check_self_collision(joint_angles_degrees);

    /* '<S2>:1:15' */
    j3 = Arm_contro_check_self_collision(scan_bottompos1);

    /* '<S2>:1:16' */
    j4 = Arm_contro_check_self_collision(scan_toppos1);
    if ((rtb_writehome != 0.0) || (j3 != 0.0) || (j4 != 0.0)) {
      /* '<S2>:1:19' */
      status_scan = -1;
    } else {
      /* '<S2>:1:21' */
      status_scan = 1;

      /* output_num = 3; */
      /* '<S2>:1:23' */
      /* '<S2>:1:24' */
      /* '<S2>:1:25' */
      for (current_config = 0; current_config < 6; current_config++) {
        rtb_scan_startpos[current_config] = joint_angles_degrees[current_config];
        rtb_scan_bottompos[current_config] = scan_bottompos1[current_config];
        rtb_scan_toppos[current_config] = scan_toppos1[current_config];
      }
    }
  } else {
    /* '<S2>:1:28' */
    status_scan = 0;
  }

  /* MATLAB Function: '<S1>/move_cam2' */
  /* MATLAB Function 'Arm_controller_v2/move_cam2': '<S5>:1' */
  /* moves the end effector of the arm by a specific pan and tilt angle */
  /* for now the pan and tilt angle are relative to the direction the rover is */
  /* facing */
  /* positive tilt = tilt up */
  /* positive pan = pan left */
  /* '<S5>:1:8' */
  status_movecam2 = 0;

  /* '<S5>:1:9' */
  for (i = 0; i < 6; i++) {
    rtb_jointangles_a[i] = 0.0;
  }

  if (rtb_enableflags[5] == 1) {
    /* '<S5>:1:10' */
    /* UNTITLED Summary of this function goes here */
    /*    Detailed explanation goes here */
    rtb_writehome = rtb_Converttodegrees[0];
    Arm_controller_v2_cosd_b(&rtb_writehome);
    wrist_vect_idx_2 = rtb_Converttodegrees[0];
    Arm_controller_v2_sind_c(&wrist_vect_idx_2);
    j = rtb_Converttodegrees[0];
    Arm_controller_v2_sind_c(&j);
    p1_idx_0 = rtb_Converttodegrees[0];
    Arm_controller_v2_cosd_b(&p1_idx_0);
    p1_idx_1 = rtb_Converttodegrees[1];
    Arm_controller_v2_cosd_b(&p1_idx_1);
    p1_idx_2 = rtb_Converttodegrees[1];
    Arm_controller_v2_sind_c(&p1_idx_2);
    wrist_vect_idx_0 = rtb_Converttodegrees[1];
    Arm_controller_v2_cosd_b(&wrist_vect_idx_0);
    wrist_vect_idx_1 = rtb_Converttodegrees[1];
    Arm_controller_v2_sind_c(&wrist_vect_idx_1);
    p = rtb_Converttodegrees[1];
    Arm_controller_v2_cosd_b(&p);
    q = rtb_Converttodegrees[1];
    Arm_controller_v2_sind_c(&q);
    r = rtb_Converttodegrees[2];
    Arm_controller_v2_cosd_b(&r);
    s = rtb_Converttodegrees[2];
    Arm_controller_v2_sind_c(&s);
    t = rtb_Converttodegrees[2];
    Arm_controller_v2_cosd_b(&t);
    u = rtb_Converttodegrees[2];
    Arm_controller_v2_sind_c(&u);
    v = rtb_Converttodegrees[2];
    Arm_controller_v2_cosd_b(&v);
    w = rtb_Converttodegrees[2];
    Arm_controller_v2_sind_c(&w);
    x = rtb_Converttodegrees[3];
    Arm_controller_v2_cosd_b(&x);
    y = rtb_Converttodegrees[3];
    Arm_controller_v2_sind_c(&y);
    ab = rtb_Converttodegrees[3];
    Arm_controller_v2_sind_c(&ab);
    bb = rtb_Converttodegrees[3];
    Arm_controller_v2_cosd_b(&bb);
    cb = rtb_Converttodegrees[4];
    Arm_controller_v2_cosd_b(&cb);
    db = rtb_Converttodegrees[4];
    Arm_controller_v2_sind_c(&db);
    eb = rtb_Converttodegrees[4];
    Arm_controller_v2_sind_c(&eb);
    fb = rtb_Converttodegrees[4];
    Arm_controller_v2_cosd_b(&fb);
    gb = rtb_Converttodegrees[5];
    Arm_controller_v2_cosd_b(&gb);
    hb = rtb_Converttodegrees[5];
    Arm_controller_v2_sind_c(&hb);
    ib = rtb_Converttodegrees[5];
    Arm_controller_v2_sind_c(&ib);
    jb = rtb_Converttodegrees[5];
    Arm_controller_v2_cosd_b(&jb);
    rtb_writehome_0[0] = rtb_writehome;
    rtb_writehome_0[4] = 0.0;
    rtb_writehome_0[8] = wrist_vect_idx_2;
    rtb_writehome_0[12] = 0.0;
    rtb_writehome_0[1] = j;
    rtb_writehome_0[5] = 0.0;
    rtb_writehome_0[9] = -p1_idx_0;
    rtb_writehome_0[13] = 0.0;
    rtb_writehome_0[2] = 0.0;
    rtb_writehome_0[6] = 1.0;
    rtb_writehome_0[10] = 0.0;
    rtb_writehome_0[14] = 0.089;
    rtb_writehome_0[3] = 0.0;
    rtb_writehome_0[7] = 0.0;
    rtb_writehome_0[11] = 0.0;
    rtb_writehome_0[15] = 1.0;
    l[0] = p1_idx_1;
    l[4] = -p1_idx_2;
    l[8] = 0.0;
    l[12] = -0.425 * wrist_vect_idx_0;
    l[1] = wrist_vect_idx_1;
    l[5] = p;
    l[9] = 0.0;
    l[13] = -0.425 * q;
    l[2] = 0.0;
    l[6] = 0.0;
    l[10] = 1.0;
    l[14] = 0.0;
    l[3] = 0.0;
    l[7] = 0.0;
    l[11] = 0.0;
    l[15] = 1.0;
    for (current_config = 0; current_config < 4; current_config++) {
      for (i = 0; i < 4; i++) {
        T1[current_config + (i << 2)] = 0.0;
        T1[current_config + (i << 2)] += l[i << 2] *
          rtb_writehome_0[current_config];
        T1[current_config + (i << 2)] += l[(i << 2) + 1] *
          rtb_writehome_0[current_config + 4];
        T1[current_config + (i << 2)] += l[(i << 2) + 2] *
          rtb_writehome_0[current_config + 8];
        T1[current_config + (i << 2)] += l[(i << 2) + 3] *
          rtb_writehome_0[current_config + 12];
      }
    }

    r_0[0] = r;
    r_0[4] = -s;
    r_0[8] = 0.0;
    r_0[12] = -0.392 * t;
    r_0[1] = u;
    r_0[5] = v;
    r_0[9] = 0.0;
    r_0[13] = -0.392 * w;
    r_0[2] = 0.0;
    r_0[6] = 0.0;
    r_0[10] = 1.0;
    r_0[14] = 0.0;
    r_0[3] = 0.0;
    r_0[7] = 0.0;
    r_0[11] = 0.0;
    r_0[15] = 1.0;
    for (current_config = 0; current_config < 4; current_config++) {
      for (i = 0; i < 4; i++) {
        rtb_writehome_0[current_config + (i << 2)] = 0.0;
        rtb_writehome_0[current_config + (i << 2)] += r_0[i << 2] *
          T1[current_config];
        rtb_writehome_0[current_config + (i << 2)] += r_0[(i << 2) + 1] *
          T1[current_config + 4];
        rtb_writehome_0[current_config + (i << 2)] += r_0[(i << 2) + 2] *
          T1[current_config + 8];
        rtb_writehome_0[current_config + (i << 2)] += r_0[(i << 2) + 3] *
          T1[current_config + 12];
      }
    }

    x_0[0] = x;
    x_0[4] = 0.0;
    x_0[8] = y;
    x_0[12] = 0.0;
    x_0[1] = ab;
    x_0[5] = 0.0;
    x_0[9] = -bb;
    x_0[13] = 0.0;
    x_0[2] = 0.0;
    x_0[6] = 1.0;
    x_0[10] = 0.0;
    x_0[14] = 0.109;
    x_0[3] = 0.0;
    x_0[7] = 0.0;
    x_0[11] = 0.0;
    x_0[15] = 1.0;
    for (current_config = 0; current_config < 4; current_config++) {
      for (i = 0; i < 4; i++) {
        T1[current_config + (i << 2)] = 0.0;
        T1[current_config + (i << 2)] += x_0[i << 2] *
          rtb_writehome_0[current_config];
        T1[current_config + (i << 2)] += x_0[(i << 2) + 1] *
          rtb_writehome_0[current_config + 4];
        T1[current_config + (i << 2)] += x_0[(i << 2) + 2] *
          rtb_writehome_0[current_config + 8];
        T1[current_config + (i << 2)] += x_0[(i << 2) + 3] *
          rtb_writehome_0[current_config + 12];
      }
    }

    cb_0[0] = cb;
    cb_0[4] = 0.0;
    cb_0[8] = -db;
    cb_0[12] = 0.0;
    cb_0[1] = eb;
    cb_0[5] = 0.0;
    cb_0[9] = fb;
    cb_0[13] = 0.0;
    cb_0[2] = 0.0;
    cb_0[6] = -1.0;
    cb_0[10] = 0.0;
    cb_0[14] = 0.095;
    cb_0[3] = 0.0;
    cb_0[7] = 0.0;
    cb_0[11] = 0.0;
    cb_0[15] = 1.0;
    for (current_config = 0; current_config < 4; current_config++) {
      for (i = 0; i < 4; i++) {
        rtb_writehome_0[current_config + (i << 2)] = 0.0;
        rtb_writehome_0[current_config + (i << 2)] += cb_0[i << 2] *
          T1[current_config];
        rtb_writehome_0[current_config + (i << 2)] += cb_0[(i << 2) + 1] *
          T1[current_config + 4];
        rtb_writehome_0[current_config + (i << 2)] += cb_0[(i << 2) + 2] *
          T1[current_config + 8];
        rtb_writehome_0[current_config + (i << 2)] += cb_0[(i << 2) + 3] *
          T1[current_config + 12];
      }
    }

    gb_0[0] = gb;
    gb_0[4] = -hb;
    gb_0[8] = 0.0;
    gb_0[12] = 0.0;
    gb_0[1] = ib;
    gb_0[5] = jb;
    gb_0[9] = 0.0;
    gb_0[13] = 0.0;
    gb_0[2] = 0.0;
    gb_0[6] = 0.0;
    gb_0[10] = 1.0;
    gb_0[14] = 0.082;
    gb_0[3] = 0.0;
    gb_0[7] = 0.0;
    gb_0[11] = 0.0;
    gb_0[15] = 1.0;
    for (current_config = 0; current_config < 4; current_config++) {
      for (i = 0; i < 4; i++) {
        total_transform[current_config + (i << 2)] = 0.0;
        total_transform[current_config + (i << 2)] += gb_0[i << 2] *
          rtb_writehome_0[current_config];
        total_transform[current_config + (i << 2)] += gb_0[(i << 2) + 1] *
          rtb_writehome_0[current_config + 4];
        total_transform[current_config + (i << 2)] += gb_0[(i << 2) + 2] *
          rtb_writehome_0[current_config + 8];
        total_transform[current_config + (i << 2)] += gb_0[(i << 2) + 3] *
          rtb_writehome_0[current_config + 12];
      }
    }

    /* laser_offset = 0.072; %negative = upwards */
    /* co-ordinate system rotational transformation */
    /* '<S5>:1:17' */
    rtb_writehome = rtb_outputs[0] - 57.295779513082323 * asin(total_transform[9]);

    /* positive = pan more left */
    /* '<S5>:1:18' */
    j3 = rtb_outputs[1] - atan(total_transform[10] / total_transform[8]) *
      57.295779513082323;

    /* positive = tilt more up */
    /* %need to test */
    if (rtb_Converttodegrees[0] > 90.0) {
      /* '<S5>:1:22' */
      /* jointangles_try = [current_position(1), current_position(2), current_position(3), current_position(4) - tilt_diff, pan_diff - 90, 180]; %need to consider that arm might be rotated */
      /* '<S5>:1:24' */
      joint_angles_degrees[0] = rtb_Converttodegrees[0];
      joint_angles_degrees[1] = rtb_Converttodegrees[1];
      joint_angles_degrees[2] = rtb_Converttodegrees[2];
      joint_angles_degrees[3] = rtb_Converttodegrees[3] - j3;
      joint_angles_degrees[4] = rtb_Converttodegrees[4] + rtb_writehome;
      joint_angles_degrees[5] = 180.0;
    } else {
      /* jointangles_try = [current_position(1), current_position(2), current_position(3), current_position(4) + tilt_diff, pan_diff + 90, 180]; %check pan angle */
      /* '<S5>:1:27' */
      joint_angles_degrees[0] = rtb_Converttodegrees[0];
      joint_angles_degrees[1] = rtb_Converttodegrees[1];
      joint_angles_degrees[2] = rtb_Converttodegrees[2];
      joint_angles_degrees[3] = rtb_Converttodegrees[3] + j3;
      joint_angles_degrees[4] = rtb_Converttodegrees[4] + rtb_writehome;
      joint_angles_degrees[5] = 180.0;
    }

    /* '<S5>:1:30' */
    rtb_writehome = Arm_controller__check_collision(joint_angles_degrees);

    /* check_col = check_collision(jointangles_try); */
    /* upgrade model to include obstacles as well */
    if (rtb_writehome == 0.0) {
      /* '<S5>:1:34' */
      /* no collision collision */
      /* '<S5>:1:35' */
      status_movecam2 = 1;

      /* '<S5>:1:36' */
      for (current_config = 0; current_config < 6; current_config++) {
        rtb_jointangles_a[current_config] = joint_angles_degrees[current_config];
      }
    } else {
      /* '<S5>:1:38' */
      status_movecam2 = -1;
    }
  }

  /* MATLAB Function: '<S1>/move_effector' */
  /* MATLAB Function 'Arm_controller_v2/move_effector': '<S6>:1' */
  /* UNTITLED Summary of this function goes here */
  /*    default values */
  /* '<S6>:1:4' */
  /* '<S6>:1:5' */
  /* '<S6>:1:6' */
  /* '<S6>:1:7' */
  /* '<S6>:1:8' */
  for (i = 0; i < 6; i++) {
    rtb_wp1_k[i] = 0.0;
    rtb_wp2_o[i] = 0.0;
    rtb_wp3_cw[i] = 0.0;
    rtb_wp4_m2[i] = 0.0;
    rtb_wp5_m[i] = 0.0;
  }

  /* wp6 = [0 0 0 0 0 0]; */
  /* '<S6>:1:10' */
  waypoint_number = 0;

  /* '<S6>:1:11' */
  status_flag = 0;
  if (rtb_enableflags[6] == 1) {
    /* '<S6>:1:13' */
    /* calculates up to 8 IK solutions */
    Arm_controller_v2_getIK(rtb_outputs[0], rtb_outputs[1], rtb_outputs[2],
      &wrist_vect_idx_2, q_sols);

    /* working well */
    /* need to sort through the eight solutions to choose best one */
    /* go through each line one by one up to numsols */
    /* check range of j1- 0 to 180 */
    /* make sure wrist is upwards */
    /* use collision checker to check if laser doesnt collide with elbow or */
    /* shouler */
    if (wrist_vect_idx_2 == 0.0) {
      /* '<S6>:1:26' */
      /* %if no solutions found then exit function */
      /* '<S6>:1:27' */
      status_flag = -1;
    } else {
      /* '<S6>:1:32' */
      for (current_config = 0; current_config < 48; current_config++) {
        q_sols[current_config] *= 57.295779513082323;
      }

      /* convert to degrees */
      /* checksol = zeros(num_sols,1); */
      /* '<S6>:1:34' */
      for (current_config = 0; current_config < 8; current_config++) {
        checksol[current_config] = 1;
      }

      /* assumes all configurations cause collisions */
      /* convert 0 to 360 to -180 to 180 degrees */
      /* '<S6>:1:37' */
      for (collision_flag = 0; collision_flag < (int32_T)wrist_vect_idx_2;
           collision_flag++) {
        /* '<S6>:1:37' */
        /* '<S6>:1:38' */
        for (current_config = 0; current_config < 6; current_config++) {
          /* '<S6>:1:38' */
          if (q_sols[(current_config << 3) + collision_flag] > 180.0) {
            /* '<S6>:1:39' */
            /* '<S6>:1:40' */
            q_sols[collision_flag + (current_config << 3)] -= 360.0;
          }

          /* '<S6>:1:38' */
        }

        /* '<S6>:1:37' */
      }

      /* '<S6>:1:45' */
      for (collision_flag = 0; collision_flag < (int32_T)wrist_vect_idx_2;
           collision_flag++) {
        /* '<S6>:1:45' */
        /* '<S6>:1:46' */
        for (current_config = 0; current_config < 6; current_config++) {
          scan_bottompos1[current_config] = q_sols[(current_config << 3) +
            collision_flag];
        }

        rtb_writehome = Arm_controlle_check_collision_k(scan_bottompos1);
        if ((q_sols[collision_flag] < -1.0) || (q_sols[collision_flag] > 181.0))
        {
          /* '<S6>:1:47' */
          /* collision with rover */
          /* disp('collision with rover') */
          /* '<S6>:1:50' */
          checksol[collision_flag] = 1;
        } else if (fabs(q_sols[40 + collision_flag]) < 1.0) {
          /* '<S6>:1:52' */
          /* wrist is rotated */
          /*  disp('end effector') */
          /* '<S6>:1:54' */
          checksol[collision_flag] = 1;
        } else if (rtb_writehome == 1.0) {
          /* '<S6>:1:56' */
          /* disp('collision detected'); */
          /* '<S6>:1:58' */
          checksol[collision_flag] = 1;

          /*  elseif or((q_sols(i,1) > 90),(q_sols(i,1) < -1)) %%right shoulder config */
          /* disp('right shoulder config') %%this should reduce the maximum number of solutions to 2- shoulder down and shoulder ups */
          /* checksol(i) = 0; */
        } else {
          /* disp('solution is ok') */
          /* '<S6>:1:64' */
          checksol[collision_flag] = 0;
        }

        /* '<S6>:1:45' */
      }

      /* finds the solutions closest to the current position */
      /* '<S6>:1:69' */
      rtb_writehome = 100000.0;

      /* '<S6>:1:70' */
      j3 = 0.0;

      /* '<S6>:1:71' */
      for (collision_flag = 0; collision_flag < (int32_T)wrist_vect_idx_2;
           collision_flag++) {
        /* '<S6>:1:71' */
        if (checksol[collision_flag] == 0) {
          /* '<S6>:1:73' */
          /* %no collision */
          /* %compute joint space distance from current position */
          /* inflate the effect of rotating the shoulder */
          /* '<S6>:1:77' */
          j4 = fabs(q_sols[collision_flag] - rtb_Converttodegrees[0]) * 10.0;

          /* %joint space  */
          /* %inflate the effect of rotating the arm */
          /* '<S6>:1:79' */
          for (current_config = 0; current_config < 5; current_config++) {
            /* '<S6>:1:79' */
            /* '<S6>:1:80' */
            /* '<S6>:1:81' */
            j4 += fabs(q_sols[((current_config + 1) << 3) + collision_flag] -
                       rtb_Converttodegrees[current_config + 1]);

            /* '<S6>:1:79' */
          }

          if (j4 < rtb_writehome) {
            /* '<S6>:1:84' */
            /* disp('dilpesh') */
            /* '<S6>:1:86' */
            rtb_writehome = j4;

            /* '<S6>:1:87' */
            j3 = 1.0 + (real_T)collision_flag;
          }
        }

        /* '<S6>:1:71' */
      }

      if (j3 == 0.0) {
        /* '<S6>:1:93' */
        /* '<S6>:1:94' */
        status_flag = -1;
      } else {
        if (rtb_Converttodegrees[0] > 90.0) {
          /* '<S6>:1:99' */
          /* '<S6>:1:100' */
          current_config = 1;

          /* arm is currently in flipped config */
        } else {
          /* '<S6>:1:102' */
          current_config = 0;

          /* arm is currently in normal config */
        }

        if (q_sols[(int32_T)j3 - 1] > 90.0) {
          /* '<S6>:1:106' */
          /* '<S6>:1:107' */
          collision_flag = 1;

          /* arm will be in flipped config */
        } else {
          /* '<S6>:1:109' */
          collision_flag = 0;

          /* arm will be in normal config */
        }

        if ((collision_flag == 0) && (current_config == 0)) {
          /* '<S6>:1:113' */
          /* arm in normal configuration to normal */
          /* '<S6>:1:114' */
          rtb_wp1_k[0] = rtb_Converttodegrees[0];
          rtb_wp1_k[1] = rtb_Converttodegrees[1];
          rtb_wp1_k[2] = rtb_Converttodegrees[2];
          rtb_wp1_k[3] = rtb_Converttodegrees[3];
          rtb_wp1_k[4] = 90.0;
          rtb_wp1_k[5] = rtb_Converttodegrees[5];

          /* wp1(5) = 90; %turn end effector so it's parallel with elbow */
          /* '<S6>:1:117' */
          rtb_wp2_o[0] = q_sols[(int32_T)j3 - 1];
          rtb_wp2_o[1] = q_sols[(int32_T)j3 + 7];
          rtb_wp2_o[2] = q_sols[(int32_T)j3 + 15];
          rtb_wp2_o[3] = q_sols[(int32_T)j3 + 23];
          rtb_wp2_o[4] = 90.0;
          rtb_wp2_o[5] = 180.0;

          /*  wp2(5) = 90; %keeps end effector facing forward */
          /* '<S6>:1:120' */
          rtb_wp3_cw[0] = q_sols[(int32_T)j3 - 1];
          rtb_wp3_cw[1] = q_sols[(int32_T)j3 + 7];
          rtb_wp3_cw[2] = q_sols[(int32_T)j3 + 15];
          rtb_wp3_cw[3] = q_sols[(int32_T)j3 + 23];
          rtb_wp3_cw[4] = q_sols[(int32_T)j3 + 31];
          rtb_wp3_cw[5] = q_sols[(int32_T)j3 + 39];
          if ((Arm_controlle_check_collision_k(rtb_wp2_o) != 0.0) ||
              (Arm_controlle_check_collision_k(rtb_wp3_cw) != 0.0)) {
            /* '<S6>:1:122' */
            status_flag = -1;

            /* '<S6>:1:123' */
          } else {
            /* '<S6>:1:125' */
            status_flag = 1;

            /* '<S6>:1:126' */
            waypoint_number = 3;
          }
        } else if ((current_config == 0) && (collision_flag == 1)) {
          /* '<S6>:1:129' */
          /* normal to reverse */
          /* wp1 = current_position; */
          /* wp1(5) = 90; %turn end effector so it's parallel with elbow */
          /* '<S6>:1:132' */
          rtb_wp1_k[0] = rtb_Converttodegrees[0];
          rtb_wp1_k[1] = rtb_Converttodegrees[1];
          rtb_wp1_k[2] = rtb_Converttodegrees[2];
          rtb_wp1_k[3] = rtb_Converttodegrees[3];
          rtb_wp1_k[4] = 90.0;
          rtb_wp1_k[5] = rtb_Converttodegrees[5];

          /* '<S6>:1:134' */
          /* goes to reverse giraffe position */
          /* '<S6>:1:135' */
          for (current_config = 0; current_config < 6; current_config++) {
            rtb_wp2_o[current_config] = b_0[current_config];
            rtb_wp3_cw[current_config] = c[current_config];
          }

          /* end effector faces forward */
          /* wp4 = q_sols(minimum_sol, :); */
          /* '<S6>:1:138' */
          rtb_wp4_m2[0] = q_sols[(int32_T)j3 - 1];
          rtb_wp4_m2[1] = q_sols[(int32_T)j3 + 7];
          rtb_wp4_m2[2] = q_sols[(int32_T)j3 + 15];
          rtb_wp4_m2[3] = q_sols[(int32_T)j3 + 23];
          rtb_wp4_m2[4] = -90.0;
          rtb_wp4_m2[5] = q_sols[(int32_T)j3 + 39];

          /* wp4(5) = -90; */
          /* '<S6>:1:141' */
          rtb_wp5_m[0] = q_sols[(int32_T)j3 - 1];
          rtb_wp5_m[1] = q_sols[(int32_T)j3 + 7];
          rtb_wp5_m[2] = q_sols[(int32_T)j3 + 15];
          rtb_wp5_m[3] = q_sols[(int32_T)j3 + 23];
          rtb_wp5_m[4] = q_sols[(int32_T)j3 + 31];
          rtb_wp5_m[5] = q_sols[(int32_T)j3 + 39];
          if ((Arm_controlle_check_collision_k(b_0) != 0.0) ||
              (Arm_controlle_check_collision_k(c) != 0.0) ||
              (Arm_controlle_check_collision_k(rtb_wp4_m2) != 0.0) ||
              (Arm_controlle_check_collision_k(rtb_wp5_m) != 0.0)) {
            /* '<S6>:1:144' */
            status_flag = -1;

            /* '<S6>:1:145' */
          } else {
            /* '<S6>:1:147' */
            status_flag = 1;

            /* '<S6>:1:148' */
            waypoint_number = 5;
          }
        } else if ((current_config == 1) && (collision_flag == 0)) {
          /* '<S6>:1:151' */
          /* reverse to normal */
          /* wp1 = current_position; */
          /* wp1(5) = -90; %turn end effector so it's parallel with elbow */
          /* '<S6>:1:154' */
          rtb_wp1_k[0] = rtb_Converttodegrees[0];
          rtb_wp1_k[1] = rtb_Converttodegrees[1];
          rtb_wp1_k[2] = rtb_Converttodegrees[2];
          rtb_wp1_k[3] = rtb_Converttodegrees[3];
          rtb_wp1_k[4] = -90.0;
          rtb_wp1_k[5] = rtb_Converttodegrees[5];

          /* check if its 90 or -90 */
          /* '<S6>:1:157' */
          /* goes to giraffe position */
          /* '<S6>:1:158' */
          for (current_config = 0; current_config < 6; current_config++) {
            rtb_wp2_o[current_config] = d[current_config];
            rtb_wp3_cw[current_config] = e[current_config];
          }

          /* end effector faces forward */
          /* wp4 = q_sols(minimum_sol, :); */
          /* wp4(5) = 90; */
          /* '<S6>:1:162' */
          rtb_wp4_m2[0] = q_sols[(int32_T)j3 - 1];
          rtb_wp4_m2[1] = q_sols[(int32_T)j3 + 7];
          rtb_wp4_m2[2] = q_sols[(int32_T)j3 + 15];
          rtb_wp4_m2[3] = q_sols[(int32_T)j3 + 23];
          rtb_wp4_m2[4] = 90.0;
          rtb_wp4_m2[5] = q_sols[(int32_T)j3 + 39];

          /* '<S6>:1:164' */
          rtb_wp5_m[0] = q_sols[(int32_T)j3 - 1];
          rtb_wp5_m[1] = q_sols[(int32_T)j3 + 7];
          rtb_wp5_m[2] = q_sols[(int32_T)j3 + 15];
          rtb_wp5_m[3] = q_sols[(int32_T)j3 + 23];
          rtb_wp5_m[4] = q_sols[(int32_T)j3 + 31];
          rtb_wp5_m[5] = q_sols[(int32_T)j3 + 39];
          if ((Arm_controlle_check_collision_k(d) != 0.0) ||
              (Arm_controlle_check_collision_k(e) != 0.0) ||
              (Arm_controlle_check_collision_k(rtb_wp4_m2) != 0.0) ||
              (Arm_controlle_check_collision_k(rtb_wp5_m) != 0.0)) {
            /* '<S6>:1:167' */
            status_flag = -1;

            /* '<S6>:1:168' */
          } else {
            /* '<S6>:1:170' */
            status_flag = 1;

            /* '<S6>:1:171' */
            waypoint_number = 5;
          }
        } else {
          if ((current_config == 1) && (collision_flag == 1)) {
            /* '<S6>:1:174' */
            /* reverse to reverse */
            /* wp1 = current_position; */
            /* wp1(5) = -90; %turn end effector so it's parallel with elbow */
            /* '<S6>:1:177' */
            rtb_wp1_k[0] = rtb_Converttodegrees[0];
            rtb_wp1_k[1] = rtb_Converttodegrees[1];
            rtb_wp1_k[2] = rtb_Converttodegrees[2];
            rtb_wp1_k[3] = rtb_Converttodegrees[3];
            rtb_wp1_k[4] = -90.0;
            rtb_wp1_k[5] = rtb_Converttodegrees[5];

            /* wp2 = q_sols(minimum_sol, :); */
            /* wp2(5) = -90; %keeps end effector facing forward */
            /* '<S6>:1:181' */
            rtb_wp2_o[0] = q_sols[(int32_T)j3 - 1];
            rtb_wp2_o[1] = q_sols[(int32_T)j3 + 7];
            rtb_wp2_o[2] = q_sols[(int32_T)j3 + 15];
            rtb_wp2_o[3] = q_sols[(int32_T)j3 + 23];
            rtb_wp2_o[4] = -90.0;
            rtb_wp2_o[5] = q_sols[(int32_T)j3 + 39];

            /* wp3 = q_sols(minimum_sol, :); */
            /* '<S6>:1:184' */
            rtb_wp3_cw[0] = q_sols[(int32_T)j3 - 1];
            rtb_wp3_cw[1] = q_sols[(int32_T)j3 + 7];
            rtb_wp3_cw[2] = q_sols[(int32_T)j3 + 15];
            rtb_wp3_cw[3] = q_sols[(int32_T)j3 + 23];
            rtb_wp3_cw[4] = q_sols[(int32_T)j3 + 31];
            rtb_wp3_cw[5] = q_sols[(int32_T)j3 + 39];
            if ((Arm_controlle_check_collision_k(rtb_wp2_o) != 0.0) ||
                (Arm_controlle_check_collision_k(rtb_wp3_cw) != 0.0)) {
              /* '<S6>:1:186' */
              status_flag = -1;

              /* '<S6>:1:187' */
            } else {
              /* '<S6>:1:189' */
              status_flag = 1;

              /* '<S6>:1:190' */
              waypoint_number = 3;
            }
          }
        }

        /* convert waypoints so that joints avoid full rotations to get to a certain */
        /* angle */
        /* %full rotations only seem to be possible on j2, j3 and j4 */
        /* %full rotations are only possible while transitioning from giraffe/reverse */
        /* %giraffe to solution and vice versa. eg. only from wp1 to wp2, wp2 to wp3 */
        /* %or wp3 to wp4 */
        if (waypoint_number == 3) {
          /* '<S6>:1:204' */
          /* '<S6>:1:205' */
          /* %transforming wp2 if needed */
          if (fabs(rtb_wp2_o[1] - rtb_wp1_k[1]) > 180.0) {
            /* '<S6>:1:206' */
            if (rtb_wp2_o[1] < 0.0) {
              /* '<S6>:1:207' */
              /* '<S6>:1:208' */
              rtb_wp2_o[1] += 360.0;
            } else {
              /* '<S6>:1:210' */
              rtb_wp2_o[1] -= 360.0;
            }

            /* '<S6>:1:212' */
            rtb_wp3_cw[1] = rtb_wp2_o[1];
          }

          /* '<S6>:1:205' */
          /* %transforming wp2 if needed */
          if (fabs(rtb_wp2_o[2] - rtb_wp1_k[2]) > 180.0) {
            /* '<S6>:1:206' */
            if (rtb_wp2_o[2] < 0.0) {
              /* '<S6>:1:207' */
              /* '<S6>:1:208' */
              rtb_wp2_o[2] += 360.0;
            } else {
              /* '<S6>:1:210' */
              rtb_wp2_o[2] -= 360.0;
            }

            /* '<S6>:1:212' */
            rtb_wp3_cw[2] = rtb_wp2_o[2];
          }

          /* '<S6>:1:205' */
          /* %transforming wp2 if needed */
          if (fabs(rtb_wp2_o[3] - rtb_wp1_k[3]) > 180.0) {
            /* '<S6>:1:206' */
            if (rtb_wp2_o[3] < 0.0) {
              /* '<S6>:1:207' */
              /* '<S6>:1:208' */
              rtb_wp2_o[3] += 360.0;
            } else {
              /* '<S6>:1:210' */
              rtb_wp2_o[3] -= 360.0;
            }

            /* '<S6>:1:212' */
            rtb_wp3_cw[3] = rtb_wp2_o[3];
          }

          /* '<S6>:1:205' */
        } else {
          if (waypoint_number == 5) {
            /* '<S6>:1:216' */
            /* '<S6>:1:217' */
            if (fabs(rtb_wp4_m2[1] - rtb_wp3_cw[1]) > 180.0) {
              /* '<S6>:1:218' */
              if (rtb_wp4_m2[1] < 0.0) {
                /* '<S6>:1:219' */
                /* '<S6>:1:220' */
                rtb_wp4_m2[1] += 360.0;
              } else {
                /* '<S6>:1:222' */
                rtb_wp4_m2[1] -= 360.0;
              }

              /* '<S6>:1:224' */
              rtb_wp5_m[1] = rtb_wp4_m2[1];
            }

            /* '<S6>:1:217' */
            if (fabs(rtb_wp4_m2[2] - rtb_wp3_cw[2]) > 180.0) {
              /* '<S6>:1:218' */
              if (rtb_wp4_m2[2] < 0.0) {
                /* '<S6>:1:219' */
                /* '<S6>:1:220' */
                rtb_wp4_m2[2] += 360.0;
              } else {
                /* '<S6>:1:222' */
                rtb_wp4_m2[2] -= 360.0;
              }

              /* '<S6>:1:224' */
              rtb_wp5_m[2] = rtb_wp4_m2[2];
            }

            /* '<S6>:1:217' */
            if (fabs(rtb_wp4_m2[3] - rtb_wp3_cw[3]) > 180.0) {
              /* '<S6>:1:218' */
              if (rtb_wp4_m2[3] < 0.0) {
                /* '<S6>:1:219' */
                /* '<S6>:1:220' */
                rtb_wp4_m2[3] += 360.0;
              } else {
                /* '<S6>:1:222' */
                rtb_wp4_m2[3] -= 360.0;
              }

              /* '<S6>:1:224' */
              rtb_wp5_m[3] = rtb_wp4_m2[3];
            }

            /* '<S6>:1:217' */
          }
        }
      }
    }
  }

  /* MATLAB Function: '<S1>/set_pos2' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read1'
   */
  /* MATLAB Function 'Arm_controller_v2/set_pos2': '<S10>:1' */
  /* '<S10>:1:3' */
  collision_flag = 0;

  /* '<S10>:1:4' */
  /* '<S10>:1:5' */
  /* '<S10>:1:6' */
  /* '<S10>:1:7' */
  /* '<S10>:1:8' */
  for (i = 0; i < 6; i++) {
    scan_bottompos1[i] = 0.0;
    scan_toppos1[i] = 0.0;
    joint_angles_degrees[i] = 0.0;
    rtb_wp4[i] = 0.0;
    rtb_wp5[i] = 0.0;
  }

  /* '<S10>:1:9' */
  waypoint_number_0 = 0;
  if (rtb_enableflags[7] == 1) {
    /* '<S10>:1:11' */
    if (rtb_outputs[0] == 1.0) {
      /* '<S10>:1:12' */
      /* home_pos */
      if (rtb_Converttodegrees[0] > 90.0) {
        /* '<S10>:1:13' */
        /* '<S10>:1:14' */
        current_config = 1;

        /* arm is currently in flipped config */
      } else {
        /* '<S10>:1:16' */
        current_config = 0;

        /* arm is currently in normal config */
      }

      /* '<S10>:1:19' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 57.295779513082323 *
          Arm_controller_v2_DW.home_position[i];
      }

      if (current_config == 0) {
        /* '<S10>:1:20' */
        /* arm in normal configuration to normal */
        /* '<S10>:1:21' */
        scan_bottompos1[0] = rtb_Converttodegrees[0];
        scan_bottompos1[1] = rtb_Converttodegrees[1];
        scan_bottompos1[2] = rtb_Converttodegrees[2];
        scan_bottompos1[3] = rtb_Converttodegrees[3];
        scan_bottompos1[4] = 90.0;
        scan_bottompos1[5] = rtb_Converttodegrees[5];

        /* wp1(5) = 90; %turn end effector so it's parallel with elbow */
        /* '<S10>:1:24' */
        scan_toppos1[0] = rtb_TmpSignalConversionAtSFunct[0];
        scan_toppos1[1] = rtb_TmpSignalConversionAtSFunct[1];
        scan_toppos1[2] = rtb_TmpSignalConversionAtSFunct[2];
        scan_toppos1[3] = rtb_TmpSignalConversionAtSFunct[3];
        scan_toppos1[4] = 90.0;
        scan_toppos1[5] = 180.0;

        /*  wp2(5) = 90; %keeps end effector facing forward */
        /* '<S10>:1:27' */
        joint_angles_degrees[0] = rtb_TmpSignalConversionAtSFunct[0];
        joint_angles_degrees[1] = rtb_TmpSignalConversionAtSFunct[1];
        joint_angles_degrees[2] = rtb_TmpSignalConversionAtSFunct[2];
        joint_angles_degrees[3] = rtb_TmpSignalConversionAtSFunct[3];
        joint_angles_degrees[4] = rtb_TmpSignalConversionAtSFunct[4];
        joint_angles_degrees[5] = rtb_TmpSignalConversionAtSFunct[5];
        if ((Arm_controlle_check_collision_f(scan_toppos1) != 0.0) ||
            (Arm_controlle_check_collision_f(joint_angles_degrees) != 0.0)) {
          /* '<S10>:1:30' */
          collision_flag = -1;

          /* '<S10>:1:31' */
        } else {
          /* '<S10>:1:33' */
          collision_flag = 1;

          /* '<S10>:1:34' */
          waypoint_number_0 = 3;
        }
      } else {
        /* '<S10>:1:37' */
        /* need to flip arm */
        /* wp1 = current_position; */
        /* wp1(5) = -90; %turn end effector so it's parallel with elbow */
        /* '<S10>:1:40' */
        scan_bottompos1[0] = rtb_Converttodegrees[0];
        scan_bottompos1[1] = rtb_Converttodegrees[1];
        scan_bottompos1[2] = rtb_Converttodegrees[2];
        scan_bottompos1[3] = rtb_Converttodegrees[3];
        scan_bottompos1[4] = -90.0;
        scan_bottompos1[5] = rtb_Converttodegrees[5];

        /* check if its 90 or -90 */
        /* '<S10>:1:43' */
        /* goes to giraffe position */
        /* '<S10>:1:44' */
        for (current_config = 0; current_config < 6; current_config++) {
          scan_toppos1[current_config] = b_1[current_config];
          joint_angles_degrees[current_config] = c_0[current_config];
        }

        /* end effector faces forward */
        /* wp4 = home_pos(:); */
        /* wp4(5) = 90; */
        /* '<S10>:1:48' */
        rtb_wp4[0] = rtb_TmpSignalConversionAtSFunct[0];
        rtb_wp4[1] = rtb_TmpSignalConversionAtSFunct[1];
        rtb_wp4[2] = rtb_TmpSignalConversionAtSFunct[2];
        rtb_wp4[3] = rtb_TmpSignalConversionAtSFunct[3];
        rtb_wp4[4] = 90.0;
        rtb_wp4[5] = rtb_TmpSignalConversionAtSFunct[5];

        /* '<S10>:1:50' */
        rtb_wp5[0] = rtb_TmpSignalConversionAtSFunct[0];
        rtb_wp5[1] = rtb_TmpSignalConversionAtSFunct[1];
        rtb_wp5[2] = rtb_TmpSignalConversionAtSFunct[2];
        rtb_wp5[3] = rtb_TmpSignalConversionAtSFunct[3];
        rtb_wp5[4] = rtb_TmpSignalConversionAtSFunct[4];
        rtb_wp5[5] = rtb_TmpSignalConversionAtSFunct[5];
        if ((Arm_controlle_check_collision_f(b_1) != 0.0) ||
            (Arm_controlle_check_collision_f(c_0) != 0.0) ||
            (Arm_controlle_check_collision_f(rtb_wp4) != 0.0) ||
            (Arm_controlle_check_collision_f(rtb_wp5) != 0.0)) {
          /* '<S10>:1:53' */
          collision_flag = -1;

          /* '<S10>:1:54' */
        } else {
          /* '<S10>:1:56' */
          collision_flag = 1;

          /* '<S10>:1:57' */
          waypoint_number_0 = 5;
        }
      }
    } else {
      if (rtb_outputs[0] == 2.0) {
        /* '<S10>:1:62' */
        /* %giraffe position */
        if (rtb_Converttodegrees[0] > 90.0) {
          /* '<S10>:1:64' */
          /* '<S10>:1:65' */
          current_config = 1;

          /* arm is currently in flipped config */
        } else {
          /* '<S10>:1:67' */
          current_config = 0;

          /* arm is currently in normal config */
        }

        if (current_config == 0) {
          /* '<S10>:1:70' */
          /* arm in normal configuration to normal */
          /* '<S10>:1:71' */
          scan_bottompos1[0] = rtb_Converttodegrees[0];
          scan_bottompos1[1] = rtb_Converttodegrees[1];
          scan_bottompos1[2] = rtb_Converttodegrees[2];
          scan_bottompos1[3] = rtb_Converttodegrees[3];
          scan_bottompos1[4] = 90.0;
          scan_bottompos1[5] = rtb_Converttodegrees[5];

          /* wp1(5) = 90; %turn end effector so it's parallel with elbow */
          /* '<S10>:1:74' */
          for (current_config = 0; current_config < 6; current_config++) {
            scan_toppos1[current_config] = c_0[current_config];
          }

          /*  wp2(5) = 90; %keeps end effector facing forward */
          if (Arm_controlle_check_collision_f(c_0) != 0.0) {
            /* '<S10>:1:78' */
            collision_flag = -1;

            /* '<S10>:1:79' */
          } else {
            /* '<S10>:1:81' */
            collision_flag = 1;

            /* '<S10>:1:82' */
            waypoint_number_0 = 2;
          }
        } else {
          /* '<S10>:1:85' */
          /* need to flip arm */
          /* wp1 = current_position; */
          /* wp1(5) = -90; %turn end effector so it's parallel with elbow */
          /* '<S10>:1:88' */
          scan_bottompos1[0] = rtb_Converttodegrees[0];
          scan_bottompos1[1] = rtb_Converttodegrees[1];
          scan_bottompos1[2] = rtb_Converttodegrees[2];
          scan_bottompos1[3] = rtb_Converttodegrees[3];
          scan_bottompos1[4] = -90.0;
          scan_bottompos1[5] = rtb_Converttodegrees[5];

          /* check if its 90 or -90 */
          /* '<S10>:1:91' */
          /* goes to giraffe position */
          /* '<S10>:1:92' */
          for (current_config = 0; current_config < 6; current_config++) {
            scan_toppos1[current_config] = b_1[current_config];
            joint_angles_degrees[current_config] = c_0[current_config];
          }

          /* end effector faces forward */
          /* wp4 = home_pos(:); */
          /* wp4(5) = 90;             */
          if ((Arm_controlle_check_collision_f(b_1) != 0.0) ||
              (Arm_controlle_check_collision_f(c_0) != 0.0)) {
            /* '<S10>:1:98' */
            collision_flag = -1;

            /* '<S10>:1:99' */
          } else {
            /* '<S10>:1:101' */
            collision_flag = 1;

            /* '<S10>:1:102' */
            waypoint_number_0 = 3;
          }
        }
      }
    }
  }

  /* MultiPortSwitch: '<S1>/Multiport Switch3' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  MATLAB Function: '<S1>/move_cam'
   *  MATLAB Function: '<S1>/move_cam2'
   *  MATLAB Function: '<S1>/move_effector'
   *  MATLAB Function: '<S1>/set_pos2'
   */
  switch (rtb_portoutput) {
   case 1:
    rtb_statusflag_setpos = statusflag_movecam;
    break;

   case 2:
    break;

   case 3:
    rtb_statusflag_setpos = rtb_statusflag_movej;
    break;

   case 4:
    rtb_statusflag_setpos = status_scan;
    break;

   case 5:
    rtb_statusflag_setpos = status_movecam2;
    break;

   case 6:
    rtb_statusflag_setpos = status_flag;
    break;

   default:
    rtb_statusflag_setpos = collision_flag;
    break;
  }

  /* End of MultiPortSwitch: '<S1>/Multiport Switch3' */

  /* MATLAB Function: '<S1>/sendcommand' incorporates:
   *  MATLAB Function: '<S1>/move_effector'
   *  MATLAB Function: '<S1>/set_pos2'
   */
  /* MATLAB Function 'Arm_controller_v2/sendcommand': '<S8>:1' */
  /* converts the joint angles into radians */
  /* if all of them are zero it is implied that no movement should occur */
  /* '<S8>:1:5' */
  /* to give matlab the size of the variable */
  /* '<S8>:1:6' */
  /* '<S8>:1:7' */
  /* '<S8>:1:8' */
  /* '<S8>:1:9' */
  for (i = 0; i < 6; i++) {
    rtb_TmpSignalConversionAtSFunct[i] = 0.0;
    rtb_outputs[i] = 0.0;
    rtb_Converttodegrees[i] = 0.0;
    rtb_wp4_h[i] = 0.0;
    rtb_wp5_o[i] = 0.0;
  }

  /* '<S8>:1:10' */
  /* '<S8>:1:11' */
  rtb_writehome = 0.0;

  /* jointangles = (pi/180)*jointangles; %convert to radians */
  /* wp1 = jointangles; */
  if (rtb_statusflag_setpos != 1) {
    /* '<S8>:1:16' */
    /* interpreted as doing nothing */
    /* '<S8>:1:17' */
    j3 = 0.0;

    /* don't send command */
  } else {
    /* '<S8>:1:19' */
    j3 = 1.0;

    /* send command to arm */
    if (rtb_portoutput == 1) {
      /* '<S8>:1:20' */
      /* '<S8>:1:21' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 0.017453292519943295 *
          rtb_jointangles_n[i];
      }

      /* '<S8>:1:22' */
      rtb_writehome = 1.0;
    } else if (rtb_portoutput == 2) {
      /* '<S8>:1:23' */
      /* %move to preset position */
      /* '<S8>:1:24' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 0.017453292519943295 *
          rtb_jointangles[i];
      }

      /* '<S8>:1:25' */
      rtb_writehome = 1.0;
    } else if (rtb_portoutput == 3) {
      /* '<S8>:1:26' */
      /* %movej command */
      /* '<S8>:1:27' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 0.017453292519943295 *
          rtb_jointangles_l[i];
      }

      /* '<S8>:1:28' */
      rtb_writehome = 1.0;
    } else if (rtb_portoutput == 4) {
      /* '<S8>:1:29' */
      /* %scan command */
      /* '<S8>:1:30' */
      /* '<S8>:1:31' */
      /* '<S8>:1:32' */
      /* '<S8>:1:33' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 0.017453292519943295 *
          rtb_scan_startpos[i];
        rtb_outputs[i] = 0.017453292519943295 * rtb_scan_bottompos[i];
        rtb_Converttodegrees[i] = 0.017453292519943295 * rtb_scan_toppos[i];
        rtb_wp4_h[i] = 0.017453292519943295 * rtb_scan_startpos[i];
      }

      /* '<S8>:1:34' */
      rtb_writehome = 4.0;
    } else if (rtb_portoutput == 5) {
      /* '<S8>:1:35' */
      /* %pan, tilt while maintaining the x, y, z */
      /* '<S8>:1:36' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 0.017453292519943295 *
          rtb_jointangles_a[i];
      }

      /* '<S8>:1:37' */
      rtb_writehome = 1.0;
    } else if (rtb_portoutput == 6) {
      /* '<S8>:1:38' */
      /* %move effector x y z position while end effector is facing forward. */
      /* '<S8>:1:39' */
      /* '<S8>:1:40' */
      /* '<S8>:1:41' */
      /* '<S8>:1:42' */
      /* '<S8>:1:43' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 0.017453292519943295 * rtb_wp1_k[i];
        rtb_outputs[i] = 0.017453292519943295 * rtb_wp2_o[i];
        rtb_Converttodegrees[i] = 0.017453292519943295 * rtb_wp3_cw[i];
        rtb_wp4_h[i] = 0.017453292519943295 * rtb_wp4_m2[i];
        rtb_wp5_o[i] = 0.017453292519943295 * rtb_wp5_m[i];
      }

      /* '<S8>:1:44' */
      rtb_writehome = waypoint_number;
    } else {
      /* '<S8>:1:45' */
      /* %setpos2 function */
      /* '<S8>:1:46' */
      /* '<S8>:1:47' */
      /* '<S8>:1:48' */
      /* '<S8>:1:49' */
      /* '<S8>:1:50' */
      for (i = 0; i < 6; i++) {
        rtb_TmpSignalConversionAtSFunct[i] = 0.017453292519943295 *
          scan_bottompos1[i];
        rtb_outputs[i] = 0.017453292519943295 * scan_toppos1[i];
        rtb_Converttodegrees[i] = 0.017453292519943295 * joint_angles_degrees[i];
        rtb_wp4_h[i] = 0.017453292519943295 * rtb_wp4[i];
        rtb_wp5_o[i] = 0.017453292519943295 * rtb_wp5[i];
      }

      /* '<S8>:1:51' */
      rtb_writehome = waypoint_number_0;
    }
  }

  /* Outport: '<Root>/arm_status' */
  Arm_controller_v2_Y.arm_status = rtb_statusflag_setpos;

  /* Outport: '<Root>/command_flag' */
  Arm_controller_v2_Y.command_flag = j3;

  /* Outport: '<Root>/number_waypoints' */
  Arm_controller_v2_Y.number_waypoints = rtb_writehome;
  for (i = 0; i < 6; i++) {
    /* Outport: '<Root>/WP1' */
    Arm_controller_v2_Y.WP1[i] = rtb_TmpSignalConversionAtSFunct[i];

    /* Outport: '<Root>/WP2' */
    Arm_controller_v2_Y.WP2[i] = rtb_outputs[i];

    /* Outport: '<Root>/WP3' */
    Arm_controller_v2_Y.WP3[i] = rtb_Converttodegrees[i];

    /* Outport: '<Root>/WP4' */
    Arm_controller_v2_Y.WP4[i] = rtb_wp4_h[i];

    /* Outport: '<Root>/WP5' */
    Arm_controller_v2_Y.WP5[i] = rtb_wp5_o[i];

    /* Outport: '<Root>/WP6' incorporates:
     *  MATLAB Function: '<S1>/sendcommand'
     */
    Arm_controller_v2_Y.WP6[i] = 0.0;

    /* Update for Memory: '<S1>/Memory1' */
    Arm_controller_v2_DW.Memory1_PreviousInput[i] = rtb_MultiportSwitch2[i];
  }
}

/* Model initialize function */
void Arm_controller_v2_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(Arm_controller_v2_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&Arm_controller_v2_DW, 0,
                sizeof(DW_Arm_controller_v2_T));

  /* external inputs */
  (void) memset((void *)&Arm_controller_v2_U, 0,
                sizeof(ExtU_Arm_controller_v2_T));

  /* external outputs */
  (void) memset((void *)&Arm_controller_v2_Y, 0,
                sizeof(ExtY_Arm_controller_v2_T));

  {
    int32_T i;

    /* Start for DataStoreMemory: '<S1>/home_pos' */
    for (i = 0; i < 6; i++) {
      Arm_controller_v2_DW.home_position[i] =
        Arm_controller_v2_P.home_pos_InitialValue[i];
    }

    /* End of Start for DataStoreMemory: '<S1>/home_pos' */
  }

  {
    int32_T i;

    /* InitializeConditions for Memory: '<S1>/Memory1' */
    for (i = 0; i < 6; i++) {
      Arm_controller_v2_DW.Memory1_PreviousInput[i] =
        Arm_controller_v2_P.Memory1_X0[i];
    }

    /* End of InitializeConditions for Memory: '<S1>/Memory1' */
  }
}

/* Model terminate function */
void Arm_controller_v2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
