/*
 * File: Arm_Controller.c
 *
 * Code generated for Simulink model 'Arm_Controller'.
 *
 * Model version                  : 1.178
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Tue Aug 19 16:41:09 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Arm_Controller.h"
#include "Arm_Controller_private.h"

/* Block states (auto storage) */
DW_Arm_Controller_T Arm_Controller_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_Arm_Controller_T Arm_Controller_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_Arm_Controller_T Arm_Controller_Y;

/* Real-time model */
RT_MODEL_Arm_Controller_T Arm_Controller_M_;
RT_MODEL_Arm_Controller_T *const Arm_Controller_M = &Arm_Controller_M_;

/* Forward declaration for local functions */
static void Arm_Controller_cosd(real_T *x);
static void Arm_Controller_sind(real_T *x);
static real_T Arm_Controller_norm(const real_T x[3]);
static real_T Arm_Controller_dot(const real_T a[3], const real_T b[3]);
static real_T Arm_Control_DistBetween2Segment(const real_T p1[3], const real_T
  p2[3], const real_T p3[3], const real_T p4[3]);
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
static void Arm_Controller_cosd(real_T *x)
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
static void Arm_Controller_sind(real_T *x)
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
static real_T Arm_Controller_norm(const real_T x[3])
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
static real_T Arm_Controller_dot(const real_T a[3], const real_T b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

/* Function for MATLAB Function: '<S1>/move_cam' */
static real_T Arm_Control_DistBetween2Segment(const real_T p1[3], const real_T
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
  a = Arm_Controller_dot(u, u);
  b = Arm_Controller_dot(u, v);
  c = Arm_Controller_dot(v, v);
  d = Arm_Controller_dot(u, w);
  e = Arm_Controller_dot(v, w);
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
  distance = Arm_Controller_norm(w_0);

  /* outV = dP; */
  /* varargout(1) = {outV};      % vector connecting the closest points */
  /* varargout(2) = {p2+sc*u};   % Closest point on object 1  */
  /* varargout(3) = {p4+tc*v};   % Closest point on object 2 */
  return distance;
}

/* Model step function */
void Arm_Controller_step(void)
{
  real_T j4;
  real_T j3;
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
  real_T q;
  static const int8_T b[6] = { 0, -90, 0, -90, 90, 0 };

  boolean_T p;
  boolean_T b_p;
  boolean_T exitg1;
  real_T rtb_writehome;
  real_T rtb_MultiportSwitch2[6];
  int32_T rtb_portoutput;
  real_T rtb_write_pos;
  real_T rtb_statusflag_movej;
  real_T rtb_TmpSignalConversionAtSFunct[6];
  real_T rtb_outputs[6];
  real_T rtb_jointangles_h[6];
  real_T rtb_jointangles[6];
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
  int32_T i_0;
  real_T T1_1[16];
  real_T rtb_writehome_0[16];
  int8_T rtb_enableflags_idx_0;
  int8_T rtb_enableflags_idx_1;
  int8_T rtb_enableflags_idx_2;
  int8_T rtb_enableflags_idx_3;
  real_T p1_idx_0;
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
   *  SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport2'
   */
  /* MATLAB Function 'Arm_Controller/functionselector': '<S2>:1' */
  /* '<S2>:1:4' */
  rtb_enableflags_idx_0 = 0;
  rtb_enableflags_idx_1 = 0;
  rtb_enableflags_idx_2 = 0;
  rtb_enableflags_idx_3 = 0;

  /* '<S2>:1:5' */
  for (i = 0; i < 6; i++) {
    rtb_outputs[i] = 0.0;
  }

  /* '<S2>:1:7' */
  rtb_portoutput = 2;
  if (Arm_Controller_U.motion_primitive == 1.0) {
    /* '<S2>:1:9' */
    /* activate move_cam and disable home position. Can expand concept to */
    /* other functions */
    /* '<S2>:1:12' */
    rtb_enableflags_idx_0 = 1;

    /* transfer the inputs received to the inputs of the move_cam function */
    /* '<S2>:1:15' */
    rtb_outputs[0] = Arm_Controller_U.Input_1;

    /* '<S2>:1:16' */
    rtb_outputs[1] = Arm_Controller_U.Input_2;

    /* '<S2>:1:17' */
    rtb_outputs[2] = Arm_Controller_U.Input_3;

    /* '<S2>:1:18' */
    rtb_portoutput = 1;
  } else if (Arm_Controller_U.motion_primitive == 2.0) {
    /* '<S2>:1:20' */
    /* activate move to home/giraffe position */
    /* '<S2>:1:22' */
    rtb_enableflags_idx_1 = 1;

    /* '<S2>:1:24' */
    rtb_outputs[0] = Arm_Controller_U.Input_1;

    /* '<S2>:1:25' */
  } else if (Arm_Controller_U.motion_primitive == 3.0) {
    /* '<S2>:1:27' */
    /* '<S2>:1:28' */
    rtb_enableflags_idx_2 = 1;

    /* set home position */
  } else if (Arm_Controller_U.motion_primitive == 4.0) {
    /* '<S2>:1:31' */
    /* movej function. full control of arm motion.  */
    /* '<S2>:1:33' */
    rtb_enableflags_idx_3 = 1;

    /* '<S2>:1:34' */
    rtb_outputs[0] = Arm_Controller_U.Input_1;

    /* '<S2>:1:35' */
    rtb_outputs[1] = Arm_Controller_U.Input_2;

    /* '<S2>:1:36' */
    rtb_outputs[2] = Arm_Controller_U.Input_3;

    /* '<S2>:1:37' */
    rtb_outputs[3] = Arm_Controller_U.Input_4;

    /* '<S2>:1:38' */
    rtb_outputs[4] = Arm_Controller_U.Input_5;

    /* '<S2>:1:39' */
    rtb_outputs[5] = Arm_Controller_U.Input_6;

    /* '<S2>:1:40' */
    rtb_portoutput = 3;
  } else {
    /* reject input and dont do anything */
    /* '<S2>:1:43' */
    rtb_portoutput = 1;
  }

  /* End of MATLAB Function: '<S1>/functionselector' */

  /* MATLAB Function: '<S1>/set_home' */
  /* MATLAB Function 'Arm_Controller/set_home': '<S6>:1' */
  /* '<S6>:1:3' */
  if (rtb_enableflags_idx_2 == 1) {
    /* '<S6>:1:4' */
    /* '<S6>:1:5' */
    rtb_writehome = 1.0;
  } else {
    /* '<S6>:1:7' */
    rtb_writehome = 2.0;
  }

  for (i = 0; i < 6; i++) {
    /* MultiPortSwitch: '<S1>/Multiport Switch2' incorporates:
     *  DataStoreRead: '<S1>/Data Store Read2'
     *  Memory: '<S1>/Memory1'
     */
    if (rtb_writehome == 1.0) {
      wrist_vect_idx_2 = Arm_Controller_DW.current_position[i];
    } else {
      wrist_vect_idx_2 = Arm_Controller_DW.Memory1_PreviousInput[i];
    }

    /* DataStoreWrite: '<S1>/Data Store Write' */
    Arm_Controller_DW.home_position[i] = wrist_vect_idx_2;

    /* MATLAB Function: '<S1>/move_cam' */
    rtb_TmpSignalConversionAtSFunct[i] = 0.0;

    /* MultiPortSwitch: '<S1>/Multiport Switch2' */
    rtb_MultiportSwitch2[i] = wrist_vect_idx_2;
  }

  /* End of MATLAB Function: '<S1>/set_home' */

  /* MATLAB Function: '<S1>/move_cam' */
  rtb_writehome = rtb_outputs[2];

  /* MATLAB Function 'Arm_Controller/move_cam': '<S3>:1' */
  /* '<S3>:1:4' */
  /* j1 and j2 will ultimately use the home position of the robot as reference */
  /* j1 = -37.93; %this angle orients the base towards the user */
  /* j2 = abs(-173.74 + 90);  */
  /* '<S3>:1:8' */
  /* this angle orients the base towards the user */
  /* '<S3>:1:9' */
  if (rtb_enableflags_idx_0 == 1) {
    /* '<S3>:1:11' */
    /* max_height = 0.392 + 0.425*cosd(j2); %max possible height of the end effector */
    /* if height > max_height, %catch invalid height */
    /* disp('The specified height is larger than the maximum possible height for this configuration. Using a height of: ') */
    /* disp(max_height) */
    /* statusflag = -2; %out of target height */
    /* height = max_height; */
    /* end */
    if (rtb_outputs[2] == 1.0) {
      /* '<S3>:1:20' */
      /* they want the giraffe position */
      /* '<S3>:1:24' */
      rtb_TmpSignalConversionAtSFunct[0] = 0.0;
      rtb_TmpSignalConversionAtSFunct[1] = -90.0;
      rtb_TmpSignalConversionAtSFunct[2] = 0.0;
      rtb_TmpSignalConversionAtSFunct[3] = rtb_outputs[1] - 90.0;
      rtb_TmpSignalConversionAtSFunct[4] = 90.0 + rtb_outputs[0];
      rtb_TmpSignalConversionAtSFunct[5] = 0.0;

      /* '<S3>:1:25' */
      collision_flag = 1;
    } else {
      /* they want standard height control */
      /* '<S3>:1:28' */
      /* max possible height of the end effector */
      if (rtb_outputs[2] > 0.8105432950301884) {
        /* '<S3>:1:29' */
        /* catch invalid height */
        /* '<S3>:1:30' */
        rtb_writehome = 0.8105432950301884;
      } else {
        if (rtb_outputs[2] < 0.13) {
          /* '<S3>:1:31' */
          /* '<S3>:1:32' */
          rtb_writehome = 0.13;
        }
      }

      /* j3 = 90 - j2 + asind((height - 0.425*cosd(j2))/0.392);  %calculates the elbow joint angle needed to reach the required height. works properly */
      /* j4 = tilt_angle - j2 - j3 + 180; %joint angle to apply to achieve target tilt position. works properly */
      /* j5 = pan_angle; %assumption. good assumption */
      /* j6 = 0; %not important at the moment */
      if (rtb_writehome < 0.41854329503018839) {
        /* '<S3>:1:40' */
        /* '<S3>:1:41' */
        /* '<S3>:1:42' */
        /* '<S3>:1:43' */
        j3 = -(asin((0.41854329503018839 - rtb_writehome) / 0.392) *
               57.295779513082323 + 100.0);
      } else {
        /* '<S3>:1:45' */
        /* '<S3>:1:46' */
        /* '<S3>:1:47' */
        j3 = -((180.0 - asin((rtb_writehome - 0.41854329503018839) / 0.392) *
                57.295779513082323) - 80.0);
      }

      /* with arm pitching forward */
      /* j4 = j2-j3-90-tilt_angle-90; */
      /* '<S3>:1:52' */
      j4 = (((80.0 - j3) - 90.0) + rtb_outputs[1]) - 90.0;

      /* with arm pitching back */
      /* j4 = j2 - j3 - 90 - tilt_angle; */
      /* assumption */
      /* very basic check to see if the robot will collide with itself */
      /* if (j4 > 205)||((180-j3) > 155)||((90+j5) > 140), %found these upper limits by observation */
      /* disp('Movement will cause collision') */
      /* statusflag_movecam = -1; */
      /* return; */
      /* end */
      /* %self collision checking %%%%%% */
      /* '<S3>:1:67' */
      /* '<S3>:1:68' */
      /* UNTITLED Summary of this function goes here */
      /*    Detailed explanation goes here */
      rtb_writehome = -0.0;
      Arm_Controller_cosd(&rtb_writehome);
      rtb_statusflag_movej = -0.0;
      Arm_Controller_sind(&rtb_statusflag_movej);
      rtb_write_pos = -0.0;
      Arm_Controller_sind(&rtb_write_pos);
      wrist_vect_idx_2 = -0.0;
      Arm_Controller_cosd(&wrist_vect_idx_2);
      T1[0] = rtb_writehome;
      T1[4] = 0.0;
      T1[8] = rtb_statusflag_movej;
      T1[12] = 0.0;
      T1[1] = rtb_write_pos;
      T1[5] = 0.0;
      T1[9] = -wrist_vect_idx_2;
      T1[13] = 0.0;
      T1[2] = 0.0;
      T1[6] = 1.0;
      T1[10] = 0.0;
      T1[14] = 0.089;
      T1[3] = 0.0;
      T1[7] = 0.0;
      T1[11] = 0.0;
      T1[15] = 1.0;
      rtb_writehome = -80.0;
      Arm_Controller_cosd(&rtb_writehome);
      rtb_statusflag_movej = -80.0;
      Arm_Controller_sind(&rtb_statusflag_movej);
      rtb_write_pos = -80.0;
      Arm_Controller_cosd(&rtb_write_pos);
      wrist_vect_idx_2 = -80.0;
      Arm_Controller_sind(&wrist_vect_idx_2);
      q = -80.0;
      Arm_Controller_cosd(&q);
      p1_idx_0 = -80.0;
      Arm_Controller_sind(&p1_idx_0);
      T2[0] = rtb_writehome;
      T2[4] = -rtb_statusflag_movej;
      T2[8] = 0.0;
      T2[12] = -0.425 * rtb_write_pos;
      T2[1] = wrist_vect_idx_2;
      T2[5] = q;
      T2[9] = 0.0;
      T2[13] = -0.425 * p1_idx_0;
      T2[2] = 0.0;
      T2[6] = 0.0;
      T2[10] = 1.0;
      T2[14] = 0.0;
      T2[3] = 0.0;
      T2[7] = 0.0;
      T2[11] = 0.0;
      T2[15] = 1.0;
      rtb_writehome = j3;
      Arm_Controller_cosd(&rtb_writehome);
      rtb_statusflag_movej = j3;
      Arm_Controller_sind(&rtb_statusflag_movej);
      rtb_write_pos = j3;
      Arm_Controller_cosd(&rtb_write_pos);
      wrist_vect_idx_2 = j3;
      Arm_Controller_sind(&wrist_vect_idx_2);
      q = j3;
      Arm_Controller_cosd(&q);
      p1_idx_0 = j3;
      Arm_Controller_sind(&p1_idx_0);
      T3[0] = rtb_writehome;
      T3[4] = -rtb_statusflag_movej;
      T3[8] = 0.0;
      T3[12] = -0.392 * rtb_write_pos;
      T3[1] = wrist_vect_idx_2;
      T3[5] = q;
      T3[9] = 0.0;
      T3[13] = -0.392 * p1_idx_0;
      T3[2] = 0.0;
      T3[6] = 0.0;
      T3[10] = 1.0;
      T3[14] = 0.0;
      T3[3] = 0.0;
      T3[7] = 0.0;
      T3[11] = 0.0;
      T3[15] = 1.0;
      rtb_writehome = j4;
      Arm_Controller_cosd(&rtb_writehome);
      rtb_statusflag_movej = j4;
      Arm_Controller_sind(&rtb_statusflag_movej);
      rtb_write_pos = j4;
      Arm_Controller_sind(&rtb_write_pos);
      wrist_vect_idx_2 = j4;
      Arm_Controller_cosd(&wrist_vect_idx_2);
      T4[0] = rtb_writehome;
      T4[4] = 0.0;
      T4[8] = rtb_statusflag_movej;
      T4[12] = 0.0;
      T4[1] = rtb_write_pos;
      T4[5] = 0.0;
      T4[9] = -wrist_vect_idx_2;
      T4[13] = 0.0;
      T4[2] = 0.0;
      T4[6] = 1.0;
      T4[10] = 0.0;
      T4[14] = 0.109;
      T4[3] = 0.0;
      T4[7] = 0.0;
      T4[11] = 0.0;
      T4[15] = 1.0;
      rtb_writehome = 90.0 + rtb_outputs[0];
      Arm_Controller_cosd(&rtb_writehome);
      rtb_statusflag_movej = 90.0 + rtb_outputs[0];
      Arm_Controller_sind(&rtb_statusflag_movej);
      rtb_write_pos = 90.0 + rtb_outputs[0];
      Arm_Controller_sind(&rtb_write_pos);
      wrist_vect_idx_2 = 90.0 + rtb_outputs[0];
      Arm_Controller_cosd(&wrist_vect_idx_2);
      T5[0] = rtb_writehome;
      T5[4] = 0.0;
      T5[8] = -rtb_statusflag_movej;
      T5[12] = 0.0;
      T5[1] = rtb_write_pos;
      T5[5] = 0.0;
      T5[9] = wrist_vect_idx_2;
      T5[13] = 0.0;
      T5[2] = 0.0;
      T5[6] = -1.0;
      T5[10] = 0.0;
      T5[14] = 0.095;
      T5[3] = 0.0;
      T5[7] = 0.0;
      T5[11] = 0.0;
      T5[15] = 1.0;
      rtb_writehome = 0.0;
      Arm_Controller_cosd(&rtb_writehome);
      rtb_statusflag_movej = 0.0;
      Arm_Controller_sind(&rtb_statusflag_movej);
      rtb_write_pos = 0.0;
      Arm_Controller_sind(&rtb_write_pos);
      wrist_vect_idx_2 = 0.0;
      Arm_Controller_cosd(&wrist_vect_idx_2);
      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_0[i_0 + (i << 2)] = 0.0;
          T1_0[i_0 + (i << 2)] += T2[i << 2] * T1[i_0];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 1] * T1[i_0 + 4];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 2] * T1[i_0 + 8];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 3] * T1[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_1[i_0 + (i << 2)] = 0.0;
          T1_1[i_0 + (i << 2)] += T3[i << 2] * T1_0[i_0];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 1] * T1_0[i_0 + 4];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 2] * T1_0[i_0 + 8];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 3] * T1_0[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_0[i_0 + (i << 2)] = 0.0;
          T1_0[i_0 + (i << 2)] += T4[i << 2] * T1_1[i_0];
          T1_0[i_0 + (i << 2)] += T4[(i << 2) + 1] * T1_1[i_0 + 4];
          T1_0[i_0 + (i << 2)] += T4[(i << 2) + 2] * T1_1[i_0 + 8];
          T1_0[i_0 + (i << 2)] += T4[(i << 2) + 3] * T1_1[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_1[i_0 + (i << 2)] = 0.0;
          T1_1[i_0 + (i << 2)] += T5[i << 2] * T1_0[i_0];
          T1_1[i_0 + (i << 2)] += T5[(i << 2) + 1] * T1_0[i_0 + 4];
          T1_1[i_0 + (i << 2)] += T5[(i << 2) + 2] * T1_0[i_0 + 8];
          T1_1[i_0 + (i << 2)] += T5[(i << 2) + 3] * T1_0[i_0 + 12];
        }
      }

      rtb_writehome_0[0] = rtb_writehome;
      rtb_writehome_0[4] = -rtb_statusflag_movej;
      rtb_writehome_0[8] = 0.0;
      rtb_writehome_0[12] = 0.0;
      rtb_writehome_0[1] = rtb_write_pos;
      rtb_writehome_0[5] = wrist_vect_idx_2;
      rtb_writehome_0[9] = 0.0;
      rtb_writehome_0[13] = 0.0;
      rtb_writehome_0[2] = 0.0;
      rtb_writehome_0[6] = 0.0;
      rtb_writehome_0[10] = 1.0;
      rtb_writehome_0[14] = 0.082;
      rtb_writehome_0[3] = 0.0;
      rtb_writehome_0[7] = 0.0;
      rtb_writehome_0[11] = 0.0;
      rtb_writehome_0[15] = 1.0;
      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          total_transform[i_0 + (i << 2)] = 0.0;
          total_transform[i_0 + (i << 2)] += rtb_writehome_0[i << 2] * T1_1[i_0];
          total_transform[i_0 + (i << 2)] += rtb_writehome_0[(i << 2) + 1] *
            T1_1[i_0 + 4];
          total_transform[i_0 + (i << 2)] += rtb_writehome_0[(i << 2) + 2] *
            T1_1[i_0 + 8];
          total_transform[i_0 + (i << 2)] += rtb_writehome_0[(i << 2) + 3] *
            T1_1[i_0 + 12];
        }
      }

      /* arm_orig = [0;0;0;1]; */
      for (i_0 = 0; i_0 < 4; i_0++) {
        wrist_vect_idx_2 = T1[i_0 + 12] + (T1[i_0 + 8] * 0.0 + (T1[i_0 + 4] *
          0.0 + T1[i_0] * 0.0));
        shoulder_start[i_0] = wrist_vect_idx_2;
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_0[i_0 + (i << 2)] = 0.0;
          T1_0[i_0 + (i << 2)] += T2[i << 2] * T1[i_0];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 1] * T1[i_0 + 4];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 2] * T1[i_0 + 8];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 3] * T1[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        wrist_vect_idx_2 = T1_0[i_0 + 12] + (T1_0[i_0 + 8] * 0.0 + (T1_0[i_0 + 4]
          * 0.0 + T1_0[i_0] * 0.0));
        shoulder_orig[i_0] = wrist_vect_idx_2;
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_0[i_0 + (i << 2)] = 0.0;
          T1_0[i_0 + (i << 2)] += T2[i << 2] * T1[i_0];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 1] * T1[i_0 + 4];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 2] * T1[i_0 + 8];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 3] * T1[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_1[i_0 + (i << 2)] = 0.0;
          T1_1[i_0 + (i << 2)] += T3[i << 2] * T1_0[i_0];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 1] * T1_0[i_0 + 4];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 2] * T1_0[i_0 + 8];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 3] * T1_0[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        wrist_vect_idx_2 = T1_1[i_0 + 12] + (T1_1[i_0 + 8] * 0.0 + (T1_1[i_0 + 4]
          * 0.0 + T1_1[i_0] * 0.0));
        elbow_orig[i_0] = wrist_vect_idx_2;
      }

      /* wrist_1 = T1*T2*T3*T4*[0;0;0;1]; */
      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_0[i_0 + (i << 2)] = 0.0;
          T1_0[i_0 + (i << 2)] += T2[i << 2] * T1[i_0];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 1] * T1[i_0 + 4];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 2] * T1[i_0 + 8];
          T1_0[i_0 + (i << 2)] += T2[(i << 2) + 3] * T1[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_1[i_0 + (i << 2)] = 0.0;
          T1_1[i_0 + (i << 2)] += T3[i << 2] * T1_0[i_0];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 1] * T1_0[i_0 + 4];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 2] * T1_0[i_0 + 8];
          T1_1[i_0 + (i << 2)] += T3[(i << 2) + 3] * T1_0[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_0[i_0 + (i << 2)] = 0.0;
          T1_0[i_0 + (i << 2)] += T4[i << 2] * T1_1[i_0];
          T1_0[i_0 + (i << 2)] += T4[(i << 2) + 1] * T1_1[i_0 + 4];
          T1_0[i_0 + (i << 2)] += T4[(i << 2) + 2] * T1_1[i_0 + 8];
          T1_0[i_0 + (i << 2)] += T4[(i << 2) + 3] * T1_1[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        for (i = 0; i < 4; i++) {
          T1_1[i_0 + (i << 2)] = 0.0;
          T1_1[i_0 + (i << 2)] += T5[i << 2] * T1_0[i_0];
          T1_1[i_0 + (i << 2)] += T5[(i << 2) + 1] * T1_0[i_0 + 4];
          T1_1[i_0 + (i << 2)] += T5[(i << 2) + 2] * T1_0[i_0 + 8];
          T1_1[i_0 + (i << 2)] += T5[(i << 2) + 3] * T1_0[i_0 + 12];
        }
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        q = T1_1[i_0 + 12] + (T1_1[i_0 + 8] * 0.0 + (T1_1[i_0 + 4] * 0.0 +
          T1_1[i_0] * 0.0));
        wrist_2[i_0] = q;
      }

      for (i_0 = 0; i_0 < 4; i_0++) {
        wrist_vect_idx_2 = total_transform[i_0 + 12] + (total_transform[i_0 + 8]
          * 0.0 + (total_transform[i_0 + 4] * 0.0 + total_transform[i_0] * 0.0));
        end_effector_orig[i_0] = wrist_vect_idx_2;
      }

      /* find distance between end effector and elbow */
      /* end effector */
      p1_idx_0 = wrist_2[0];
      rtb_write_pos = wrist_2[1];
      rtb_statusflag_movej = wrist_2[2];

      /* wrist */
      p2_0[0] = end_effector_orig[0] - wrist_2[0];
      p2_0[1] = end_effector_orig[1] - wrist_2[1];
      p2_0[2] = end_effector_orig[2] - wrist_2[2];
      rtb_writehome = Arm_Controller_norm(p2_0);
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
      for (i_0 = 0; i_0 < 4; i_0++) {
        q = total_transform[i_0 + 12] + (total_transform[i_0 + 8] * 0.0 +
          (total_transform[i_0 + 4] * -0.072 + total_transform[i_0] * 0.0));
        wrist_2[i_0] = q;
      }

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
        q = T1[i_0 + 12] + (T1[i_0 + 8] * 0.135 + (T1[i_0 + 4] * 0.0 + T1[i_0] *
          0.0));
        wrist_2[i_0] = q;
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
      p1[1] = rtb_write_pos - 0.114 * wrist_vect_idx_1;
      p1[2] = rtb_statusflag_movej - 0.114 * wrist_vect_idx_2;
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
      if ((Arm_Control_DistBetween2Segment(p3offset, pwirestart, elbow_orig_0,
            shoulder_orig_0) < 0.1) || (Arm_Control_DistBetween2Segment(p2,
            wrist_back, elbow_orig_1, shoulder_orig_1) < 0.1) ||
          (Arm_Control_DistBetween2Segment(wrist_back, p1, elbow_orig_2,
            shoulder_orig_2) < 0.054) || (Arm_Control_DistBetween2Segment
           (pwirestart, p1offset, elbow_orig_3, shoulder_orig_3) < 0.054) ||
          (Arm_Control_DistBetween2Segment(p3offset, p1offset, wrist_2_0,
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
        /* '<S3>:1:72' */
        /* '<S3>:1:73' */
        collision_flag = -1;
      } else {
        /* '<S3>:1:76' */
        rtb_TmpSignalConversionAtSFunct[0] = -0.0;
        rtb_TmpSignalConversionAtSFunct[1] = -80.0;
        rtb_TmpSignalConversionAtSFunct[2] = j3;
        rtb_TmpSignalConversionAtSFunct[3] = j4;
        rtb_TmpSignalConversionAtSFunct[4] = 90.0 + rtb_outputs[0];
        rtb_TmpSignalConversionAtSFunct[5] = 0.0;

        /* '<S3>:1:77' */
        collision_flag = 1;

        /* %%%%%%% */
        /* jointangles = [-37.93 -173.74 180-j3 -j4 90+j5 0]; %adjusts the calculated joint angles so it matches the current robot configuration */
      }
    }
  } else {
    /* '<S3>:1:90' */
    collision_flag = 0;

    /* no movement carried out */
    /* '<S3>:1:91' */
    /* interpreted as doing nothing */
  }

  /* MATLAB Function: '<S1>/setpos' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read3'
   */
  /* MATLAB Function 'Arm_Controller/setpos': '<S7>:1' */
  if (rtb_enableflags_idx_1 == 1) {
    /* '<S7>:1:5' */
    if (rtb_outputs[0] == 1.0) {
      /* '<S7>:1:6' */
      /* replace 'home' with a integer */
      /* move to the defined home position */
      /* '<S7>:1:8' */
      rtb_jointangles[0] = 57.295779513082323 * Arm_Controller_DW.home_position
        [0];
      rtb_jointangles[1] = 57.295779513082323 * Arm_Controller_DW.home_position
        [1];
      rtb_jointangles[2] = 57.295779513082323 * Arm_Controller_DW.home_position
        [2];
      rtb_jointangles[3] = 57.295779513082323 * Arm_Controller_DW.home_position
        [3];
      rtb_jointangles[4] = 57.295779513082323 * Arm_Controller_DW.home_position
        [4];
      rtb_jointangles[5] = 57.295779513082323 * Arm_Controller_DW.home_position
        [5];

      /* jointangles = [0 0 0 0 0 0]; %need to read some sort of internal memory */
      /* '<S7>:1:10' */
      rtb_writehome = 1.0;
    } else if (rtb_outputs[0] == 2.0) {
      /* '<S7>:1:11' */
      /* move to giraffe position- replace with integer */
      /* '<S7>:1:12' */
      for (i_0 = 0; i_0 < 6; i_0++) {
        rtb_jointangles[i_0] = b[i_0];
      }

      /* '<S7>:1:13' */
      rtb_writehome = 1.0;
    } else {
      /* output error message */
      /* '<S7>:1:16' */
      rtb_writehome = -3.0;

      /* invalid argument */
      /* '<S7>:1:17' */
      for (i = 0; i < 6; i++) {
        rtb_jointangles[i] = 0.0;
      }

      /* interpreted at do nothing */
    }
  } else {
    /* '<S7>:1:20' */
    rtb_writehome = 0.0;

    /* function not executed */
    /* '<S7>:1:21' */
    for (i = 0; i < 6; i++) {
      rtb_jointangles[i] = 0.0;
    }
  }

  /* End of MATLAB Function: '<S1>/setpos' */

  /* MATLAB Function: '<S1>/move_joints' */
  /* MATLAB Function 'Arm_Controller/move_joints': '<S4>:1' */
  /* need to implement some collision checking */
  /* '<S4>:1:4' */
  for (i = 0; i < 6; i++) {
    rtb_jointangles_h[i] = 0.0;
  }

  if (rtb_enableflags_idx_3 == 1) {
    /* '<S4>:1:6' */
    /* check collisions */
    /* '<S4>:1:8' */
    rtb_statusflag_movej = 1.0;

    /* '<S4>:1:9' */
    rtb_jointangles_h[0] = rtb_outputs[0];

    /* '<S4>:1:10' */
    rtb_jointangles_h[1] = rtb_outputs[1];

    /* '<S4>:1:11' */
    rtb_jointangles_h[2] = rtb_outputs[2];

    /* '<S4>:1:12' */
    rtb_jointangles_h[3] = rtb_outputs[3];

    /* '<S4>:1:13' */
    rtb_jointangles_h[4] = rtb_outputs[4];

    /* '<S4>:1:14' */
    rtb_jointangles_h[5] = rtb_outputs[5];
  } else {
    /* '<S4>:1:16' */
    rtb_statusflag_movej = 0.0;
  }

  /* MATLAB Function 'Arm_Controller/sendcommand': '<S5>:1' */
  /* converts the joint angles into radians */
  /* if all of them are zero it is implied that no movement should occur */
  /* to give matlab the size of the variable */
  /* '<S5>:1:6' */
  for (i = 0; i < 6; i++) {
    /* MATLAB Function: '<S1>/sendcommand' */
    wrist_vect_idx_2 = rtb_jointangles_h[i];

    /* MultiPortSwitch: '<S1>/Multiport Switch' */
    switch (rtb_portoutput) {
     case 1:
      wrist_vect_idx_2 = rtb_TmpSignalConversionAtSFunct[i];
      break;

     case 2:
      wrist_vect_idx_2 = rtb_jointangles[i];
      break;
    }

    /* End of MultiPortSwitch: '<S1>/Multiport Switch' */

    /* MATLAB Function: '<S1>/sendcommand' */
    wrist_vect_idx_2 *= 0.017453292519943295;
    rtb_jointangles_h[i] = wrist_vect_idx_2;
  }

  /* End of MATLAB Function: '<S1>/move_joints' */

  /* MATLAB Function: '<S1>/sendcommand' */
  /* '<S5>:1:7' */
  p = false;
  b_p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 6)) {
    if (!(rtb_jointangles_h[i] == 0.0)) {
      b_p = false;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (b_p) {
    p = true;
  }

  if (p) {
    /* Outport: '<Root>/command_flag' */
    /* '<S5>:1:10' */
    Arm_Controller_Y.command_flag = 0.0;

    /* '<S5>:1:11' */
    rtb_write_pos = 2.0;
  } else {
    /* Outport: '<Root>/command_flag' */
    /* '<S5>:1:13' */
    Arm_Controller_Y.command_flag = 1.0;

    /* '<S5>:1:14' */
    rtb_write_pos = 1.0;
  }

  for (i = 0; i < 6; i++) {
    /* MultiPortSwitch: '<S1>/Multiport Switch1' incorporates:
     *  Memory: '<S1>/Memory'
     */
    if (rtb_write_pos == 1.0) {
      wrist_vect_idx_2 = rtb_jointangles_h[i];
    } else {
      wrist_vect_idx_2 = Arm_Controller_DW.Memory_PreviousInput[i];
    }

    /* DataStoreWrite: '<S1>/Data Store Write1' */
    Arm_Controller_DW.current_position[i] = wrist_vect_idx_2;

    /* MultiPortSwitch: '<S1>/Multiport Switch1' */
    rtb_TmpSignalConversionAtSFunct[i] = wrist_vect_idx_2;
  }

  /* MultiPortSwitch: '<S1>/Multiport Switch3' */
  switch (rtb_portoutput) {
   case 1:
    /* Outport: '<Root>/arm_status' incorporates:
     *  MATLAB Function: '<S1>/move_cam'
     */
    Arm_Controller_Y.arm_status = collision_flag;
    break;

   case 2:
    /* Outport: '<Root>/arm_status' */
    Arm_Controller_Y.arm_status = rtb_writehome;
    break;

   default:
    /* Outport: '<Root>/arm_status' */
    Arm_Controller_Y.arm_status = rtb_statusflag_movej;
    break;
  }

  for (i = 0; i < 6; i++) {
    /* Outport: '<Root>/joint_angle_vector' */
    Arm_Controller_Y.joint_angle_vector[i] = rtb_jointangles_h[i];

    /* Outport: '<Root>/arm_position' incorporates:
     *  DataStoreRead: '<S1>/Data Store Read1'
     */
    Arm_Controller_Y.arm_position[i] = Arm_Controller_DW.current_position[i];

    /* Update for Memory: '<S1>/Memory1' */
    Arm_Controller_DW.Memory1_PreviousInput[i] = rtb_MultiportSwitch2[i];

    /* Update for Memory: '<S1>/Memory' */
    Arm_Controller_DW.Memory_PreviousInput[i] =
      rtb_TmpSignalConversionAtSFunct[i];
  }

  /* End of MultiPortSwitch: '<S1>/Multiport Switch3' */
}

/* Model initialize function */
void Arm_Controller_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(Arm_Controller_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&Arm_Controller_DW, 0,
                sizeof(DW_Arm_Controller_T));

  /* external inputs */
  (void) memset((void *)&Arm_Controller_U, 0,
                sizeof(ExtU_Arm_Controller_T));

  /* external outputs */
  (void) memset((void *)&Arm_Controller_Y, 0,
                sizeof(ExtY_Arm_Controller_T));

  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      /* Start for DataStoreMemory: '<S1>/current_pos' */
      Arm_Controller_DW.current_position[i] =
        Arm_Controller_P.current_pos_InitialValue[i];

      /* Start for DataStoreMemory: '<S1>/home_pos' */
      Arm_Controller_DW.home_position[i] =
        Arm_Controller_P.home_pos_InitialValue[i];
    }
  }

  {
    int32_T i;
    for (i = 0; i < 6; i++) {
      /* InitializeConditions for Memory: '<S1>/Memory1' */
      Arm_Controller_DW.Memory1_PreviousInput[i] = Arm_Controller_P.Memory1_X0[i];

      /* InitializeConditions for Memory: '<S1>/Memory' */
      Arm_Controller_DW.Memory_PreviousInput[i] = Arm_Controller_P.Memory_X0[i];
    }
  }
}

/* Model terminate function */
void Arm_Controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
