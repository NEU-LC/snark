/*
 * File: Arm_Controller.c
 *
 * Code generated for Simulink model 'Arm_Controller'.
 *
 * Model version                  : 1.92
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Fri Jun 13 11:52:31 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Arm_Controller.h"
#include "Arm_Controller_private.h"

/* External inputs (root inport signals with auto storage) */
ExtU_Arm_Controller_T Arm_Controller_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_Arm_Controller_T Arm_Controller_Y;

/* Real-time model */
RT_MODEL_Arm_Controller_T Arm_Controller_M_;
RT_MODEL_Arm_Controller_T *const Arm_Controller_M = &Arm_Controller_M_;

/* Model step function */
void Arm_Controller_step(void)
{
  boolean_T p;
  boolean_T b_p;
  boolean_T exitg1;
  int32_T rtb_portoutput;
  real_T rtb_jointangles[6];
  real_T rtb_jointangles_h[6];
  int32_T rtb_enableflags_idx_0;
  int32_T rtb_enableflags_idx_1;
  real_T rtb_outputs_idx_0;
  real_T rtb_outputs_idx_1;
  real_T rtb_outputs_idx_2;

  /* MATLAB Function: '<S1>/functionselector' incorporates:
   *  Inport: '<Root>/Input_1'
   *  Inport: '<Root>/Input_2'
   *  Inport: '<Root>/Input_3'
   *  Inport: '<Root>/motion_primitive'
   *  SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport2'
   */
  /* MATLAB Function 'Arm_Controller/functionselector': '<S2>:1' */
  /* '<S2>:1:4' */
  rtb_enableflags_idx_0 = 0;
  rtb_enableflags_idx_1 = 0;

  /* '<S2>:1:5' */
  rtb_outputs_idx_0 = 0.0;
  rtb_outputs_idx_1 = 0.0;
  rtb_outputs_idx_2 = 0.0;
  if (Arm_Controller_U.motion_primitive == 1.0) {
    /* '<S2>:1:7' */
    /* activate move_cam and disable home position. Can expand concept to */
    /* other functions */
    /* '<S2>:1:10' */
    rtb_enableflags_idx_0 = 1;

    /* '<S2>:1:11' */
    /* transfer the inputs received to the inputs of the move_cam function */
    /* '<S2>:1:14' */
    rtb_outputs_idx_0 = Arm_Controller_U.Input_1;

    /* '<S2>:1:15' */
    rtb_outputs_idx_1 = Arm_Controller_U.Input_2;

    /* '<S2>:1:16' */
    rtb_outputs_idx_2 = Arm_Controller_U.Input_3;

    /* '<S2>:1:17' */
    rtb_portoutput = 1;
  } else if (Arm_Controller_U.motion_primitive == 2.0) {
    /* '<S2>:1:19' */
    /* activate move to home/giraffe position */
    /* '<S2>:1:21' */
    /* '<S2>:1:22' */
    rtb_enableflags_idx_1 = 1;

    /* '<S2>:1:23' */
    rtb_outputs_idx_0 = Arm_Controller_U.Input_1;

    /* '<S2>:1:24' */
    rtb_portoutput = 2;
  } else {
    /* reject input and dont do anything */
    /* '<S2>:1:27' */
    rtb_portoutput = 1;
  }

  /* End of MATLAB Function: '<S1>/functionselector' */

  /* MATLAB Function: '<S1>/setpos' */
  /* MATLAB Function 'Arm_Controller/setpos': '<S5>:1' */
  if (rtb_enableflags_idx_1 == 1) {
    /* Outport: '<Root>/status_setpos' */
    /* '<S5>:1:5' */
    /* output error message */
    /* '<S5>:1:15' */
    Arm_Controller_Y.status_setpos = -3.0;

    /* invalid argument */
    /* '<S5>:1:16' */
    for (rtb_enableflags_idx_1 = 0; rtb_enableflags_idx_1 < 6;
         rtb_enableflags_idx_1++) {
      rtb_jointangles[rtb_enableflags_idx_1] = 0.0;
    }
  } else {
    /* Outport: '<Root>/status_setpos' */
    /* '<S5>:1:19' */
    Arm_Controller_Y.status_setpos = 0.0;

    /* function not executed */
    /* '<S5>:1:20' */
    for (rtb_enableflags_idx_1 = 0; rtb_enableflags_idx_1 < 6;
         rtb_enableflags_idx_1++) {
      rtb_jointangles[rtb_enableflags_idx_1] = 0.0;
    }
  }

  /* End of MATLAB Function: '<S1>/setpos' */

  /* MATLAB Function: '<S1>/move_cam' */
  /* MATLAB Function 'Arm_Controller/move_cam': '<S3>:1' */
  /* '<S3>:1:4' */
  for (rtb_enableflags_idx_1 = 0; rtb_enableflags_idx_1 < 6;
       rtb_enableflags_idx_1++) {
    rtb_jointangles_h[rtb_enableflags_idx_1] = 0.0;
  }

  /* j1 and j2 will ultimately use the home position of the robot as reference */
  /* this angle orients the base towards the user */
  /* '<S3>:1:7' */
  if (rtb_enableflags_idx_0 == 1) {
    /* '<S3>:1:9' */
    /* '<S3>:1:10' */
    /* max possible height of the end effector */
    if (rtb_outputs_idx_2 > 0.43834215672022442) {
      /* '<S3>:1:12' */
      /* catch invalid height */
      /* disp('The specified height is larger than the maximum possible height for this configuration. Using a height of: ') */
      /* disp(max_height) */
      /* statusflag = -2; %out of target height */
      /* '<S3>:1:16' */
      rtb_outputs_idx_2 = 0.43834215672022442;
    }

    /* '<S3>:1:19' */
    rtb_outputs_idx_2 = asin((rtb_outputs_idx_2 - 0.046342156720224424) / 0.392)
      * 57.295779513082323;

    /* calculates the elbow joint angle needed to reach the required height. works properly */
    /* '<S3>:1:20' */
    rtb_outputs_idx_1 = ((rtb_outputs_idx_1 - 83.740000000000009) -
                         (6.2599999999999909 + rtb_outputs_idx_2)) + 180.0;

    /* joint angle to apply to achieve target tilt position. works properly */
    /* assumption. good assumption */
    /* not important at the moment */
    /* very basic check to see if the robot will collide with itself */
    if ((rtb_outputs_idx_1 > 205.0) || (180.0 - (6.2599999999999909 +
          rtb_outputs_idx_2) > 155.0) || (90.0 + rtb_outputs_idx_0 > 140.0)) {
      /* '<S3>:1:25' */
      /* found these upper limits by observation */
      /* disp('Movement will cause collision') */
      /* '<S3>:1:27' */
      rtb_outputs_idx_2 = -1.0;
    } else {
      /* '<S3>:1:31' */
      rtb_jointangles_h[0] = -37.93;
      rtb_jointangles_h[1] = -173.74;
      rtb_jointangles_h[2] = 180.0 - (6.2599999999999909 + rtb_outputs_idx_2);
      rtb_jointangles_h[3] = -rtb_outputs_idx_1;
      rtb_jointangles_h[4] = 90.0 + rtb_outputs_idx_0;
      rtb_jointangles_h[5] = 0.0;

      /* adjusts the calculated joint angles so it matches the current robot configuration */
      /* '<S3>:1:32' */
      rtb_outputs_idx_2 = 1.0;
    }
  } else {
    /* '<S3>:1:35' */
    rtb_outputs_idx_2 = 0.0;

    /* no movement carried out */
    /* '<S3>:1:36' */
    for (rtb_enableflags_idx_1 = 0; rtb_enableflags_idx_1 < 6;
         rtb_enableflags_idx_1++) {
      rtb_jointangles_h[rtb_enableflags_idx_1] = 0.0;
    }

    /* interpreted as doing nothing */
  }

  /* MATLAB Function 'Arm_Controller/sendcommand': '<S4>:1' */
  /* converts the joint angles into radians */
  /* if all of them are zero it is implied that no movement should occur */
  /* to give matlab the size of the variable */
  /* '<S4>:1:6' */
  for (rtb_enableflags_idx_1 = 0; rtb_enableflags_idx_1 < 6;
       rtb_enableflags_idx_1++) {
    /* MATLAB Function: '<S1>/sendcommand' */
    rtb_outputs_idx_1 = rtb_jointangles[rtb_enableflags_idx_1];

    /* MultiPortSwitch: '<S1>/Multiport Switch' */
    if (rtb_portoutput == 1) {
      rtb_outputs_idx_1 = rtb_jointangles_h[rtb_enableflags_idx_1];
    }

    /* End of MultiPortSwitch: '<S1>/Multiport Switch' */

    /* MATLAB Function: '<S1>/sendcommand' */
    rtb_outputs_idx_1 *= 0.017453292519943295;
    rtb_jointangles[rtb_enableflags_idx_1] = rtb_outputs_idx_1;
  }

  /* End of MATLAB Function: '<S1>/move_cam' */

  /* MATLAB Function: '<S1>/sendcommand' */
  /* '<S4>:1:7' */
  p = false;
  b_p = true;
  rtb_enableflags_idx_1 = 0;
  exitg1 = false;
  while ((!exitg1) && (rtb_enableflags_idx_1 < 6)) {
    if (!(rtb_jointangles[rtb_enableflags_idx_1] == 0.0)) {
      b_p = false;
      exitg1 = true;
    } else {
      rtb_enableflags_idx_1++;
    }
  }

  if (b_p) {
    p = true;
  }

  if (p) {
    /* Outport: '<Root>/command_flag' */
    /* '<S4>:1:10' */
    Arm_Controller_Y.command_flag = 0.0;
  } else {
    /* Outport: '<Root>/command_flag' */
    /* '<S4>:1:13' */
    Arm_Controller_Y.command_flag = 1.0;
  }

  /* Outport: '<Root>/joint_angle_vector' */
  for (rtb_enableflags_idx_1 = 0; rtb_enableflags_idx_1 < 6;
       rtb_enableflags_idx_1++) {
    Arm_Controller_Y.joint_angle_vector[rtb_enableflags_idx_1] =
      rtb_jointangles[rtb_enableflags_idx_1];
  }

  /* End of Outport: '<Root>/joint_angle_vector' */

  /* Outport: '<Root>/status_movecam' */
  Arm_Controller_Y.status_movecam = rtb_outputs_idx_2;
}

/* Model initialize function */
void Arm_Controller_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(Arm_Controller_M, (NULL));

  /* external inputs */
  (void) memset((void *)&Arm_Controller_U, 0,
                sizeof(ExtU_Arm_Controller_T));

  /* external outputs */
  (void) memset((void *)&Arm_Controller_Y, 0,
                sizeof(ExtY_Arm_Controller_T));
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
