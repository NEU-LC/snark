/*
 * File: Arm_Controller.c
 *
 * Code generated for Simulink model 'Arm_Controller'.
 *
 * Model version                  : 1.133
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Thu Jun 26 15:25:05 2014
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

/* Model step function */
void Arm_Controller_step(void)
{
  real_T j4;
  static const int8_T b[6] = { 0, -90, 0, -90, 0, 0 };

  boolean_T p;
  boolean_T b_p;
  boolean_T exitg1;
  real_T rtb_writehome;
  real_T rtb_MultiportSwitch2[6];
  int32_T rtb_portoutput;
  int32_T rtb_write_pos;
  int32_T rtb_statusflag_movej;
  real_T rtb_TmpSignalConversionAtSFunct[6];
  real_T rtb_outputs[6];
  real_T rtb_jointangles_h[6];
  real_T rtb_jointangles[6];
  int32_T i;
  int8_T rtb_enableflags_idx_0;
  int8_T rtb_enableflags_idx_1;
  int8_T rtb_enableflags_idx_2;
  int8_T rtb_enableflags_idx_3;
  real_T rtb_TmpSignalConversionAtSFun_h;

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
  /* MATLAB Function 'Arm_Controller/functionselector': '<S3>:1' */
  /* '<S3>:1:4' */
  rtb_enableflags_idx_0 = 0;
  rtb_enableflags_idx_1 = 0;
  rtb_enableflags_idx_2 = 0;
  rtb_enableflags_idx_3 = 0;

  /* '<S3>:1:5' */
  for (i = 0; i < 6; i++) {
    rtb_outputs[i] = 0.0;
  }

  /* '<S3>:1:7' */
  rtb_portoutput = 2;
  if (Arm_Controller_U.motion_primitive == 1.0) {
    /* '<S3>:1:9' */
    /* activate move_cam and disable home position. Can expand concept to */
    /* other functions */
    /* '<S3>:1:12' */
    rtb_enableflags_idx_0 = 1;

    /* transfer the inputs received to the inputs of the move_cam function */
    /* '<S3>:1:15' */
    rtb_outputs[0] = Arm_Controller_U.Input_1;

    /* '<S3>:1:16' */
    rtb_outputs[1] = Arm_Controller_U.Input_2;

    /* '<S3>:1:17' */
    rtb_outputs[2] = Arm_Controller_U.Input_3;

    /* '<S3>:1:18' */
    rtb_portoutput = 1;
  } else if (Arm_Controller_U.motion_primitive == 2.0) {
    /* '<S3>:1:20' */
    /* activate move to home/giraffe position */
    /* '<S3>:1:22' */
    rtb_enableflags_idx_1 = 1;

    /* '<S3>:1:24' */
    rtb_outputs[0] = Arm_Controller_U.Input_1;

    /* '<S3>:1:25' */
  } else if (Arm_Controller_U.motion_primitive == 3.0) {
    /* '<S3>:1:27' */
    /* '<S3>:1:28' */
    rtb_enableflags_idx_2 = 1;

    /* set home position */
  } else if (Arm_Controller_U.motion_primitive == 4.0) {
    /* '<S3>:1:31' */
    /* movej function. full control of arm motion.  */
    /* '<S3>:1:33' */
    rtb_enableflags_idx_3 = 1;

    /* '<S3>:1:34' */
    rtb_outputs[0] = Arm_Controller_U.Input_1;

    /* '<S3>:1:35' */
    rtb_outputs[1] = Arm_Controller_U.Input_2;

    /* '<S3>:1:36' */
    rtb_outputs[2] = Arm_Controller_U.Input_3;

    /* '<S3>:1:37' */
    rtb_outputs[3] = Arm_Controller_U.Input_4;

    /* '<S3>:1:38' */
    rtb_outputs[4] = Arm_Controller_U.Input_5;

    /* '<S3>:1:39' */
    rtb_outputs[5] = Arm_Controller_U.Input_6;

    /* '<S3>:1:40' */
    rtb_portoutput = 3;
  } else {
    /* reject input and dont do anything */
    /* '<S3>:1:43' */
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
      rtb_TmpSignalConversionAtSFun_h = Arm_Controller_DW.current_position[i];
    } else {
      rtb_TmpSignalConversionAtSFun_h =
        Arm_Controller_DW.Memory1_PreviousInput[i];
    }

    /* DataStoreWrite: '<S1>/Data Store Write' */
    Arm_Controller_DW.home_position[i] = rtb_TmpSignalConversionAtSFun_h;

    /* MATLAB Function: '<S1>/move_cam' */
    rtb_TmpSignalConversionAtSFunct[i] = 0.0;

    /* MultiPortSwitch: '<S1>/Multiport Switch2' */
    rtb_MultiportSwitch2[i] = rtb_TmpSignalConversionAtSFun_h;
  }

  /* End of MATLAB Function: '<S1>/set_home' */

  /* MATLAB Function: '<S1>/move_cam' */
  rtb_writehome = rtb_outputs[2];

  /* MATLAB Function 'Arm_Controller/move_cam': '<S4>:1' */
  /* '<S4>:1:4' */
  /* j1 and j2 will ultimately use the home position of the robot as reference */
  /* this angle orients the base towards the user */
  /* '<S4>:1:7' */
  if (rtb_enableflags_idx_0 == 1) {
    /* '<S4>:1:9' */
    /* '<S4>:1:10' */
    /* max possible height of the end effector */
    if (rtb_outputs[2] > 0.43834215672022442) {
      /* '<S4>:1:12' */
      /* catch invalid height */
      /* disp('The specified height is larger than the maximum possible height for this configuration. Using a height of: ') */
      /* disp(max_height) */
      /* statusflag = -2; %out of target height */
      /* '<S4>:1:16' */
      rtb_writehome = 0.43834215672022442;
    }

    /* '<S4>:1:19' */
    rtb_writehome = asin((rtb_writehome - 0.046342156720224424) / 0.392) *
      57.295779513082323;

    /* calculates the elbow joint angle needed to reach the required height. works properly */
    /* '<S4>:1:20' */
    j4 = ((rtb_outputs[1] - 83.740000000000009) - (6.2599999999999909 +
           rtb_writehome)) + 180.0;

    /* joint angle to apply to achieve target tilt position. works properly */
    /* assumption. good assumption */
    /* not important at the moment */
    /* very basic check to see if the robot will collide with itself */
    if ((j4 > 205.0) || (180.0 - (6.2599999999999909 + rtb_writehome) > 155.0) ||
        (90.0 + rtb_outputs[0] > 140.0)) {
      /* '<S4>:1:25' */
      /* found these upper limits by observation */
      /* disp('Movement will cause collision') */
      /* '<S4>:1:27' */
      rtb_writehome = -1.0;
    } else {
      /* '<S4>:1:31' */
      rtb_TmpSignalConversionAtSFunct[0] = -37.93;
      rtb_TmpSignalConversionAtSFunct[1] = -173.74;
      rtb_TmpSignalConversionAtSFunct[2] = 180.0 - (6.2599999999999909 +
        rtb_writehome);
      rtb_TmpSignalConversionAtSFunct[3] = -j4;
      rtb_TmpSignalConversionAtSFunct[4] = 90.0 + rtb_outputs[0];
      rtb_TmpSignalConversionAtSFunct[5] = 0.0;

      /* adjusts the calculated joint angles so it matches the current robot configuration */
      /* '<S4>:1:32' */
      rtb_writehome = 1.0;
    }
  } else {
    /* '<S4>:1:35' */
    rtb_writehome = 0.0;

    /* no movement carried out */
    /* '<S4>:1:36' */
    for (i = 0; i < 6; i++) {
      rtb_TmpSignalConversionAtSFunct[i] = 0.0;
    }

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
      j4 = 1.0;
    } else if (rtb_outputs[0] == 2.0) {
      /* '<S7>:1:11' */
      /* move to giraffe position- replace with integer */
      /* '<S7>:1:12' */
      for (i = 0; i < 6; i++) {
        rtb_jointangles[i] = b[i];
      }

      /* '<S7>:1:13' */
      j4 = 1.0;
    } else {
      /* output error message */
      /* '<S7>:1:16' */
      j4 = -3.0;

      /* invalid argument */
      /* '<S7>:1:17' */
      for (i = 0; i < 6; i++) {
        rtb_jointangles[i] = 0.0;
      }

      /* interpreted at do nothing */
    }
  } else {
    /* '<S7>:1:20' */
    j4 = 0.0;

    /* function not executed */
    /* '<S7>:1:21' */
    for (i = 0; i < 6; i++) {
      rtb_jointangles[i] = 0.0;
    }
  }

  /* End of MATLAB Function: '<S1>/setpos' */

  /* MATLAB Function: '<S1>/MATLAB Function' */
  /* MATLAB Function 'Arm_Controller/MATLAB Function': '<S2>:1' */
  /* need to implement some collision checking */
  /* '<S2>:1:4' */
  for (i = 0; i < 6; i++) {
    rtb_jointangles_h[i] = 0.0;
  }

  if (rtb_enableflags_idx_3 == 1) {
    /* '<S2>:1:6' */
    /* check collisions */
    /* '<S2>:1:8' */
    rtb_statusflag_movej = 1;

    /* '<S2>:1:9' */
    rtb_jointangles_h[0] = rtb_outputs[0];

    /* '<S2>:1:10' */
    rtb_jointangles_h[1] = rtb_outputs[1];

    /* '<S2>:1:11' */
    rtb_jointangles_h[2] = rtb_outputs[2];

    /* '<S2>:1:12' */
    rtb_jointangles_h[3] = rtb_outputs[3];

    /* '<S2>:1:13' */
    rtb_jointangles_h[4] = rtb_outputs[4];

    /* '<S2>:1:14' */
    rtb_jointangles_h[5] = rtb_outputs[5];
  } else {
    /* '<S2>:1:16' */
    rtb_statusflag_movej = 0;
  }

  /* MATLAB Function 'Arm_Controller/sendcommand': '<S5>:1' */
  /* converts the joint angles into radians */
  /* if all of them are zero it is implied that no movement should occur */
  /* to give matlab the size of the variable */
  /* '<S5>:1:6' */
  for (i = 0; i < 6; i++) {
    /* MATLAB Function: '<S1>/sendcommand' */
    rtb_TmpSignalConversionAtSFun_h = rtb_jointangles_h[i];

    /* MultiPortSwitch: '<S1>/Multiport Switch' */
    switch (rtb_portoutput) {
     case 1:
      rtb_TmpSignalConversionAtSFun_h = rtb_TmpSignalConversionAtSFunct[i];
      break;

     case 2:
      rtb_TmpSignalConversionAtSFun_h = rtb_jointangles[i];
      break;
    }

    /* End of MultiPortSwitch: '<S1>/Multiport Switch' */

    /* MATLAB Function: '<S1>/sendcommand' */
    rtb_TmpSignalConversionAtSFun_h *= 0.017453292519943295;
    rtb_jointangles_h[i] = rtb_TmpSignalConversionAtSFun_h;
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function' */

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
    rtb_write_pos = 2;
  } else {
    /* Outport: '<Root>/command_flag' */
    /* '<S5>:1:13' */
    Arm_Controller_Y.command_flag = 1.0;

    /* '<S5>:1:14' */
    rtb_write_pos = 1;
  }

  for (i = 0; i < 6; i++) {
    /* MultiPortSwitch: '<S1>/Multiport Switch1' incorporates:
     *  Memory: '<S1>/Memory'
     */
    if (rtb_write_pos == 1) {
      rtb_TmpSignalConversionAtSFun_h = rtb_jointangles_h[i];
    } else {
      rtb_TmpSignalConversionAtSFun_h = Arm_Controller_DW.Memory_PreviousInput[i];
    }

    /* DataStoreWrite: '<S1>/Data Store Write1' */
    Arm_Controller_DW.current_position[i] = rtb_TmpSignalConversionAtSFun_h;

    /* MultiPortSwitch: '<S1>/Multiport Switch1' */
    rtb_TmpSignalConversionAtSFunct[i] = rtb_TmpSignalConversionAtSFun_h;
  }

  /* MultiPortSwitch: '<S1>/Multiport Switch3' */
  switch (rtb_portoutput) {
   case 1:
    /* Outport: '<Root>/arm_status' */
    Arm_Controller_Y.arm_status = rtb_writehome;
    break;

   case 2:
    /* Outport: '<Root>/arm_status' */
    Arm_Controller_Y.arm_status = j4;
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
