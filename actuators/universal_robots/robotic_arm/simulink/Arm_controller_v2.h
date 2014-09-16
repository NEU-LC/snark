/*
 * File: Arm_controller_v2.h
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

#ifndef RTW_HEADER_Arm_controller_v2_h_
#define RTW_HEADER_Arm_controller_v2_h_
#include <float.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef Arm_controller_v2_COMMON_INCLUDES_
# define Arm_controller_v2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Arm_controller_v2_COMMON_INCLUDES_ */

#include "Arm_controller_v2_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Memory1_PreviousInput[6];     /* '<S1>/Memory1' */
  real_T home_position[6];             /* '<S1>/home_pos' */
} DW_Arm_controller_v2_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T motion_primitive;             /* '<Root>/motion_primitive' */
  real_T Input_1;                      /* '<Root>/Input_1' */
  real_T Input_2;                      /* '<Root>/Input_2' */
  real_T Input_3;                      /* '<Root>/Input_3' */
  real_T Input_4;                      /* '<Root>/Input_4' */
  real_T Input_5;                      /* '<Root>/Input_5' */
  real_T Input_6;                      /* '<Root>/Input_6' */
  real_T Joint1;                       /* '<Root>/Joint1' */
  real_T Joint2;                       /* '<Root>/Joint2' */
  real_T Joint3;                       /* '<Root>/Joint3' */
  real_T Joint4;                       /* '<Root>/Joint4' */
  real_T Joint5;                       /* '<Root>/Joint5' */
  real_T Joint6;                       /* '<Root>/Joint6' */
} ExtU_Arm_controller_v2_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T arm_status;                   /* '<Root>/arm_status' */
  real_T command_flag;                 /* '<Root>/command_flag' */
  real_T number_waypoints;             /* '<Root>/number_waypoints' */
  real_T WP1[6];                       /* '<Root>/WP1' */
  real_T WP2[6];                       /* '<Root>/WP2' */
  real_T WP3[6];                       /* '<Root>/WP3' */
  real_T WP4[6];                       /* '<Root>/WP4' */
  real_T WP5[6];                       /* '<Root>/WP5' */
  real_T WP6[6];                       /* '<Root>/WP6' */
} ExtY_Arm_controller_v2_T;

/* Parameters (auto storage) */
struct P_Arm_controller_v2_T_ {
  real_T Converttodegrees_Gain;        /* Expression: (180/pi)
                                        * Referenced by: '<S1>/Convert to degrees'
                                        */
  real_T Memory1_X0[6];                /* Expression: [0 -1.396 -2.611 0.8658 1.571 3.141]
                                        * Referenced by: '<S1>/Memory1'
                                        */
  real_T home_pos_InitialValue[6];     /* Expression: [0 -1.396 -2.611 0.8658 1.571 3.141]
                                        * Referenced by: '<S1>/home_pos'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Arm_controller_v2_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_Arm_controller_v2_T Arm_controller_v2_P;

/* Block states (auto storage) */
extern DW_Arm_controller_v2_T Arm_controller_v2_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_Arm_controller_v2_T Arm_controller_v2_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_Arm_controller_v2_T Arm_controller_v2_Y;

/* Model entry point functions */
extern void Arm_controller_v2_initialize(void);
extern void Arm_controller_v2_step(void);
extern void Arm_controller_v2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Arm_controller_v2_T *const Arm_controller_v2_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('fsm_v2/Arm_controller_v2')    - opens subsystem fsm_v2/Arm_controller_v2
 * hilite_system('fsm_v2/Arm_controller_v2/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'fsm_v2'
 * '<S1>'   : 'fsm_v2/Arm_controller_v2'
 * '<S2>'   : 'fsm_v2/Arm_controller_v2/MATLAB Function'
 * '<S3>'   : 'fsm_v2/Arm_controller_v2/functionselector'
 * '<S4>'   : 'fsm_v2/Arm_controller_v2/move_cam'
 * '<S5>'   : 'fsm_v2/Arm_controller_v2/move_cam2'
 * '<S6>'   : 'fsm_v2/Arm_controller_v2/move_effector'
 * '<S7>'   : 'fsm_v2/Arm_controller_v2/move_joints'
 * '<S8>'   : 'fsm_v2/Arm_controller_v2/sendcommand'
 * '<S9>'   : 'fsm_v2/Arm_controller_v2/set_home'
 * '<S10>'  : 'fsm_v2/Arm_controller_v2/set_pos2'
 * '<S11>'  : 'fsm_v2/Arm_controller_v2/setpos'
 */
#endif                                 /* RTW_HEADER_Arm_controller_v2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
