/*
 * File: Arm_Controller.h
 *
 * Code generated for Simulink model 'Arm_Controller'.
 *
 * Model version                  : 1.167
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Tue Jul 29 17:02:52 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Arm_Controller_h_
#define RTW_HEADER_Arm_Controller_h_
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef Arm_Controller_COMMON_INCLUDES_
# define Arm_Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Arm_Controller_COMMON_INCLUDES_ */

#include "Arm_Controller_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
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
  real_T Memory_PreviousInput[6];      /* '<S1>/Memory' */
  real_T current_position[6];          /* '<S1>/current_pos' */
  real_T home_position[6];             /* '<S1>/home_pos' */
} DW_Arm_Controller_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T motion_primitive;             /* '<Root>/motion_primitive' */
  real_T Input_1;                      /* '<Root>/Input_1' */
  real_T Input_2;                      /* '<Root>/Input_2' */
  real_T Input_3;                      /* '<Root>/Input_3' */
  real_T Input_4;                      /* '<Root>/Input_4' */
  real_T Input_5;                      /* '<Root>/Input_5' */
  real_T Input_6;                      /* '<Root>/Input_6' */
} ExtU_Arm_Controller_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T command_flag;                 /* '<Root>/command_flag' */
  real_T joint_angle_vector[6];        /* '<Root>/joint_angle_vector' */
  real_T arm_status;                   /* '<Root>/arm_status' */
  real_T arm_position[6];              /* '<Root>/arm_position' */
} ExtY_Arm_Controller_T;

/* Parameters (auto storage) */
struct P_Arm_Controller_T_ {
  real_T Memory1_X0[6];                /* Expression: [0 -1.396 -2.611 0.8658 1.571 0]
                                        * Referenced by: '<S1>/Memory1'
                                        */
  real_T Memory_X0[6];                 /* Expression: [0 0 0 0 0 0]
                                        * Referenced by: '<S1>/Memory'
                                        */
  real_T current_pos_InitialValue[6];  /* Expression: [0 0 0 0 0 0]
                                        * Referenced by: '<S1>/current_pos'
                                        */
  real_T home_pos_InitialValue[6];     /* Expression: [0 -1.396 -2.611 0.8658 1.571 0]
                                        * Referenced by: '<S1>/home_pos'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Arm_Controller_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_Arm_Controller_T Arm_Controller_P;

/* Block states (auto storage) */
extern DW_Arm_Controller_T Arm_Controller_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_Arm_Controller_T Arm_Controller_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_Arm_Controller_T Arm_Controller_Y;

/* Model entry point functions */
extern void Arm_Controller_initialize(void);
extern void Arm_Controller_step(void);
extern void Arm_Controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Arm_Controller_T *const Arm_Controller_M;

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
 * hilite_system('fsm_toplevel/Arm_Controller')    - opens subsystem fsm_toplevel/Arm_Controller
 * hilite_system('fsm_toplevel/Arm_Controller/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'fsm_toplevel'
 * '<S1>'   : 'fsm_toplevel/Arm_Controller'
 * '<S2>'   : 'fsm_toplevel/Arm_Controller/MATLAB Function'
 * '<S3>'   : 'fsm_toplevel/Arm_Controller/functionselector'
 * '<S4>'   : 'fsm_toplevel/Arm_Controller/move_cam'
 * '<S5>'   : 'fsm_toplevel/Arm_Controller/sendcommand'
 * '<S6>'   : 'fsm_toplevel/Arm_Controller/set_home'
 * '<S7>'   : 'fsm_toplevel/Arm_Controller/setpos'
 */
#endif                                 /* RTW_HEADER_Arm_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
