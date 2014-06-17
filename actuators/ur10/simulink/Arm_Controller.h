/*
 * File: Arm_Controller.h
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

#ifndef RTW_HEADER_Arm_Controller_h_
#define RTW_HEADER_Arm_Controller_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef Arm_Controller_COMMON_INCLUDES_
# define Arm_Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Arm_Controller_COMMON_INCLUDES_ */

#include "Arm_Controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T motion_primitive;             /* '<Root>/motion_primitive' */
  real_T Input_1;                      /* '<Root>/Input_1' */
  real_T Input_2;                      /* '<Root>/Input_2' */
  real_T Input_3;                      /* '<Root>/Input_3' */
} ExtU_Arm_Controller_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T command_flag;                 /* '<Root>/command_flag' */
  real_T joint_angle_vector[6];        /* '<Root>/joint_angle_vector' */
  real_T status_movecam;               /* '<Root>/status_movecam' */
  real_T status_setpos;                /* '<Root>/status_setpos' */
} ExtY_Arm_Controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_Arm_Controller_T {
  const char_T * volatile errorStatus;
};

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
 * '<S2>'   : 'fsm_toplevel/Arm_Controller/functionselector'
 * '<S3>'   : 'fsm_toplevel/Arm_Controller/move_cam'
 * '<S4>'   : 'fsm_toplevel/Arm_Controller/sendcommand'
 * '<S5>'   : 'fsm_toplevel/Arm_Controller/setpos'
 */
#endif                                 /* RTW_HEADER_Arm_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
