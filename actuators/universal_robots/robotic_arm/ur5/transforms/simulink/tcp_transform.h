/*
 * File: tcp_transform.h
 *
 * Code generated for Simulink model 'tcp_transform'.
 *
 * Model version                  : 1.13
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Thu Aug 07 16:00:31 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_tcp_transform_h_
#define RTW_HEADER_tcp_transform_h_
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef tcp_transform_COMMON_INCLUDES_
# define tcp_transform_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* tcp_transform_COMMON_INCLUDES_ */

#include "tcp_transform_types.h"
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

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T Joint1;                       /* '<Root>/Joint1' */
  real_T Joint2;                       /* '<Root>/Joint2' */
  real_T Joint3;                       /* '<Root>/Joint3' */
  real_T Joint4;                       /* '<Root>/Joint4' */
  real_T Joint5;                       /* '<Root>/Joint5' */
  real_T Joint6;                       /* '<Root>/Joint6' */
} ExtU_tcp_transform_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T x;                            /* '<Root>/x' */
  real_T y;                            /* '<Root>/y' */
  real_T z;                            /* '<Root>/z' */
  real_T Pan;                          /* '<Root>/Pan' */
  real_T Tilt;                         /* '<Root>/Tilt' */
  real_T Roll;                         /* '<Root>/Roll' */
  real_T x_laser;                      /* '<Root>/x_laser' */
  real_T y_laser;                      /* '<Root>/y_laser' */
  real_T z_laser;                      /* '<Root>/z_laser' */
  real_T pan_laser;                    /* '<Root>/pan_laser' */
  real_T tilt_laser;                   /* '<Root>/tilt_laser' */
  real_T roll_laser;                   /* '<Root>/roll_laser' */
} ExtY_tcp_transform_T;

/* Real-time Model Data Structure */
struct tag_RTM_tcp_transform_T {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with auto storage) */
extern ExtU_tcp_transform_T tcp_transform_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_tcp_transform_T tcp_transform_Y;

/* Model entry point functions */
extern void tcp_transform_initialize(void);
extern void tcp_transform_step(void);
extern void tcp_transform_terminate(void);

/* Real-time Model object */
extern RT_MODEL_tcp_transform_T *const tcp_transform_M;

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
 * hilite_system('laser_func/tcp_transform')    - opens subsystem laser_func/tcp_transform
 * hilite_system('laser_func/tcp_transform/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'laser_func'
 * '<S1>'   : 'laser_func/tcp_transform'
 * '<S2>'   : 'laser_func/tcp_transform/getToolLaser'
 */
#endif                                 /* RTW_HEADER_tcp_transform_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
