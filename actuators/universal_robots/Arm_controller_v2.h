// this is a leftover from simulink

#ifndef RTW_HEADER_Arm_controller_v2_h_
#define RTW_HEADER_Arm_controller_v2_h_

struct ExtU_Arm_controller_v2_T {
  double motion_primitive;             /* '<Root>/motion_primitive' */
  double Input_1;                      /* '<Root>/Input_1' */
  double Input_2;                      /* '<Root>/Input_2' */
  double Input_3;                      /* '<Root>/Input_3' */
  double Input_4;                      /* '<Root>/Input_4' */
  double Input_5;                      /* '<Root>/Input_5' */
  double Input_6;                      /* '<Root>/Input_6' */
  double Joint1;                       /* '<Root>/Joint1' */
  double Joint2;                       /* '<Root>/Joint2' */
  double Joint3;                       /* '<Root>/Joint3' */
  double Joint4;                       /* '<Root>/Joint4' */
  double Joint5;                       /* '<Root>/Joint5' */
  double Joint6;                       /* '<Root>/Joint6' */
};

struct ExtY_Arm_controller_v2_T {
  double arm_status;                   /* '<Root>/arm_status' */
  double command_flag;                 /* '<Root>/command_flag' */
  double number_waypoints;             /* '<Root>/number_waypoints' */
  double WP1[6];                       /* '<Root>/WP1' */
  double WP2[6];                       /* '<Root>/WP2' */
  double WP3[6];                       /* '<Root>/WP3' */
  double WP4[6];                       /* '<Root>/WP4' */
  double WP5[6];                       /* '<Root>/WP5' */
  double WP6[6];                       /* '<Root>/WP6' */
};

inline void Arm_controller_v2_initialize() {}
inline void Arm_controller_v2_terminate() {}
inline void Arm_controller_v2_step() {}

#endif // RTW_HEADER_Arm_controller_v2_h_
