//
//  robotROSFeedbackControlExample.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "robotROSFeedbackControlExample".
//
//  Model version              : 1.78
//  Simulink Coder version : 9.0 (R2018b) 24-May-2018
//  C++ source code generated on : Sun Feb 17 11:38:35 2019
//
//  Target selection: ert.tlc
//  Embedded hardware selection: ARM Compatible->ARM Cortex
//  Code generation objectives: Unspecified
//  Validation result: Not run


#ifndef RTW_HEADER_robotROSFeedbackControlExample_h_
#define RTW_HEADER_robotROSFeedbackControlExample_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef robotROSFeedbackControlExample_COMMON_INCLUDES_
# define robotROSFeedbackControlExample_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 // robotROSFeedbackControlExample_COMMON_INCLUDES_ 

#include "robotROSFeedbackControlExample_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define robotROSFeedbackControlExample_M (robotROSFeedbackControlExamp_M)

// Block signals (default storage)
typedef struct {
  SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry In1;// '<S10>/In1'
  SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry b_varargout_2;
  SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist BusAssignment1;// '<S1>/Bus Assignment1' 
  real_T Sum1;                         // '<S3>/Sum1'
} B_robotROSFeedbackControlExam_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slros_internal_block_T obj; // '<S6>/SinkBlock'
  robotics_slros_internal_blo_j_T obj_n;// '<S4>/SourceBlock'
} DW_robotROSFeedbackControlExa_T;

// Parameters (default storage)
struct P_robotROSFeedbackControlExam_T_ {
  real_T DistanceThreshold_const;      // Mask Parameter: DistanceThreshold_const
                                       //  Referenced by: '<S7>/Constant'

  real_T GainSlider_gain;              // Mask Parameter: GainSlider_gain
                                       //  Referenced by: '<S8>/Slider Gain'

  real_T LinearVelocitySlider_gain;    // Mask Parameter: LinearVelocitySlider_gain
                                       //  Referenced by: '<S9>/Slider Gain'

  SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry Out1_Y0;// Computed Parameter: Out1_Y0
                                                                  //  Referenced by: '<S10>/Out1'

  SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S4>/Constant'

  SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                      //  Referenced by: '<S5>/Constant'

  real_T Saturation_UpperSat;          // Expression: 0.5
                                       //  Referenced by: '<S3>/Saturation'

  real_T Saturation_LowerSat;          // Expression: -0.5
                                       //  Referenced by: '<S3>/Saturation'

  real_T LinearVelocityv_Y0;           // Computed Parameter: LinearVelocityv_Y0
                                       //  Referenced by: '<S3>/Linear Velocity (v)'

  real_T AngularVelocityw_Y0;          // Computed Parameter: AngularVelocityw_Y0
                                       //  Referenced by: '<S3>/Angular Velocity (w)'

  real_T Constant_Value_m;             // Expression: 1
                                       //  Referenced by: '<S3>/Constant'

  real_T Stop_Value;                   // Expression: 0
                                       //  Referenced by: '<S3>/Stop'

  real_T DesiredPosition_Value[2];     // Expression: [-10 10]
                                       //  Referenced by: '<Root>/Desired Position'

};

// Real-time Model Data Structure
struct tag_RTM_robotROSFeedbackContr_T {
  const char_T *errorStatus;
};

// Class declaration for model robotROSFeedbackControlExample
class Example3ModelClass {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  Example3ModelClass();

  // Destructor
  ~Example3ModelClass();

  // Real-Time Model get method
  RT_MODEL_robotROSFeedbackCont_T * getRTM();

  // private data and function members
 private:
  // Tunable parameters
  P_robotROSFeedbackControlExam_T robotROSFeedbackControlExampl_P;

  // Block signals
  B_robotROSFeedbackControlExam_T robotROSFeedbackControlExampl_B;

  // Block states
  DW_robotROSFeedbackControlExa_T robotROSFeedbackControlExamp_DW;

  // Real-Time Model
  RT_MODEL_robotROSFeedbackCont_T robotROSFeedbackControlExamp_M;

  // private member function(s) for subsystem '<Root>'
  void matlabCodegenHandle_matlabCod_j(robotics_slros_internal_blo_j_T *obj);
  void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj);
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'robotROSFeedbackControlExample'
//  '<S1>'   : 'robotROSFeedbackControlExample/Command Velocity Publisher'
//  '<S2>'   : 'robotROSFeedbackControlExample/Conversion'
//  '<S3>'   : 'robotROSFeedbackControlExample/Proportional Controller'
//  '<S4>'   : 'robotROSFeedbackControlExample/Subscribe'
//  '<S5>'   : 'robotROSFeedbackControlExample/Command Velocity Publisher/Blank Message'
//  '<S6>'   : 'robotROSFeedbackControlExample/Command Velocity Publisher/Publish2'
//  '<S7>'   : 'robotROSFeedbackControlExample/Proportional Controller/Distance Threshold'
//  '<S8>'   : 'robotROSFeedbackControlExample/Proportional Controller/Gain (Slider)'
//  '<S9>'   : 'robotROSFeedbackControlExample/Proportional Controller/Linear Velocity (Slider)'
//  '<S10>'  : 'robotROSFeedbackControlExample/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_robotROSFeedbackControlExample_h_ 
