//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: myROScomm.h
//
// Code generated for Simulink model 'myROScomm'.
//
// Model version                  : 1.7
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Thu Mar 14 15:03:05 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_myROScomm_h_
#define RTW_HEADER_myROScomm_h_
#include <stddef.h>
#include <string.h>
#ifndef myROScomm_COMMON_INCLUDES_
# define myROScomm_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // myROScomm_COMMON_INCLUDES_

#include "myROScomm_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  char_T cv0[39];
  SL_Bus_myROScomm_semi_truck_Teensy_Sensors In1;// '<S3>/In1'
} B_myROScomm_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slros_internal_block_T obj; // '<S1>/SinkBlock'
  robotics_slros_internal_blo_l_T obj_n;// '<S2>/SourceBlock'
} DW_myROScomm_T;

// Parameters (default storage)
struct P_myROScomm_T_ {
  SL_Bus_myROScomm_semi_truck_Teensy_Sensors Out1_Y0;// Computed Parameter: Out1_Y0
                                                     //  Referenced by: '<S3>/Out1'

  SL_Bus_myROScomm_semi_truck_Teensy_Sensors Constant_Value;// Computed Parameter: Constant_Value
                                                            //  Referenced by: '<S2>/Constant'

  int32_T Constant_Value_c;            // Computed Parameter: Constant_Value_c
                                       //  Referenced by: '<Root>/Constant'

  int32_T Constant2_Value;             // Computed Parameter: Constant2_Value
                                       //  Referenced by: '<Root>/Constant2'

  int32_T Constant1_Value;             // Computed Parameter: Constant1_Value
                                       //  Referenced by: '<Root>/Constant1'

  int32_T Constant3_Value;             // Computed Parameter: Constant3_Value
                                       //  Referenced by: '<Root>/Constant3'

};

// Real-time Model Data Structure
struct tag_RTM_myROScomm_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_myROScomm_T myROScomm_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
extern B_myROScomm_T myROScomm_B;

// Block states (default storage)
extern DW_myROScomm_T myROScomm_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void myROScomm_initialize(void);
  extern void myROScomm_step(void);
  extern void myROScomm_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_myROScomm_T *const myROScomm_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Scope' : Unused code path elimination


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
//  '<Root>' : 'myROScomm'
//  '<S1>'   : 'myROScomm/Publish'
//  '<S2>'   : 'myROScomm/Subscribe'
//  '<S3>'   : 'myROScomm/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_myROScomm_h_

//
// File trailer for generated code.
//
// [EOF]
//
