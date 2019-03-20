//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: myROScomm_types.h
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
#ifndef RTW_HEADER_myROScomm_types_h_
#define RTW_HEADER_myROScomm_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_myROScomm_semi_truck_Teensy_Sensors_
#define DEFINED_TYPEDEF_FOR_SL_Bus_myROScomm_semi_truck_Teensy_Sensors_

// MsgType=semi_truck/Teensy_Sensors
typedef struct {
  int32_T WheelSpeed;
  int32_T ImuAngle;
  int32_T RightURF;
  int32_T LeftURF;
} SL_Bus_myROScomm_semi_truck_Teensy_Sensors;

#endif

#ifndef typedef_ExampleHelperSimulationRateCo_T
#define typedef_ExampleHelperSimulationRateCo_T

typedef struct {
  int32_T isInitialized;
} ExampleHelperSimulationRateCo_T;

#endif                                 //typedef_ExampleHelperSimulationRateCo_T

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_l_T
#define typedef_robotics_slros_internal_blo_l_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_blo_l_T;

#endif                                 //typedef_robotics_slros_internal_blo_l_T

// Parameters (default storage)
typedef struct P_myROScomm_T_ P_myROScomm_T;

// Forward declaration for rtModel
typedef struct tag_RTM_myROScomm_T RT_MODEL_myROScomm_T;

#endif                                 // RTW_HEADER_myROScomm_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
