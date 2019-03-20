//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: myROScomm.cpp
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
#include "myROScomm.h"
#include "myROScomm_private.h"

// Block signals (default storage)
B_myROScomm_T myROScomm_B;

// Block states (default storage)
DW_myROScomm_T myROScomm_DW;

// Real-time model
RT_MODEL_myROScomm_T myROScomm_M_;
RT_MODEL_myROScomm_T *const myROScomm_M = &myROScomm_M_;

// Forward declaration for local functions
static void matlabCodegenHandle_matlabCod_l(robotics_slros_internal_blo_l_T *obj);
static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj);
static void matlabCodegenHandle_matlabCod_l(robotics_slros_internal_blo_l_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void myROScomm_step(void)
{
  SL_Bus_myROScomm_semi_truck_Teensy_Sensors b_varargout_2;
  boolean_T b_varargout_1;
  SL_Bus_myROScomm_semi_truck_Teensy_Sensors rtb_BusAssignment;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S2>/SourceBlock' incorporates:
  //   Inport: '<S3>/In1'

  b_varargout_1 = Sub_myROScomm_1.getLatestMessage(&b_varargout_2);

  // Outputs for Enabled SubSystem: '<S2>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S3>/Enable'

  if (b_varargout_1) {
    myROScomm_B.In1 = b_varargout_2;
  }

  // End of MATLABSystem: '<S2>/SourceBlock'
  // End of Outputs for SubSystem: '<S2>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Constant: '<Root>/Constant'
  //   Constant: '<Root>/Constant1'
  //   Constant: '<Root>/Constant2'
  //   Constant: '<Root>/Constant3'
  //   Sum: '<Root>/Add'
  //   Sum: '<Root>/Add1'
  //   Sum: '<Root>/Add2'
  //   Sum: '<Root>/Add3'

  rtb_BusAssignment.WheelSpeed = myROScomm_P.Constant_Value_c +
    myROScomm_B.In1.WheelSpeed;
  rtb_BusAssignment.ImuAngle = myROScomm_P.Constant2_Value +
    myROScomm_B.In1.ImuAngle;
  rtb_BusAssignment.RightURF = myROScomm_B.In1.RightURF +
    myROScomm_P.Constant1_Value;
  rtb_BusAssignment.LeftURF = myROScomm_B.In1.LeftURF +
    myROScomm_P.Constant3_Value;

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S1>/SinkBlock'
  Pub_myROScomm_9.publish(&rtb_BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void myROScomm_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(myROScomm_M, (NULL));

  // block I/O
  (void) memset(((void *) &myROScomm_B), 0,
                sizeof(B_myROScomm_T));

  // states (dwork)
  (void) memset((void *)&myROScomm_DW, 0,
                sizeof(DW_myROScomm_T));

  {
    static const char_T tmp[38] = { '/', 't', 'e', 'e', 'n', 's', 'y', '_', 'c',
      'o', 'm', 'm', '_', 'n', 'o', 'd', 'e', '/', 's', 'e', 'n', 's', 'o', 'r',
      '_', 'd', 'a', 't', 'a', '_', 'm', 'o', 'd', 'i', 'f', 'i', 'e', 'd' };

    static const char_T tmp_0[29] = { '/', 't', 'e', 'e', 'n', 's', 'y', '_',
      'c', 'o', 'm', 'm', '_', 'n', 'o', 'd', 'e', '/', 's', 'e', 'n', 's', 'o',
      'r', '_', 'd', 'a', 't', 'a' };

    char_T tmp_1[30];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S2>/SourceBlock'
    myROScomm_DW.obj_n.matlabCodegenIsDeleted = true;
    myROScomm_DW.obj_n.isInitialized = 0;
    myROScomm_DW.obj_n.matlabCodegenIsDeleted = false;
    myROScomm_DW.obj_n.isSetupComplete = false;
    myROScomm_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      tmp_1[i] = tmp_0[i];
    }

    tmp_1[29] = '\x00';
    Sub_myROScomm_1.createSubscriber(tmp_1, 51);
    myROScomm_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S1>/SinkBlock'
    myROScomm_DW.obj.matlabCodegenIsDeleted = true;
    myROScomm_DW.obj.isInitialized = 0;
    myROScomm_DW.obj.matlabCodegenIsDeleted = false;
    myROScomm_DW.obj.isSetupComplete = false;
    myROScomm_DW.obj.isInitialized = 1;
    for (i = 0; i < 38; i++) {
      myROScomm_B.cv0[i] = tmp[i];
    }

    myROScomm_B.cv0[38] = '\x00';
    Pub_myROScomm_9.createPublisher(myROScomm_B.cv0, 1);
    myROScomm_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S1>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S2>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S3>/Out1'
    myROScomm_B.In1 = myROScomm_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S2>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void myROScomm_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S2>/SourceBlock'
  matlabCodegenHandle_matlabCod_l(&myROScomm_DW.obj_n);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S1>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&myROScomm_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
