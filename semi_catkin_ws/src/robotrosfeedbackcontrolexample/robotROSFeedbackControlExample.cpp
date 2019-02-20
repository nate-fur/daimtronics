//
//  robotROSFeedbackControlExample.cpp
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


#include "robotROSFeedbackControlExample.h"
#include "robotROSFeedbackControlExample_private.h"

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2((real_T)u0_0, (real_T)u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

void Example3ModelClass::matlabCodegenHandle_matlabCod_j
  (robotics_slros_internal_blo_j_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void Example3ModelClass::matlabCodegenHandle_matlabCodeg
  (robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void Example3ModelClass::step()
{
  real_T rtb_Distance;
  real_T rtb_Sum3;
  boolean_T rtb_Compare;
  real_T Switch;
  real_T q_idx_1;
  real_T q_idx_2;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S4>/SourceBlock' incorporates:
  //   Inport: '<S10>/In1'

  rtb_Compare = Sub_robotROSFeedbackControlExample_126.getLatestMessage
    (&robotROSFeedbackControlExampl_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S10>/Enable'

  if (rtb_Compare) {
    robotROSFeedbackControlExampl_B.In1 =
      robotROSFeedbackControlExampl_B.b_varargout_2;
  }

  // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLAB Function: '<Root>/Conversion'
  // MATLAB Function 'Conversion': '<S2>:1'
  // '<S2>:1:5'
  rtb_Sum3 = 1.0 / sqrt
    (((robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.W *
       robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.W +
       robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.X *
       robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.X) +
      robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.Y *
      robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.Y) +
     robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.Z *
     robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.Z);
  Switch = robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.W *
    rtb_Sum3;
  q_idx_1 = robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.X *
    rtb_Sum3;
  q_idx_2 = robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.Y *
    rtb_Sum3;
  rtb_Sum3 *= robotROSFeedbackControlExampl_B.In1.Pose.Pose.Orientation.Z;

  // Outputs for Enabled SubSystem: '<Root>/Command Velocity Publisher' incorporates:
  //   EnablePort: '<S1>/Enable'

  // Outputs for Enabled SubSystem: '<Root>/Proportional Controller' incorporates:
  //   EnablePort: '<S3>/Enable'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S4>/SourceBlock'
  // '<S2>:1:6'
  if (rtb_Compare) {
    // Sum: '<S3>/Sum1' incorporates:
    //   Constant: '<Root>/Desired Position'

    robotROSFeedbackControlExampl_B.Sum1 =
      robotROSFeedbackControlExampl_P.DesiredPosition_Value[1] -
      robotROSFeedbackControlExampl_B.In1.Pose.Pose.Position.Y;

    // Sum: '<S3>/Sum' incorporates:
    //   Constant: '<Root>/Desired Position'

    rtb_Distance = robotROSFeedbackControlExampl_P.DesiredPosition_Value[0] -
      robotROSFeedbackControlExampl_B.In1.Pose.Pose.Position.X;

    // Sum: '<S3>/Sum3' incorporates:
    //   MATLAB Function: '<Root>/Conversion'
    //   Trigonometry: '<S3>/Desired Yaw'

    rtb_Sum3 = rt_atan2d_snf(robotROSFeedbackControlExampl_B.Sum1, rtb_Distance)
      - rt_atan2d_snf((q_idx_1 * q_idx_2 + Switch * rtb_Sum3) * 2.0, ((Switch *
      Switch + q_idx_1 * q_idx_1) - q_idx_2 * q_idx_2) - rtb_Sum3 * rtb_Sum3);

    // Fcn: '<S3>/Distance'
    robotROSFeedbackControlExampl_B.Sum1 = rt_powd_snf(rtb_Distance, 2.0) +
      rt_powd_snf(robotROSFeedbackControlExampl_B.Sum1, 2.0);
    if (robotROSFeedbackControlExampl_B.Sum1 < 0.0) {
      robotROSFeedbackControlExampl_B.Sum1 = -sqrt
        (-robotROSFeedbackControlExampl_B.Sum1);
    } else {
      robotROSFeedbackControlExampl_B.Sum1 = sqrt
        (robotROSFeedbackControlExampl_B.Sum1);
    }

    // End of Fcn: '<S3>/Distance'

    // Switch: '<S3>/Switch' incorporates:
    //   Constant: '<S3>/Constant'
    //   Constant: '<S3>/Stop'
    //   Constant: '<S7>/Constant'
    //   Gain: '<S9>/Slider Gain'
    //   RelationalOperator: '<S7>/Compare'
    //   Switch: '<S3>/Switch1'

    if (robotROSFeedbackControlExampl_B.Sum1 <=
        robotROSFeedbackControlExampl_P.DistanceThreshold_const) {
      Switch = robotROSFeedbackControlExampl_P.Stop_Value;
      rtb_Sum3 = robotROSFeedbackControlExampl_P.Stop_Value;
    } else {
      Switch = robotROSFeedbackControlExampl_P.LinearVelocitySlider_gain *
        robotROSFeedbackControlExampl_P.Constant_Value_m;

      // Gain: '<S8>/Slider Gain' incorporates:
      //   Constant: '<S3>/Constant'
      //   Fcn: '<S3>/Bound [-pi,pi]'
      //   Gain: '<S9>/Slider Gain'

      rtb_Sum3 = robotROSFeedbackControlExampl_P.GainSlider_gain * rt_atan2d_snf
        (sin(rtb_Sum3), cos(rtb_Sum3));

      // Saturate: '<S3>/Saturation'
      if (rtb_Sum3 > robotROSFeedbackControlExampl_P.Saturation_UpperSat) {
        rtb_Sum3 = robotROSFeedbackControlExampl_P.Saturation_UpperSat;
      } else {
        if (rtb_Sum3 < robotROSFeedbackControlExampl_P.Saturation_LowerSat) {
          rtb_Sum3 = robotROSFeedbackControlExampl_P.Saturation_LowerSat;
        }
      }

      // End of Saturate: '<S3>/Saturation'
    }

    // End of Switch: '<S3>/Switch'

    // BusAssignment: '<S1>/Bus Assignment1' incorporates:
    //   Constant: '<S5>/Constant'

    robotROSFeedbackControlExampl_B.BusAssignment1 =
      robotROSFeedbackControlExampl_P.Constant_Value_o;
    robotROSFeedbackControlExampl_B.BusAssignment1.Linear.X = Switch;
    robotROSFeedbackControlExampl_B.BusAssignment1.Angular.Z = rtb_Sum3;

    // Outputs for Atomic SubSystem: '<S1>/Publish2'
    // MATLABSystem: '<S6>/SinkBlock'
    Pub_robotROSFeedbackControlExample_128.publish
      (&robotROSFeedbackControlExampl_B.BusAssignment1);

    // End of Outputs for SubSystem: '<S1>/Publish2'
  }

  // End of Outputs for SubSystem: '<Root>/Subscribe'
  // End of Outputs for SubSystem: '<Root>/Proportional Controller'
  // End of Outputs for SubSystem: '<Root>/Command Velocity Publisher'
}

// Model initialize function
void Example3ModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // initialize error status
  rtmSetErrorStatus(getRTM(), (NULL));

  // block I/O
  (void) memset(((void *) &robotROSFeedbackControlExampl_B), 0,
                sizeof(B_robotROSFeedbackControlExam_T));

  // states (dwork)
  (void) memset((void *)&robotROSFeedbackControlExamp_DW, 0,
                sizeof(DW_robotROSFeedbackControlExa_T));

  {
    static const char_T tmp[30] = { '/', 'm', 'o', 'b', 'i', 'l', 'e', '_', 'b',
      'a', 's', 'e', '/', 'c', 'o', 'm', 'm', 'a', 'n', 'd', 's', '/', 'v', 'e',
      'l', 'o', 'c', 'i', 't', 'y' };

    static const char_T tmp_0[5] = { '/', 'o', 'd', 'o', 'm' };

    char_T tmp_1[31];
    char_T tmp_2[6];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S4>/SourceBlock'
    robotROSFeedbackControlExamp_DW.obj_n.matlabCodegenIsDeleted = true;
    robotROSFeedbackControlExamp_DW.obj_n.isInitialized = 0;
    robotROSFeedbackControlExamp_DW.obj_n.matlabCodegenIsDeleted = false;
    robotROSFeedbackControlExamp_DW.obj_n.isSetupComplete = false;
    robotROSFeedbackControlExamp_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 5; i++) {
      tmp_2[i] = tmp_0[i];
    }

    tmp_2[5] = '\x00';
    Sub_robotROSFeedbackControlExample_126.createSubscriber(tmp_2, 51);
    robotROSFeedbackControlExamp_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Enabled SubSystem: '<Root>/Command Velocity Publisher'
    // Start for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S6>/SinkBlock'
    robotROSFeedbackControlExamp_DW.obj.matlabCodegenIsDeleted = true;
    robotROSFeedbackControlExamp_DW.obj.isInitialized = 0;
    robotROSFeedbackControlExamp_DW.obj.matlabCodegenIsDeleted = false;
    robotROSFeedbackControlExamp_DW.obj.isSetupComplete = false;
    robotROSFeedbackControlExamp_DW.obj.isInitialized = 1;
    for (i = 0; i < 30; i++) {
      tmp_1[i] = tmp[i];
    }

    tmp_1[30] = '\x00';
    Pub_robotROSFeedbackControlExample_128.createPublisher(tmp_1, 105);
    robotROSFeedbackControlExamp_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SinkBlock'
    // End of Start for SubSystem: '<S1>/Publish2'
    // End of Start for SubSystem: '<Root>/Command Velocity Publisher'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S10>/Out1'
    robotROSFeedbackControlExampl_B.In1 =
      robotROSFeedbackControlExampl_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void Example3ModelClass::terminate()
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  matlabCodegenHandle_matlabCod_j(&robotROSFeedbackControlExamp_DW.obj_n);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Enabled SubSystem: '<Root>/Command Velocity Publisher'
  // Terminate for Atomic SubSystem: '<S1>/Publish2'
  // Terminate for MATLABSystem: '<S6>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&robotROSFeedbackControlExamp_DW.obj);

  // End of Terminate for SubSystem: '<S1>/Publish2'
  // End of Terminate for SubSystem: '<Root>/Command Velocity Publisher'
}

// Constructor
Example3ModelClass::Example3ModelClass()
{
  static const P_robotROSFeedbackControlExam_T
    robotROSFeedbackControlExampl_P_temp = {
    // Mask Parameter: DistanceThreshold_const
    //  Referenced by: '<S7>/Constant'

    0.5,

    // Mask Parameter: GainSlider_gain
    //  Referenced by: '<S8>/Slider Gain'

    5.0,

    // Mask Parameter: LinearVelocitySlider_gain
    //  Referenced by: '<S9>/Slider Gain'

    1.0,

    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S10>/Out1'

    {
      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // ChildFrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // ChildFrameId_SL_Info

      {
        0U,                            // Seq

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U }
        ,                              // FrameId

        {
          0U,                          // CurrentLength
          0U                           // ReceivedLength
        },                             // FrameId_SL_Info

        {
          0.0,                         // Sec
          0.0                          // Nsec
        }                              // Stamp
      },                               // Header

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              // Covariance

        {
          {
            0.0,                       // X
            0.0,                       // Y
            0.0                        // Z
          },                           // Position

          {
            0.0,                       // X
            0.0,                       // Y
            0.0,                       // Z
            0.0                        // W
          }                            // Orientation
        }                              // Pose
      },                               // Pose

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              // Covariance

        {
          {
            0.0,                       // X
            0.0,                       // Y
            0.0                        // Z
          },                           // Linear

          {
            0.0,                       // X
            0.0,                       // Y
            0.0                        // Z
          }                            // Angular
        }                              // Twist
      }                                // Twist
    },

    // Computed Parameter: Constant_Value
    //  Referenced by: '<S4>/Constant'

    {
      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // ChildFrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // ChildFrameId_SL_Info

      {
        0U,                            // Seq

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U }
        ,                              // FrameId

        {
          0U,                          // CurrentLength
          0U                           // ReceivedLength
        },                             // FrameId_SL_Info

        {
          0.0,                         // Sec
          0.0                          // Nsec
        }                              // Stamp
      },                               // Header

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              // Covariance

        {
          {
            0.0,                       // X
            0.0,                       // Y
            0.0                        // Z
          },                           // Position

          {
            0.0,                       // X
            0.0,                       // Y
            0.0,                       // Z
            0.0                        // W
          }                            // Orientation
        }                              // Pose
      },                               // Pose

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              // Covariance

        {
          {
            0.0,                       // X
            0.0,                       // Y
            0.0                        // Z
          },                           // Linear

          {
            0.0,                       // X
            0.0,                       // Y
            0.0                        // Z
          }                            // Angular
        }                              // Twist
      }                                // Twist
    },

    // Computed Parameter: Constant_Value_o
    //  Referenced by: '<S5>/Constant'

    {
      {
        0.0,                           // X
        0.0,                           // Y
        0.0                            // Z
      },                               // Linear

      {
        0.0,                           // X
        0.0,                           // Y
        0.0                            // Z
      }                                // Angular
    },

    // Expression: 0.5
    //  Referenced by: '<S3>/Saturation'

    0.5,

    // Expression: -0.5
    //  Referenced by: '<S3>/Saturation'

    -0.5,

    // Computed Parameter: LinearVelocityv_Y0
    //  Referenced by: '<S3>/Linear Velocity (v)'

    0.0,

    // Computed Parameter: AngularVelocityw_Y0
    //  Referenced by: '<S3>/Angular Velocity (w)'

    0.0,

    // Expression: 1
    //  Referenced by: '<S3>/Constant'

    1.0,

    // Expression: 0
    //  Referenced by: '<S3>/Stop'

    0.0,

    // Expression: [-10 10]
    //  Referenced by: '<Root>/Desired Position'

    { -10.0, 10.0 }
  };                                   // Modifiable parameters

  // Initialize tunable parameters
  robotROSFeedbackControlExampl_P = robotROSFeedbackControlExampl_P_temp;
}

// Destructor
Example3ModelClass::~Example3ModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_robotROSFeedbackCont_T * Example3ModelClass::getRTM()
{
  return (&robotROSFeedbackControlExamp_M);
}
