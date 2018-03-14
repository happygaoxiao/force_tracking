//
// File: force_loop.cpp
//
// Code generated for Simulink model 'force_loop'.
//
// Model version                  : 1.150
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Feb 28 14:51:04 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "force_loop.h"

// Exported block parameters
real_T kd = 1.0;                       // Variable: kd
                                       //  Referenced by: '<S1>/Derivative Gain'

real_T ki = 1.0;                       // Variable: ki
                                       //  Referenced by: '<S1>/Integral Gain'

real_T kp = 1.0;                       // Variable: kp
                                       //  Referenced by: '<S1>/Proportional Gain'

real_T limit_dx = 1.0;                 // Variable: limit_dx
                                       //  Referenced by: '<Root>/Constant'


// Model step function
void force_PID_loop::step(real_T &arg_ref_force, real_T &arg_contact_force,
  real_T &arg_delta_x)
{
  real_T rtb_Sum_b;
  real_T rtb_FilterCoefficient;
  real_T rtb_Sum;

  // Sum: '<Root>/Sum' incorporates:
  //   Inport: '<Root>/contact_force'
  //   Inport: '<Root>/ref_force'

  rtb_Sum_b = arg_ref_force - arg_contact_force;

  // Gain: '<S1>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S1>/Filter'
  //   Gain: '<S1>/Derivative Gain'
  //   Sum: '<S1>/SumD'

  rtb_FilterCoefficient = (kd * rtb_Sum_b - force_loop_DW.Filter_DSTATE) * 5.0;

  // Sum: '<S1>/Sum' incorporates:
  //   DiscreteIntegrator: '<S1>/Integrator'
  //   Gain: '<S1>/Proportional Gain'

  rtb_Sum = (kp * rtb_Sum_b + force_loop_DW.Integrator_DSTATE) +
    rtb_FilterCoefficient;

  // Switch: '<S3>/Switch2' incorporates:
  //   Constant: '<Root>/Constant'
  //   Gain: '<Root>/Gain'
  //   RelationalOperator: '<S3>/LowerRelop1'
  //   RelationalOperator: '<S3>/UpperRelop'
  //   Switch: '<S3>/Switch'

  if (rtb_Sum > limit_dx) {
    // Outport: '<Root>/delta_x'
    arg_delta_x = limit_dx;
  } else if (rtb_Sum < -limit_dx) {
    // Switch: '<S3>/Switch' incorporates:
    //   Gain: '<Root>/Gain'
    //   Outport: '<Root>/delta_x'

    arg_delta_x = -limit_dx;
  } else {
    // Outport: '<Root>/delta_x' incorporates:
    //   Switch: '<S3>/Switch'

    arg_delta_x = rtb_Sum;
  }

  // End of Switch: '<S3>/Switch2'

  // Update for DiscreteIntegrator: '<S1>/Integrator' incorporates:
  //   Gain: '<S1>/Integral Gain'

  force_loop_DW.Integrator_DSTATE += ki * rtb_Sum_b * 0.008;

  // Update for DiscreteIntegrator: '<S1>/Filter'
  force_loop_DW.Filter_DSTATE += 0.008 * rtb_FilterCoefficient;
}

// Model initialize function
void force_PID_loop::initialize()
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus((&force_loop_M), (NULL));

  // states (dwork)
  (void) memset((void *)&force_loop_DW, 0,
                sizeof(DW_force_loop_T));
}

// Model terminate function
void force_PID_loop::terminate()
{
  // (no terminate code required)
}

// Constructor
force_PID_loop::force_PID_loop()
{
}

// Destructor
force_PID_loop::~force_PID_loop()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_force_loop_T * force_PID_loop::getRTM()
{
  return (&force_loop_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
