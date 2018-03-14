//
// File: force_loop.h
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
#ifndef RTW_HEADER_force_loop_h_
#define RTW_HEADER_force_loop_h_
#include <stddef.h>
#include <string.h>
#ifndef force_loop_COMMON_INCLUDES_
# define force_loop_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // force_loop_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_force_loop_T RT_MODEL_force_loop_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T Integrator_DSTATE;            // '<S1>/Integrator'
  real_T Filter_DSTATE;                // '<S1>/Filter'
} DW_force_loop_T;

// Real-time Model Data Structure
struct tag_RTM_force_loop_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern real_T kd;                      // Variable: kd
                                       //  Referenced by: '<S1>/Derivative Gain'

extern real_T ki;                      // Variable: ki
                                       //  Referenced by: '<S1>/Integral Gain'

extern real_T kp;                      // Variable: kp
                                       //  Referenced by: '<S1>/Proportional Gain'

extern real_T limit_dx;                // Variable: limit_dx
                                       //  Referenced by: '<Root>/Constant'


// Class declaration for model force_loop
class force_PID_loop {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(real_T &arg_ref_force, real_T &arg_contact_force, real_T
            &arg_delta_x);

  // model terminate function
  void terminate();

  // Constructor
  force_PID_loop();

  // Destructor
  ~force_PID_loop();

  // Real-Time Model get method
  RT_MODEL_force_loop_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_force_loop_T force_loop_DW;

  // Real-Time Model
  RT_MODEL_force_loop_T force_loop_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S3>/Data Type Duplicate' : Unused code path elimination
//  Block '<S3>/Data Type Propagation' : Unused code path elimination


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
//  '<Root>' : 'force_loop'
//  '<S1>'   : 'force_loop/Discrete PID Controller'
//  '<S2>'   : 'force_loop/MATLAB Function1'
//  '<S3>'   : 'force_loop/Saturation Dynamic'

#endif                                 // RTW_HEADER_force_loop_h_

//
// File trailer for generated code.
//
// [EOF]
//
