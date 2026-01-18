/*
 * CAVE_MachE_sil_test.h
 *
 * Code generation for model "CAVE_MachE_sil_test".
 *
 * Model version              : 1.49
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Tue Aug 22 23:31:18 2023
 *
 * Target selection: dsrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_CAVE_MachE_sil_test_h_
#define RTW_HEADER_CAVE_MachE_sil_test_h_
#include <string.h>
#include <math.h>
#ifndef CAVE_MachE_sil_test_COMMON_INCLUDES_
# define CAVE_MachE_sil_test_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                /* CAVE_MachE_sil_test_COMMON_INCLUDES_ */

#include "CAVE_MachE_sil_test_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals for system '<S80>/MATLAB Function' */
typedef struct {
  real_T switchFlag;                   /* '<S80>/MATLAB Function' */
} B_MATLABFunction_CAVE_MachE_sil_test_T;

/* Block signals for system '<S107>/Defloater' */
typedef struct {
  uint8_T byte1;                       /* '<S107>/Defloater' */
  uint8_T byte2;                       /* '<S107>/Defloater' */
  uint8_T byte3;                       /* '<S107>/Defloater' */
  uint8_T byte4;                       /* '<S107>/Defloater' */
} B_Defloater_CAVE_MachE_sil_test_T;

/* Block signals for system '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */
typedef struct {
  real_T Selector6;                    /* '<S142>/Selector6' */
  real_T TrigonometricFunction;        /* '<S142>/Trigonometric Function' */
  real_T Gain;                         /* '<S142>/Gain' */
  real_T Sum;                          /* '<S146>/Sum' */
  real_T DCMStaringRow;                /* '<S146>/DCM Staring Row' */
  real_T Sum1;                         /* '<S146>/Sum1' */
  real_T SelectDCM[9];                 /* '<S146>/Select DCM' */
  real_T MathFunction[9];              /* '<S142>/Math Function' */
  real_T Selector1[3];                 /* '<S142>/Selector1' */
  real_T MatrixMultiply1[3];           /* '<S142>/Matrix Multiply1' */
  real_T Translationeffectonpositions[3];/* '<S142>/Selector2' */
  real_T Sum1_l[3];                    /* '<S142>/Sum1' */
  real_T Selector5[3];                 /* '<S142>/Selector5' */
  real_T TrigonometricFunction1;       /* '<S142>/Trigonometric Function1' */
  real_T MatrixConcatenate4[3];        /* '<S142>/Matrix Concatenate4' */
  real_T Product2[3];                  /* '<S142>/Product2' */
  real_T Sum3[3];                      /* '<S142>/Sum3' */
  real_T Selector[3];                  /* '<S142>/Selector' */
  real_T Rotationeffectonpositions[3]; /* '<S142>/Matrix Multiply3' */
  real_T Sum4[3];                      /* '<S142>/Sum4' */
  real_T Product1[3];                  /* '<S142>/Product1' */
  real_T Sum2[3];                      /* '<S142>/Sum2' */
} B_CoreSubsys_CAVE_MachE_sil_test_T;

/* Block signals for system '<S135>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T Reshape[3];                   /* '<S154>/Reshape' */
  real_T u;                            /* '<S154>/ ' */
  real_T TrigonometricFunction;        /* '<S154>/Trigonometric Function' */
  real_T TrigonometricFunction1;       /* '<S154>/Trigonometric Function1' */
  real_T MatrixConcatenate4[3];        /* '<S154>/Matrix Concatenate4' */
  real_T MatrixConcatenate5[3];        /* '<S154>/Matrix Concatenate5' */
  real_T MatrixConcatenate3[9];        /* '<S154>/Matrix Concatenate3' */
  real_T cgcoordinates[3];             /* '<S150>/cg coordinates' */
  real_T MathFunction1[9];             /* '<S150>/Math Function1' */
  real_T zdot;                         /* '<S148>/Vz' */
  real_T TmpSignalConversionAtMatrixMultiply2Inport2[3];/* '<S150>/cgV in' */
  real_T MatrixMultiply2[3];           /* '<S150>/Matrix Multiply2' */
  real_T p;                            /* '<S148>/Vy1' */
  real_T yaxistrackcoordinates[2];     /* '<S151>/y axis track coordinates' */
  real_T MathFunction[2];              /* '<S151>/Math Function' */
  real_T yaxistrackcoordinates_o[2];   /* '<S149>/y axis track coordinates' */
  real_T Product1;                     /* '<S148>/Product1' */
  real_T Sum;                          /* '<S148>/Sum' */
  real_T ydot;                         /* '<S148>/Vy' */
  real_T ydotp;                        /* '<S148>/Vyp' */
  real_T DataTypeConversion[2];        /* '<S151>/Data Type Conversion' */
  real_T Product[2];                   /* '<S151>/Product' */
  real_T DotProduct1;                  /* '<S151>/Dot Product1' */
  real_T DataTypeConversion_j;         /* '<S155>/Data Type Conversion' */
  real_T Product_g;                    /* '<S155>/Product' */
  real_T SumofElements;                /* '<S155>/Sum of Elements' */
  real_T Product2[2];                  /* '<S151>/Product2' */
  real_T DataTypeConversion_a;         /* '<S156>/Data Type Conversion' */
  real_T Product_k;                    /* '<S156>/Product' */
  real_T SumofElements_b;              /* '<S156>/Sum of Elements' */
  real_T Sum_e[2];                     /* '<S151>/Sum' */
  real_T Product3[2];                  /* '<S151>/Product3' */
  real_T Product4[2];                  /* '<S151>/Product4' */
  real_T xaxiswheelmoments[2];         /* '<S151>/x axis wheel moments' */
  real_T DotProduct2;                  /* '<S151>/Dot Product2' */
  real_T Sum1;                         /* '<S151>/Sum1' */
  real_T DataTypeConversion_k[2];      /* '<S149>/Data Type Conversion' */
  real_T Product_b[2];                 /* '<S149>/Product' */
  real_T DotProduct1_h;                /* '<S149>/Dot Product1' */
  real_T SuspensionMomentDirectionOnSolidAxle;
                        /* '<S149>/Suspension Moment Direction On Solid Axle' */
  real_T Sum1_n;                       /* '<S147>/Sum1' */
  real_T DataTypeConversion_e;         /* '<S153>/Data Type Conversion' */
  real_T Product_p;                    /* '<S153>/Product' */
  real_T SumofElements_h;              /* '<S153>/Sum of Elements' */
  real_T pdot;                         /* '<S148>/Divide' */
  real_T DotProduct;                   /* '<S151>/Dot Product' */
  real_T DotProduct_o;                 /* '<S149>/Dot Product' */
  real_T SuspensionForceDirectionOnSolidAxle;
                         /* '<S149>/Suspension Force Direction On Solid Axle' */
  real_T Sum_f;                        /* '<S147>/Sum' */
  real_T DataTypeConversion_g;         /* '<S152>/Data Type Conversion' */
  real_T Product_j;                    /* '<S152>/Product' */
  real_T SumofElements_h5;             /* '<S152>/Sum of Elements' */
  real_T Divide1;                      /* '<S148>/Divide1' */
  real_T Sum2;                         /* '<S148>/Sum2' */
  real_T zddot;                        /* '<S148>/Sum1' */
  boolean_T RelationalOperator[2];     /* '<S151>/Relational Operator' */
  boolean_T RelationalOperator_j;      /* '<S155>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S156>/Relational Operator' */
  boolean_T RelationalOperator_i[2];   /* '<S149>/Relational Operator' */
  boolean_T RelationalOperator_d;      /* '<S153>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S152>/Relational Operator' */
} B_CoreSubsys_CAVE_MachE_sil_test_l_T;

/* Continuous states for system '<S135>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T _CSTATE;                      /* '<S154>/ ' */
  real_T cgcoordinates_CSTATE[3];      /* '<S150>/cg coordinates' */
  real_T Vz_CSTATE;                    /* '<S148>/Vz' */
  real_T Vy1_CSTATE;                   /* '<S148>/Vy1' */
  real_T Vy_CSTATE;                    /* '<S148>/Vy' */
} X_CoreSubsys_CAVE_MachE_sil_test_o_T;

/* State derivatives for system '<S135>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T _CSTATE;                      /* '<S154>/ ' */
  real_T cgcoordinates_CSTATE[3];      /* '<S150>/cg coordinates' */
  real_T Vz_CSTATE;                    /* '<S148>/Vz' */
  real_T Vy1_CSTATE;                   /* '<S148>/Vy1' */
  real_T Vy_CSTATE;                    /* '<S148>/Vy' */
} XDot_CoreSubsys_CAVE_MachE_sil_test_k_T;

/* State Disabled for system '<S135>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  boolean_T _CSTATE;                   /* '<S154>/ ' */
  boolean_T cgcoordinates_CSTATE[3];   /* '<S150>/cg coordinates' */
  boolean_T Vz_CSTATE;                 /* '<S148>/Vz' */
  boolean_T Vy1_CSTATE;                /* '<S148>/Vy1' */
  boolean_T Vy_CSTATE;                 /* '<S148>/Vy' */
} XDis_CoreSubsys_CAVE_MachE_sil_test_e_T;

/* Block signals for system '<S172>/Min stop reached' */
typedef struct {
  real_T Sum1;                         /* '<S178>/Sum1' */
  real_T Gain5;                        /* '<S178>/Gain5' */
  real_T Gain4;                        /* '<S178>/Gain4' */
  real_T Product3;                     /* '<S178>/Product3' */
  real_T Abs1;                         /* '<S178>/Abs1' */
  real_T Saturation;                   /* '<S178>/Saturation' */
  real_T TrigonometricFunction;        /* '<S178>/Trigonometric Function' */
  real_T Gain;                         /* '<S178>/Gain' */
  real_T Sum2;                         /* '<S178>/Sum2' */
  real_T LowerHardStopBlendMult;       /* '<S178>/Lower Hard Stop Blend Mult' */
  real_T MathFunction;                 /* '<S178>/Math Function' */
  real_T Product2;                     /* '<S178>/Product2' */
  real_T Product;                      /* '<S178>/Product' */
  real_T Product1;                     /* '<S178>/Product1' */
  real_T Sum;                          /* '<S178>/Sum' */
  real_T Product4;                     /* '<S178>/Product4' */
} B_Minstopreached_CAVE_MachE_sil_test_T;

/* Block signals for system '<S172>/Max stop reached' */
typedef struct {
  real_T Sum1;                         /* '<S177>/Sum1' */
  real_T Gain5;                        /* '<S177>/Gain5' */
  real_T Gain4;                        /* '<S177>/Gain4' */
  real_T Product3;                     /* '<S177>/Product3' */
  real_T Abs1;                         /* '<S177>/Abs1' */
  real_T Saturation;                   /* '<S177>/Saturation' */
  real_T TrigonometricFunction;        /* '<S177>/Trigonometric Function' */
  real_T Gain;                         /* '<S177>/Gain' */
  real_T MathFunction;                 /* '<S177>/Math Function' */
  real_T Product2;                     /* '<S177>/Product2' */
  real_T Product;                      /* '<S177>/Product' */
  real_T Product1;                     /* '<S177>/Product1' */
  real_T Sum;                          /* '<S177>/Sum' */
  real_T Sum2;                         /* '<S177>/Sum2' */
  real_T UpperHardStopBlendMult;       /* '<S177>/Upper Hard Stop Blend Mult' */
  real_T Product4;                     /* '<S177>/Product4' */
} B_Maxstopreached_CAVE_MachE_sil_test_T;

/* Block signals for system '<S135>/For each track and axle combination calculate suspension forces and moments' */
typedef struct {
  real_T DataTypeConversion;           /* '<S164>/Data Type Conversion' */
  real_T Product;                      /* '<S164>/Product' */
  real_T SumofElements;                /* '<S164>/Sum of Elements' */
  real_T DataTypeConversion_m;         /* '<S163>/Data Type Conversion' */
  real_T Product_d;                    /* '<S163>/Product' */
  real_T SumofElements_f;              /* '<S163>/Sum of Elements' */
  real_T TmpSignalConversionAtSelector3Inport1[2];/* '<S158>/Mux' */
  real_T Selector1;                    /* '<S157>/Selector1' */
  real_T Sum2;                         /* '<S157>/Sum2' */
  real_T Selector5;                    /* '<S158>/Selector5' */
  real_T Product_g;                    /* '<S158>/Product' */
  real_T Selector3;                    /* '<S158>/Selector3' */
  real_T Abs;                          /* '<S169>/Abs' */
  real_T Product_dj;                   /* '<S169>/Product' */
  real_T Selector2[2];                 /* '<S157>/Selector2' */
  real_T Selector[2];                  /* '<S157>/Selector' */
  real_T Add;                          /* '<S169>/Add' */
  real_T DataTypeConversion_c;         /* '<S175>/Data Type Conversion' */
  real_T Product_b;                    /* '<S175>/Product' */
  real_T SumofElements_g;              /* '<S175>/Sum of Elements' */
  real_T DataTypeConversion_l;         /* '<S174>/Data Type Conversion' */
  real_T Product_h;                    /* '<S174>/Product' */
  real_T SumofElements_m;              /* '<S174>/Sum of Elements' */
  real_T Product4;                     /* '<S170>/Product4' */
  real_T Add4;                         /* '<S170>/Add4' */
  real_T HeightSignConvention;         /* '<S170>/Height Sign Convention' */
  real_T Product3;                     /* '<S162>/Product3' */
  real_T Sum2_b;                       /* '<S162>/Sum2' */
  real_T DataTypeConversion_d;         /* '<S166>/Data Type Conversion' */
  real_T Product_l;                    /* '<S166>/Product' */
  real_T SumofElements_mz;             /* '<S166>/Sum of Elements' */
  real_T DataTypeConversion_k;         /* '<S165>/Data Type Conversion' */
  real_T Product_bk;                   /* '<S165>/Product' */
  real_T SumofElements_b;              /* '<S165>/Sum of Elements' */
  real_T Product5;                     /* '<S162>/Product5' */
  real_T Sum1;                         /* '<S162>/Sum1' */
  real_T DataTypeConversion_a;         /* '<S168>/Data Type Conversion' */
  real_T Product_j;                    /* '<S168>/Product' */
  real_T SumofElements_d;              /* '<S168>/Sum of Elements' */
  real_T DataTypeConversion_p;         /* '<S167>/Data Type Conversion' */
  real_T Product_l2;                   /* '<S167>/Product' */
  real_T SumofElements_i;              /* '<S167>/Sum of Elements' */
  real_T Product1;                     /* '<S162>/Product1' */
  real_T Sum;                          /* '<S162>/Sum' */
  real_T Product3_d;                   /* '<S170>/Product3' */
  real_T DataTypeConversion_f;         /* '<S173>/Data Type Conversion' */
  real_T Product_c;                    /* '<S173>/Product' */
  real_T SumofElements_e;              /* '<S173>/Sum of Elements' */
  real_T Add2;                         /* '<S169>/Add2' */
  real_T Product5_d;                   /* '<S170>/Product5' */
  real_T Add1;                         /* '<S170>/Add1' */
  real_T Product1_a;                   /* '<S170>/Product1' */
  real_T Sign1;                        /* '<S170>/Sign1' */
  real_T Product2;                     /* '<S170>/Product2' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[6];/* '<S144>/Suspension' */
  real_T DataTypeConversion_fh;        /* '<S176>/Data Type Conversion' */
  real_T Product_i;                    /* '<S176>/Product' */
  real_T SumofElements_fu;             /* '<S176>/Sum of Elements' */
  real_T Sum_a;                        /* '<S172>/Sum' */
  real_T Sum_e;                        /* '<S170>/Sum' */
  real_T VehicleForceSign;             /* '<S169>/Vehicle Force Sign' */
  real_T Selector1_g;                  /* '<S160>/Selector1' */
  real_T Selector_d;                   /* '<S160>/Selector' */
  real_T VehicleHeight;                /* '<S169>/Sign convention' */
  real_T Sum_h;                        /* '<S160>/Sum' */
  real_T Product_l5;                   /* '<S160>/Product' */
  real_T UnaryMinus;                   /* '<S160>/Unary Minus' */
  real_T Selector2_g;                  /* '<S160>/Selector2' */
  real_T Product1_p;                   /* '<S160>/Product1' */
  real_T Reshape[3];                   /* '<S160>/Reshape' */
  real_T Selector3_o[3];               /* '<S160>/Selector3' */
  real_T Sum1_n[3];                    /* '<S160>/Sum1' */
  real_T Sum3;                         /* '<S162>/Sum3' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[3];/* '<S144>/Suspension' */
  real_T Reshape19[2];                 /* '<S144>/Reshape19' */
  real_T Selector3_e;                  /* '<S157>/Selector3' */
  boolean_T RelationalOperator;        /* '<S164>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S163>/Relational Operator' */
  boolean_T RelationalOperator_j;      /* '<S175>/Relational Operator' */
  boolean_T RelationalOperator_d;      /* '<S174>/Relational Operator' */
  boolean_T RelationalOperator_l;      /* '<S166>/Relational Operator' */
  boolean_T RelationalOperator_b;      /* '<S165>/Relational Operator' */
  boolean_T RelationalOperator_c;      /* '<S168>/Relational Operator' */
  boolean_T RelationalOperator_a;      /* '<S167>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S173>/Relational Operator' */
  boolean_T RelationalOperator_e;      /* '<S176>/Relational Operator' */
  B_Maxstopreached_CAVE_MachE_sil_test_T Maxstopreached;/* '<S172>/Max stop reached' */
  B_Minstopreached_CAVE_MachE_sil_test_T Minstopreached;/* '<S172>/Min stop reached' */
} B_CoreSubsys_CAVE_MachE_sil_test_l0_T;

/* Block signals for system '<S182>/For Each Axle With Anti-Sway' */
typedef struct {
  real_T DataTypeConversion;           /* '<S186>/Data Type Conversion' */
  real_T Product;                      /* '<S186>/Product' */
  real_T SumofElements;                /* '<S186>/Sum of Elements' */
  real_T DataTypeConversion_b;         /* '<S185>/Data Type Conversion' */
  real_T Product_o;                    /* '<S185>/Product' */
  real_T SumofElements_a;              /* '<S185>/Sum of Elements' */
  real_T TrigonometricFunction;        /* '<S184>/Trigonometric Function' */
  real_T Z0;                           /* '<S184>/Product' */
  real_T Selector5[2];                 /* '<S183>/Selector5' */
  real_T Selector1;                    /* '<S183>/Selector1' */
  real_T Sum2[2];                      /* '<S183>/Sum2' */
  real_T Selector3[2];                 /* '<S183>/Selector3' */
  real_T Selector6[2];                 /* '<S183>/Selector6' */
  real_T Selector4[2];                 /* '<S183>/Selector4' */
  real_T Sum3;                         /* '<S184>/Sum3' */
  real_T Sum6;                         /* '<S184>/Sum6' */
  real_T Sum[2];                       /* '<S184>/Sum' */
  real_T Product1[2];                  /* '<S184>/Product1' */
  real_T AngleTangentLimit[2];         /* '<S184>/Angle Tangent Limit' */
  real_T DataTypeConversion_bj;        /* '<S187>/Data Type Conversion' */
  real_T Product_p;                    /* '<S187>/Product' */
  real_T SumofElements_e;              /* '<S187>/Sum of Elements' */
  real_T TrigonometricFunction1[2];    /* '<S184>/Trigonometric Function1' */
  real_T Sum1[2];                      /* '<S184>/Sum1' */
  real_T deltaTheta;                   /* '<S184>/Sum2' */
  real_T antiswaybartorque;            /* '<S184>/Product4' */
  real_T Product2;                     /* '<S184>/Product2' */
  real_T TrigonometricFunction2[2];    /* '<S184>/Trigonometric Function2' */
  real_T Product3[2];                  /* '<S184>/Product3' */
  real_T Selector[2];                  /* '<S183>/Selector' */
  real_T Sum4[2];                      /* '<S184>/Sum4' */
  real_T Selector2[2];                 /* '<S183>/Selector2' */
  real_T Sum5[2];                      /* '<S184>/Sum5' */
  boolean_T RelationalOperator;        /* '<S186>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S185>/Relational Operator' */
  boolean_T RelationalOperator_o;      /* '<S187>/Relational Operator' */
} B_CoreSubsys_CAVE_MachE_sil_test_o_T;

/* Block signals for system '<S140>/For each track and axle combination calculate suspension forces and moments' */
typedef struct {
  real_T TmpSignalConversionAtSelector3Inport1[3];/* '<S189>/Mux' */
  real_T Selector1;                    /* '<S188>/Selector1' */
  real_T Sum2;                         /* '<S188>/Sum2' */
  real_T Selector5;                    /* '<S189>/Selector5' */
  real_T Product;                      /* '<S189>/Product' */
  real_T Selector3;                    /* '<S189>/Selector3' */
  real_T Abs2;                         /* '<S193>/Abs2' */
  real_T DataTypeConversion;           /* '<S196>/Data Type Conversion' */
  real_T Product_n;                    /* '<S196>/Product' */
  real_T SumofElements;                /* '<S196>/Sum of Elements' */
  real_T Product2;                     /* '<S193>/Product2' */
  real_T DataTypeConversion_p;         /* '<S195>/Data Type Conversion' */
  real_T Product_h;                    /* '<S195>/Product' */
  real_T SumofElements_d;              /* '<S195>/Sum of Elements' */
  real_T DataTypeConversion_p5;        /* '<S194>/Data Type Conversion' */
  real_T Product_d;                    /* '<S194>/Product' */
  real_T SumofElements_i;              /* '<S194>/Sum of Elements' */
  real_T DataTypeConversion_a;         /* '<S214>/Data Type Conversion' */
  real_T Product_m;                    /* '<S214>/Product' */
  real_T SumofElements_k;              /* '<S214>/Sum of Elements' */
  real_T Abs;                          /* '<S203>/Abs' */
  real_T Product_hq;                   /* '<S203>/Product' */
  real_T Selector2[2];                 /* '<S188>/Selector2' */
  real_T Selector[2];                  /* '<S188>/Selector' */
  real_T Add;                          /* '<S203>/Add' */
  real_T DataTypeConversion_c;         /* '<S209>/Data Type Conversion' */
  real_T Product_k;                    /* '<S209>/Product' */
  real_T SumofElements_g;              /* '<S209>/Sum of Elements' */
  real_T DataTypeConversion_o;         /* '<S208>/Data Type Conversion' */
  real_T Product_c;                    /* '<S208>/Product' */
  real_T SumofElements_o;              /* '<S208>/Sum of Elements' */
  real_T Product4;                     /* '<S204>/Product4' */
  real_T Add4;                         /* '<S204>/Add4' */
  real_T HeightSignConvention;         /* '<S204>/Height Sign Convention' */
  real_T Product3;                     /* '<S193>/Product3' */
  real_T Sum2_b;                       /* '<S193>/Sum2' */
  real_T Abs1;                         /* '<S193>/Abs1' */
  real_T DataTypeConversion_m;         /* '<S199>/Data Type Conversion' */
  real_T Product_b;                    /* '<S199>/Product' */
  real_T SumofElements_o0;             /* '<S199>/Sum of Elements' */
  real_T Product4_j;                   /* '<S193>/Product4' */
  real_T DataTypeConversion_ck;        /* '<S198>/Data Type Conversion' */
  real_T Product_l;                    /* '<S198>/Product' */
  real_T SumofElements_j;              /* '<S198>/Sum of Elements' */
  real_T DataTypeConversion_b;         /* '<S197>/Data Type Conversion' */
  real_T Product_g;                    /* '<S197>/Product' */
  real_T SumofElements_c;              /* '<S197>/Sum of Elements' */
  real_T Product5;                     /* '<S193>/Product5' */
  real_T Sum1;                         /* '<S193>/Sum1' */
  real_T Abs_d;                        /* '<S193>/Abs' */
  real_T DataTypeConversion_d;         /* '<S202>/Data Type Conversion' */
  real_T Product_i;                    /* '<S202>/Product' */
  real_T SumofElements_oi;             /* '<S202>/Sum of Elements' */
  real_T Product_lx;                   /* '<S193>/Product' */
  real_T DataTypeConversion_f;         /* '<S201>/Data Type Conversion' */
  real_T Product_e;                    /* '<S201>/Product' */
  real_T SumofElements_f;              /* '<S201>/Sum of Elements' */
  real_T DataTypeConversion_pi;        /* '<S200>/Data Type Conversion' */
  real_T Product_o;                    /* '<S200>/Product' */
  real_T SumofElements_h;              /* '<S200>/Sum of Elements' */
  real_T Product1;                     /* '<S193>/Product1' */
  real_T Sum;                          /* '<S193>/Sum' */
  real_T Product3_n;                   /* '<S204>/Product3' */
  real_T DataTypeConversion_p4;        /* '<S207>/Data Type Conversion' */
  real_T Product_l4;                   /* '<S207>/Product' */
  real_T SumofElements_p;              /* '<S207>/Sum of Elements' */
  real_T Add2;                         /* '<S203>/Add2' */
  real_T Product5_c;                   /* '<S204>/Product5' */
  real_T Add1;                         /* '<S204>/Add1' */
  real_T Product1_o;                   /* '<S204>/Product1' */
  real_T Sign1;                        /* '<S204>/Sign1' */
  real_T Product2_d;                   /* '<S204>/Product2' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[6];/* '<S181>/Suspension' */
  real_T DataTypeConversion_g;         /* '<S210>/Data Type Conversion' */
  real_T Product_gk;                   /* '<S210>/Product' */
  real_T SumofElements_dv;             /* '<S210>/Sum of Elements' */
  real_T Sum_l;                        /* '<S206>/Sum' */
  real_T Sum_p;                        /* '<S204>/Sum' */
  real_T VehicleForceSign;             /* '<S203>/Vehicle Force Sign' */
  real_T Selector1_c;                  /* '<S191>/Selector1' */
  real_T Selector_h;                   /* '<S191>/Selector' */
  real_T VehicleHeight;                /* '<S203>/Sign convention' */
  real_T Sum_f;                        /* '<S191>/Sum' */
  real_T Product_j;                    /* '<S191>/Product' */
  real_T UnaryMinus;                   /* '<S191>/Unary Minus' */
  real_T Selector2_g;                  /* '<S191>/Selector2' */
  real_T Product1_l;                   /* '<S191>/Product1' */
  real_T Reshape[3];                   /* '<S191>/Reshape' */
  real_T Selector3_k[3];               /* '<S191>/Selector3' */
  real_T Sum1_f[3];                    /* '<S191>/Sum1' */
  real_T Sum3;                         /* '<S193>/Sum3' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[3];/* '<S181>/Suspension' */
  real_T Selector3_i;                  /* '<S188>/Selector3' */
  boolean_T RelationalOperator;        /* '<S196>/Relational Operator' */
  boolean_T RelationalOperator_l;      /* '<S195>/Relational Operator' */
  boolean_T RelationalOperator_o;      /* '<S194>/Relational Operator' */
  boolean_T RelationalOperator_a;      /* '<S214>/Relational Operator' */
  boolean_T RelationalOperator_c;      /* '<S209>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S208>/Relational Operator' */
  boolean_T RelationalOperator_ag;     /* '<S199>/Relational Operator' */
  boolean_T RelationalOperator_l5;     /* '<S198>/Relational Operator' */
  boolean_T RelationalOperator_e;      /* '<S197>/Relational Operator' */
  boolean_T RelationalOperator_n;      /* '<S202>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S201>/Relational Operator' */
  boolean_T RelationalOperator_m;      /* '<S200>/Relational Operator' */
  boolean_T RelationalOperator_mm;     /* '<S207>/Relational Operator' */
  boolean_T RelationalOperator_nm;     /* '<S210>/Relational Operator' */
  B_Maxstopreached_CAVE_MachE_sil_test_T Maxstopreached;/* '<S206>/Max stop reached' */
  B_Minstopreached_CAVE_MachE_sil_test_T Minstopreached;/* '<S206>/Min stop reached' */
} B_CoreSubsys_CAVE_MachE_sil_test_n_T;

/* Block signals for system '<S246>/For Each Subsystem' */
typedef struct {
  real_T Product[3];                   /* '<S247>/Product' */
} B_CoreSubsys_CAVE_MachE_sil_test_b_T;

/* Block signals for system '<S313>/Wheel to Body Transform' */
typedef struct {
  real_T TmpSignalConversionAtsincosInport1[3];/* '<S320>/In1' */
  real_T sincos_o1[3];                 /* '<S320>/sincos' */
  real_T sincos_o2[3];                 /* '<S320>/sincos' */
  real_T VectorConcatenate[9];         /* '<S321>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor[9];
                                /* '<S321>/Reshape (9) to [3x3] column-major' */
  real_T Divide1[3];                   /* '<S319>/Divide1' */
} B_CoreSubsys_CAVE_MachE_sil_test_c_T;

/* Block signals for system '<S344>/Magic Tire Const Input' */
typedef struct {
  real_T Fx;                           /* '<S344>/Magic Tire Const Input' */
  real_T Fy;                           /* '<S344>/Magic Tire Const Input' */
  real_T FzTire;                       /* '<S344>/Magic Tire Const Input' */
  real_T My;                           /* '<S344>/Magic Tire Const Input' */
  real_T Re;                           /* '<S344>/Magic Tire Const Input' */
  real_T sig_x;                        /* '<S344>/Magic Tire Const Input' */
  real_T sig_y;                        /* '<S344>/Magic Tire Const Input' */
  real_T a;                            /* '<S344>/Magic Tire Const Input' */
  real_T b;                            /* '<S344>/Magic Tire Const Input' */
} B_MagicTireConstInput_CAVE_MachE_sil_test_T;

/* Block signals for system '<S346>/LockUp' */
typedef struct {
  real_T Tout;                         /* '<S346>/LockUp' */
  real_T Tfmaxs;                       /* '<S346>/LockUp' */
  real_T Tout_l;                       /* '<S346>/LockUp' */
  real_T Tfmaxs_b;                     /* '<S346>/LockUp' */
  real_T Omega;                        /* '<S346>/LockUp' */
  real_T Omegadot;                     /* '<S346>/LockUp' */
  real_T ReactionTorque;               /* '<S346>/LockUp' */
  real_T Myb;                          /* '<S346>/LockUp' */
  real_T omegawheel;                   /* '<S357>/omega wheel' */
  real_T u;                            /* '<S357>/-4' */
  real_T TrigonometricFunction;        /* '<S357>/Trigonometric Function' */
  real_T MaxDynamicFrictionTorque1;  /* '<S357>/Max Dynamic Friction Torque1' */
  real_T OutputDamping;                /* '<S357>/Output Damping' */
  real_T OutputSum;                    /* '<S357>/Output Sum' */
  real_T OutputInertia;                /* '<S357>/Output Inertia' */
  real_T OutputDamping_j;              /* '<S364>/Output Damping' */
  real_T Sum2;                         /* '<S364>/Sum2' */
  real_T Sum1;                         /* '<S364>/Sum1' */
  real_T Abs;                          /* '<S361>/Abs' */
  real_T UnaryMinus;                   /* '<S365>/Unary Minus' */
  real_T Abs_e;                        /* '<S366>/Abs' */
  real_T Abs_l;                        /* '<S367>/Abs' */
  boolean_T RelationalOperator;        /* '<S361>/Relational Operator' */
  boolean_T RelationalOperator_c;      /* '<S366>/Relational Operator' */
  boolean_T UnitDelay;                 /* '<S363>/Unit Delay' */
  boolean_T CombinatorialLogic;        /* '<S363>/Combinatorial  Logic' */
  boolean_T RelationalOperator_d;      /* '<S367>/Relational Operator' */
} B_LockUp_CAVE_MachE_sil_test_T;

/* Block states (default storage) for system '<S346>/LockUp' */
typedef struct {
  real_T lastMajorTime;                /* '<S346>/LockUp' */
  boolean_T UnitDelay_DSTATE;          /* '<S363>/Unit Delay' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c6_autolibshared;  /* '<S346>/LockUp' */
  uint8_T is_c6_autolibshared;         /* '<S346>/LockUp' */
} DW_LockUp_CAVE_MachE_sil_test_T;

/* Continuous states for system '<S346>/LockUp' */
typedef struct {
  real_T omegaWheel_l;                 /* '<S357>/omega wheel' */
} X_LockUp_CAVE_MachE_sil_test_T;

/* State derivatives for system '<S346>/LockUp' */
typedef struct {
  real_T omegaWheel_l;                 /* '<S357>/omega wheel' */
} XDot_LockUp_CAVE_MachE_sil_test_T;

/* State Disabled for system '<S346>/LockUp' */
typedef struct {
  boolean_T omegaWheel_l;              /* '<S357>/omega wheel' */
} XDis_LockUp_CAVE_MachE_sil_test_T;

/* Block signals for system '<S315>/Wheel to Body Transform' */
typedef struct {
  real_T TmpSignalConversionAtsincosInport1[3];/* '<S445>/In1' */
  real_T sincos_o1[3];                 /* '<S445>/sincos' */
  real_T sincos_o2[3];                 /* '<S445>/sincos' */
  real_T VectorConcatenate[9];         /* '<S446>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor[9];
                                /* '<S446>/Reshape (9) to [3x3] column-major' */
  real_T Divide1[3];                   /* '<S444>/Divide1' */
} B_CoreSubsys_CAVE_MachE_sil_test_ca_T;

/* Block signals (default storage) */
typedef struct {
  real_T xeyeze[3];                    /* '<S218>/xe,ye,ze' */
  real_T phithetapsi[3];               /* '<S230>/phi theta psi' */
  real_T VectorConcatenate[4];         /* '<S1>/Vector Concatenate' */
  real_T ubvbwb[3];                    /* '<S218>/ub,vb,wb' */
  real_T UnitConversion[3];            /* '<S234>/Unit Conversion' */
  real_T EnrgyTrans;                   /* '<S9>/Integrator' */
  real_T UnitConversion_c;             /* '<S9>/Unit Conversion' */
  real_T Constant;                     /* '<S1>/Constant' */
  real_T GearCmd;                      /* '<S1>/Constant2' */
  real_T CltchCmd;                     /* '<S1>/Constant3' */
  real_T lgSw;                         /* '<S1>/Constant4' */
  real_T Switch2;                      /* '<S1>/Switch2' */
  real_T SteeringCmd;                  /* '<S1>/EV Bolt Steer' */
  real_T Constant_i;                   /* '<S6>/Constant' */
  real_T Constant1;                    /* '<S6>/Constant1' */
  real_T VectorConcatenate_j[3];       /* '<S6>/Vector Concatenate' */
  real_T AccelCmd;                     /* '<S1>/Step' */
  real_T AccelCmd_b;                   /* '<S1>/Switch' */
  real_T DecelCmd;                     /* '<S1>/Switch1' */
  real_T CarTrq_T2WFL;                 /* '<S120>/Transfer Fcn3' */
  real_T CarTrq_T2WFR;                 /* '<S120>/Transfer Fcn1' */
  real_T CarTrq_T2WRL;                 /* '<S120>/Transfer Fcn2' */
  real_T CarTrq_T2WRR;                 /* '<S120>/Transfer Fcn4' */
  real_T TransferFcn;                  /* '<S120>/Transfer Fcn' */
  real_T TransferFcn9;                 /* '<S120>/Transfer Fcn9' */
  real_T TransferFcn10;                /* '<S120>/Transfer Fcn10' */
  real_T TransferFcn11;                /* '<S120>/Transfer Fcn11' */
  real_T Memory;                       /* '<S30>/Memory' */
  real_T Gain3;                        /* '<S30>/Gain3' */
  real_T T_FL;                         /* '<S30>/Gain4' */
  real_T VectorConcatenate_l[4];       /* '<S30>/Vector Concatenate' */
  real_T Memory1;                      /* '<S30>/Memory1' */
  real_T Abs;                          /* '<S39>/Abs' */
  real_T Divide;                       /* '<S39>/Divide' */
  real_T Integrator;                   /* '<S39>/Integrator' */
  real_T Gain1;                        /* '<S39>/Gain1' */
  real_T Merge;                        /* '<S39>/Merge' */
  real_T VectorConcatenate_p[4];       /* '<S4>/Vector Concatenate' */
  real_T Curr;                         /* '<S12>/Memory' */
  real_T Gain1_i;                      /* '<S23>/Gain1' */
  real_T Switch;                       /* '<S23>/Switch' */
  real_T IntegratorLimited;            /* '<S23>/Integrator Limited' */
  real_T Divide_a;                     /* '<S24>/Divide' */
  real_T Em;                           /* '<S25>/Em' */
  real_T R;                            /* '<S25>/R' */
  real_T Gain2;                        /* '<S25>/Gain2' */
  real_T Product;                      /* '<S25>/Product' */
  real_T Subtract;                     /* '<S25>/Subtract' */
  real_T Gain1_c;                      /* '<S25>/Gain1' */
  real_T Product1;                     /* '<S25>/Product1' */
  real_T Gain3_o;                      /* '<S25>/Gain3' */
  real_T Gain4;                        /* '<S25>/Gain4' */
  real_T Product_b;                    /* '<S20>/Product' */
  real_T Gain;                         /* '<S20>/Gain' */
  real_T Gain1_e;                      /* '<S20>/Gain1' */
  real_T Add;                          /* '<S20>/Add' */
  real_T AxlTrqLump;                   /* '<S30>/Add' */
  real_T Reshape4[4];                  /* '<S315>/Reshape4' */
  real_T Gain4_a[4];                   /* '<S315>/Gain4' */
  real_T Integrator_k;                 /* '<S350>/Integrator' */
  real_T Integrator_a;                 /* '<S347>/Integrator' */
  real_T VectorConcatenate_g[4];       /* '<S120>/Vector Concatenate' */
  real_T IntegratorSecondOrder_o1;     /* '<S345>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2;     /* '<S345>/Integrator, Second-Order' */
  real_T Sum6;                         /* '<S345>/Sum6' */
  real_T Saturation;                   /* '<S345>/Saturation' */
  real_T Add2;                         /* '<S346>/Add2' */
  real_T Product3;                     /* '<S346>/Product3' */
  real_T Add1;                         /* '<S346>/Add1' */
  real_T Saturation_f;                 /* '<S54>/Saturation' */
  real_T BrkTrqReqTotal;               /* '<S54>/Gain1' */
  real_T Saturation1;                  /* '<S70>/Saturation1' */
  real_T Divide_m;                     /* '<S70>/Divide' */
  real_T Saturation_n;                 /* '<S70>/Saturation' */
  real_T MotTrqMaxWhls;                /* '<S54>/MotTrqReflectedToWheels' */
  real_T min;                          /* '<S54>/MinMax' */
  real_T RegenBrakingCutoff;           /* '<S54>/RegenBrakingCutoff' */
  real_T ChrgLmt;                      /* '<S54>/ChrgLmt' */
  real_T RegenFactor;                  /* '<S54>/Product1' */
  real_T MotTrqRegenWhl;               /* '<S54>/Product3' */
  real_T Subtract_k;                   /* '<S54>/Subtract' */
  real_T BrkTrqReqTotal_o;             /* '<S54>/Gain2' */
  real_T Saturation1_g;                /* '<S54>/Saturation1' */
  real_T Gain_j;                       /* '<S121>/Gain' */
  real_T Gain2_p;                      /* '<S121>/Gain2' */
  real_T Gain3_n;                      /* '<S121>/Gain3' */
  real_T Gain1_n;                      /* '<S121>/Gain1' */
  real_T Gain4_f;                      /* '<S121>/Gain4' */
  real_T VectorConcatenate2[4];        /* '<S121>/Vector Concatenate2' */
  real_T Reshape3[4];                  /* '<S315>/Reshape3' */
  real_T TorqueConversion1;            /* '<S355>/Torque Conversion1' */
  real_T product;                      /* '<S355>/product' */
  real_T DisallowNegativeBrakeTorque;
                                   /* '<S355>/Disallow Negative Brake Torque' */
  real_T TorqueConversion;             /* '<S355>/Torque Conversion' */
  real_T Ratioofstatictokinetic;       /* '<S352>/Ratio of static to kinetic' */
  real_T Integrator_f;                 /* '<S375>/Integrator' */
  real_T Integrator_l;                 /* '<S372>/Integrator' */
  real_T IntegratorSecondOrder_o1_a;   /* '<S370>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_n;   /* '<S370>/Integrator, Second-Order' */
  real_T Sum6_p;                       /* '<S370>/Sum6' */
  real_T Saturation_d;                 /* '<S370>/Saturation' */
  real_T Add2_l;                       /* '<S371>/Add2' */
  real_T Product3_h;                   /* '<S371>/Product3' */
  real_T Add1_g;                       /* '<S371>/Add1' */
  real_T TorqueConversion1_e;          /* '<S380>/Torque Conversion1' */
  real_T product_l;                    /* '<S380>/product' */
  real_T DisallowNegativeBrakeTorque_c;
                                   /* '<S380>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_p;           /* '<S380>/Torque Conversion' */
  real_T Ratioofstatictokinetic_n;     /* '<S377>/Ratio of static to kinetic' */
  real_T Integrator_ah;                /* '<S400>/Integrator' */
  real_T Integrator_fm;                /* '<S397>/Integrator' */
  real_T IntegratorSecondOrder_o1_o;   /* '<S395>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_p;   /* '<S395>/Integrator, Second-Order' */
  real_T Sum6_f;                       /* '<S395>/Sum6' */
  real_T Saturation_j;                 /* '<S395>/Saturation' */
  real_T Add2_e;                       /* '<S396>/Add2' */
  real_T Product3_l;                   /* '<S396>/Product3' */
  real_T Add1_k;                       /* '<S396>/Add1' */
  real_T TorqueConversion1_b;          /* '<S405>/Torque Conversion1' */
  real_T product_c;                    /* '<S405>/product' */
  real_T DisallowNegativeBrakeTorque_d;
                                   /* '<S405>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_e;           /* '<S405>/Torque Conversion' */
  real_T Ratioofstatictokinetic_c;     /* '<S402>/Ratio of static to kinetic' */
  real_T Integrator_p;                 /* '<S425>/Integrator' */
  real_T Integrator_b;                 /* '<S422>/Integrator' */
  real_T IntegratorSecondOrder_o1_h;   /* '<S420>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_h;   /* '<S420>/Integrator, Second-Order' */
  real_T Sum6_o;                       /* '<S420>/Sum6' */
  real_T Saturation_nz;                /* '<S420>/Saturation' */
  real_T Add2_d;                       /* '<S421>/Add2' */
  real_T Product3_i;                   /* '<S421>/Product3' */
  real_T Add1_ge;                      /* '<S421>/Add1' */
  real_T TorqueConversion1_g;          /* '<S430>/Torque Conversion1' */
  real_T product_m;                    /* '<S430>/product' */
  real_T DisallowNegativeBrakeTorque_a;
                                   /* '<S430>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_n;           /* '<S430>/Torque Conversion' */
  real_T Ratioofstatictokinetic_b;     /* '<S427>/Ratio of static to kinetic' */
  real_T VectorConcatenate_h[4];       /* '<S340>/Vector Concatenate' */
  real_T MultiportSwitch[4];           /* '<S4>/Multiport Switch' */
  real_T Add1_j;                       /* '<S30>/Add1' */
  real_T Gain1_k;                      /* '<S30>/Gain1' */
  real_T Spd;                          /* '<S30>/Gain2' */
  real_T TransferredPower;             /* '<S31>/Transferred Power' */
  real_T PwrMechLoss;                  /* '<S31>/Constant' */
  real_T PwrDampLoss;                  /* '<S31>/Constant1' */
  real_T PwrStoredShft;                /* '<S31>/Constant2' */
  real_T VectorConcatenate_e[3];       /* '<S31>/Vector Concatenate' */
  real_T VehSpdKph;                    /* '<S4>/Gain1' */
  real_T Product_o;                    /* '<S41>/Product' */
  real_T uDLookupTable;                /* '<S41>/2-D Lookup Table' */
  real_T Add_d;                        /* '<S38>/Add' */
  real_T Saturation_g;                 /* '<S38>/Saturation' */
  real_T Divide_f;                     /* '<S38>/Divide' */
  real_T Saturation1_d;                /* '<S66>/Saturation1' */
  real_T Divide_o;                     /* '<S66>/Divide' */
  real_T Saturation_l;                 /* '<S66>/Saturation' */
  real_T Saturation1_a;                /* '<S55>/Saturation1' */
  real_T Divide_l;                     /* '<S55>/Divide' */
  real_T Saturation_h;                 /* '<S55>/Saturation' */
  real_T Product1_g;                   /* '<S51>/Product1' */
  real_T WhlTrqReflectedToMot;         /* '<S54>/WhlTrqReflectedToMot' */
  real_T MotTrqCmdRegen;               /* '<S54>/Gain' */
  real_T AccelDecelSwitch;             /* '<S53>/Accel Decel Switch' */
  real_T Abs_l;                        /* '<S57>/Abs' */
  real_T DischrgLmt;                   /* '<S52>/DischrgLmt' */
  real_T Product_f;                    /* '<S52>/Product' */
  real_T Product3_f;                   /* '<S60>/Product3' */
  real_T rads_to_rpm;                  /* '<S60>/rads_to_rpm' */
  real_T Abs_f;                        /* '<S60>/Abs' */
  real_T Abs1;                         /* '<S60>/Abs1' */
  real_T EffMap;                       /* '<S60>/Eff Map' */
  real_T Gain1_h;                      /* '<S60>/Gain1' */
  real_T Product_o4;                   /* '<S60>/Product' */
  real_T Switch2_d;                    /* '<S60>/Switch2' */
  real_T MathFunction;                 /* '<S60>/Math Function' */
  real_T Product4;                     /* '<S60>/Product4' */
  real_T Subtract_e;                   /* '<S58>/Subtract' */
  real_T ChrgLmt_c;                    /* '<S52>/ChrgLmt' */
  real_T Product1_h;                   /* '<S52>/Product1' */
  real_T Subtract1;                    /* '<S58>/Subtract1' */
  real_T Switch_b;                     /* '<S65>/Switch' */
  real_T Switch2_h;                    /* '<S65>/Switch2' */
  real_T ElecToMechPwr;                /* '<S57>/ElecToMechPwr' */
  real_T UnaryMinus;                   /* '<S62>/Unary Minus' */
  real_T Switch1;                      /* '<S62>/Switch1' */
  real_T Fcn;                          /* '<S62>/Fcn' */
  real_T Product_d;                    /* '<S62>/Product' */
  real_T Switch_bq;                    /* '<S62>/Switch' */
  real_T MechPwrToTrq;                 /* '<S57>/MechPwrToTrq' */
  real_T Switch_n;                     /* '<S57>/Switch' */
  real_T Switch1_n;                    /* '<S57>/Switch1' */
  real_T Gain_j4;                      /* '<S61>/Gain' */
  real_T Switch_bu;                    /* '<S67>/Switch' */
  real_T Switch2_k;                    /* '<S67>/Switch2' */
  real_T Sum;                          /* '<S39>/Sum' */
  real_T Add_p;                        /* '<S40>/Add' */
  real_T Gain_k;                       /* '<S40>/Gain' */
  real_T Gain1_l;                      /* '<S40>/Gain1' */
  real_T Subtract_i;                   /* '<S40>/Subtract' */
  real_T mph2ms;                       /* '<S14>/mph2m//s' */
  real_T utireRadius;                  /* '<S14>/1//tireRadius' */
  real_T Constant_c;                   /* '<S14>/Constant' */
  real_T APPTorque;                    /* '<S71>/APPToTorque' */
  real_T APPToTorque1;                 /* '<S71>/APPToTorque1' */
  real_T BPPTorque;                    /* '<S71>/Flip' */
  real_T TorqueTotal;                  /* '<S71>/Sum' */
  real_T omegawheel;                   /* '<S80>/omega wheel' */
  real_T omegawheel_d;                 /* '<S81>/omega wheel' */
  real_T omegawheel_g;                 /* '<S82>/omega wheel' */
  real_T omegawheel_k;                 /* '<S83>/omega wheel' */
  real_T Saturation_fw[4];             /* '<S73>/Saturation' */
  real_T Speed_mps[4];                 /* '<S71>/WheelRadius' */
  real_T Speed_kph[4];                 /* '<S71>/mps2kph' */
  real_T Speed_mph[4];                 /* '<S71>/kph2mph' */
  real_T Gear[4];                      /* '<S71>/NissanTrans' */
  real_T TransTorqueOut[4];            /* '<S71>/Gear' */
  real_T WheelTorqueOut[4];            /* '<S71>/FDR' */
  real_T Memory_j[4];                  /* '<S71>/Memory' */
  real_T MultiportSwitch1[4];          /* '<S14>/Multiport Switch1' */
  real_T PTWFLrot;                     /* '<S72>/Discrete-Time Integrator4' */
  real_T PTWFRrot;                     /* '<S72>/Discrete-Time Integrator5' */
  real_T PTWRLrot;                     /* '<S72>/Discrete-Time Integrator6' */
  real_T PtTWRRrot;                    /* '<S72>/Discrete-Time Integrator7' */
  real_T Memory3;                      /* '<S72>/Memory3' */
  real_T Saturation3;                  /* '<S72>/Saturation3' */
  real_T Product3_j;                   /* '<S72>/Product3' */
  real_T Gain5;                        /* '<S72>/Gain5' */
  real_T Memory1_m;                    /* '<S72>/Memory1' */
  real_T Saturation1_f;                /* '<S72>/Saturation1' */
  real_T Product1_p;                   /* '<S72>/Product1' */
  real_T Gain6;                        /* '<S72>/Gain6' */
  real_T Memory2;                      /* '<S72>/Memory2' */
  real_T Saturation2;                  /* '<S72>/Saturation2' */
  real_T Product2;                     /* '<S72>/Product2' */
  real_T Gain7;                        /* '<S72>/Gain7' */
  real_T Memory4;                      /* '<S72>/Memory4' */
  real_T Saturation4;                  /* '<S72>/Saturation4' */
  real_T Product4_g;                   /* '<S72>/Product4' */
  real_T Gain8;                        /* '<S72>/Gain8' */
  real_T PTIgnition;                   /* '<S72>/Ignition' */
  real_T OperationError;               /* '<S72>/Operation Error' */
  real_T OperationStateDriving;        /* '<S72>/Operation State Driving' */
  real_T Zero1;                        /* '<S72>/Zero1' */
  real_T Zero2;                        /* '<S72>/Zero2' */
  real_T Zero3;                        /* '<S72>/Zero3' */
  real_T Zero4;                        /* '<S72>/Zero4' */
  real_T Zero5;                        /* '<S72>/Zero5' */
  real_T PTWFLrot_a;                   /* '<S78>/Discrete-Time Integrator4' */
  real_T PTWFRrot_n;                   /* '<S78>/Discrete-Time Integrator5' */
  real_T PTWRLrot_n;                   /* '<S78>/Discrete-Time Integrator6' */
  real_T PtTWRRrot_c;                  /* '<S78>/Discrete-Time Integrator7' */
  real_T Memory3_d;                    /* '<S78>/Memory3' */
  real_T Saturation3_d;                /* '<S78>/Saturation3' */
  real_T BrakeTrqFL;                   /* '<S78>/Product3' */
  real_T Gain5_m;                      /* '<S78>/Gain5' */
  real_T Memory1_o;                    /* '<S78>/Memory1' */
  real_T Saturation1_b;                /* '<S78>/Saturation1' */
  real_T BrakeTrqFR;                   /* '<S78>/Product1' */
  real_T Gain6_b;                      /* '<S78>/Gain6' */
  real_T Memory2_n;                    /* '<S78>/Memory2' */
  real_T Saturation2_f;                /* '<S78>/Saturation2' */
  real_T BrakeTrqRL;                   /* '<S78>/Product2' */
  real_T Gain7_g;                      /* '<S78>/Gain7' */
  real_T Memory4_m;                    /* '<S78>/Memory4' */
  real_T Saturation4_g;                /* '<S78>/Saturation4' */
  real_T BrakeTrqRR;                   /* '<S78>/Product4' */
  real_T Gain8_a;                      /* '<S78>/Gain8' */
  real_T PTIgnition_f;                 /* '<S78>/Ignition' */
  real_T OperationError_b;             /* '<S78>/Operation Error' */
  real_T OperationStateDriving_j;      /* '<S78>/Operation State Driving' */
  real_T MinMax;                       /* '<S98>/MinMax' */
  real_T Divide1;                      /* '<S98>/Divide1' */
  real_T Product_k;                    /* '<S98>/Product' */
  real_T Sum3;                         /* '<S98>/Sum3' */
  real_T Memory_h;                     /* '<S98>/Memory' */
  real_T Product2_c;                   /* '<S98>/Product2' */
  real_T Add_b;                        /* '<S98>/Add' */
  real_T LF;                           /* '<S74>/Data Type Conversion1' */
  real_T MinMax_f;                     /* '<S99>/MinMax' */
  real_T Divide1_o;                    /* '<S99>/Divide1' */
  real_T Product_p;                    /* '<S99>/Product' */
  real_T Sum3_h;                       /* '<S99>/Sum3' */
  real_T Memory_c;                     /* '<S99>/Memory' */
  real_T Product2_i;                   /* '<S99>/Product2' */
  real_T Add_k;                        /* '<S99>/Add' */
  real_T RF;                           /* '<S74>/Data Type Conversion2' */
  real_T MinMax_l;                     /* '<S100>/MinMax' */
  real_T Divide1_d;                    /* '<S100>/Divide1' */
  real_T Product_j;                    /* '<S100>/Product' */
  real_T Sum3_b;                       /* '<S100>/Sum3' */
  real_T Memory_b;                     /* '<S100>/Memory' */
  real_T Product2_a;                   /* '<S100>/Product2' */
  real_T Add_d3;                       /* '<S100>/Add' */
  real_T LR;                           /* '<S74>/Data Type Conversion3' */
  real_T MinMax_h;                     /* '<S101>/MinMax' */
  real_T Divide1_n;                    /* '<S101>/Divide1' */
  real_T Product_b3;                   /* '<S101>/Product' */
  real_T Sum3_k;                       /* '<S101>/Sum3' */
  real_T Memory_a;                     /* '<S101>/Memory' */
  real_T Product2_n;                   /* '<S101>/Product2' */
  real_T Add_c;                        /* '<S101>/Add' */
  real_T RR;                           /* '<S74>/Data Type Conversion4' */
  real_T TorqueIn[4];                  /* '<S74>/TorqueIn' */
  real_T SatTrq[4];                    /* '<S74>/SatTrq' */
  real_T Trq[4];                       /* '<S74>/TorqueIn ' */
  real_T MultiportSwitch_n[4];         /* '<S14>/Multiport Switch' */
  real_T IntegratorSecondOrder_o1_d;   /* '<S334>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_b;   /* '<S334>/Integrator, Second-Order' */
  real_T Sum6_a;                       /* '<S334>/Sum6' */
  real_T Saturation_m;                 /* '<S334>/Saturation' */
  real_T Add2_g;                       /* '<S80>/Add2' */
  real_T Saturation_a;                 /* '<S80>/Saturation' */
  real_T Product3_k;                   /* '<S80>/Product3' */
  real_T Add1_c;                       /* '<S80>/Add1' */
  real_T Memory_m;                     /* '<S80>/Memory' */
  real_T Switch_c;                     /* '<S86>/Switch' */
  real_T Divide_fj;                    /* '<S86>/Divide' */
  real_T OutputDamping;                /* '<S80>/Output Damping' */
  real_T Sum_d;                        /* '<S80>/Sum' */
  real_T Switch_p;                     /* '<S80>/Switch' */
  real_T Sum3_c;                       /* '<S80>/Sum3' */
  real_T Switch_cx;                    /* '<S85>/Switch' */
  real_T Divide_b;                     /* '<S85>/Divide' */
  real_T IntegratorSecondOrder_o1_n;   /* '<S335>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_g;   /* '<S335>/Integrator, Second-Order' */
  real_T Sum6_k;                       /* '<S335>/Sum6' */
  real_T Saturation_b;                 /* '<S335>/Saturation' */
  real_T Add2_j;                       /* '<S81>/Add2' */
  real_T Saturation_k;                 /* '<S81>/Saturation' */
  real_T Product3_lg;                  /* '<S81>/Product3' */
  real_T Add1_l;                       /* '<S81>/Add1' */
  real_T Memory_cj;                    /* '<S81>/Memory' */
  real_T Switch_f;                     /* '<S89>/Switch' */
  real_T Divide_n;                     /* '<S89>/Divide' */
  real_T OutputDamping_k;              /* '<S81>/Output Damping' */
  real_T Sum3_ko;                      /* '<S81>/Sum3' */
  real_T Switch_k;                     /* '<S81>/Switch' */
  real_T Switch_j;                     /* '<S88>/Switch' */
  real_T Divide_k;                     /* '<S88>/Divide' */
  real_T IntegratorSecondOrder_o1_o4;  /* '<S336>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_d;   /* '<S336>/Integrator, Second-Order' */
  real_T Sum6_k0;                      /* '<S336>/Sum6' */
  real_T Saturation_i;                 /* '<S336>/Saturation' */
  real_T Add2_b;                       /* '<S82>/Add2' */
  real_T Saturation_d0;                /* '<S82>/Saturation' */
  real_T Product3_e;                   /* '<S82>/Product3' */
  real_T Add1_ge1;                     /* '<S82>/Add1' */
  real_T Memory_n;                     /* '<S82>/Memory' */
  real_T OutputDamping_m;              /* '<S82>/Output Damping' */
  real_T Switch_pa;                    /* '<S92>/Switch' */
  real_T Divide_ox;                    /* '<S92>/Divide' */
  real_T Sum_h;                        /* '<S82>/Sum' */
  real_T Sum3_f;                       /* '<S82>/Sum3' */
  real_T Switch_cv;                    /* '<S82>/Switch' */
  real_T Switch_o;                     /* '<S91>/Switch' */
  real_T Divide_d;                     /* '<S91>/Divide' */
  real_T IntegratorSecondOrder_o1_p;   /* '<S337>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_hy;  /* '<S337>/Integrator, Second-Order' */
  real_T Sum6_fh;                      /* '<S337>/Sum6' */
  real_T Saturation_o;                 /* '<S337>/Saturation' */
  real_T Add2_n;                       /* '<S83>/Add2' */
  real_T Saturation_c;                 /* '<S83>/Saturation' */
  real_T Product3_m;                   /* '<S83>/Product3' */
  real_T Add1_m;                       /* '<S83>/Add1' */
  real_T Switch_m;                     /* '<S95>/Switch' */
  real_T Divide_h;                     /* '<S95>/Divide' */
  real_T Gain_n;                       /* '<S83>/Gain' */
  real_T Memory_b2;                    /* '<S83>/Memory' */
  real_T Sum3_d;                       /* '<S83>/Sum3' */
  real_T Switch_cn;                    /* '<S83>/Switch' */
  real_T Switch_cw;                    /* '<S94>/Switch' */
  real_T Divide_i;                     /* '<S94>/Divide' */
  real_T Zero1_k;                      /* '<S78>/Zero1' */
  real_T Zero2_o;                      /* '<S78>/Zero2' */
  real_T Zero3_o;                      /* '<S78>/Zero3' */
  real_T Zero4_h;                      /* '<S78>/Zero4' */
  real_T Zero5_n;                      /* '<S78>/Zero5' */
  real_T DiffTrq;                      /* '<S79>/Product' */
  real_T Gain10;                       /* '<S79>/Gain10' */
  real_T Saturation1_i;                /* '<S73>/Saturation1' */
  real_T BrakeIFTrq_Reg_trgRR;         /* '<S75>/Zero1' */
  real_T BrakeIFTrq_DriveSrc_trgd3;    /* '<S75>/Zero2' */
  real_T BrakeIFTrq_PBRR;              /* '<S75>/Zero5' */
  real_T rads2rpm[4];                  /* '<S76>/rads2rpm' */
  real_T BrkLatchThr_Nm;               /* '<S102>/BrkLatchThr_Nm' */
  real_T ModeOut[4];                   /* '<S76>/ModeOut' */
  real_T SpeedOut[4];                  /* '<S76>/SpeedOut' */
  real_T RateLimSpd[4];                /* '<S76>/RateLimSpd' */
  real_T Spd_d[4];                     /* '<S76>/SatSpd' */
  real_T ModeSwitch[4];                /* '<S76>/ModeSwitch' */
  real_T SpTqRF;                       /* '<S112>/Bit2' */
  real_T Backlash;                     /* '<S130>/Backlash' */
  real_T PwrLoss;                      /* '<S130>/Constant' */
  real_T InstStrgRatio;                /* '<S131>/Constant' */
  real_T Saturation_oi;                /* '<S130>/Saturation' */
  real_T Gain_e;                       /* '<S131>/Gain' */
  real_T Gain1_g;                      /* '<S131>/Gain1' */
  real_T Add_bk;                       /* '<S132>/Add' */
  real_T TrqIn;                        /* '<S131>/Gain2' */
  real_T TrqIn_p;                      /* '<S130>/Unary Minus' */
  real_T UnaryMinus1;                  /* '<S129>/Unary Minus1' */
  real_T UnaryMinus_k;                 /* '<S129>/Unary Minus' */
  real_T VectorConcatenate2_b[4];      /* '<S122>/Vector Concatenate2' */
  real_T VectorConcatenate_n[4];       /* '<S327>/Vector Concatenate' */
  real_T Integrator_fu;                /* '<S348>/Integrator' */
  real_T Integrator_n;                 /* '<S373>/Integrator' */
  real_T Integrator_e;                 /* '<S398>/Integrator' */
  real_T Integrator_d;                 /* '<S423>/Integrator' */
  real_T VectorConcatenate_pf[4];      /* '<S328>/Vector Concatenate' */
  real_T Integrator1[4];               /* '<S310>/Integrator1' */
  real_T Saturation_du[4];             /* '<S128>/Saturation' */
  real_T VectorConcatenate8[12];       /* '<S313>/Vector Concatenate8' */
  real_T MathFunction_d[12];           /* '<S313>/Math Function' */
  real_T Integrator1_d[12];            /* '<S311>/Integrator1' */
  real_T CamberAngles[4];              /* '<S128>/Selector3' */
  real_T CamberAngles_l[4];            /* '<S128>/Manual Switch6' */
  real_T Add2_k[4];                    /* '<S318>/Add2' */
  real_T Reshape1[4];                  /* '<S318>/Reshape1' */
  real_T Reshape2[4];                  /* '<S318>/Reshape2' */
  real_T WheelAngles[4];               /* '<S128>/Selector2' */
  real_T Add1_f[4];                    /* '<S318>/Add1' */
  real_T Reshape[4];                   /* '<S318>/Reshape' */
  real_T VectorConcatenate3[12];       /* '<S318>/Vector Concatenate3' */
  real_T Reshape6[4];                  /* '<S139>/Reshape6' */
  real_T Reshape3_i[2];                /* '<S140>/Reshape3' */
  real_T Reshape7[4];                  /* '<S139>/Reshape7' */
  real_T Reshape4_e[2];                /* '<S140>/Reshape4' */
  real_T MatrixConcatenate4[4];        /* '<S140>/Matrix Concatenate4' */
  real_T VectorConcatenate_er[4];      /* '<S341>/Vector Concatenate' */
  real_T z[4];                         /* '<S317>/Unary Minus1' */
  real_T z_k[4];                       /* '<S139>/Reshape' */
  real_T Reshape9[2];                  /* '<S140>/Reshape9' */
  real_T VectorConcatenate_m[4];       /* '<S342>/Vector Concatenate' */
  real_T zdot[4];                      /* '<S317>/Unary Minus2' */
  real_T zdot_o[4];                    /* '<S139>/Reshape2' */
  real_T Reshape8[2];                  /* '<S140>/Reshape8' */
  real_T MatrixConcatenate2[4];        /* '<S140>/Matrix Concatenate2' */
  real_T TmpSignalConversionAtsincosInport1[3];/* '<S238>/In1' */
  real_T sincos_o1[3];                 /* '<S257>/sincos' */
  real_T sincos_o2[3];                 /* '<S257>/sincos' */
  real_T VectorConcatenate_l5[9];      /* '<S261>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor[9];
                                /* '<S261>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1[9];                /* '<S256>/Transpose1' */
  real_T Selector1[3];                 /* '<S251>/Selector1' */
  real_T Reshape1_n[3];                /* '<S259>/Reshape1' */
  real_T Product_l[3];                 /* '<S259>/Product' */
  real_T Reshape2_l[3];                /* '<S259>/Reshape2' */
  real_T Add_b3[3];                    /* '<S256>/Add' */
  real_T sincos_o1_h[3];               /* '<S265>/sincos' */
  real_T sincos_o2_g[3];               /* '<S265>/sincos' */
  real_T VectorConcatenate_ps[9];      /* '<S269>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_m[9];
                                /* '<S269>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_p[9];              /* '<S264>/Transpose1' */
  real_T Selector1_h[3];               /* '<S252>/Selector1' */
  real_T Reshape1_k[3];                /* '<S267>/Reshape1' */
  real_T Product_a[3];                 /* '<S267>/Product' */
  real_T Reshape2_i[3];                /* '<S267>/Reshape2' */
  real_T Add_kn[3];                    /* '<S264>/Add' */
  real_T sincos_o1_l[3];               /* '<S285>/sincos' */
  real_T sincos_o2_m[3];               /* '<S285>/sincos' */
  real_T VectorConcatenate_o[9];       /* '<S289>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_mx[9];
                                /* '<S289>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_f[9];              /* '<S284>/Transpose1' */
  real_T Selector1_c[3];               /* '<S254>/Selector1' */
  real_T Reshape1_b[3];                /* '<S287>/Reshape1' */
  real_T Product_aw[3];                /* '<S287>/Product' */
  real_T Reshape2_n[3];                /* '<S287>/Reshape2' */
  real_T Add_cn[3];                    /* '<S284>/Add' */
  real_T sincos_o1_c[3];               /* '<S293>/sincos' */
  real_T sincos_o2_gs[3];              /* '<S293>/sincos' */
  real_T VectorConcatenate_n5[9];      /* '<S297>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_k[9];
                                /* '<S297>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_fn[9];             /* '<S292>/Transpose1' */
  real_T Selector1_l[3];               /* '<S255>/Selector1' */
  real_T Reshape1_kj[3];               /* '<S295>/Reshape1' */
  real_T Product_e[3];                 /* '<S295>/Product' */
  real_T Reshape2_e[3];                /* '<S295>/Reshape2' */
  real_T Add_f[3];                     /* '<S292>/Add' */
  real_T Reshape_i[12];                /* '<S138>/Reshape' */
  real_T P[12];                        /* '<S138>/Sum' */
  real_T Selector10[6];                /* '<S134>/Selector10' */
  real_T Selector[2];                  /* '<S140>/Selector' */
  real_T Reshape7_p[2];                /* '<S140>/Reshape7' */
  real_T pqr[3];                       /* '<S218>/p,q,r ' */
  real_T jxk;                          /* '<S262>/j x k' */
  real_T kxi;                          /* '<S262>/k x i' */
  real_T ixj;                          /* '<S262>/i x j' */
  real_T kxj;                          /* '<S263>/k x j' */
  real_T ixk;                          /* '<S263>/i x k' */
  real_T jxi;                          /* '<S263>/j x i' */
  real_T Sum_j[3];                     /* '<S260>/Sum' */
  real_T Add1_a[3];                    /* '<S256>/Add1' */
  real_T jxk_f;                        /* '<S270>/j x k' */
  real_T kxi_g;                        /* '<S270>/k x i' */
  real_T ixj_o;                        /* '<S270>/i x j' */
  real_T kxj_b;                        /* '<S271>/k x j' */
  real_T ixk_k;                        /* '<S271>/i x k' */
  real_T jxi_b;                        /* '<S271>/j x i' */
  real_T Sum_k[3];                     /* '<S268>/Sum' */
  real_T Add1_my[3];                   /* '<S264>/Add1' */
  real_T jxk_g;                        /* '<S290>/j x k' */
  real_T kxi_k;                        /* '<S290>/k x i' */
  real_T ixj_b;                        /* '<S290>/i x j' */
  real_T kxj_d;                        /* '<S291>/k x j' */
  real_T ixk_c;                        /* '<S291>/i x k' */
  real_T jxi_j;                        /* '<S291>/j x i' */
  real_T Sum_hm[3];                    /* '<S288>/Sum' */
  real_T Add1_b[3];                    /* '<S284>/Add1' */
  real_T jxk_gi;                       /* '<S298>/j x k' */
  real_T kxi_o;                        /* '<S298>/k x i' */
  real_T ixj_e;                        /* '<S298>/i x j' */
  real_T kxj_e;                        /* '<S299>/k x j' */
  real_T ixk_b;                        /* '<S299>/i x k' */
  real_T jxi_o;                        /* '<S299>/j x i' */
  real_T Sum_p[3];                     /* '<S296>/Sum' */
  real_T Add1_p[3];                    /* '<S292>/Add1' */
  real_T V[12];                        /* '<S138>/Reshape2' */
  real_T Selector11[6];                /* '<S134>/Selector11' */
  real_T Selector1_f[2];               /* '<S140>/Selector1' */
  real_T Reshape6_l[2];                /* '<S140>/Reshape6' */
  real_T MatrixConcatenate3[4];        /* '<S140>/Matrix Concatenate3' */
  real_T Reshape1_a[2];                /* '<S137>/Reshape1' */
  real_T Selector4[2];                 /* '<S140>/Selector4' */
  real_T Reshape15[2];                 /* '<S140>/Reshape15' */
  real_T Selector5[2];                 /* '<S140>/Selector5' */
  real_T Reshape13[2];                 /* '<S140>/Reshape13' */
  real_T MatrixConcatenate6[6];        /* '<S140>/Matrix Concatenate6' */
  real_T Selector18[6];                /* '<S134>/Selector18' */
  real_T Selector7[2];                 /* '<S135>/Selector7' */
  real_T Selector8[2];                 /* '<S135>/Selector8' */
  real_T Reshape15_o[2];               /* '<S135>/Reshape15' */
  real_T MatrixConcatenate5[6];        /* '<S135>/Matrix Concatenate5' */
  real_T Reshape1_p;                   /* '<S141>/Reshape1' */
  real_T Memory1_b[2];                 /* '<S141>/Memory1' */
  real_T Reshape3_c[2];                /* '<S135>/Reshape3' */
  real_T Switch_k4[2];                 /* '<S141>/Switch' */
  real_T Reshape21[2];                 /* '<S141>/Reshape21' */
  real_T SumofElements;                /* '<S141>/Sum of Elements' */
  real_T MatrixConcatenate[3];         /* '<S141>/Matrix Concatenate' */
  real_T Reshape8_d[3];                /* '<S135>/Reshape8' */
  real_T Sum_f[3];                     /* '<S141>/Sum' */
  real_T Reshape9_c[3];                /* '<S135>/Reshape9' */
  real_T Reshape5[6];                  /* '<S135>/Reshape5' */
  real_T Sum2[6];                      /* '<S135>/Sum2' */
  real_T xdot[4];                      /* '<S134>/Matrix Concatenate1' */
  real_T ydot[4];                      /* '<S134>/Matrix Concatenate' */
  real_T Reshape1_o[4];                /* '<S317>/Reshape1' */
  real_T MatrixConcatenate_j[12];      /* '<S128>/Matrix Concatenate' */
  real_T Add2_f[4];                    /* '<S443>/Add2' */
  real_T Reshape1_i[4];                /* '<S443>/Reshape1' */
  real_T Reshape2_p[4];                /* '<S443>/Reshape2' */
  real_T Add1_h[4];                    /* '<S443>/Add1' */
  real_T Reshape_p[4];                 /* '<S443>/Reshape' */
  real_T VectorConcatenate3_p[12];     /* '<S443>/Vector Concatenate3' */
  real_T Reshape2_b[4];                /* '<S136>/Reshape2' */
  real_T MatrixConcatenate1[12];       /* '<S136>/Matrix Concatenate1' */
  real_T Reshape3_n[4];                /* '<S136>/Reshape3' */
  real_T Reshape4_g[4];                /* '<S136>/Reshape4' */
  real_T MatrixConcatenate_f[12];      /* '<S136>/Matrix Concatenate' */
  real_T AngVel[12];                   /* '<S136>/Add' */
  real_T Selector1_d[4];               /* '<S128>/Selector1' */
  real_T Reshape1_e[4];                /* '<S128>/Reshape1' */
  real_T UnaryMinus_m[4];              /* '<S315>/Unary Minus' */
  real_T VectorConcatenate1[4];        /* '<S120>/Vector Concatenate1' */
  real_T Reshape_ic[4];                /* '<S316>/Reshape' */
  real_T Reshape1_j[4];                /* '<S316>/Reshape1' */
  real_T VectorConcatenate_pm[108];    /* '<S316>/Vector Concatenate' */
  real_T Selector9[27];                /* '<S322>/Selector9' */
  real_T Selector19[27];               /* '<S322>/Selector19' */
  real_T Selector29[27];               /* '<S322>/Selector29' */
  real_T Selector39[27];               /* '<S322>/Selector39' */
  real_T VectorConcatenate_md[4];      /* '<S333>/Vector Concatenate' */
  real_T Re[4];                        /* '<S139>/Reshape1' */
  real_T Reshape17[2];                 /* '<S140>/Reshape17' */
  real_T VectorConcatenate_jk[4];      /* '<S330>/Vector Concatenate' */
  real_T Reshape3_cq[4];               /* '<S139>/Reshape3' */
  real_T VectorConcatenate_g3[4];      /* '<S331>/Vector Concatenate' */
  real_T Reshape4_ee[4];               /* '<S139>/Reshape4' */
  real_T VectorConcatenate_b[4];       /* '<S332>/Vector Concatenate' */
  real_T Reshape5_j[4];                /* '<S139>/Reshape5' */
  real_T M[12];                        /* '<S139>/Matrix Concatenate' */
  real_T Selector9_l[6];               /* '<S134>/Selector9' */
  real_T Reshape18[6];                 /* '<S140>/Reshape18' */
  real_T Reshape19[2];                 /* '<S140>/Reshape19' */
  real_T AssignWhlFz[2];               /* '<S182>/Assign WhlFz' */
  real_T Reshape2_m[2];                /* '<S140>/Reshape2' */
  real_T MatrixConcatenate1_p[6];      /* '<S140>/Matrix Concatenate1' */
  real_T Reshape10[2];                 /* '<S135>/Reshape10' */
  real_T Reshape11[2];                 /* '<S135>/Reshape11' */
  real_T MatrixConcatenate2_h[4];      /* '<S135>/Matrix Concatenate2' */
  real_T Reshape7_j[6];                /* '<S135>/Reshape7' */
  real_T Selector_i[2];                /* '<S135>/Selector' */
  real_T Sum_n[2];                     /* '<S145>/Sum' */
  real_T CarriertoAxleCompliance[2];   /* '<S145>/Carrier to Axle Compliance' */
  real_T Reshape6_a[6];                /* '<S135>/Reshape6' */
  real_T Selector1_fq[2];              /* '<S135>/Selector1' */
  real_T Reshape2_ms[2];               /* '<S135>/Reshape2' */
  real_T Sum2_h[2];                    /* '<S145>/Sum2' */
  real_T CarriertoAxleDamping[2];      /* '<S145>/Carrier to Axle Damping' */
  real_T Sum1[2];                      /* '<S145>/Sum1' */
  real_T Gain_g[2];                    /* '<S135>/Gain' */
  real_T MatrixConcatenate6_c[6];      /* '<S135>/Matrix Concatenate6' */
  real_T Reshape14[6];                 /* '<S140>/Reshape14' */
  real_T MatrixConcatenate1_f[4];      /* '<S135>/Matrix Concatenate1' */
  real_T Selector1_fc[6];              /* '<S134>/Selector1' */
  real_T Selector3[2];                 /* '<S135>/Selector3' */
  real_T Reshape12[2];                 /* '<S135>/Reshape12' */
  real_T Selector4_n[2];               /* '<S135>/Selector4' */
  real_T Reshape13_c[2];               /* '<S135>/Reshape13' */
  real_T MatrixConcatenate3_c[4];      /* '<S135>/Matrix Concatenate3' */
  real_T Reshape4_j[2];                /* '<S135>/Reshape4' */
  real_T Selector17[6];                /* '<S134>/Selector17' */
  real_T Reshape18_p[6];               /* '<S135>/Reshape18' */
  real_T AssignVehFz[2];               /* '<S182>/Assign VehFz' */
  real_T Reshape1_g[2];                /* '<S140>/Reshape1' */
  real_T MatrixConcatenate_fi[6];      /* '<S140>/Matrix Concatenate' */
  real_T Reshape1_h[2];                /* '<S135>/Reshape1' */
  real_T MatrixConcatenate_o[6];       /* '<S135>/Matrix Concatenate' */
  real_T Ang[12];                      /* '<S134>/Matrix Concatenate2' */
  real_T F[12];                        /* '<S134>/Matrix Concatenate4' */
  real_T M_i[12];                      /* '<S134>/Matrix Concatenate5' */
  real_T F_k[12];                      /* '<S134>/Matrix Concatenate6' */
  real_T Reshape_ps[12];               /* '<S135>/Reshape' */
  real_T Camberselect[2];              /* '<S135>/Camber select' */
  real_T Casterselect[2];              /* '<S135>/Caster select' */
  real_T Energyselect[2];              /* '<S135>/Energy select' */
  real_T Heightselect[2];              /* '<S135>/Height select' */
  real_T Powerselect[2];               /* '<S135>/Power select' */
  real_T Reshape16[6];                 /* '<S135>/Reshape16' */
  real_T Reshape17_m[6];               /* '<S135>/Reshape17' */
  real_T Reshape19_o[6];               /* '<S135>/Reshape19' */
  real_T Reshape20[6];                 /* '<S135>/Reshape20' */
  real_T Toeselect[2];                 /* '<S135>/Toe select' */
  real_T Reshape_m[12];                /* '<S140>/Reshape' */
  real_T Camberselect_m[2];            /* '<S140>/Camber select' */
  real_T Casterselect_l[2];            /* '<S140>/Caster select' */
  real_T Energyselect_p[2];            /* '<S140>/Energy select' */
  real_T Heightselect_l[2];            /* '<S140>/Height select' */
  real_T Powerselect_n[2];             /* '<S140>/Power select' */
  real_T Reshape10_g[6];               /* '<S140>/Reshape10' */
  real_T Reshape11_g[6];               /* '<S140>/Reshape11' */
  real_T Reshape12_b[6];               /* '<S140>/Reshape12' */
  real_T Reshape16_i[6];               /* '<S140>/Reshape16' */
  real_T Toeselect_h[2];               /* '<S140>/Toe select' */
  real_T ActTqLF_Nm_;                  /* '<S120>/Transfer Fcn5' */
  real_T ActTqRF_Nm_;                  /* '<S120>/Transfer Fcn6' */
  real_T ActTqLR_Nm_;                  /* '<S120>/Transfer Fcn7' */
  real_T ActTqRR_Nm_;                  /* '<S120>/Transfer Fcn8' */
  real_T sincos_o1_a[3];               /* '<S238>/sincos' */
  real_T sincos_o2_p[3];               /* '<S238>/sincos' */
  real_T VectorConcatenate_b5[9];      /* '<S240>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_i[9];
                                /* '<S240>/Reshape (9) to [3x3] column-major' */
  real_T sincos_o1_o[3];               /* '<S239>/sincos' */
  real_T sincos_o2_gn[3];              /* '<S239>/sincos' */
  real_T phidot;                       /* '<S239>/phidot' */
  real_T thetadot;                     /* '<S239>/thetadot' */
  real_T psidot;                       /* '<S239>/psidot' */
  real_T TmpSignalConversionAtphithetapsiInport1[3];/* '<S230>/phidot thetadot psidot' */
  real_T MatrixConcatenation[18];      /* '<S232>/Matrix Concatenation' */
  real_T Selector_f[9];                /* '<S231>/Selector' */
  real_T Reshape1_ar[3];               /* '<S242>/Reshape1' */
  real_T Product_g[3];                 /* '<S242>/Product' */
  real_T Reshape2_f[3];                /* '<S242>/Reshape2' */
  real_T ixj_p;                        /* '<S244>/i x j' */
  real_T jxk_j;                        /* '<S244>/j x k' */
  real_T kxi_e;                        /* '<S244>/k x i' */
  real_T ixk_k3;                       /* '<S245>/i x k' */
  real_T jxi_b1;                       /* '<S245>/j x i' */
  real_T kxj_f;                        /* '<S245>/k x j' */
  real_T Sum_a[3];                     /* '<S241>/Sum' */
  real_T Selector1_g[9];               /* '<S231>/Selector1' */
  real_T Reshape1_f[3];                /* '<S243>/Reshape1' */
  real_T Product_je[3];                /* '<S243>/Product' */
  real_T Reshape2_g[3];                /* '<S243>/Reshape2' */
  real_T Reshape_c[12];                /* '<S227>/Reshape' */
  real_T Selector3_g[4];               /* '<S227>/Selector3' */
  real_T Reshape3_j[4];                /* '<S227>/Reshape3' */
  real_T Product2_h[4];                /* '<S222>/Product2' */
  real_T SumofElements1;               /* '<S222>/Sum of Elements1' */
  real_T Selector2[4];                 /* '<S227>/Selector2' */
  real_T Reshape2_pd[4];               /* '<S227>/Reshape2' */
  real_T Product1_k[4];                /* '<S222>/Product1' */
  real_T SumofElements15;              /* '<S222>/Sum of Elements15' */
  real_T SumofElements2;               /* '<S222>/Sum of Elements2' */
  real_T Product4_i;                   /* '<S222>/Product4' */
  real_T SumofElements4;               /* '<S222>/Sum of Elements4' */
  real_T Product5;                     /* '<S222>/Product5' */
  real_T SumofElements3;               /* '<S222>/Sum of Elements3' */
  real_T Selector1_hl[4];              /* '<S227>/Selector1' */
  real_T Reshape1_b3[4];               /* '<S227>/Reshape1' */
  real_T Product_m[4];                 /* '<S222>/Product' */
  real_T SumofElements16;              /* '<S222>/Sum of Elements16' */
  real_T SumofElements10;              /* '<S222>/Sum of Elements10' */
  real_T Product6;                     /* '<S222>/Product6' */
  real_T SumofElements11;              /* '<S222>/Sum of Elements11' */
  real_T Product7;                     /* '<S222>/Product7' */
  real_T SumofElements8;               /* '<S222>/Sum of Elements8' */
  real_T Product3_o[4];                /* '<S222>/Product3' */
  real_T SumofElements7;               /* '<S222>/Sum of Elements7' */
  real_T VectorConcatenate_f[3];       /* '<S217>/Vector Concatenate' */
  real_T Selector1_a[4];               /* '<S228>/Selector1' */
  real_T Selector2_n[4];               /* '<S228>/Selector2' */
  real_T Selector3_i[4];               /* '<S228>/Selector3' */
  real_T VectorConcatenate_nl[3];      /* '<S228>/Vector Concatenate' */
  real_T Reshape1_p5[3];               /* '<S219>/Reshape1' */
  real_T InertialtoBody[3];            /* '<S219>/Inertial to Body' */
  real_T Add1_n[3];                    /* '<S250>/Add1' */
  real_T Product_kk[3];                /* '<S250>/Product' */
  real_T SumofElements_l;              /* '<S250>/Sum of Elements' */
  real_T Sqrt;                         /* '<S250>/Sqrt' */
  real_T Product2_m;                   /* '<S250>/Product2' */
  real_T TrigonometricFunction;        /* '<S250>/Trigonometric Function' */
  real_T u[3];                         /* '<S250>/4' */
  real_T Tanh[3];                      /* '<S250>/Tanh' */
  real_T VectorConcatenate_fs[6];      /* '<S250>/Vector Concatenate' */
  real_T Product1_g3[6];               /* '<S250>/Product1' */
  real_T uAPabsRT[6];                  /* '<S250>/.5.*A.*Pabs.//R.//T' */
  real_T Product4_p[3];                /* '<S250>/Product4' */
  real_T UnaryMinus1_g[3];             /* '<S217>/Unary Minus1' */
  real_T Add_n[3];                     /* '<S217>/Add' */
  real_T Sum2_a[3];                    /* '<S231>/Sum2' */
  real_T Reshape1_m[3];                /* '<S231>/Reshape1' */
  real_T Selector2_k[9];               /* '<S231>/Selector2' */
  real_T Product2_d[3];                /* '<S231>/Product2' */
  real_T Reshape_o[3];                 /* '<S231>/Reshape' */
  real_T UnitConversion_k[3];          /* '<S236>/Unit Conversion' */
  real_T SumofElements_e[3];           /* '<S246>/Sum of Elements' */
  real_T Reshape_h[3];                 /* '<S232>/Reshape' */
  real_T Inertialgravityvector[3];     /* '<S221>/Vector Concatenate' */
  real_T Fg_I[3];                      /* '<S221>/Product' */
  real_T Reshape_ox[3];                /* '<S221>/Reshape' */
  real_T Fg_B[3];                      /* '<S221>/Inertial to Body' */
  real_T Product3_kb[3];               /* '<S250>/Product3' */
  real_T VectorConcatenate_pf5[3];     /* '<S227>/Vector Concatenate' */
  real_T Sum_o[3];                     /* '<S217>/Sum' */
  real_T Sum_op[3];                    /* '<S232>/Sum' */
  real_T Product_fl[3];                /* '<S218>/Product' */
  real_T jxk_h;                        /* '<S248>/j x k' */
  real_T kxi_f;                        /* '<S248>/k x i' */
  real_T ixj_ol;                       /* '<S248>/i x j' */
  real_T kxj_k;                        /* '<S249>/k x j' */
  real_T ixk_d;                        /* '<S249>/i x k' */
  real_T jxi_jh;                       /* '<S249>/j x i' */
  real_T Sum_kr[3];                    /* '<S233>/Sum' */
  real_T Sum_dx[3];                    /* '<S218>/Sum' */
  real_T Transpose[9];                 /* '<S218>/Transpose' */
  real_T Reshape1_d[3];                /* '<S237>/Reshape1' */
  real_T Product_aa[3];                /* '<S237>/Product' */
  real_T Reshape2_me[3];               /* '<S237>/Reshape2' */
  real_T UnitConversion_j[3];          /* '<S235>/Unit Conversion' */
  real_T Constant_o[12];               /* '<S217>/Constant' */
  real_T VectorConcatenate_a[6];       /* '<S224>/Vector Concatenate' */
  real_T Abs_f5[6];                    /* '<S224>/Abs' */
  real_T UnaryMinus_o[3];              /* '<S217>/Unary Minus' */
  real_T VectorConcatenate1_j[6];      /* '<S224>/Vector Concatenate1' */
  real_T Abs1_b[6];                    /* '<S224>/Abs1' */
  real_T SumofElements_g;              /* '<S224>/Sum of Elements' */
  real_T SumofElements1_d;             /* '<S224>/Sum of Elements1' */
  real_T Reshape1_k0[3];               /* '<S258>/Reshape1' */
  real_T Product_c[3];                 /* '<S258>/Product' */
  real_T Reshape2_a[3];                /* '<S258>/Reshape2' */
  real_T V_wb[3];                      /* '<S256>/Add4' */
  real_T Reshape1_dx[3];               /* '<S266>/Reshape1' */
  real_T Product_cv[3];                /* '<S266>/Product' */
  real_T Reshape2_c[3];                /* '<S266>/Reshape2' */
  real_T V_wb_k[3];                    /* '<S264>/Add4' */
  real_T sincos_o1_hn[3];              /* '<S274>/sincos' */
  real_T sincos_o2_d[3];               /* '<S274>/sincos' */
  real_T VectorConcatenate_am[9];      /* '<S281>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_e[9];
                                /* '<S281>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_e[9];              /* '<S272>/Transpose1' */
  real_T VectorConcatenate_i[3];       /* '<S253>/Vector Concatenate' */
  real_T Subtract_g[3];                /* '<S253>/Subtract' */
  real_T Reshape1_iq[3];               /* '<S276>/Reshape1' */
  real_T Product_kf[3];                /* '<S276>/Product' */
  real_T Reshape2_j[3];                /* '<S276>/Reshape2' */
  real_T Add_h[3];                     /* '<S272>/Add' */
  real_T jxk_c;                        /* '<S282>/j x k' */
  real_T kxi_o2;                       /* '<S282>/k x i' */
  real_T ixj_f;                        /* '<S282>/i x j' */
  real_T kxj_j;                        /* '<S283>/k x j' */
  real_T ixk_m;                        /* '<S283>/i x k' */
  real_T jxi_f;                        /* '<S283>/j x i' */
  real_T Sum_l[3];                     /* '<S277>/Sum' */
  real_T Add1_o[3];                    /* '<S272>/Add1' */
  real_T Reshape1_ff[3];               /* '<S275>/Reshape1' */
  real_T Product_i[3];                 /* '<S275>/Product' */
  real_T Reshape2_ca[3];               /* '<S275>/Reshape2' */
  real_T V_wb_o[3];                    /* '<S272>/Add4' */
  real_T Fcn_i;                        /* '<S278>/Fcn' */
  real_T Abs_la;                       /* '<S278>/Abs' */
  real_T Switch_d;                     /* '<S278>/Switch' */
  real_T Divide_f2;                    /* '<S273>/Divide' */
  real_T Beta;                         /* '<S273>/Trigonometric Function' */
  real_T Reshape1_kx[3];               /* '<S286>/Reshape1' */
  real_T Product_d4[3];                /* '<S286>/Product' */
  real_T Reshape2_js[3];               /* '<S286>/Reshape2' */
  real_T V_wb_n[3];                    /* '<S284>/Add4' */
  real_T Reshape1_bj[3];               /* '<S294>/Reshape1' */
  real_T Product_dl[3];                /* '<S294>/Product' */
  real_T Reshape2_n1[3];               /* '<S294>/Reshape2' */
  real_T V_wb_j[3];                    /* '<S292>/Add4' */
  real_T sincos_o1_m[3];               /* '<S302>/sincos' */
  real_T sincos_o2_md[3];              /* '<S302>/sincos' */
  real_T thetadot_e;                   /* '<S302>/thetadot' */
  real_T jxk_e;                        /* '<S308>/j x k' */
  real_T psidot_g;                     /* '<S302>/psidot' */
  real_T kxi_ki;                       /* '<S308>/k x i' */
  real_T phidot_m;                     /* '<S302>/phidot' */
  real_T ixj_d;                        /* '<S308>/i x j' */
  real_T kxj_g;                        /* '<S309>/k x j' */
  real_T ixk_k2;                       /* '<S309>/i x k' */
  real_T jxi_d;                        /* '<S309>/j x i' */
  real_T Sum_e[3];                     /* '<S303>/Sum' */
  real_T Add_hc[3];                    /* '<S226>/Add' */
  real_T Fcn_c;                        /* '<S305>/Fcn' */
  real_T Abs_h;                        /* '<S305>/Abs' */
  real_T Switch_nu;                    /* '<S305>/Switch' */
  real_T Divide_g;                     /* '<S301>/Divide' */
  real_T Beta_j;                       /* '<S301>/Trigonometric Function' */
  real_T Integrator_i[3];              /* '<S226>/Integrator' */
  real_T UnitConversion1[3];           /* '<S226>/Unit Conversion1' */
  real_T UnitConversion3[3];           /* '<S226>/Unit Conversion3' */
  real_T VectorConcatenate2_k[12];     /* '<S227>/Vector Concatenate2' */
  real_T VectorConcatenate4[4];        /* '<S127>/Vector Concatenate4' */
  real_T VectorConcatenate3_a[4];      /* '<S127>/Vector Concatenate3' */
  real_T Subtract_p[4];                /* '<S127>/Subtract' */
  real_T VectorConcatenate1_l[4];      /* '<S127>/Vector Concatenate1' */
  real_T VectorConcatenate2_bt[4];     /* '<S127>/Vector Concatenate2' */
  real_T Selector_b[4];                /* '<S128>/Selector' */
  real_T Reshape2_o[4];                /* '<S128>/Reshape2' */
  real_T Sum_dw[4];                    /* '<S310>/Sum' */
  real_T Divide_ah[4];                 /* '<S310>/Divide' */
  real_T Sum_ja[12];                   /* '<S311>/Sum' */
  real_T Divide_d4[12];                /* '<S311>/Divide' */
  real_T Reshape6_ll[4];               /* '<S313>/Reshape6' */
  real_T Reshape_h0[4];                /* '<S317>/Reshape' */
  real_T VectorConcatenate5[4];        /* '<S313>/Vector Concatenate5' */
  real_T VectorConcatenate6[4];        /* '<S313>/Vector Concatenate6' */
  real_T SignalCopy;                   /* '<S343>/Signal Copy' */
  real_T Product2_mt;                  /* '<S347>/Product2' */
  real_T Add1_cq;                      /* '<S347>/Add1' */
  real_T Abs_e;                        /* '<S347>/Abs' */
  real_T Add_j;                        /* '<S347>/Add' */
  real_T DeadZone;                     /* '<S347>/Dead Zone' */
  real_T Saturation1_h;                /* '<S347>/Saturation1' */
  real_T Product3_es;                  /* '<S347>/Product3' */
  real_T Saturation_lr;                /* '<S347>/Saturation' */
  real_T Fdot;                         /* '<S347>/Product1' */
  real_T Product2_j;                   /* '<S348>/Product2' */
  real_T Add1_k4;                      /* '<S348>/Add1' */
  real_T Abs_g;                        /* '<S348>/Abs' */
  real_T Add_l;                        /* '<S348>/Add' */
  real_T DeadZone_h;                   /* '<S348>/Dead Zone' */
  real_T Saturation1_c;                /* '<S348>/Saturation1' */
  real_T Product3_fx;                  /* '<S348>/Product3' */
  real_T Saturation_g5;                /* '<S348>/Saturation' */
  real_T Fdot_a;                       /* '<S348>/Product1' */
  real_T Product2_ht;                  /* '<S350>/Product2' */
  real_T Add1_al;                      /* '<S350>/Add1' */
  real_T Abs_n;                        /* '<S350>/Abs' */
  real_T Add_m;                        /* '<S350>/Add' */
  real_T DeadZone_j;                   /* '<S350>/Dead Zone' */
  real_T Saturation1_bf;               /* '<S350>/Saturation1' */
  real_T Product3_ix;                  /* '<S350>/Product3' */
  real_T Saturation_dk;                /* '<S350>/Saturation' */
  real_T Fdot_h;                       /* '<S350>/Product1' */
  real_T Switch_a;                     /* '<S345>/Switch' */
  real_T Gain2_m;                      /* '<S345>/Gain2' */
  real_T Sum2_g;                       /* '<S345>/Sum2' */
  real_T Gain1_ge;                     /* '<S345>/Gain1' */
  real_T SignalCopy_e;                 /* '<S368>/Signal Copy' */
  real_T Product2_k;                   /* '<S372>/Product2' */
  real_T Add1_k1;                      /* '<S372>/Add1' */
  real_T Abs_k;                        /* '<S372>/Abs' */
  real_T Add_bv;                       /* '<S372>/Add' */
  real_T DeadZone_f;                   /* '<S372>/Dead Zone' */
  real_T Saturation1_fy;               /* '<S372>/Saturation1' */
  real_T Product3_oh;                  /* '<S372>/Product3' */
  real_T Saturation_jp;                /* '<S372>/Saturation' */
  real_T Fdot_f;                       /* '<S372>/Product1' */
  real_T Product2_o;                   /* '<S373>/Product2' */
  real_T Add1_nb;                      /* '<S373>/Add1' */
  real_T Abs_o;                        /* '<S373>/Abs' */
  real_T Add_mi;                       /* '<S373>/Add' */
  real_T DeadZone_b;                   /* '<S373>/Dead Zone' */
  real_T Saturation1_b1;               /* '<S373>/Saturation1' */
  real_T Product3_a;                   /* '<S373>/Product3' */
  real_T Saturation_p;                 /* '<S373>/Saturation' */
  real_T Fdot_l;                       /* '<S373>/Product1' */
  real_T Product2_kj;                  /* '<S375>/Product2' */
  real_T Add1_bl;                      /* '<S375>/Add1' */
  real_T Abs_a;                        /* '<S375>/Abs' */
  real_T Add_fx;                       /* '<S375>/Add' */
  real_T DeadZone_p;                   /* '<S375>/Dead Zone' */
  real_T Saturation1_k;                /* '<S375>/Saturation1' */
  real_T Product3_j1;                  /* '<S375>/Product3' */
  real_T Saturation_fp;                /* '<S375>/Saturation' */
  real_T Fdot_o;                       /* '<S375>/Product1' */
  real_T Switch_o0;                    /* '<S370>/Switch' */
  real_T Gain2_o;                      /* '<S370>/Gain2' */
  real_T Sum2_d;                       /* '<S370>/Sum2' */
  real_T Gain1_p;                      /* '<S370>/Gain1' */
  real_T SignalCopy_g;                 /* '<S393>/Signal Copy' */
  real_T Product2_ji;                  /* '<S397>/Product2' */
  real_T Add1_oe;                      /* '<S397>/Add1' */
  real_T Abs_kh;                       /* '<S397>/Abs' */
  real_T Add_du;                       /* '<S397>/Add' */
  real_T DeadZone_d;                   /* '<S397>/Dead Zone' */
  real_T Saturation1_dh;               /* '<S397>/Saturation1' */
  real_T Product3_oq;                  /* '<S397>/Product3' */
  real_T Saturation_gr;                /* '<S397>/Saturation' */
  real_T Fdot_d;                       /* '<S397>/Product1' */
  real_T Product2_ii;                  /* '<S398>/Product2' */
  real_T Add1_gr;                      /* '<S398>/Add1' */
  real_T Abs_hy;                       /* '<S398>/Abs' */
  real_T Add_dy;                       /* '<S398>/Add' */
  real_T DeadZone_e;                   /* '<S398>/Dead Zone' */
  real_T Saturation1_ke;               /* '<S398>/Saturation1' */
  real_T Product3_ol;                  /* '<S398>/Product3' */
  real_T Saturation_ku;                /* '<S398>/Saturation' */
  real_T Fdot_b;                       /* '<S398>/Product1' */
  real_T Product2_f;                   /* '<S400>/Product2' */
  real_T Add1_ln;                      /* '<S400>/Add1' */
  real_T Abs_fr;                       /* '<S400>/Abs' */
  real_T Add_o;                        /* '<S400>/Add' */
  real_T DeadZone_a;                   /* '<S400>/Dead Zone' */
  real_T Saturation1_hj;               /* '<S400>/Saturation1' */
  real_T Product3_lt;                  /* '<S400>/Product3' */
  real_T Saturation_gl;                /* '<S400>/Saturation' */
  real_T Fdot_c;                       /* '<S400>/Product1' */
  real_T Switch_bz;                    /* '<S395>/Switch' */
  real_T Gain2_h;                      /* '<S395>/Gain2' */
  real_T Sum2_dp;                      /* '<S395>/Sum2' */
  real_T Gain1_o;                      /* '<S395>/Gain1' */
  real_T SignalCopy_k;                 /* '<S418>/Signal Copy' */
  real_T Product2_b;                   /* '<S422>/Product2' */
  real_T Add1_e;                       /* '<S422>/Add1' */
  real_T Abs_j;                        /* '<S422>/Abs' */
  real_T Add_lr;                       /* '<S422>/Add' */
  real_T DeadZone_l;                   /* '<S422>/Dead Zone' */
  real_T Saturation1_kd;               /* '<S422>/Saturation1' */
  real_T Product3_ip;                  /* '<S422>/Product3' */
  real_T Saturation_e;                 /* '<S422>/Saturation' */
  real_T Fdot_m;                       /* '<S422>/Product1' */
  real_T Product2_ip;                  /* '<S423>/Product2' */
  real_T Add1_ok;                      /* '<S423>/Add1' */
  real_T Abs_az;                       /* '<S423>/Abs' */
  real_T Add_j5;                       /* '<S423>/Add' */
  real_T DeadZone_bq;                  /* '<S423>/Dead Zone' */
  real_T Saturation1_o;                /* '<S423>/Saturation1' */
  real_T Product3_e2;                  /* '<S423>/Product3' */
  real_T Saturation_fl;                /* '<S423>/Saturation' */
  real_T Fdot_g;                       /* '<S423>/Product1' */
  real_T Product2_ol;                  /* '<S425>/Product2' */
  real_T Add1_lr;                      /* '<S425>/Add1' */
  real_T Abs_fn;                       /* '<S425>/Abs' */
  real_T Add_fb;                       /* '<S425>/Add' */
  real_T DeadZone_i;                   /* '<S425>/Dead Zone' */
  real_T Saturation1_l;                /* '<S425>/Saturation1' */
  real_T Product3_d;                   /* '<S425>/Product3' */
  real_T Saturation_pu;                /* '<S425>/Saturation' */
  real_T Fdot_bw;                      /* '<S425>/Product1' */
  real_T Switch_i;                     /* '<S420>/Switch' */
  real_T Gain2_c;                      /* '<S420>/Gain2' */
  real_T Sum2_p;                       /* '<S420>/Sum2' */
  real_T Gain1_b;                      /* '<S420>/Gain1' */
  real_T VectorConcatenate_ag[4];      /* '<S329>/Vector Concatenate' */
  real_T Switch_cu;                    /* '<S334>/Switch' */
  real_T Gain2_b;                      /* '<S334>/Gain2' */
  real_T Sum2_e;                       /* '<S334>/Sum2' */
  real_T Gain1_m;                      /* '<S334>/Gain1' */
  real_T Switch_e;                     /* '<S335>/Switch' */
  real_T Gain2_g;                      /* '<S335>/Gain2' */
  real_T Sum2_dd;                      /* '<S335>/Sum2' */
  real_T Gain1_na;                     /* '<S335>/Gain1' */
  real_T Switch_op;                    /* '<S336>/Switch' */
  real_T Gain2_k;                      /* '<S336>/Gain2' */
  real_T Sum2_f;                       /* '<S336>/Sum2' */
  real_T Gain1_er;                     /* '<S336>/Gain1' */
  real_T Switch_fy;                    /* '<S337>/Switch' */
  real_T Gain2_d;                      /* '<S337>/Gain2' */
  real_T Sum2_hm;                      /* '<S337>/Sum2' */
  real_T Gain1_n4;                     /* '<S337>/Gain1' */
  real_T VectorConcatenate_bc[4];      /* '<S338>/Vector Concatenate' */
  real_T VectorConcatenate_ol[4];      /* '<S339>/Vector Concatenate' */
  real_T SteeringCmd_c;                /* '<S1>/Ackerman steer' */
  real_T Gain_f;                       /* '<Root>/Gain' */
  real_T Gain1_la;                     /* '<Root>/Gain1' */
  real_T ImpAsg_InsertedFor_zdotWheel_at_inport_0[4];/* '<S444>/Demux1' */
  real_T ImpAsg_InsertedFor_ydotWheel_at_inport_0[4];/* '<S444>/Demux1' */
  real_T ImpAsg_InsertedFor_xdotWheel_at_inport_0[4];/* '<S444>/Demux1' */
  real_T ImpAsg_InsertedFor_Fz_at_inport_0[4];/* '<S319>/Demux1' */
  real_T ImpAsg_InsertedFor_Fy_at_inport_0[4];/* '<S319>/Demux1' */
  real_T ImpAsg_InsertedFor_Fx_at_inport_0[4];/* '<S319>/Demux1' */
  real_T Mbar;                         /* '<S217>/vehdyncginert' */
  real_T Ibar[9];                      /* '<S217>/vehdyncginert' */
  real_T Rbar[3];                      /* '<S217>/vehdyncginert' */
  real_T Xbar[3];                      /* '<S217>/vehdyncginert' */
  real_T Wbar[4];                      /* '<S217>/vehdyncginert' */
  real_T HPbar[12];                    /* '<S217>/vehdyncginert' */
  real_T ImpAsg_InsertedFor_F_at_inport_0[3];/* '<S247>/Product' */
  real_T ImpAsg_InsertedFor_WhlFz_at_inport_0[2];/* '<S181>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlAng_at_inport_0[6];/* '<S181>/Suspension' */
  real_T ImpAsg_InsertedFor_VehM_at_inport_0[6];/* '<S181>/Suspension' */
  real_T ImpAsg_InsertedFor_VehFz_at_inport_0[2];/* '<S181>/Suspension' */
  real_T ImpAsg_InsertedFor_Info_at_inport_0[12];/* '<S181>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlFzAs_at_inport_0[2];/* '<S183>/Mux' */
  real_T ImpAsg_InsertedFor_VehFzAs_at_inport_0[2];/* '<S183>/Mux1' */
  real_T ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[2];/* '<S183>/Sum2' */
  real_T ImpAsg_InsertedFor_WhlFz_at_inport_0_l[2];/* '<S144>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlAng_at_inport_0_p[6];/* '<S144>/Suspension' */
  real_T ImpAsg_InsertedFor_VehM_at_inport_0_b[6];/* '<S144>/Suspension' */
  real_T ImpAsg_InsertedFor_VehFz_at_inport_0_j[2];/* '<S144>/Suspension' */
  real_T ImpAsg_InsertedFor_Info_at_inport_0_i[12];/* '<S144>/Suspension' */
  real_T ImpAsg_InsertedFor_p_at_inport_0;
                                     /* '<S143>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_cgV_at_inport_0[3];
                                     /* '<S143>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_cgP_at_inport_0[3];
                                     /* '<S143>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_DCM_at_inport_0[9];
                                     /* '<S143>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_WhlV_at_inport_0[6];/* '<S142>/Sum2' */
  real_T ImpAsg_InsertedFor_WhlP_at_inport_0[6];/* '<S142>/Sum4' */
  real_T ImpAsg_InsertedFor_SuspV_at_inport_0[6];/* '<S142>/Sum3' */
  real_T ImpAsg_InsertedFor_SuspP_at_inport_0[6];/* '<S142>/Sum1' */
  real_T TmpSignalConversionAtSFunctionInport6[4];/* '<S102>/Band-Aid' */
  real_T OutMode;                      /* '<S102>/Band-Aid' */
  real_T OutSpd[4];                    /* '<S102>/Band-Aid' */
  real_T State;                        /* '<S102>/Band-Aid' */
  real_T Divide_l4;                    /* '<S43>/Divide' */
  real_T Divide1_f;                    /* '<S43>/Divide1' */
  real_T Interpolatedzerocrossing;     /* '<S43>/Interpolated zero-crossing' */
  real_T Divide2;                      /* '<S43>/Divide2' */
  real32_T DataTypeConversion;         /* '<S98>/Data Type Conversion' */
  real32_T DataTypeConversion_a;       /* '<S99>/Data Type Conversion' */
  real32_T DataTypeConversion_c;       /* '<S100>/Data Type Conversion' */
  real32_T DataTypeConversion_n;       /* '<S101>/Data Type Conversion' */
  uint8_T Reserved;                    /* '<S112>/Bit7' */
  uint8_T Reserved_o;                  /* '<S112>/Bit6' */
  uint8_T Reserved_j;                  /* '<S112>/Bit5' */
  uint8_T SpTqRR;                      /* '<S112>/Bit4' */
  uint8_T SpTqLR;                      /* '<S112>/Bit3' */
  uint8_T SpTqLF;                      /* '<S112>/Bit1' */
  uint8_T SystemActive;                /* '<S112>/Bit0' */
  uint8_T SystemCtrlBits;              /* '<S112>/Add' */
  uint8_T Output;                      /* '<S113>/Output' */
  uint8_T FixPtSum1;                   /* '<S118>/FixPt Sum1' */
  uint8_T FixPtSwitch;                 /* '<S119>/FixPt Switch' */
  uint8_T uint16unsigned162[2];        /* '<S104>/uint16 (unsigned 16) 2' */
  uint8_T uint8unsigned81;             /* '<S104>/uint8 (unsigned 8) 1' */
  uint8_T uint8unsigned82;             /* '<S104>/uint8 (unsigned 8) 2' */
  uint8_T uint8unsigned83;             /* '<S104>/uint8 (unsigned 8) 3' */
  uint8_T uint8unsigned84;             /* '<S104>/uint8 (unsigned 8) 4' */
  uint8_T uint8unsigned85;             /* '<S104>/uint8 (unsigned 8) 5' */
  uint8_T uint8unsigned87;             /* '<S104>/uint8 (unsigned 8) 7' */
  uint8_T uint8unsigned88;             /* '<S104>/uint8 (unsigned 8) 8' */
  uint8_T uint8unsigned89;             /* '<S104>/uint8 (unsigned 8) 9' */
  uint8_T uint8unsigned86;             /* '<S104>/uint8 (unsigned 8) 6' */
  uint8_T y;                           /* '<S104>/MATLAB Function' */
  boolean_T Compare;                   /* '<S56>/Compare' */
  boolean_T Compare_e;                 /* '<S59>/Compare' */
  boolean_T Compare_l;                 /* '<S63>/Compare' */
  boolean_T Compare_a;                 /* '<S64>/Compare' */
  boolean_T LogicalOperator;           /* '<S58>/Logical Operator' */
  boolean_T LowerRelop1;               /* '<S65>/LowerRelop1' */
  boolean_T UpperRelop;                /* '<S65>/UpperRelop' */
  boolean_T Compare_m;                 /* '<S68>/Compare' */
  boolean_T Compare_j;                 /* '<S69>/Compare' */
  boolean_T LogicalOperator_a;         /* '<S62>/Logical Operator' */
  boolean_T LowerRelop1_a;             /* '<S67>/LowerRelop1' */
  boolean_T UpperRelop_p;              /* '<S67>/UpperRelop' */
  boolean_T Compare_p[4];              /* '<S96>/Compare' */
  boolean_T Compare_k[4];              /* '<S97>/Compare' */
  boolean_T LogicalOperator_ap[4];     /* '<S74>/Logical Operator' */
  boolean_T Memory_d;                  /* '<S141>/Memory' */
  boolean_T Compare_ku;                /* '<S279>/Compare' */
  boolean_T Compare_o;                 /* '<S280>/Compare' */
  boolean_T LogicalOperator_p;         /* '<S278>/Logical Operator' */
  boolean_T Compare_lx;                /* '<S306>/Compare' */
  boolean_T Compare_b;                 /* '<S307>/Compare' */
  boolean_T LogicalOperator_j;         /* '<S305>/Logical Operator' */
  B_CoreSubsys_CAVE_MachE_sil_test_ca_T CoreSubsys_pna[4];/* '<S315>/Wheel to Body Transform' */
  B_LockUp_CAVE_MachE_sil_test_T sf_LockUp_c;/* '<S421>/LockUp' */
  B_MagicTireConstInput_CAVE_MachE_sil_test_T sf_MagicTireConstInput_k;/* '<S419>/Magic Tire Const Input' */
  B_LockUp_CAVE_MachE_sil_test_T sf_LockUp_h;/* '<S396>/LockUp' */
  B_MagicTireConstInput_CAVE_MachE_sil_test_T sf_MagicTireConstInput_f;/* '<S394>/Magic Tire Const Input' */
  B_LockUp_CAVE_MachE_sil_test_T sf_LockUp_n;/* '<S371>/LockUp' */
  B_MagicTireConstInput_CAVE_MachE_sil_test_T sf_MagicTireConstInput_j;/* '<S369>/Magic Tire Const Input' */
  B_LockUp_CAVE_MachE_sil_test_T sf_LockUp;/* '<S346>/LockUp' */
  B_MagicTireConstInput_CAVE_MachE_sil_test_T sf_MagicTireConstInput;/* '<S344>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_sil_test_c_T CoreSubsys_pn[4];/* '<S313>/Wheel to Body Transform' */
  B_CoreSubsys_CAVE_MachE_sil_test_b_T CoreSubsys_b[1];/* '<S246>/For Each Subsystem' */
  B_CoreSubsys_CAVE_MachE_sil_test_n_T CoreSubsys_d[2];
  /* '<S140>/For each track and axle combination calculate suspension forces and moments' */
  B_CoreSubsys_CAVE_MachE_sil_test_o_T CoreSubsys_n[1];
                                     /* '<S182>/For Each Axle With Anti-Sway' */
  B_CoreSubsys_CAVE_MachE_sil_test_l0_T CoreSubsys_p[2];
  /* '<S135>/For each track and axle combination calculate suspension forces and moments' */
  B_CoreSubsys_CAVE_MachE_sil_test_l_T CoreSubsys_m[1];
         /* '<S135>/For each axle calculate axle cg positions and velocities' */
  B_CoreSubsys_CAVE_MachE_sil_test_T CoreSubsys[2];
  /* '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */
  B_Defloater_CAVE_MachE_sil_test_T sf_Defloater_e;/* '<S110>/Defloater' */
  B_Defloater_CAVE_MachE_sil_test_T sf_Defloater_d;/* '<S109>/Defloater' */
  B_Defloater_CAVE_MachE_sil_test_T sf_Defloater_l;/* '<S108>/Defloater' */
  B_Defloater_CAVE_MachE_sil_test_T sf_Defloater;/* '<S107>/Defloater' */
  B_MATLABFunction_CAVE_MachE_sil_test_T sf_MATLABFunction_g;/* '<S83>/MATLAB Function' */
  B_MATLABFunction_CAVE_MachE_sil_test_T sf_MATLABFunction_j;/* '<S82>/MATLAB Function' */
  B_MATLABFunction_CAVE_MachE_sil_test_T sf_MATLABFunction_h;/* '<S81>/MATLAB Function' */
  B_MATLABFunction_CAVE_MachE_sil_test_T sf_MATLABFunction;/* '<S80>/MATLAB Function' */
} B_CAVE_MachE_sil_test_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator4_DSTATE;/* '<S72>/Discrete-Time Integrator4' */
  real_T DiscreteTimeIntegrator5_DSTATE;/* '<S72>/Discrete-Time Integrator5' */
  real_T DiscreteTimeIntegrator6_DSTATE;/* '<S72>/Discrete-Time Integrator6' */
  real_T DiscreteTimeIntegrator7_DSTATE;/* '<S72>/Discrete-Time Integrator7' */
  real_T DiscreteTimeIntegrator4_DSTATE_k;/* '<S78>/Discrete-Time Integrator4' */
  real_T DiscreteTimeIntegrator5_DSTATE_a;/* '<S78>/Discrete-Time Integrator5' */
  real_T DiscreteTimeIntegrator6_DSTATE_j;/* '<S78>/Discrete-Time Integrator6' */
  real_T DiscreteTimeIntegrator7_DSTATE_i;/* '<S78>/Discrete-Time Integrator7' */
  real_T Memory_PreviousInput;         /* '<S30>/Memory' */
  real_T Memory1_PreviousInput;        /* '<S30>/Memory1' */
  real_T Memory_PreviousInput_f;       /* '<S12>/Memory' */
  real_T Memory_PreviousInput_j[4];    /* '<S71>/Memory' */
  real_T Memory3_PreviousInput;        /* '<S72>/Memory3' */
  real_T Memory1_PreviousInput_f;      /* '<S72>/Memory1' */
  real_T Memory2_PreviousInput;        /* '<S72>/Memory2' */
  real_T Memory4_PreviousInput;        /* '<S72>/Memory4' */
  real_T Memory3_PreviousInput_c;      /* '<S78>/Memory3' */
  real_T Memory1_PreviousInput_d;      /* '<S78>/Memory1' */
  real_T Memory2_PreviousInput_f;      /* '<S78>/Memory2' */
  real_T Memory4_PreviousInput_h;      /* '<S78>/Memory4' */
  real_T Memory_PreviousInput_g;       /* '<S98>/Memory' */
  real_T Memory_PreviousInput_p;       /* '<S99>/Memory' */
  real_T Memory_PreviousInput_h;       /* '<S100>/Memory' */
  real_T Memory_PreviousInput_e;       /* '<S101>/Memory' */
  real_T Memory_PreviousInput_o;       /* '<S80>/Memory' */
  real_T Memory_PreviousInput_gr;      /* '<S81>/Memory' */
  real_T Memory_PreviousInput_b;       /* '<S82>/Memory' */
  real_T Memory_PreviousInput_eq;      /* '<S83>/Memory' */
  real_T PrevY[4];                     /* '<S76>/RateLimSpd' */
  real_T Add_DWORK1;                   /* '<S112>/Add' */
  real_T PrevY_e;                      /* '<S130>/Backlash' */
  real_T Memory1_PreviousInput_n[2];   /* '<S141>/Memory1' */
  real_T Product2_DWORK1[9];           /* '<S231>/Product2' */
  real_T Product2_DWORK3[9];           /* '<S231>/Product2' */
  real_T Product2_DWORK4[9];           /* '<S231>/Product2' */
  real_T Product2_DWORK5[9];           /* '<S231>/Product2' */
  struct {
    void *LoggedData;
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData;
  } Scope1_PWORK;                      /* '<Root>/Scope1' */

  struct {
    void *LoggedData;
  } Scope2_PWORK;                      /* '<Root>/Scope2' */

  struct {
    void *LoggedData;
  } Scope3_PWORK;                      /* '<Root>/Scope3' */

  struct {
    void *LoggedData;
  } Scope_PWORK_c;                     /* '<S4>/Scope' */

  struct {
    void *LoggedData;
  } Scope1_PWORK_d;                    /* '<S4>/Scope1' */

  struct {
    void *LoggedData;
  } Scope2_PWORK_o;                    /* '<S4>/Scope2' */

  struct {
    void *LoggedData;
  } Scope3_PWORK_k;                    /* '<S4>/Scope3' */

  struct {
    void *LoggedData;
  } Scope4_PWORK;                      /* '<S4>/Scope4' */

  struct {
    void *LoggedData;
  } Scope_PWORK_b;                     /* '<S12>/Scope' */

  struct {
    void *LoggedData;
  } Scope_PWORK_h;                     /* '<S80>/Scope' */

  struct {
    void *LoggedData;
  } Scope1_PWORK_a;                    /* '<S80>/Scope1' */

  struct {
    void *LoggedData;
  } Scope2_PWORK_b;                    /* '<S80>/Scope2' */

  struct {
    void *LoggedData;
  } Scope3_PWORK_i;                    /* '<S80>/Scope3' */

  int32_T Product2_DWORK2[3];          /* '<S231>/Product2' */
  uint32_T temporalCounter_i1;         /* '<S102>/Band-Aid' */
  int_T IntegratorLimited_IWORK;       /* '<S23>/Integrator Limited' */
  int_T IntegratorSecondOrder_MODE;    /* '<S345>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_d;  /* '<S370>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_h;  /* '<S395>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_l;  /* '<S420>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_a;  /* '<S334>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_o;  /* '<S335>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_dh; /* '<S336>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_hl; /* '<S337>/Integrator, Second-Order' */
  uint8_T Output_DSTATE;               /* '<S113>/Output' */
  int8_T If_ActiveSubsystem;           /* '<S39>/If' */
  uint8_T DiscreteTimeIntegrator4_SYSTEM_ENABLE;/* '<S72>/Discrete-Time Integrator4' */
  uint8_T DiscreteTimeIntegrator5_SYSTEM_ENABLE;/* '<S72>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator6_SYSTEM_ENABLE;/* '<S72>/Discrete-Time Integrator6' */
  uint8_T DiscreteTimeIntegrator7_SYSTEM_ENABLE;/* '<S72>/Discrete-Time Integrator7' */
  uint8_T DiscreteTimeIntegrator4_SYSTEM_ENABLE_b;/* '<S78>/Discrete-Time Integrator4' */
  uint8_T DiscreteTimeIntegrator5_SYSTEM_ENABLE_g;/* '<S78>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator6_SYSTEM_ENABLE_p;/* '<S78>/Discrete-Time Integrator6' */
  uint8_T DiscreteTimeIntegrator7_SYSTEM_ENABLE_g;/* '<S78>/Discrete-Time Integrator7' */
  uint8_T is_active_c9_CAVE_MachE_sil_test;/* '<S102>/Band-Aid' */
  uint8_T is_c9_CAVE_MachE_sil_test;   /* '<S102>/Band-Aid' */
  uint8_T is_Running;                  /* '<S102>/Band-Aid' */
  uint8_T is_SpeedMode;                /* '<S102>/Band-Aid' */
  boolean_T IntegratorSecondOrder_DWORK1;/* '<S345>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_m;/* '<S370>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_mp;/* '<S395>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_h;/* '<S420>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_i;/* '<S334>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_d;/* '<S335>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_b;/* '<S336>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_c;/* '<S337>/Integrator, Second-Order' */
  boolean_T Memory_PreviousInput_b0;   /* '<S141>/Memory' */
  DW_LockUp_CAVE_MachE_sil_test_T sf_LockUp_c;/* '<S421>/LockUp' */
  DW_LockUp_CAVE_MachE_sil_test_T sf_LockUp_h;/* '<S396>/LockUp' */
  DW_LockUp_CAVE_MachE_sil_test_T sf_LockUp_n;/* '<S371>/LockUp' */
  DW_LockUp_CAVE_MachE_sil_test_T sf_LockUp;/* '<S346>/LockUp' */
} DW_CAVE_MachE_sil_test_T;

/* Continuous states (default storage) */
typedef struct {
  real_T xeyeze_CSTATE[3];             /* '<S218>/xe,ye,ze' */
  real_T phithetapsi_CSTATE[3];        /* '<S230>/phi theta psi' */
  real_T ubvbwb_CSTATE[3];             /* '<S218>/ub,vb,wb' */
  real_T Integrator_CSTATE;            /* '<S9>/Integrator' */
  real_T TransferFcn3_CSTATE;          /* '<S120>/Transfer Fcn3' */
  real_T TransferFcn1_CSTATE;          /* '<S120>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S120>/Transfer Fcn2' */
  real_T TransferFcn4_CSTATE;          /* '<S120>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE;           /* '<S120>/Transfer Fcn' */
  real_T TransferFcn9_CSTATE;          /* '<S120>/Transfer Fcn9' */
  real_T TransferFcn10_CSTATE;         /* '<S120>/Transfer Fcn10' */
  real_T TransferFcn11_CSTATE;         /* '<S120>/Transfer Fcn11' */
  real_T Integrator_CSTATE_k;          /* '<S39>/Integrator' */
  real_T TransferFcn_CSTATE_i;         /* '<S4>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_i;        /* '<S4>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_h;        /* '<S4>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_h;        /* '<S4>/Transfer Fcn3' */
  real_T IntegratorLimited_CSTATE;     /* '<S23>/Integrator Limited' */
  real_T Integrator_CSTATE_p;          /* '<S350>/Integrator' */
  real_T Integrator_CSTATE_o;          /* '<S347>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE[2];/* '<S345>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_c;          /* '<S375>/Integrator' */
  real_T Integrator_CSTATE_ch;         /* '<S372>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_l[2];/* '<S370>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_d;          /* '<S400>/Integrator' */
  real_T Integrator_CSTATE_b;          /* '<S397>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_ln[2];/* '<S395>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_a;          /* '<S425>/Integrator' */
  real_T Integrator_CSTATE_g;          /* '<S422>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_j[2];/* '<S420>/Integrator, Second-Order' */
  real_T omegaWheel;                   /* '<S80>/omega wheel' */
  real_T omegaWheel_j;                 /* '<S81>/omega wheel' */
  real_T omegaWheel_d;                 /* '<S82>/omega wheel' */
  real_T omegaWheel_jl;                /* '<S83>/omega wheel' */
  real_T IntegratorSecondOrder_CSTATE_m[2];/* '<S334>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_i[2];/* '<S335>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_e[2];/* '<S336>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_jk[2];/* '<S337>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_i;          /* '<S348>/Integrator' */
  real_T Integrator_CSTATE_b0;         /* '<S373>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S398>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S423>/Integrator' */
  real_T Integrator1_CSTATE[4];        /* '<S310>/Integrator1' */
  real_T Integrator1_CSTATE_m[12];     /* '<S311>/Integrator1' */
  real_T pqr_CSTATE[3];                /* '<S218>/p,q,r ' */
  real_T TransferFcn5_CSTATE;          /* '<S120>/Transfer Fcn5' */
  real_T TransferFcn6_CSTATE;          /* '<S120>/Transfer Fcn6' */
  real_T TransferFcn7_CSTATE;          /* '<S120>/Transfer Fcn7' */
  real_T TransferFcn8_CSTATE;          /* '<S120>/Transfer Fcn8' */
  real_T Integrator_CSTATE_j[3];       /* '<S226>/Integrator' */
  X_LockUp_CAVE_MachE_sil_test_T sf_LockUp_c;/* '<S346>/LockUp' */
  X_LockUp_CAVE_MachE_sil_test_T sf_LockUp_h;/* '<S346>/LockUp' */
  X_LockUp_CAVE_MachE_sil_test_T sf_LockUp_n;/* '<S346>/LockUp' */
  X_LockUp_CAVE_MachE_sil_test_T sf_LockUp;/* '<S346>/LockUp' */
  X_CoreSubsys_CAVE_MachE_sil_test_o_T CoreSubsys_m[1];/* '<S143>/CoreSubsys' */
} X_CAVE_MachE_sil_test_T;

/* Periodic continuous state vector (global) */
typedef int_T PeriodicIndX_CAVE_MachE_sil_test_T[3];
typedef real_T PeriodicRngX_CAVE_MachE_sil_test_T[6];

/* State derivatives (default storage) */
typedef struct {
  real_T xeyeze_CSTATE[3];             /* '<S218>/xe,ye,ze' */
  real_T phithetapsi_CSTATE[3];        /* '<S230>/phi theta psi' */
  real_T ubvbwb_CSTATE[3];             /* '<S218>/ub,vb,wb' */
  real_T Integrator_CSTATE;            /* '<S9>/Integrator' */
  real_T TransferFcn3_CSTATE;          /* '<S120>/Transfer Fcn3' */
  real_T TransferFcn1_CSTATE;          /* '<S120>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S120>/Transfer Fcn2' */
  real_T TransferFcn4_CSTATE;          /* '<S120>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE;           /* '<S120>/Transfer Fcn' */
  real_T TransferFcn9_CSTATE;          /* '<S120>/Transfer Fcn9' */
  real_T TransferFcn10_CSTATE;         /* '<S120>/Transfer Fcn10' */
  real_T TransferFcn11_CSTATE;         /* '<S120>/Transfer Fcn11' */
  real_T Integrator_CSTATE_k;          /* '<S39>/Integrator' */
  real_T TransferFcn_CSTATE_i;         /* '<S4>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_i;        /* '<S4>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_h;        /* '<S4>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_h;        /* '<S4>/Transfer Fcn3' */
  real_T IntegratorLimited_CSTATE;     /* '<S23>/Integrator Limited' */
  real_T Integrator_CSTATE_p;          /* '<S350>/Integrator' */
  real_T Integrator_CSTATE_o;          /* '<S347>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE[2];/* '<S345>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_c;          /* '<S375>/Integrator' */
  real_T Integrator_CSTATE_ch;         /* '<S372>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_l[2];/* '<S370>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_d;          /* '<S400>/Integrator' */
  real_T Integrator_CSTATE_b;          /* '<S397>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_ln[2];/* '<S395>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_a;          /* '<S425>/Integrator' */
  real_T Integrator_CSTATE_g;          /* '<S422>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_j[2];/* '<S420>/Integrator, Second-Order' */
  real_T omegaWheel;                   /* '<S80>/omega wheel' */
  real_T omegaWheel_j;                 /* '<S81>/omega wheel' */
  real_T omegaWheel_d;                 /* '<S82>/omega wheel' */
  real_T omegaWheel_jl;                /* '<S83>/omega wheel' */
  real_T IntegratorSecondOrder_CSTATE_m[2];/* '<S334>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_i[2];/* '<S335>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_e[2];/* '<S336>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_jk[2];/* '<S337>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_i;          /* '<S348>/Integrator' */
  real_T Integrator_CSTATE_b0;         /* '<S373>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S398>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S423>/Integrator' */
  real_T Integrator1_CSTATE[4];        /* '<S310>/Integrator1' */
  real_T Integrator1_CSTATE_m[12];     /* '<S311>/Integrator1' */
  real_T pqr_CSTATE[3];                /* '<S218>/p,q,r ' */
  real_T TransferFcn5_CSTATE;          /* '<S120>/Transfer Fcn5' */
  real_T TransferFcn6_CSTATE;          /* '<S120>/Transfer Fcn6' */
  real_T TransferFcn7_CSTATE;          /* '<S120>/Transfer Fcn7' */
  real_T TransferFcn8_CSTATE;          /* '<S120>/Transfer Fcn8' */
  real_T Integrator_CSTATE_j[3];       /* '<S226>/Integrator' */
  XDot_LockUp_CAVE_MachE_sil_test_T sf_LockUp_c;/* '<S346>/LockUp' */
  XDot_LockUp_CAVE_MachE_sil_test_T sf_LockUp_h;/* '<S346>/LockUp' */
  XDot_LockUp_CAVE_MachE_sil_test_T sf_LockUp_n;/* '<S346>/LockUp' */
  XDot_LockUp_CAVE_MachE_sil_test_T sf_LockUp;/* '<S346>/LockUp' */
  XDot_CoreSubsys_CAVE_MachE_sil_test_k_T CoreSubsys_m[1];/* '<S143>/CoreSubsys' */
} XDot_CAVE_MachE_sil_test_T;

/* State disabled  */
typedef struct {
  boolean_T xeyeze_CSTATE[3];          /* '<S218>/xe,ye,ze' */
  boolean_T phithetapsi_CSTATE[3];     /* '<S230>/phi theta psi' */
  boolean_T ubvbwb_CSTATE[3];          /* '<S218>/ub,vb,wb' */
  boolean_T Integrator_CSTATE;         /* '<S9>/Integrator' */
  boolean_T TransferFcn3_CSTATE;       /* '<S120>/Transfer Fcn3' */
  boolean_T TransferFcn1_CSTATE;       /* '<S120>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE;       /* '<S120>/Transfer Fcn2' */
  boolean_T TransferFcn4_CSTATE;       /* '<S120>/Transfer Fcn4' */
  boolean_T TransferFcn_CSTATE;        /* '<S120>/Transfer Fcn' */
  boolean_T TransferFcn9_CSTATE;       /* '<S120>/Transfer Fcn9' */
  boolean_T TransferFcn10_CSTATE;      /* '<S120>/Transfer Fcn10' */
  boolean_T TransferFcn11_CSTATE;      /* '<S120>/Transfer Fcn11' */
  boolean_T Integrator_CSTATE_k;       /* '<S39>/Integrator' */
  boolean_T TransferFcn_CSTATE_i;      /* '<S4>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE_i;     /* '<S4>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE_h;     /* '<S4>/Transfer Fcn2' */
  boolean_T TransferFcn3_CSTATE_h;     /* '<S4>/Transfer Fcn3' */
  boolean_T IntegratorLimited_CSTATE;  /* '<S23>/Integrator Limited' */
  boolean_T Integrator_CSTATE_p;       /* '<S350>/Integrator' */
  boolean_T Integrator_CSTATE_o;       /* '<S347>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE[2];/* '<S345>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_c;       /* '<S375>/Integrator' */
  boolean_T Integrator_CSTATE_ch;      /* '<S372>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_l[2];/* '<S370>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_d;       /* '<S400>/Integrator' */
  boolean_T Integrator_CSTATE_b;       /* '<S397>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_ln[2];/* '<S395>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_a;       /* '<S425>/Integrator' */
  boolean_T Integrator_CSTATE_g;       /* '<S422>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_j[2];/* '<S420>/Integrator, Second-Order' */
  boolean_T omegaWheel;                /* '<S80>/omega wheel' */
  boolean_T omegaWheel_j;              /* '<S81>/omega wheel' */
  boolean_T omegaWheel_d;              /* '<S82>/omega wheel' */
  boolean_T omegaWheel_jl;             /* '<S83>/omega wheel' */
  boolean_T IntegratorSecondOrder_CSTATE_m[2];/* '<S334>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_i[2];/* '<S335>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_e[2];/* '<S336>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_jk[2];/* '<S337>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_i;       /* '<S348>/Integrator' */
  boolean_T Integrator_CSTATE_b0;      /* '<S373>/Integrator' */
  boolean_T Integrator_CSTATE_n;       /* '<S398>/Integrator' */
  boolean_T Integrator_CSTATE_m;       /* '<S423>/Integrator' */
  boolean_T Integrator1_CSTATE[4];     /* '<S310>/Integrator1' */
  boolean_T Integrator1_CSTATE_m[12];  /* '<S311>/Integrator1' */
  boolean_T pqr_CSTATE[3];             /* '<S218>/p,q,r ' */
  boolean_T TransferFcn5_CSTATE;       /* '<S120>/Transfer Fcn5' */
  boolean_T TransferFcn6_CSTATE;       /* '<S120>/Transfer Fcn6' */
  boolean_T TransferFcn7_CSTATE;       /* '<S120>/Transfer Fcn7' */
  boolean_T TransferFcn8_CSTATE;       /* '<S120>/Transfer Fcn8' */
  boolean_T Integrator_CSTATE_j[3];    /* '<S226>/Integrator' */
  XDis_LockUp_CAVE_MachE_sil_test_T sf_LockUp_c;/* '<S346>/LockUp' */
  XDis_LockUp_CAVE_MachE_sil_test_T sf_LockUp_h;/* '<S346>/LockUp' */
  XDis_LockUp_CAVE_MachE_sil_test_T sf_LockUp_n;/* '<S346>/LockUp' */
  XDis_LockUp_CAVE_MachE_sil_test_T sf_LockUp;/* '<S346>/LockUp' */
  XDis_CoreSubsys_CAVE_MachE_sil_test_e_T CoreSubsys_m[1];/* '<S143>/CoreSubsys' */
} XDis_CAVE_MachE_sil_test_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Parameters for system: '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */
struct P_CoreSubsys_CAVE_MachE_sil_test_T_ {
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S142>/Constant1'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S142>/Gain'
                                        */
  real_T Constant1_Value_b;            /* Expression: 1
                                        * Referenced by: '<S146>/Constant1'
                                        */
  real_T DCMStaringRow_Gain;           /* Expression: 3
                                        * Referenced by: '<S146>/DCM Staring Row'
                                        */
};

/* Parameters for system: '<S135>/For each axle calculate axle cg positions and velocities' */
struct P_CoreSubsys_CAVE_MachE_sil_test_g_T_ {
  real_T SelectAxleMassByAxle_AxleNums;
                                /* Mask Parameter: SelectAxleMassByAxle_AxleNums
                                 * Referenced by: '<S155>/Axle Numbers'
                                 */
  real_T SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums;
            /* Mask Parameter: SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums
             * Referenced by: '<S156>/Axle Numbers'
             */
  real_T SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums;
            /* Mask Parameter: SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums
             * Referenced by: '<S153>/Axle Numbers'
             */
  real_T SelectAxleMassByAxle_AxleNums_h;
                              /* Mask Parameter: SelectAxleMassByAxle_AxleNums_h
                               * Referenced by: '<S152>/Axle Numbers'
                               */
  real_T SuspensionMomentDirectionOnSolidAxle_Gain;/* Expression: -1
                                                    * Referenced by: '<S149>/Suspension Moment Direction On Solid Axle'
                                                    */
  real_T SuspensionForceDirectionOnSolidAxle_Gain;/* Expression: -1
                                                   * Referenced by: '<S149>/Suspension Force Direction On Solid Axle'
                                                   */
  real_T Trackcoordinatesinaxlebodyframe1_Value[3];/* Expression: [1 0 0]
                                                    * Referenced by: '<S154>/Track coordinates in axle body frame1'
                                                    */
  real_T Trackcoordinatesinaxlebodyframe2_Value;/* Expression: 0
                                                 * Referenced by: '<S154>/Track coordinates in axle body frame2'
                                                 */
  real_T _IC;                          /* Expression: 0
                                        * Referenced by: '<S154>/ '
                                        */
  real_T _UpperSat;                    /* Expression: 0.99*pi/2
                                        * Referenced by: '<S154>/ '
                                        */
  real_T _LowerSat;                    /* Expression: -0.99*pi/2
                                        * Referenced by: '<S154>/ '
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S154>/Gain'
                                        */
  real_T cgcoordinates_IC;             /* Expression: 0
                                        * Referenced by: '<S150>/cg coordinates'
                                        */
  real_T Fy1_Value;                    /* Expression: 0
                                        * Referenced by: '<S148>/Fy1'
                                        */
  real_T Vz_IC;                        /* Expression: 0
                                        * Referenced by: '<S148>/Vz'
                                        */
  real_T Vy1_IC;                       /* Expression: 0
                                        * Referenced by: '<S148>/Vy1'
                                        */
  real_T Vy1_UpperSat;                 /* Expression: 0.99*pi/2
                                        * Referenced by: '<S148>/Vy1'
                                        */
  real_T Vy1_LowerSat;                 /* Expression: -0.99*pi/2
                                        * Referenced by: '<S148>/Vy1'
                                        */
  real_T Fy_Value;                     /* Expression: 0
                                        * Referenced by: '<S148>/Fy'
                                        */
  real_T Vy_IC;                        /* Expression: 0
                                        * Referenced by: '<S148>/Vy'
                                        */
  real_T gEarth_Value;                 /* Expression: 9.807
                                        * Referenced by: '<S148>/g (Earth)'
                                        */
};

/* Parameters for system: '<S172>/Min stop reached' */
struct P_Minstopreached_CAVE_MachE_sil_test_T_ {
  real_T Gain5_Gain;                   /* Expression: 4
                                        * Referenced by: '<S178>/Gain5'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<S178>/Gain4'
                                        */
  real_T Constant_Value;               /* Expression: 3
                                        * Referenced by: '<S178>/Constant'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 4
                                        * Referenced by: '<S178>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S178>/Saturation'
                                        */
  real_T Gain_Gain;                    /* Expression: 3
                                        * Referenced by: '<S178>/Gain'
                                        */
  real_T LowerHardStopBlendMult_tableData[3];/* Expression: [1 1 0]
                                              * Referenced by: '<S178>/Lower Hard Stop Blend Mult'
                                              */
  real_T LowerHardStopBlendMult_bp01Data[3];/* Expression: [-0.02 -0.01 0]
                                             * Referenced by: '<S178>/Lower Hard Stop Blend Mult'
                                             */
};

/* Parameters for system: '<S172>/Max stop reached' */
struct P_Maxstopreached_CAVE_MachE_sil_test_T_ {
  real_T Gain5_Gain;                   /* Expression: 4
                                        * Referenced by: '<S177>/Gain5'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<S177>/Gain4'
                                        */
  real_T Constant_Value;               /* Expression: 3
                                        * Referenced by: '<S177>/Constant'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 4
                                        * Referenced by: '<S177>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S177>/Saturation'
                                        */
  real_T Gain_Gain;                    /* Expression: 3
                                        * Referenced by: '<S177>/Gain'
                                        */
  real_T UpperHardStopBlendMult_tableData[3];/* Expression: [0 1 1]
                                              * Referenced by: '<S177>/Upper Hard Stop Blend Mult'
                                              */
  real_T UpperHardStopBlendMult_bp01Data[3];/* Expression: [0 0.01 0.02]
                                             * Referenced by: '<S177>/Upper Hard Stop Blend Mult'
                                             */
};

/* Parameters for system: '<S135>/For each track and axle combination calculate suspension forces and moments' */
struct P_CoreSubsys_CAVE_MachE_sil_test_d_T_ {
  real_T SelectCamberSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCamberSteeringCenter_AxleNums
                           * Referenced by: '<S164>/Axle Numbers'
                           */
  real_T SelectCamberHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCamberHeightSlope_AxleNums
                              * Referenced by: '<S163>/Axle Numbers'
                              */
  real_T Constrainedspringdampercombination_AxleNums;
                  /* Mask Parameter: Constrainedspringdampercombination_AxleNums
                   * Referenced by:
                   *   '<S173>/Axle Numbers'
                   *   '<S174>/Axle Numbers'
                   *   '<S175>/Axle Numbers'
                   *   '<S176>/Axle Numbers'
                   */
  real_T SelectCasterSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCasterSteeringCenter_AxleNums
                           * Referenced by: '<S166>/Axle Numbers'
                           */
  real_T SelectCasterHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCasterHeightSlope_AxleNums
                              * Referenced by: '<S165>/Axle Numbers'
                              */
  real_T SelectToeSteeringCenter_AxleNums;
                             /* Mask Parameter: SelectToeSteeringCenter_AxleNums
                              * Referenced by: '<S168>/Axle Numbers'
                              */
  real_T SelectRollSteerSlope_AxleNums;
                                /* Mask Parameter: SelectRollSteerSlope_AxleNums
                                 * Referenced by: '<S167>/Axle Numbers'
                                 */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S179>/Constant'
                                        */
  real_T Constant_Value_m;             /* Expression: 0
                                        * Referenced by: '<S158>/Constant'
                                        */
  real_T VehicleVehicleTrackOffset3_Value[2];/* Expression: [0 cumsum(NumTracksByAxl)]
                                              * Referenced by: '<S157>/Vehicle Vehicle Track Offset3'
                                              */
  real_T VehicleVehicleTrackOffset1_Value;/* Expression: StrgEnByAxl
                                           * Referenced by: '<S158>/Vehicle Vehicle Track Offset1'
                                           */
  real_T HeightSignConvention_Gain;    /* Expression: -1
                                        * Referenced by: '<S170>/Height Sign Convention'
                                        */
  real_T VehicleForceSign_Gain;        /* Expression: -1
                                        * Referenced by: '<S169>/Vehicle Force Sign'
                                        */
  real_T Signconvention_Gain;          /* Expression: -1
                                        * Referenced by: '<S169>/Sign convention'
                                        */
  real_T Constant_Value_a;             /* Expression: 0
                                        * Referenced by: '<S160>/Constant'
                                        */
  P_Maxstopreached_CAVE_MachE_sil_test_T Maxstopreached;/* '<S172>/Max stop reached' */
  P_Minstopreached_CAVE_MachE_sil_test_T Minstopreached;/* '<S172>/Min stop reached' */
};

/* Parameters for system: '<S182>/For Each Axle With Anti-Sway' */
struct P_CoreSubsys_CAVE_MachE_sil_test_e_T_ {
  real_T AntiSwayArmRadiusByAxle_AxleNums;
                             /* Mask Parameter: AntiSwayArmRadiusByAxle_AxleNums
                              * Referenced by: '<S186>/Axle Numbers'
                              */
  real_T AntiSwayArmNeutralAngleByAxle_AxleNums;
                       /* Mask Parameter: AntiSwayArmNeutralAngleByAxle_AxleNums
                        * Referenced by: '<S185>/Axle Numbers'
                        */
  real_T AntiSwayBarTorsionSpringConstantByAxle_AxleNums;
              /* Mask Parameter: AntiSwayBarTorsionSpringConstantByAxle_AxleNums
               * Referenced by: '<S187>/Axle Numbers'
               */
  real_T VehicleVehicleTrackOffset3_Value[2];/* Expression: [0 cumsum(NumTracksByAxl)]
                                              * Referenced by: '<S183>/Vehicle Vehicle Track Offset3'
                                              */
  real_T Constant_Value[2];            /* Expression: 1:2
                                        * Referenced by: '<S183>/Constant'
                                        */
  real_T AngleTangentLimit_tableData[2];/* Expression: [-1 1]
                                         * Referenced by: '<S184>/Angle Tangent Limit'
                                         */
  real_T AngleTangentLimit_bp01Data[2];/* Expression: [-1 1]
                                        * Referenced by: '<S184>/Angle Tangent Limit'
                                        */
};

/* Parameters for system: '<S140>/For each track and axle combination calculate suspension forces and moments' */
struct P_CoreSubsys_CAVE_MachE_sil_test_a_T_ {
  real_T SelectCamberSteeringSlope_AxleNums;
                           /* Mask Parameter: SelectCamberSteeringSlope_AxleNums
                            * Referenced by: '<S196>/Axle Numbers'
                            */
  real_T SelectCamberSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCamberSteeringCenter_AxleNums
                           * Referenced by: '<S195>/Axle Numbers'
                           */
  real_T SelectCamberHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCamberHeightSlope_AxleNums
                              * Referenced by: '<S194>/Axle Numbers'
                              */
  real_T SteeringHeightSlopeBySteeredAxle_AxleNums;
                    /* Mask Parameter: SteeringHeightSlopeBySteeredAxle_AxleNums
                     * Referenced by: '<S214>/Axle Numbers'
                     */
  real_T Constrainedspringdampercombination_AxleNums;
                  /* Mask Parameter: Constrainedspringdampercombination_AxleNums
                   * Referenced by:
                   *   '<S207>/Axle Numbers'
                   *   '<S208>/Axle Numbers'
                   *   '<S209>/Axle Numbers'
                   *   '<S210>/Axle Numbers'
                   */
  real_T SelectCasterSteeringSlope_AxleNums;
                           /* Mask Parameter: SelectCasterSteeringSlope_AxleNums
                            * Referenced by: '<S199>/Axle Numbers'
                            */
  real_T SelectCasterSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCasterSteeringCenter_AxleNums
                           * Referenced by: '<S198>/Axle Numbers'
                           */
  real_T SelectCasterHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCasterHeightSlope_AxleNums
                              * Referenced by: '<S197>/Axle Numbers'
                              */
  real_T SelectToeSteeringSlope_AxleNums;
                              /* Mask Parameter: SelectToeSteeringSlope_AxleNums
                               * Referenced by: '<S202>/Axle Numbers'
                               */
  real_T SelectToeSteeringCenter_AxleNums;
                             /* Mask Parameter: SelectToeSteeringCenter_AxleNums
                              * Referenced by: '<S201>/Axle Numbers'
                              */
  real_T SelectRollSteerSlope_AxleNums;
                                /* Mask Parameter: SelectRollSteerSlope_AxleNums
                                 * Referenced by: '<S200>/Axle Numbers'
                                 */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S189>/Constant'
                                        */
  real_T VehicleVehicleTrackOffset3_Value[2];/* Expression: [0 cumsum(NumTracksByAxl)]
                                              * Referenced by: '<S188>/Vehicle Vehicle Track Offset3'
                                              */
  real_T VehicleVehicleTrackOffset1_Value;/* Expression: StrgEnByAxl
                                           * Referenced by: '<S189>/Vehicle Vehicle Track Offset1'
                                           */
  real_T HeightSignConvention_Gain;    /* Expression: -1
                                        * Referenced by: '<S204>/Height Sign Convention'
                                        */
  real_T VehicleForceSign_Gain;        /* Expression: -1
                                        * Referenced by: '<S203>/Vehicle Force Sign'
                                        */
  real_T Signconvention_Gain;          /* Expression: -1
                                        * Referenced by: '<S203>/Sign convention'
                                        */
  real_T Constant_Value_j;             /* Expression: 0
                                        * Referenced by: '<S191>/Constant'
                                        */
  P_Maxstopreached_CAVE_MachE_sil_test_T Maxstopreached;/* '<S206>/Max stop reached' */
  P_Minstopreached_CAVE_MachE_sil_test_T Minstopreached;/* '<S206>/Min stop reached' */
};

/* Parameters for system: '<S346>/LockUp' */
struct P_LockUp_CAVE_MachE_sil_test_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S358>/Constant'
                                        */
  real_T locked_Value;                 /* Expression: 0
                                        * Referenced by: '<S356>/locked'
                                        */
  real_T locked1_Value;                /* Expression: 0
                                        * Referenced by: '<S356>/locked1'
                                        */
  real_T locked2_Value;                /* Expression: 0
                                        * Referenced by: '<S356>/locked2'
                                        */
  real_T u_Gain;                       /* Expression: -4
                                        * Referenced by: '<S357>/-4'
                                        */
  boolean_T yn_Y0;                     /* Computed Parameter: yn_Y0
                                        * Referenced by: '<S359>/yn'
                                        */
  boolean_T yn_Y0_m;                   /* Computed Parameter: yn_Y0_m
                                        * Referenced by: '<S358>/yn'
                                        */
  boolean_T UnitDelay_InitialCondition;
                               /* Computed Parameter: UnitDelay_InitialCondition
                                * Referenced by: '<S363>/Unit Delay'
                                */
  boolean_T CombinatorialLogic_table[8];
                                 /* Computed Parameter: CombinatorialLogic_table
                                  * Referenced by: '<S363>/Combinatorial  Logic'
                                  */
};

/* Parameters (default storage) */
struct P_CAVE_MachE_sil_test_T_ {
  struct_m2VjwNiXoluKspK4Fr7zNG InitStates[34];/* Variable: InitStates
                                                * Referenced by: '<S19>/Constant1'
                                                */
  real_T BattChargeLimitMaxPwr;        /* Variable: BattChargeLimitMaxPwr
                                        * Referenced by: '<S52>/MaxChrg'
                                        */
  real_T BattChargeLimitSocBpts[12];   /* Variable: BattChargeLimitSocBpts
                                        * Referenced by:
                                        *   '<S52>/ChrgLmt'
                                        *   '<S54>/ChrgLmt'
                                        */
  real_T BattChargeLimitTbl[12];       /* Variable: BattChargeLimitTbl
                                        * Referenced by:
                                        *   '<S52>/ChrgLmt'
                                        *   '<S54>/ChrgLmt'
                                        */
  real_T BattChrgCapcty;               /* Variable: BattChrgCapcty
                                        * Referenced by:
                                        *   '<S19>/Constant1'
                                        *   '<S23>/Constant1'
                                        *   '<S23>/Integrator Limited'
                                        *   '<S23>/Switch'
                                        *   '<S24>/Constant1'
                                        */
  real_T BattDischargeLimitSocBpts[11];/* Variable: BattDischargeLimitSocBpts
                                        * Referenced by: '<S52>/DischrgLmt'
                                        */
  real_T BattDischargeLimitTbl[11];    /* Variable: BattDischargeLimitTbl
                                        * Referenced by: '<S52>/DischrgLmt'
                                        */
  real_T BattDischargeMaxPwr;          /* Variable: BattDischargeMaxPwr
                                        * Referenced by: '<S52>/MaxDischrg'
                                        */
  real_T BattNumCellsParallel;         /* Variable: BattNumCellsParallel
                                        * Referenced by:
                                        *   '<S23>/Gain1'
                                        *   '<S25>/Gain2'
                                        *   '<S25>/Gain4'
                                        */
  real_T BattNumCellsSeries;           /* Variable: BattNumCellsSeries
                                        * Referenced by:
                                        *   '<S25>/Gain1'
                                        *   '<S25>/Gain3'
                                        */
  real_T BattOpenVoltCapBpts[11];      /* Variable: BattOpenVoltCapBpts
                                        * Referenced by: '<S25>/Em'
                                        */
  real_T BattOpenVoltTbl[11];          /* Variable: BattOpenVoltTbl
                                        * Referenced by: '<S25>/Em'
                                        */
  real_T BattResistSocBpts[6];         /* Variable: BattResistSocBpts
                                        * Referenced by: '<S25>/R'
                                        */
  real_T BattResistTbl[42];            /* Variable: BattResistTbl
                                        * Referenced by: '<S25>/R'
                                        */
  real_T BattResistTempBpts[7];        /* Variable: BattResistTempBpts
                                        * Referenced by: '<S25>/R'
                                        */
  real_T BrakeMaxPrs;                  /* Variable: BrakeMaxPrs
                                        * Referenced by:
                                        *   '<S121>/Gain'
                                        *   '<S121>/Saturation'
                                        *   '<S121>/Saturation1'
                                        *   '<S121>/Saturation2'
                                        *   '<S121>/Saturation3'
                                        */
  real_T BrakeMaxTrq;                  /* Variable: BrakeMaxTrq
                                        * Referenced by:
                                        *   '<S54>/Gain1'
                                        *   '<S54>/Gain2'
                                        */
  real_T ChassisDistCg2FrontAxle;      /* Variable: ChassisDistCg2FrontAxle
                                        * Referenced by:
                                        *   '<S217>/vehdyncginert'
                                        *   '<S250>/Constant3'
                                        */
  real_T ChassisDistCg2RearAxle;       /* Variable: ChassisDistCg2RearAxle
                                        * Referenced by:
                                        *   '<S217>/vehdyncginert'
                                        *   '<S250>/Constant3'
                                        */
  real_T ChassisFrontalArea;           /* Variable: ChassisFrontalArea
                                        * Referenced by: '<S250>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T DiffRatio;                    /* Variable: DiffRatio
                                        * Referenced by:
                                        *   '<S30>/Gain2'
                                        *   '<S30>/Gain3'
                                        *   '<S54>/MotTrqReflectedToWheels'
                                        *   '<S54>/WhlTrqReflectedToMot'
                                        */
  real_T EnvGravCnst;                  /* Variable: EnvGravCnst
                                        * Referenced by: '<S221>/g'
                                        */
  real_T EnvPrs;                       /* Variable: EnvPrs
                                        * Referenced by:
                                        *   '<S6>/Constant'
                                        *   '<S250>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T EnvTemp;                      /* Variable: EnvTemp
                                        * Referenced by:
                                        *   '<S6>/Constant1'
                                        *   '<S16>/Constant'
                                        *   '<S217>/AirTempConstant'
                                        */
  real_T MtrEffSpdBpts[12];            /* Variable: MtrEffSpdBpts
                                        * Referenced by: '<S60>/Eff Map'
                                        */
  real_T MtrEffTbl[180];               /* Variable: MtrEffTbl
                                        * Referenced by: '<S60>/Eff Map'
                                        */
  real_T MtrEffTrqBpts[15];            /* Variable: MtrEffTrqBpts
                                        * Referenced by: '<S60>/Eff Map'
                                        */
  real_T MtrPwrMax;                    /* Variable: MtrPwrMax
                                        * Referenced by:
                                        *   '<S39>/Constant'
                                        *   '<S55>/Constant'
                                        *   '<S70>/Constant'
                                        *   '<S43>/Constant'
                                        *   '<S66>/Constant'
                                        */
  real_T MtrTrqMax;                    /* Variable: MtrTrqMax
                                        * Referenced by:
                                        *   '<S39>/Constant1'
                                        *   '<S55>/Saturation'
                                        *   '<S70>/Saturation'
                                        *   '<S42>/Saturation'
                                        *   '<S66>/Saturation'
                                        */
  real_T MtrTrqTimeCnst;               /* Variable: MtrTrqTimeCnst
                                        * Referenced by: '<S39>/Gain1'
                                        */
  real_T SupvsryCtrlRegenBrkCutOffTbl[2];/* Variable: SupvsryCtrlRegenBrkCutOffTbl
                                          * Referenced by: '<S54>/RegenBrakingCutoff'
                                          */
  real_T SupvsryCtrlRegenSpdBpts[2];   /* Variable: SupvsryCtrlRegenSpdBpts
                                        * Referenced by: '<S54>/RegenBrakingCutoff'
                                        */
  real_T TestScnrioCycleNum;           /* Variable: TestScnrioCycleNum
                                        * Referenced by: '<S19>/Constant1'
                                        */
  real_T CombinedSlipWheel2DOF1_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_ALPMAX
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_ALPMAX
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_ALPMAX
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_ALPMAX
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_ALPMIN
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_ALPMIN
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_ALPMIN
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_ALPMIN
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T independentSuspensionsMacPherson_AntiSwayNtrlAng;
             /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayNtrlAng
              * Referenced by: '<S184>/Constant2'
              */
  real_T independentSuspensionsMacPherson_AntiSwayR;
                   /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayR
                    * Referenced by: '<S184>/Constant1'
                    */
  real_T independentSuspensionsMacPherson_AntiSwayTrsK;
                /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayTrsK
                 * Referenced by: '<S184>/Constant3'
                 */
  real_T SolidAxleSuspensionCoilSpring_AxlIxx;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxlIxx
                          * Referenced by:
                          *   '<S148>/Axle I'
                          *   '<S151>/Axle I1'
                          */
  real_T SolidAxleSuspensionCoilSpring_AxlM;
                           /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxlM
                            * Referenced by:
                            *   '<S148>/Axle M'
                            *   '<S151>/Axle M'
                            */
  real_T SolidAxleSuspensionCoilSpring_AxleNumVec[2];
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxleNumVec
                      * Referenced by:
                      *   '<S135>/Axle Number'
                      *   '<S135>/Axle Number2'
                      *   '<S149>/Constant2'
                      *   '<S151>/Constant2'
                      */
  real_T independentSuspensionsMacPherson_AxleNumVec[2];
                  /* Mask Parameter: independentSuspensionsMacPherson_AxleNumVec
                   * Referenced by: '<S140>/Axle Number'
                   */
  real_T CombinedSlipWheel2DOF1_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_BREFF
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_BREFF
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_BREFF
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_BREFF
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_CAMMAX
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_CAMMAX
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_CAMMAX
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_CAMMAX
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_CAMMIN
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_CAMMIN
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_CAMMIN
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_CAMMIN
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T SolidAxleSuspensionCoilSpring_Camber;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_Camber
                          * Referenced by: '<S162>/Constant4'
                          */
  real_T independentSuspensionsMacPherson_Camber;
                      /* Mask Parameter: independentSuspensionsMacPherson_Camber
                       * Referenced by: '<S193>/Constant4'
                       */
  real_T SolidAxleSuspensionCoilSpring_CamberHslp;
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_CamberHslp
                      * Referenced by: '<S162>/Constant5'
                      */
  real_T independentSuspensionsMacPherson_CamberHslp;
                  /* Mask Parameter: independentSuspensionsMacPherson_CamberHslp
                   * Referenced by: '<S193>/Constant5'
                   */
  real_T independentSuspensionsMacPherson_CamberStrgSlp;
               /* Mask Parameter: independentSuspensionsMacPherson_CamberStrgSlp
                * Referenced by: '<S193>/Constant3'
                */
  real_T SolidAxleSuspensionCoilSpring_Caster;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_Caster
                          * Referenced by: '<S162>/Constant7'
                          */
  real_T independentSuspensionsMacPherson_Caster;
                      /* Mask Parameter: independentSuspensionsMacPherson_Caster
                       * Referenced by: '<S193>/Constant7'
                       */
  real_T SolidAxleSuspensionCoilSpring_CasterHslp;
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_CasterHslp
                      * Referenced by: '<S162>/Constant8'
                      */
  real_T independentSuspensionsMacPherson_CasterHslp;
                  /* Mask Parameter: independentSuspensionsMacPherson_CasterHslp
                   * Referenced by: '<S193>/Constant8'
                   */
  real_T independentSuspensionsMacPherson_CasterStrgSlp;
               /* Mask Parameter: independentSuspensionsMacPherson_CasterStrgSlp
                * Referenced by: '<S193>/Constant6'
                */
  real_T VehicleBody6DOF1_Cd;          /* Mask Parameter: VehicleBody6DOF1_Cd
                                        * Referenced by: '<S250>/Constant'
                                        */
  real_T VehicleBody6DOF1_Cl;          /* Mask Parameter: VehicleBody6DOF1_Cl
                                        * Referenced by: '<S250>/Constant1'
                                        */
  real_T VehicleBody6DOF1_Cpm;         /* Mask Parameter: VehicleBody6DOF1_Cpm
                                        * Referenced by: '<S250>/Constant2'
                                        */
  real_T VehicleBody6DOF1_Cs[31];      /* Mask Parameter: VehicleBody6DOF1_Cs
                                        * Referenced by: '<S250>/Cs'
                                        */
  real_T VehicleBody6DOF1_Cym[31];     /* Mask Parameter: VehicleBody6DOF1_Cym
                                        * Referenced by: '<S250>/Cym'
                                        */
  real_T SolidAxleSuspensionCoilSpring_Cz;
                             /* Mask Parameter: SolidAxleSuspensionCoilSpring_Cz
                              * Referenced by: '<S170>/Constant2'
                              */
  real_T independentSuspensionsMacPherson_Cz;
                          /* Mask Parameter: independentSuspensionsMacPherson_Cz
                           * Referenced by: '<S204>/Constant2'
                           */
  real_T SolidAxleSuspensionCoilSpring_CzWhlAxl;
                       /* Mask Parameter: SolidAxleSuspensionCoilSpring_CzWhlAxl
                        * Referenced by: '<S145>/Carrier to Axle Damping'
                        */
  real_T CombinedSlipWheel2DOF1_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_DREFF
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_DREFF
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_DREFF
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_DREFF
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T KinematicSteering_Db;         /* Mask Parameter: KinematicSteering_Db
                                        * Referenced by: '<S130>/Backlash'
                                        */
  real_T SolidAxleSuspensionCoilSpring_F0z;
                            /* Mask Parameter: SolidAxleSuspensionCoilSpring_F0z
                             * Referenced by: '<S170>/Constant1'
                             */
  real_T independentSuspensionsMacPherson_F0z;
                         /* Mask Parameter: independentSuspensionsMacPherson_F0z
                          * Referenced by: '<S204>/Constant1'
                          */
  real_T SolidAxleSuspensionCoilSpring_F0zWhlAxl;
                      /* Mask Parameter: SolidAxleSuspensionCoilSpring_F0zWhlAxl
                       * Referenced by: '<S145>/Preload'
                       */
  real_T CombinedSlipWheel2DOF1_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_FNOMIN
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_FNOMIN
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_FNOMIN
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_FNOMIN
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FREFF
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FREFF
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FREFF
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FREFF
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FZMAX
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FZMAX
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FZMAX
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FZMAX
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FZMIN
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FZMIN
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FZMIN
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FZMIN
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_GRAVITY
                                * Referenced by: '<S345>/Fg'
                                */
  real_T CombinedSlipWheel2DOF2_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_GRAVITY
                                * Referenced by: '<S370>/Fg'
                                */
  real_T CombinedSlipWheel2DOF3_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_GRAVITY
                                * Referenced by: '<S395>/Fg'
                                */
  real_T CombinedSlipWheel2DOF4_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_GRAVITY
                                * Referenced by: '<S420>/Fg'
                                */
  real_T SolidAxleSuspensionCoilSpring_Hmax;
                           /* Mask Parameter: SolidAxleSuspensionCoilSpring_Hmax
                            * Referenced by:
                            *   '<S170>/Constant3'
                            *   '<S172>/Max stop reached'
                            *   '<S172>/Min stop reached'
                            */
  real_T independentSuspensionsMacPherson_Hmax;
                        /* Mask Parameter: independentSuspensionsMacPherson_Hmax
                         * Referenced by:
                         *   '<S204>/Constant3'
                         *   '<S206>/Max stop reached'
                         *   '<S206>/Min stop reached'
                         */
  real_T CombinedSlipWheel2DOF1_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF1_IYY
                                    * Referenced by: '<S346>/LockUp'
                                    */
  real_T CombinedSlipWheel2DOF2_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF2_IYY
                                    * Referenced by: '<S371>/LockUp'
                                    */
  real_T CombinedSlipWheel2DOF3_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF3_IYY
                                    * Referenced by: '<S396>/LockUp'
                                    */
  real_T CombinedSlipWheel2DOF4_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF4_IYY
                                    * Referenced by: '<S421>/LockUp'
                                    */
  real_T VehicleBody6DOF1_Iveh[9];     /* Mask Parameter: VehicleBody6DOF1_Iveh
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T CombinedSlipWheel2DOF1_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_KPUMAX
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_KPUMAX
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_KPUMAX
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_KPUMAX
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_KPUMIN
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_KPUMIN
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_KPUMIN
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_KPUMIN
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T SolidAxleSuspensionCoilSpring_Kz;
                             /* Mask Parameter: SolidAxleSuspensionCoilSpring_Kz
                              * Referenced by: '<S170>/Constant'
                              */
  real_T independentSuspensionsMacPherson_Kz;
                          /* Mask Parameter: independentSuspensionsMacPherson_Kz
                           * Referenced by: '<S204>/Constant'
                           */
  real_T SolidAxleSuspensionCoilSpring_KzWhlAxl;
                       /* Mask Parameter: SolidAxleSuspensionCoilSpring_KzWhlAxl
                        * Referenced by: '<S145>/Carrier to Axle Compliance'
                        */
  real_T CombinedSlipWheel2DOF1_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF1_LATERAL_STIFFNESS
                      * Referenced by: '<S344>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF2_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF2_LATERAL_STIFFNESS
                      * Referenced by: '<S369>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF3_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF3_LATERAL_STIFFNESS
                      * Referenced by: '<S394>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF4_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF4_LATERAL_STIFFNESS
                      * Referenced by: '<S419>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF1_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF1_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S344>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF2_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF2_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S369>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF3_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF3_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S394>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF4_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF4_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S419>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF1_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_LONGVL
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_LONGVL
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_LONGVL
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_LONGVL
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_MASS
                                   * Referenced by:
                                   *   '<S345>/Fg'
                                   *   '<S345>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF2_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_MASS
                                   * Referenced by:
                                   *   '<S370>/Fg'
                                   *   '<S370>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF3_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_MASS
                                   * Referenced by:
                                   *   '<S395>/Fg'
                                   *   '<S395>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF4_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_MASS
                                   * Referenced by:
                                   *   '<S420>/Fg'
                                   *   '<S420>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF1_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_NOMPRES
                                * Referenced by: '<S344>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_NOMPRES
                                * Referenced by: '<S369>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_NOMPRES
                                * Referenced by: '<S394>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_NOMPRES
                                * Referenced by: '<S419>/Magic Tire Const Input'
                                */
  real_T LockUp_OmegaTol;              /* Mask Parameter: LockUp_OmegaTol
                                        * Referenced by: '<S346>/LockUp'
                                        */
  real_T LockUp_OmegaTol_p;            /* Mask Parameter: LockUp_OmegaTol_p
                                        * Referenced by: '<S371>/LockUp'
                                        */
  real_T LockUp_OmegaTol_l;            /* Mask Parameter: LockUp_OmegaTol_l
                                        * Referenced by: '<S396>/LockUp'
                                        */
  real_T LockUp_OmegaTol_d;            /* Mask Parameter: LockUp_OmegaTol_d
                                        * Referenced by: '<S421>/LockUp'
                                        */
  real_T CombinedSlipWheel2DOF1_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX3
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX3
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX3
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX3
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY3
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY3
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY3
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY3
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PCX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PCX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PCX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PCX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PCY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PCY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PCY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PCY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP3
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP3
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP3
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP3
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP3
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP3
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP3
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP3
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP4
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP4
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP4
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP4
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PECP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PECP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PECP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PECP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PECP2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PECP2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PECP2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PECP2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PFZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PFZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PFZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PFZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP3
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP3
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP3
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP3
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP4
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP4
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP4
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP4
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY6
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY6
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY6
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY6
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY7
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY7
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY7
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY7
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PKYP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PKYP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PKYP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PKYP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PPMX1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PPMX1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PPMX1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PPMX1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPZ2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPZ2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPZ2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPZ2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_PRESMAX
                                * Referenced by: '<S344>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_PRESMAX
                                * Referenced by: '<S369>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_PRESMAX
                                * Referenced by: '<S394>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_PRESMAX
                                * Referenced by: '<S419>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF1_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_PRESMIN
                                * Referenced by: '<S344>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_PRESMIN
                                * Referenced by: '<S369>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_PRESMIN
                                * Referenced by: '<S394>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_PRESMIN
                                * Referenced by: '<S419>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF1_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QBRP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QBRP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QBRP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QBRP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ10
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ10
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ10
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ10
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ6
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ6
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ6
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ6
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ9
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ9
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ9
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ9
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QCRP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QCRP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QCRP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QCRP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QCRP2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QCRP2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QCRP2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QCRP2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QCZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QCZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QCZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QCZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDRP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDRP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDRP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDRP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDRP2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDRP2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDRP2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDRP2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDTP1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDTP1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDTP1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDTP1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ10
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ10
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ10
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ10
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ11
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ11
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ11
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ11
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ6
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ6
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ6
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ6
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ7
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ7
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ7
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ7
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ8
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ8
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ8
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ8
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ9
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ9
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ9
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ9
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX10
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX10
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX10
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX10
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX11
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX11
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX11
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX11
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX12
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX12
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX12
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX12
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX13
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX13
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX13
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX13
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX14
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX14
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX14
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX14
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX6
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX6
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX6
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX6
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX7
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX7
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX7
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX7
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX8
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX8
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX8
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX8
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX9
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX9
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX9
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX9
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY6
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY6
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY6
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY6
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY7
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY7
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY7
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY7
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY8
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY8
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY8
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY8
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCX
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCX
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCX
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCX
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCY
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCY
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCY
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCY
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCY2
                                 * Referenced by: '<S344>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCY2
                                 * Referenced by: '<S369>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCY2
                                 * Referenced by: '<S394>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCY2
                                 * Referenced by: '<S419>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ3
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ3
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ3
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ3
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RA1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RA1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RA1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RA1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RA2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RA2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RA2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RA2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RB1
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RB1
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RB1
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RB1
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RB2
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RB2
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RB2
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RB2
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RE0
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RE0
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RE0
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RE0
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_Q_V1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_Q_V1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_Q_V1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_Q_V1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_Q_V2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_Q_V2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_Q_V2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_Q_V2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T DragForce_R;                  /* Mask Parameter: DragForce_R
                                        * Referenced by: '<S250>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T CombinedSlipWheel2DOF1_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RCX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RCX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RCX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RCX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RCY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RCY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RCY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RCY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REX2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REX2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REX2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REX2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHX1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHX1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHX1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHX1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY5
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY5
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY5
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY5
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY6
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY6
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY6
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY6
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Rm; /* Mask Parameter: CombinedSlipWheel2DOF1_Rm
                                     * Referenced by: '<S355>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF2_Rm; /* Mask Parameter: CombinedSlipWheel2DOF2_Rm
                                     * Referenced by: '<S380>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF3_Rm; /* Mask Parameter: CombinedSlipWheel2DOF3_Rm
                                     * Referenced by: '<S405>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF4_Rm; /* Mask Parameter: CombinedSlipWheel2DOF4_Rm
                                     * Referenced by: '<S430>/Torque Conversion'
                                     */
  real_T SolidAxleSuspensionCoilSpring_RollStrgSlp;
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_RollStrgSlp
                     * Referenced by: '<S162>/Constant2'
                     */
  real_T independentSuspensionsMacPherson_RollStrgSlp;
                 /* Mask Parameter: independentSuspensionsMacPherson_RollStrgSlp
                  * Referenced by: '<S193>/Constant2'
                  */
  real_T CombinedSlipWheel2DOF1_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ1
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ1
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ1
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ1
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ2
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ2
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ2
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ2
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ3
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ3
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ3
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ3
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ4
                                   * Referenced by: '<S344>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ4
                                   * Referenced by: '<S369>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ4
                                   * Referenced by: '<S394>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ4
                                   * Referenced by: '<S419>/Magic Tire Const Input'
                                   */
  real_T independentSuspensionsMacPherson_StrgHgtSlp;
                  /* Mask Parameter: independentSuspensionsMacPherson_StrgHgtSlp
                   * Referenced by: '<S213>/Constant'
                   */
  real_T KinematicSteering_StrgRatio;
                                  /* Mask Parameter: KinematicSteering_StrgRatio
                                   * Referenced by:
                                   *   '<S131>/Constant'
                                   *   '<S131>/Gain'
                                   *   '<S131>/Gain1'
                                   *   '<S131>/Gain2'
                                   */
  real_T KinematicSteering_StrgRng; /* Mask Parameter: KinematicSteering_StrgRng
                                     * Referenced by: '<S130>/Saturation'
                                     */
  real_T SolidAxleSuspensionCoilSpring_SuspCoords[6];
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_SuspCoords
                      * Referenced by:
                      *   '<S142>/Suspension axle connection coordinates in axle body frame'
                      *   '<S149>/Suspension connection point coordinates in axle body frame'
                      */
  real_T SolidAxleSuspensionCoilSpring_Toe;
                            /* Mask Parameter: SolidAxleSuspensionCoilSpring_Toe
                             * Referenced by: '<S162>/Constant1'
                             */
  real_T independentSuspensionsMacPherson_Toe;
                         /* Mask Parameter: independentSuspensionsMacPherson_Toe
                          * Referenced by: '<S193>/Constant1'
                          */
  real_T independentSuspensionsMacPherson_ToeStrgSlp;
                  /* Mask Parameter: independentSuspensionsMacPherson_ToeStrgSlp
                   * Referenced by: '<S193>/Constant'
                   */
  real_T SolidAxleSuspensionCoilSpring_TrackCoords[6];
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_TrackCoords
                     * Referenced by:
                     *   '<S142>/Track coordinates in axle body frame'
                     *   '<S151>/Track coordinates in axle body frame'
                     */
  real_T independentSuspensionsMacPherson_TrackNumVec[2];
                 /* Mask Parameter: independentSuspensionsMacPherson_TrackNumVec
                  * Referenced by: '<S140>/Track Number'
                  */
  real_T SolidAxleSuspensionCoilSpring_TrackNumVec[2];
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_TrackNumVec
                     * Referenced by: '<S135>/Track Number'
                     */
  real_T CombinedSlipWheel2DOF1_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF1_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S344>/Magic Tire Const Input'
                        *   '<S346>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF2_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF2_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S369>/Magic Tire Const Input'
                        *   '<S371>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF3_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF3_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S394>/Magic Tire Const Input'
                        *   '<S396>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF4_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF4_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S419>/Magic Tire Const Input'
                        *   '<S421>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF1_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF1_VERTICAL_DAMPING
                       * Referenced by: '<S345>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF2_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF2_VERTICAL_DAMPING
                       * Referenced by: '<S370>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF3_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF3_VERTICAL_DAMPING
                       * Referenced by: '<S395>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF4_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF4_VERTICAL_DAMPING
                       * Referenced by: '<S420>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF1_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF1_VERTICAL_STIFFNESS
                     * Referenced by: '<S344>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF2_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF2_VERTICAL_STIFFNESS
                     * Referenced by: '<S369>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF3_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF3_VERTICAL_STIFFNESS
                     * Referenced by: '<S394>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF4_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF4_VERTICAL_STIFFNESS
                     * Referenced by: '<S419>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF1_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_VXLOW
                                  * Referenced by:
                                  *   '<S344>/Magic Tire Const Input'
                                  *   '<S347>/Dead Zone'
                                  *   '<S348>/Dead Zone'
                                  *   '<S350>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF2_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_VXLOW
                                  * Referenced by:
                                  *   '<S369>/Magic Tire Const Input'
                                  *   '<S372>/Dead Zone'
                                  *   '<S373>/Dead Zone'
                                  *   '<S375>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF3_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_VXLOW
                                  * Referenced by:
                                  *   '<S394>/Magic Tire Const Input'
                                  *   '<S397>/Dead Zone'
                                  *   '<S398>/Dead Zone'
                                  *   '<S400>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF4_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_VXLOW
                                  * Referenced by:
                                  *   '<S419>/Magic Tire Const Input'
                                  *   '<S422>/Dead Zone'
                                  *   '<S423>/Dead Zone'
                                  *   '<S425>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF1_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_WIDTH
                                  * Referenced by: '<S344>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_WIDTH
                                  * Referenced by: '<S369>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_WIDTH
                                  * Referenced by: '<S394>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_WIDTH
                                  * Referenced by: '<S419>/Magic Tire Const Input'
                                  */
  real_T VehicleBody6DOF1_Xe_o[3];     /* Mask Parameter: VehicleBody6DOF1_Xe_o
                                        * Referenced by: '<S218>/xe,ye,ze'
                                        */
  real_T VehicleBody6DOF1_beta_w[31]; /* Mask Parameter: VehicleBody6DOF1_beta_w
                                       * Referenced by:
                                       *   '<S250>/Cs'
                                       *   '<S250>/Cym'
                                       */
  real_T CombinedSlipWheel2DOF1_br; /* Mask Parameter: CombinedSlipWheel2DOF1_br
                                     * Referenced by: '<S346>/LockUp'
                                     */
  real_T CombinedSlipWheel2DOF2_br; /* Mask Parameter: CombinedSlipWheel2DOF2_br
                                     * Referenced by: '<S371>/LockUp'
                                     */
  real_T CombinedSlipWheel2DOF3_br; /* Mask Parameter: CombinedSlipWheel2DOF3_br
                                     * Referenced by: '<S396>/LockUp'
                                     */
  real_T CombinedSlipWheel2DOF4_br; /* Mask Parameter: CombinedSlipWheel2DOF4_br
                                     * Referenced by: '<S421>/LockUp'
                                     */
  real_T VerticalWheelandUnsprungMassResponse_bz;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse_bz
                       * Referenced by: '<S334>/Gain2'
                       */
  real_T VerticalWheelandUnsprungMassResponse1_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_bz
                      * Referenced by: '<S335>/Gain2'
                      */
  real_T VerticalWheelandUnsprungMassResponse2_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_bz
                      * Referenced by: '<S336>/Gain2'
                      */
  real_T VerticalWheelandUnsprungMassResponse3_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_bz
                      * Referenced by: '<S337>/Gain2'
                      */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S56>/Constant'
                                       */
  real_T CompareToConstant_const_l; /* Mask Parameter: CompareToConstant_const_l
                                     * Referenced by: '<S59>/Constant'
                                     */
  real_T CompareToConstant_const_m; /* Mask Parameter: CompareToConstant_const_m
                                     * Referenced by: '<S96>/Constant'
                                     */
  real_T CompareToConstant1_const;   /* Mask Parameter: CompareToConstant1_const
                                      * Referenced by: '<S97>/Constant'
                                      */
  real_T Pressure_const;               /* Mask Parameter: Pressure_const
                                        * Referenced by: '<S312>/Pressure'
                                        */
  real_T VehicleBody6DOF1_d;           /* Mask Parameter: VehicleBody6DOF1_d
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T CombinedSlipWheel2DOF1_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF1_disk_abore
                             * Referenced by: '<S355>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF2_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF2_disk_abore
                             * Referenced by: '<S380>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF3_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF3_disk_abore
                             * Referenced by: '<S405>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF4_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF4_disk_abore
                             * Referenced by: '<S430>/Disk brake actuator bore'
                             */
  real_T VehicleBody6DOF1_eul_o[3];    /* Mask Parameter: VehicleBody6DOF1_eul_o
                                        * Referenced by: '<S230>/phi theta psi'
                                        */
  real_T VerticalWheelandUnsprungMassResponse_g;
                       /* Mask Parameter: VerticalWheelandUnsprungMassResponse_g
                        * Referenced by: '<S334>/Fg'
                        */
  real_T VerticalWheelandUnsprungMassResponse1_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_g
                       * Referenced by: '<S335>/Fg'
                       */
  real_T VerticalWheelandUnsprungMassResponse2_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_g
                       * Referenced by: '<S336>/Fg'
                       */
  real_T VerticalWheelandUnsprungMassResponse3_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_g
                       * Referenced by: '<S337>/Fg'
                       */
  real_T VehicleBody6DOF1_h;           /* Mask Parameter: VehicleBody6DOF1_h
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_latOff;     /* Mask Parameter: VehicleBody6DOF1_latOff
                                       * Referenced by: '<S253>/latOff'
                                       */
  real_T VehicleBody6DOF1_longOff;   /* Mask Parameter: VehicleBody6DOF1_longOff
                                      * Referenced by: '<S253>/longOff'
                                      */
  real_T VehicleBody6DOF1_m;           /* Mask Parameter: VehicleBody6DOF1_m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VerticalWheelandUnsprungMassResponse_m;
                       /* Mask Parameter: VerticalWheelandUnsprungMassResponse_m
                        * Referenced by:
                        *   '<S334>/Fg'
                        *   '<S334>/Gain1'
                        */
  real_T VerticalWheelandUnsprungMassResponse1_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_m
                       * Referenced by:
                       *   '<S335>/Fg'
                       *   '<S335>/Gain1'
                       */
  real_T VerticalWheelandUnsprungMassResponse2_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_m
                       * Referenced by:
                       *   '<S336>/Fg'
                       *   '<S336>/Gain1'
                       */
  real_T VerticalWheelandUnsprungMassResponse3_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_m
                       * Referenced by:
                       *   '<S337>/Fg'
                       *   '<S337>/Gain1'
                       */
  real_T CombinedSlipWheel2DOF1_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF1_mu_kinetic
                             * Referenced by:
                             *   '<S352>/Ratio of static to kinetic'
                             *   '<S355>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF2_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF2_mu_kinetic
                             * Referenced by:
                             *   '<S377>/Ratio of static to kinetic'
                             *   '<S380>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF3_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF3_mu_kinetic
                             * Referenced by:
                             *   '<S402>/Ratio of static to kinetic'
                             *   '<S405>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF4_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF4_mu_kinetic
                             * Referenced by:
                             *   '<S427>/Ratio of static to kinetic'
                             *   '<S430>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF1_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF1_mu_static
                              * Referenced by: '<S352>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF2_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF2_mu_static
                              * Referenced by: '<S377>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF3_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF3_mu_static
                              * Referenced by: '<S402>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF4_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF4_mu_static
                              * Referenced by: '<S427>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF1_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF1_num_pads
                               * Referenced by: '<S355>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF2_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF2_num_pads
                               * Referenced by: '<S380>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF3_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF3_num_pads
                               * Referenced by: '<S405>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF4_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF4_num_pads
                               * Referenced by: '<S430>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF1_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_omegao
                                 * Referenced by: '<S346>/LockUp'
                                 */
  real_T CombinedSlipWheel2DOF2_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_omegao
                                 * Referenced by: '<S371>/LockUp'
                                 */
  real_T CombinedSlipWheel2DOF3_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_omegao
                                 * Referenced by: '<S396>/LockUp'
                                 */
  real_T CombinedSlipWheel2DOF4_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_omegao
                                 * Referenced by: '<S421>/LockUp'
                                 */
  real_T VehicleBody6DOF1_p_o[3];      /* Mask Parameter: VehicleBody6DOF1_p_o
                                        * Referenced by: '<S218>/p,q,r '
                                        */
  real_T div0protectpoly_thresh;       /* Mask Parameter: div0protectpoly_thresh
                                        * Referenced by:
                                        *   '<S68>/Constant'
                                        *   '<S69>/Constant'
                                        */
  real_T VehicleBody6DOF1_vertOff;   /* Mask Parameter: VehicleBody6DOF1_vertOff
                                      * Referenced by: '<S253>/vertOff '
                                      */
  real_T VehicleBody6DOF1_w[2];        /* Mask Parameter: VehicleBody6DOF1_w
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T ContLPF_wc;                   /* Mask Parameter: ContLPF_wc
                                        * Referenced by: '<S310>/Constant'
                                        */
  real_T ContLPF1_wc;                  /* Mask Parameter: ContLPF1_wc
                                        * Referenced by: '<S311>/Constant'
                                        */
  real_T VehicleBody6DOF1_xbdot_o[3];/* Mask Parameter: VehicleBody6DOF1_xbdot_o
                                      * Referenced by: '<S218>/ub,vb,wb'
                                      */
  real_T VehicleBody6DOF1_xdot_tol; /* Mask Parameter: VehicleBody6DOF1_xdot_tol
                                     * Referenced by:
                                     *   '<S306>/Constant'
                                     *   '<S307>/Constant'
                                     *   '<S279>/Constant'
                                     *   '<S280>/Constant'
                                     */
  real_T VehicleBody6DOF1_z1I[9];      /* Mask Parameter: VehicleBody6DOF1_z1I
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z1R[3];      /* Mask Parameter: VehicleBody6DOF1_z1R
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z1m;         /* Mask Parameter: VehicleBody6DOF1_z1m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z2I[9];      /* Mask Parameter: VehicleBody6DOF1_z2I
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z2R[3];      /* Mask Parameter: VehicleBody6DOF1_z2R
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z2m;         /* Mask Parameter: VehicleBody6DOF1_z2m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z3I[9];      /* Mask Parameter: VehicleBody6DOF1_z3I
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z3R[3];      /* Mask Parameter: VehicleBody6DOF1_z3R
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z3m;         /* Mask Parameter: VehicleBody6DOF1_z3m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z4I[9];      /* Mask Parameter: VehicleBody6DOF1_z4I
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z4R[3];      /* Mask Parameter: VehicleBody6DOF1_z4R
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z4m;         /* Mask Parameter: VehicleBody6DOF1_z4m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z5I[9];      /* Mask Parameter: VehicleBody6DOF1_z5I
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z5R[3];      /* Mask Parameter: VehicleBody6DOF1_z5R
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z5m;         /* Mask Parameter: VehicleBody6DOF1_z5m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z6I[9];      /* Mask Parameter: VehicleBody6DOF1_z6I
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z6R[3];      /* Mask Parameter: VehicleBody6DOF1_z6R
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z6m;         /* Mask Parameter: VehicleBody6DOF1_z6m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z7I[9];      /* Mask Parameter: VehicleBody6DOF1_z7I
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z7R[3];      /* Mask Parameter: VehicleBody6DOF1_z7R
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z7m;         /* Mask Parameter: VehicleBody6DOF1_z7m
                                        * Referenced by: '<S217>/vehdyncginert'
                                        */
  real_T CombinedSlipWheel2DOF1_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_zdoto
                                  * Referenced by: '<S345>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF2_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_zdoto
                                  * Referenced by: '<S370>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF3_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_zdoto
                                  * Referenced by: '<S395>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF4_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_zdoto
                                  * Referenced by: '<S420>/Integrator, Second-Order'
                                  */
  real_T VerticalWheelandUnsprungMassResponse_zdoto;
                   /* Mask Parameter: VerticalWheelandUnsprungMassResponse_zdoto
                    * Referenced by: '<S334>/Integrator, Second-Order'
                    */
  real_T VerticalWheelandUnsprungMassResponse1_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_zdoto
                   * Referenced by: '<S335>/Integrator, Second-Order'
                   */
  real_T VerticalWheelandUnsprungMassResponse2_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_zdoto
                   * Referenced by: '<S336>/Integrator, Second-Order'
                   */
  real_T VerticalWheelandUnsprungMassResponse3_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_zdoto
                   * Referenced by: '<S337>/Integrator, Second-Order'
                   */
  real_T CombinedSlipWheel2DOF1_zo; /* Mask Parameter: CombinedSlipWheel2DOF1_zo
                                     * Referenced by: '<S345>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF2_zo; /* Mask Parameter: CombinedSlipWheel2DOF2_zo
                                     * Referenced by: '<S370>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF3_zo; /* Mask Parameter: CombinedSlipWheel2DOF3_zo
                                     * Referenced by: '<S395>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF4_zo; /* Mask Parameter: CombinedSlipWheel2DOF4_zo
                                     * Referenced by: '<S420>/Integrator, Second-Order'
                                     */
  real_T VerticalWheelandUnsprungMassResponse_zo;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse_zo
                       * Referenced by: '<S334>/Integrator, Second-Order'
                       */
  real_T VerticalWheelandUnsprungMassResponse1_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_zo
                      * Referenced by: '<S335>/Integrator, Second-Order'
                      */
  real_T VerticalWheelandUnsprungMassResponse2_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_zo
                      * Referenced by: '<S336>/Integrator, Second-Order'
                      */
  real_T VerticalWheelandUnsprungMassResponse3_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_zo
                      * Referenced by: '<S337>/Integrator, Second-Order'
                      */
  uint8_T WrapToZero_Threshold;        /* Mask Parameter: WrapToZero_Threshold
                                        * Referenced by: '<S119>/FixPt Switch'
                                        */
  real_T Interpolatedzerocrossing_tableData[2];/* Expression: [-1 1]
                                                * Referenced by: '<S43>/Interpolated zero-crossing'
                                                */
  real_T Interpolatedzerocrossing_bp01Data[2];/* Expression: [-1 1]
                                               * Referenced by: '<S43>/Interpolated zero-crossing'
                                               */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S63>/Constant'
                                        */
  real_T Constant_Value_d;             /* Expression: 0
                                        * Referenced by: '<S64>/Constant'
                                        */
  real_T phithetapsi_WrappedStateUpperValue;/* Expression: pi
                                             * Referenced by: '<S230>/phi theta psi'
                                             */
  real_T phithetapsi_WrappedStateLowerValue;/* Expression: -pi
                                             * Referenced by: '<S230>/phi theta psi'
                                             */
  real_T Throttle_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/Throttle'
                                        */
  real_T Brake_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/Brake'
                                        */
  real_T Steer_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/Steer'
                                        */
  real_T DriveOption_Value;            /* Expression: 0
                                        * Referenced by: '<Root>/DriveOption'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S9>/Integrator'
                                        */
  real_T Constant_Value_b;             /* Expression: 0
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S1>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 1
                                        * Referenced by: '<S1>/Constant2'
                                        */
  real_T Constant3_Value;              /* Expression: 0
                                        * Referenced by: '<S1>/Constant3'
                                        */
  real_T Constant4_Value;              /* Expression: 1
                                        * Referenced by: '<S1>/Constant4'
                                        */
  real_T Constant6_Value;              /* Expression: 0
                                        * Referenced by: '<S1>/Constant6'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0
                                        * Referenced by: '<S1>/Switch2'
                                        */
  real_T EVBoltSteer_tableData[183];
  /* Expression: [-9.10000000000000	-9	-8.90000000000000	-8.80000000000000	-8.70000000000000	-8.60000000000000	-8.50000000000000	-8.40000000000000	-8.30000000000000	-8.20000000000000	-8.10000000000000	-8	-7.90000000000000	-7.80000000000000	-7.70000000000000	-7.60000000000000	-7.50000000000000	-7.40000000000000	-7.30000000000000	-7.20000000000000	-7.10000000000000	-7	-6.90000000000000	-6.80000000000000	-6.70000000000000	-6.60000000000000	-6.50000000000000	-6.40000000000000	-6.30000000000000	-6.20000000000000	-6.10000000000000	-6	-5.90000000000000	-5.80000000000000	-5.70000000000000	-5.60000000000000	-5.50000000000000	-5.40000000000000	-5.30000000000000	-5.20000000000000	-5.10000000000000	-5	-4.90000000000000	-4.80000000000000	-4.70000000000000	-4.60000000000000	-4.50000000000000	-4.40000000000000	-4.30000000000000	-4.20000000000000	-4.10000000000000	-4	-3.90000000000000	-3.80000000000000	-3.70000000000000	-3.60000000000000	-3.50000000000000	-3.40000000000000	-3.30000000000000	-3.20000000000000	-3.10000000000000	-3	-2.90000000000000	-2.80000000000000	-2.70000000000000	-2.60000000000000	-2.50000000000000	-2.40000000000000	-2.30000000000000	-2.20000000000000	-2.10000000000000	-2	-1.90000000000000	-1.80000000000000	-1.70000000000000	-1.60000000000000	-1.50000000000000	-1.40000000000000	-1.30000000000000	-1.20000000000000	-1.10000000000000	-1	-0.900000000000000	-0.800000000000000	-0.700000000000000	-0.600000000000000	-0.500000000000000	-0.400000000000000	-0.300000000000000	-0.200000000000000	-0.100000000000000	0	0.100000000000000	0.200000000000000	0.300000000000000	0.400000000000000	0.500000000000000	0.600000000000000	0.700000000000000	0.800000000000000	0.900000000000000	1	1.10000000000000	1.20000000000000	1.30000000000000	1.40000000000000	1.50000000000000	1.60000000000000	1.70000000000000	1.80000000000000	1.90000000000000	2	2.10000000000000	2.20000000000000	2.30000000000000	2.40000000000000	2.50000000000000	2.60000000000000	2.70000000000000	2.80000000000000	2.90000000000000	3	3.10000000000000	3.20000000000000	3.30000000000000	3.40000000000000	3.50000000000000	3.60000000000000	3.70000000000000	3.80000000000000	3.90000000000000	4	4.10000000000000	4.20000000000000	4.30000000000000	4.40000000000000	4.50000000000000	4.60000000000000	4.70000000000000	4.80000000000000	4.90000000000000	5	5.10000000000000	5.20000000000000	5.30000000000000	5.40000000000000	5.50000000000000	5.60000000000000	5.70000000000000	5.80000000000000	5.90000000000000	6	6.10000000000000	6.20000000000000	6.30000000000000	6.40000000000000	6.50000000000000	6.60000000000000	6.70000000000000	6.80000000000000	6.90000000000000	7	7.10000000000000	7.20000000000000	7.30000000000000	7.40000000000000	7.50000000000000	7.60000000000000	7.70000000000000	7.80000000000000	7.90000000000000	8	8.10000000000000	8.20000000000000	8.30000000000000	8.40000000000000	8.50000000000000	8.60000000000000	8.70000000000000	8.80000000000000	8.90000000000000	9	9.10000000000000]
   * Referenced by: '<S1>/EV Bolt Steer'
   */
  real_T EVBoltSteer_bp01Data[183];
  /* Expression: [-0.541571931581492	-0.535619550629112	-0.529667169676731	-0.523714788724350	-0.517762407771969	-0.511810026819588	-0.505857645867207	-0.499905264914826	-0.493952883962445	-0.488000503010064	-0.482048122057683	-0.476095741105302	-0.470143360152921	-0.464190979200541	-0.458238598248160	-0.452286217295779	-0.446333836343398	-0.440381455391017	-0.434429074438636	-0.428476693486255	-0.422524312533874	-0.416571931581493	-0.410619550629112	-0.404667169676731	-0.398714788724350	-0.392762407771969	-0.386810026819588	-0.380857645867207	-0.374905264914826	-0.368952883962445	-0.363000503010064	-0.357048122057683	-0.351095741105302	-0.345143360152921	-0.339190979200541	-0.333238598248160	-0.327286217295779	-0.321333836343398	-0.315381455391017	-0.309429074438636	-0.303476693486255	-0.297524312533874	-0.291571931581493	-0.285619550629112	-0.279667169676731	-0.273714788724350	-0.267762407771969	-0.261810026819588	-0.255857645867207	-0.249905264914826	-0.243952883962445	-0.238000503010065	-0.232048122057684	-0.226095741105303	-0.220143360152922	-0.214190979200541	-0.208238598248160	-0.202286217295779	-0.196333836343398	-0.190381455391017	-0.184429074438636	-0.178476693486255	-0.172524312533874	-0.166571931581493	-0.160619550629112	-0.154667169676731	-0.148714788724350	-0.142762407771969	-0.136810026819588	-0.130857645867207	-0.124905264914826	-0.118952883962445	-0.113000503010064	-0.107048122057683	-0.101095741105303	-0.0951433601529216	-0.0891909792005406	-0.0832385982481596	-0.0772862172957787	-0.0713338363433977	-0.0653814553910168	-0.0594290744386358	-0.0534766934862548	-0.0475243125338739	-0.0415719315814929	-0.0356195506291120	-0.0296671696767310	-0.0237147887243500	-0.0177624077719691	-0.0118100268195881	-0.00585764586720720	0	0.00585764586720720	0.0118100268195881	0.0177624077719691	0.0237147887243500	0.0296671696767310	0.0356195506291120	0.0415719315814929	0.0475243125338739	0.0534766934862548	0.0594290744386358	0.0653814553910168	0.0713338363433977	0.0772862172957787	0.0832385982481596	0.0891909792005406	0.0951433601529216	0.101095741105303	0.107048122057683	0.113000503010064	0.118952883962445	0.124905264914826	0.130857645867207	0.136810026819588	0.142762407771969	0.148714788724350	0.154667169676731	0.160619550629112	0.166571931581493	0.172524312533874	0.178476693486255	0.184429074438636	0.190381455391017	0.196333836343398	0.202286217295779	0.208238598248160	0.214190979200541	0.220143360152922	0.226095741105303	0.232048122057684	0.238000503010065	0.243952883962445	0.249905264914826	0.255857645867207	0.261810026819588	0.267762407771969	0.273714788724350	0.279667169676731	0.285619550629112	0.291571931581493	0.297524312533874	0.303476693486255	0.309429074438636	0.315381455391017	0.321333836343398	0.327286217295779	0.333238598248160	0.339190979200541	0.345143360152921	0.351095741105302	0.357048122057683	0.363000503010064	0.368952883962445	0.374905264914826	0.380857645867207	0.386810026819588	0.392762407771969	0.398714788724350	0.404667169676731	0.410619550629112	0.416571931581493	0.422524312533874	0.428476693486255	0.434429074438636	0.440381455391017	0.446333836343398	0.452286217295779	0.458238598248160	0.464190979200541	0.470143360152921	0.476095741105302	0.482048122057683	0.488000503010064	0.493952883962445	0.499905264914826	0.505857645867207	0.511810026819588	0.517762407771969	0.523714788724350	0.529667169676731	0.535619550629112	0.541571931581492]
   * Referenced by: '<S1>/EV Bolt Steer'
   */
  real_T Friction_Value[4];            /* Expression: ones(4,1).*1
                                        * Referenced by: '<S2>/Friction'
                                        */
  real_T GroundZLevel_Value[4];        /* Expression: zeros(4,1)
                                        * Referenced by: '<S2>/Ground Z Level'
                                        */
  real_T Constant2_Value_m;            /* Expression: 0
                                        * Referenced by: '<S6>/Constant2'
                                        */
  real_T Constant3_Value_p;            /* Expression: 0
                                        * Referenced by: '<S6>/Constant3'
                                        */
  real_T Constant4_Value_e;            /* Expression: 0
                                        * Referenced by: '<S6>/Constant4'
                                        */
  real_T Step_Time;                    /* Expression: 3
                                        * Referenced by: '<S1>/Step'
                                        */
  real_T Step_Y0;                      /* Expression: 0
                                        * Referenced by: '<S1>/Step'
                                        */
  real_T Step_YFinal;                  /* Expression: 0.1
                                        * Referenced by: '<S1>/Step'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S1>/Switch'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<S1>/Switch1'
                                        */
  real_T TransferFcn3_A;               /* Computed Parameter: TransferFcn3_A
                                        * Referenced by: '<S120>/Transfer Fcn3'
                                        */
  real_T TransferFcn3_C;               /* Computed Parameter: TransferFcn3_C
                                        * Referenced by: '<S120>/Transfer Fcn3'
                                        */
  real_T TransferFcn1_A;               /* Computed Parameter: TransferFcn1_A
                                        * Referenced by: '<S120>/Transfer Fcn1'
                                        */
  real_T TransferFcn1_C;               /* Computed Parameter: TransferFcn1_C
                                        * Referenced by: '<S120>/Transfer Fcn1'
                                        */
  real_T TransferFcn2_A;               /* Computed Parameter: TransferFcn2_A
                                        * Referenced by: '<S120>/Transfer Fcn2'
                                        */
  real_T TransferFcn2_C;               /* Computed Parameter: TransferFcn2_C
                                        * Referenced by: '<S120>/Transfer Fcn2'
                                        */
  real_T TransferFcn4_A;               /* Computed Parameter: TransferFcn4_A
                                        * Referenced by: '<S120>/Transfer Fcn4'
                                        */
  real_T TransferFcn4_C;               /* Computed Parameter: TransferFcn4_C
                                        * Referenced by: '<S120>/Transfer Fcn4'
                                        */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S120>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S120>/Transfer Fcn'
                                        */
  real_T TransferFcn9_A;               /* Computed Parameter: TransferFcn9_A
                                        * Referenced by: '<S120>/Transfer Fcn9'
                                        */
  real_T TransferFcn9_C;               /* Computed Parameter: TransferFcn9_C
                                        * Referenced by: '<S120>/Transfer Fcn9'
                                        */
  real_T TransferFcn10_A;              /* Computed Parameter: TransferFcn10_A
                                        * Referenced by: '<S120>/Transfer Fcn10'
                                        */
  real_T TransferFcn10_C;              /* Computed Parameter: TransferFcn10_C
                                        * Referenced by: '<S120>/Transfer Fcn10'
                                        */
  real_T TransferFcn11_A;              /* Computed Parameter: TransferFcn11_A
                                        * Referenced by: '<S120>/Transfer Fcn11'
                                        */
  real_T TransferFcn11_C;              /* Computed Parameter: TransferFcn11_C
                                        * Referenced by: '<S120>/Transfer Fcn11'
                                        */
  real_T Memory_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S30>/Memory'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S30>/Gain4'
                                        */
  real_T Constant_Value_e;             /* Expression: 0
                                        * Referenced by: '<S30>/Constant'
                                        */
  real_T Memory1_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S30>/Memory1'
                                        */
  real_T Integrator_IC_m;              /* Expression: 0
                                        * Referenced by: '<S39>/Integrator'
                                        */
  real_T Merge_InitialOutput;          /* Expression: 0
                                        * Referenced by: '<S39>/Merge'
                                        */
  real_T TransferFcn_A_k;              /* Computed Parameter: TransferFcn_A_k
                                        * Referenced by: '<S4>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_a;              /* Computed Parameter: TransferFcn_C_a
                                        * Referenced by: '<S4>/Transfer Fcn'
                                        */
  real_T TransferFcn1_A_c;             /* Computed Parameter: TransferFcn1_A_c
                                        * Referenced by: '<S4>/Transfer Fcn1'
                                        */
  real_T TransferFcn1_C_a;             /* Computed Parameter: TransferFcn1_C_a
                                        * Referenced by: '<S4>/Transfer Fcn1'
                                        */
  real_T TransferFcn2_A_j;             /* Computed Parameter: TransferFcn2_A_j
                                        * Referenced by: '<S4>/Transfer Fcn2'
                                        */
  real_T TransferFcn2_C_h;             /* Computed Parameter: TransferFcn2_C_h
                                        * Referenced by: '<S4>/Transfer Fcn2'
                                        */
  real_T TransferFcn3_A_j;             /* Computed Parameter: TransferFcn3_A_j
                                        * Referenced by: '<S4>/Transfer Fcn3'
                                        */
  real_T TransferFcn3_C_m;             /* Computed Parameter: TransferFcn3_C_m
                                        * Referenced by: '<S4>/Transfer Fcn3'
                                        */
  real_T Memory_InitialCondition_p;    /* Expression: 0
                                        * Referenced by: '<S12>/Memory'
                                        */
  real_T IntegratorLimited_LowerSat;   /* Expression: 0
                                        * Referenced by: '<S23>/Integrator Limited'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S20>/Gain'
                                        */
  real_T Gain1_Gain;                   /* Expression: -1
                                        * Referenced by: '<S20>/Gain1'
                                        */
  real_T Constant_Value_k;             /* Expression: 0
                                        * Referenced by: '<S4>/Constant'
                                        */
  real_T VehicleSimulationType_Value;  /* Expression: 1
                                        * Referenced by: '<S4>/VehicleSimulationType'
                                        */
  real_T Gain4_Gain_e[4];              /* Expression: [1;-1;1;-1]*0+1
                                        * Referenced by: '<S315>/Gain4'
                                        */
  real_T Integrator_IC_f;              /* Expression: 0
                                        * Referenced by: '<S350>/Integrator'
                                        */
  real_T Integrator_IC_a;              /* Expression: 0
                                        * Referenced by: '<S347>/Integrator'
                                        */
  real_T Saturation_UpperSat;          /* Expression: inf
                                        * Referenced by: '<S345>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S345>/Saturation'
                                        */
  real_T Saturation_UpperSat_a;        /* Expression: 1
                                        * Referenced by: '<S54>/Saturation'
                                        */
  real_T Saturation_LowerSat_h;        /* Expression: 0
                                        * Referenced by: '<S54>/Saturation'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: inf
                                        * Referenced by: '<S70>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: 1
                                        * Referenced by: '<S70>/Saturation1'
                                        */
  real_T Saturation_LowerSat_hi;       /* Expression: 0
                                        * Referenced by: '<S70>/Saturation'
                                        */
  real_T Saturation1_UpperSat_a;       /* Expression: 1
                                        * Referenced by: '<S54>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_l;       /* Expression: 0
                                        * Referenced by: '<S54>/Saturation1'
                                        */
  real_T Gain2_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S121>/Gain2'
                                        */
  real_T Gain3_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S121>/Gain3'
                                        */
  real_T Saturation_LowerSat_j;        /* Expression: 0
                                        * Referenced by: '<S121>/Saturation'
                                        */
  real_T Saturation1_LowerSat_i;       /* Expression: 0
                                        * Referenced by: '<S121>/Saturation1'
                                        */
  real_T Gain1_Gain_e;                 /* Expression: 0.5
                                        * Referenced by: '<S121>/Gain1'
                                        */
  real_T Gain4_Gain_h;                 /* Expression: 0.5
                                        * Referenced by: '<S121>/Gain4'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S121>/Saturation2'
                                        */
  real_T Saturation3_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S121>/Saturation3'
                                        */
  real_T TorqueConversion1_Gain;       /* Expression: pi/4
                                        * Referenced by: '<S355>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat;/* Expression: inf
                                               * Referenced by: '<S355>/Disallow Negative Brake Torque'
                                               */
  real_T DisallowNegativeBrakeTorque_LowerSat;/* Expression: eps
                                               * Referenced by: '<S355>/Disallow Negative Brake Torque'
                                               */
  real_T Integrator_IC_fs;             /* Expression: 0
                                        * Referenced by: '<S375>/Integrator'
                                        */
  real_T Integrator_IC_k;              /* Expression: 0
                                        * Referenced by: '<S372>/Integrator'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: inf
                                        * Referenced by: '<S370>/Saturation'
                                        */
  real_T Saturation_LowerSat_p;        /* Expression: 0
                                        * Referenced by: '<S370>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_p;     /* Expression: pi/4
                                        * Referenced by: '<S380>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_k;/* Expression: inf
                                                 * Referenced by: '<S380>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_f;/* Expression: eps
                                                 * Referenced by: '<S380>/Disallow Negative Brake Torque'
                                                 */
  real_T Integrator_IC_e;              /* Expression: 0
                                        * Referenced by: '<S400>/Integrator'
                                        */
  real_T Integrator_IC_mh;             /* Expression: 0
                                        * Referenced by: '<S397>/Integrator'
                                        */
  real_T Saturation_UpperSat_b;        /* Expression: inf
                                        * Referenced by: '<S395>/Saturation'
                                        */
  real_T Saturation_LowerSat_e;        /* Expression: 0
                                        * Referenced by: '<S395>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_b;     /* Expression: pi/4
                                        * Referenced by: '<S405>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_i;/* Expression: inf
                                                 * Referenced by: '<S405>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_i;/* Expression: eps
                                                 * Referenced by: '<S405>/Disallow Negative Brake Torque'
                                                 */
  real_T Integrator_IC_d;              /* Expression: 0
                                        * Referenced by: '<S425>/Integrator'
                                        */
  real_T Integrator_IC_eh;             /* Expression: 0
                                        * Referenced by: '<S422>/Integrator'
                                        */
  real_T Saturation_UpperSat_d;        /* Expression: inf
                                        * Referenced by: '<S420>/Saturation'
                                        */
  real_T Saturation_LowerSat_o;        /* Expression: 0
                                        * Referenced by: '<S420>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_n;     /* Expression: pi/4
                                        * Referenced by: '<S430>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_c;/* Expression: inf
                                                 * Referenced by: '<S430>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_k;/* Expression: eps
                                                 * Referenced by: '<S430>/Disallow Negative Brake Torque'
                                                 */
  real_T Gain1_Gain_p;                 /* Expression: 0.5
                                        * Referenced by: '<S30>/Gain1'
                                        */
  real_T Constant_Value_dv;            /* Expression: 0
                                        * Referenced by: '<S31>/Constant'
                                        */
  real_T Constant1_Value_i;            /* Expression: 0
                                        * Referenced by: '<S31>/Constant1'
                                        */
  real_T Constant2_Value_o;            /* Expression: 0
                                        * Referenced by: '<S31>/Constant2'
                                        */
  real_T Gain1_Gain_f;                 /* Expression: 3.6
                                        * Referenced by: '<S4>/Gain1'
                                        */
  real_T Constant_Value_l;             /* Expression: 0
                                        * Referenced by: '<S37>/Constant'
                                        */
  real_T uDLookupTable_tableData[667]; /* Expression: x_losses_mat
                                        * Referenced by: '<S41>/2-D Lookup Table'
                                        */
  real_T uDLookupTable_bp01Data[23];   /* Expression: x_w_eff_vec
                                        * Referenced by: '<S41>/2-D Lookup Table'
                                        */
  real_T uDLookupTable_bp02Data[29];   /* Expression: x_T_eff_vec
                                        * Referenced by: '<S41>/2-D Lookup Table'
                                        */
  real_T Saturation_UpperSat_h;        /* Expression: Inf
                                        * Referenced by: '<S38>/Saturation'
                                        */
  real_T Saturation_LowerSat_oa;       /* Expression: 0.0001
                                        * Referenced by: '<S38>/Saturation'
                                        */
  real_T Saturation1_UpperSat_l;       /* Expression: inf
                                        * Referenced by: '<S66>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_j;       /* Expression: 1
                                        * Referenced by: '<S66>/Saturation1'
                                        */
  real_T Saturation_LowerSat_ej;       /* Expression: 0
                                        * Referenced by: '<S66>/Saturation'
                                        */
  real_T Saturation1_UpperSat_ak;      /* Expression: inf
                                        * Referenced by: '<S55>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_c;       /* Expression: 1
                                        * Referenced by: '<S55>/Saturation1'
                                        */
  real_T Saturation_LowerSat_p0;       /* Expression: 0
                                        * Referenced by: '<S55>/Saturation'
                                        */
  real_T Gain_Gain_e;                  /* Expression: -1
                                        * Referenced by: '<S54>/Gain'
                                        */
  real_T rads_to_rpm_Gain;             /* Expression: 30/pi
                                        * Referenced by: '<S60>/rads_to_rpm'
                                        */
  real_T Gain1_Gain_o;                 /* Expression: 1/100
                                        * Referenced by: '<S60>/Gain1'
                                        */
  real_T Constant1_Value_g;            /* Expression: -1
                                        * Referenced by: '<S60>/Constant1'
                                        */
  real_T Constant2_Value_k;            /* Expression: 1
                                        * Referenced by: '<S60>/Constant2'
                                        */
  real_T Switch2_Threshold_o;          /* Expression: 0
                                        * Referenced by: '<S60>/Switch2'
                                        */
  real_T Constant_Value_j;             /* Expression: 1
                                        * Referenced by: '<S62>/Constant'
                                        */
  real_T Switch1_Threshold_k;          /* Expression: 0
                                        * Referenced by: '<S62>/Switch1'
                                        */
  real_T Gain_Gain_k;                  /* Expression: -1
                                        * Referenced by: '<S61>/Gain'
                                        */
  real_T Gain_Gain_o;                  /* Expression: -1
                                        * Referenced by: '<S40>/Gain'
                                        */
  real_T Gain1_Gain_g;                 /* Expression: 1
                                        * Referenced by: '<S40>/Gain1'
                                        */
  real_T CmdSpeedmph_Value;            /* Expression: 0
                                        * Referenced by: '<S14>/CmdSpeed (mph)'
                                        */
  real_T mph2ms_Gain;                  /* Expression: 0.44704
                                        * Referenced by: '<S14>/mph2m//s'
                                        */
  real_T utireRadius_Gain;             /* Expression: 1/0.3232
                                        * Referenced by: '<S14>/1//tireRadius'
                                        */
  real_T ChargingMode_Value;           /* Expression: 0
                                        * Referenced by: '<S14>/ChargingMode'
                                        */
  real_T Constant_Value_i;             /* Expression: 3000
                                        * Referenced by: '<S14>/Constant'
                                        */
  real_T DriverSwitch_Value;           /* Expression: 0
                                        * Referenced by: '<S14>/DriverSwitch'
                                        */
  real_T APP_pct_Value;                /* Expression: 0
                                        * Referenced by: '<S71>/APP_pct'
                                        */
  real_T APPToTorque_tableData[2];     /* Expression: [0 184]
                                        * Referenced by: '<S71>/APPToTorque'
                                        */
  real_T APPToTorque_bp01Data[2];      /* Expression: [0 100]
                                        * Referenced by: '<S71>/APPToTorque'
                                        */
  real_T BPP_pct_Value;                /* Expression: 0
                                        * Referenced by: '<S71>/BPP_pct'
                                        */
  real_T APPToTorque1_tableData[2];    /* Expression: [0 184]
                                        * Referenced by: '<S71>/APPToTorque1'
                                        */
  real_T APPToTorque1_bp01Data[2];     /* Expression: [0 100]
                                        * Referenced by: '<S71>/APPToTorque1'
                                        */
  real_T Flip_Gain;                    /* Expression: -1
                                        * Referenced by: '<S71>/Flip'
                                        */
  real_T omegawheel_IC;                /* Expression: 0
                                        * Referenced by: '<S80>/omega wheel'
                                        */
  real_T omegawheel_IC_g;              /* Expression: 0
                                        * Referenced by: '<S81>/omega wheel'
                                        */
  real_T omegawheel_IC_gv;             /* Expression: 0
                                        * Referenced by: '<S82>/omega wheel'
                                        */
  real_T omegawheel_IC_m;              /* Expression: 0
                                        * Referenced by: '<S83>/omega wheel'
                                        */
  real_T Saturation_UpperSat_j;        /* Expression: 100
                                        * Referenced by: '<S73>/Saturation'
                                        */
  real_T Saturation_LowerSat_j4;       /* Expression: -1
                                        * Referenced by: '<S73>/Saturation'
                                        */
  real_T WheelRadius_Gain;             /* Expression: 0.3235
                                        * Referenced by: '<S71>/WheelRadius'
                                        */
  real_T mps2kph_Gain;                 /* Expression: 3.6
                                        * Referenced by: '<S71>/mps2kph'
                                        */
  real_T kph2mph_Gain;                 /* Expression: 0.621
                                        * Referenced by: '<S71>/kph2mph'
                                        */
  real_T NissanTrans_tableData[8];
                        /* Expression: [5.25 3.03 1.95 1.46 1.22 1.00 0.81 0.67]
                         * Referenced by: '<S71>/NissanTrans'
                         */
  real_T NissanTrans_bp01Data[8];      /* Expression: [0 5 10 15 25 34 45 55]
                                        * Referenced by: '<S71>/NissanTrans'
                                        */
  real_T FDR_Gain;                     /* Expression: 3.18
                                        * Referenced by: '<S71>/FDR'
                                        */
  real_T Memory_InitialCondition_o;    /* Expression: 0
                                        * Referenced by: '<S71>/Memory'
                                        */
  real_T DiscreteTimeIntegrator4_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator4_gainval
                           * Referenced by: '<S72>/Discrete-Time Integrator4'
                           */
  real_T DiscreteTimeIntegrator4_IC;   /* Expression: 0
                                        * Referenced by: '<S72>/Discrete-Time Integrator4'
                                        */
  real_T DiscreteTimeIntegrator5_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator5_gainval
                           * Referenced by: '<S72>/Discrete-Time Integrator5'
                           */
  real_T DiscreteTimeIntegrator5_IC;   /* Expression: 0
                                        * Referenced by: '<S72>/Discrete-Time Integrator5'
                                        */
  real_T DiscreteTimeIntegrator6_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator6_gainval
                           * Referenced by: '<S72>/Discrete-Time Integrator6'
                           */
  real_T DiscreteTimeIntegrator6_IC;   /* Expression: 0
                                        * Referenced by: '<S72>/Discrete-Time Integrator6'
                                        */
  real_T DiscreteTimeIntegrator7_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator7_gainval
                           * Referenced by: '<S72>/Discrete-Time Integrator7'
                           */
  real_T DiscreteTimeIntegrator7_IC;   /* Expression: 0
                                        * Referenced by: '<S72>/Discrete-Time Integrator7'
                                        */
  real_T Constant1_Value_m;            /* Expression: 0
                                        * Referenced by: '<S75>/Constant1'
                                        */
  real_T Memory3_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S72>/Memory3'
                                        */
  real_T Saturation3_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S72>/Saturation3'
                                        */
  real_T Saturation3_LowerSat_c;       /* Expression: -1
                                        * Referenced by: '<S72>/Saturation3'
                                        */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<S72>/Gain5'
                                        */
  real_T Memory1_InitialCondition_f;   /* Expression: 0
                                        * Referenced by: '<S72>/Memory1'
                                        */
  real_T Saturation1_UpperSat_f;       /* Expression: 1
                                        * Referenced by: '<S72>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_jv;      /* Expression: -1
                                        * Referenced by: '<S72>/Saturation1'
                                        */
  real_T Gain6_Gain;                   /* Expression: -1
                                        * Referenced by: '<S72>/Gain6'
                                        */
  real_T Constant_Value_m;             /* Expression: 0
                                        * Referenced by: '<S75>/Constant'
                                        */
  real_T Memory2_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S72>/Memory2'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S72>/Saturation2'
                                        */
  real_T Saturation2_LowerSat_h;       /* Expression: -1
                                        * Referenced by: '<S72>/Saturation2'
                                        */
  real_T Gain7_Gain;                   /* Expression: -1
                                        * Referenced by: '<S72>/Gain7'
                                        */
  real_T Memory4_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S72>/Memory4'
                                        */
  real_T Saturation4_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S72>/Saturation4'
                                        */
  real_T Saturation4_LowerSat;         /* Expression: -1
                                        * Referenced by: '<S72>/Saturation4'
                                        */
  real_T Gain8_Gain;                   /* Expression: -1
                                        * Referenced by: '<S72>/Gain8'
                                        */
  real_T Ignition_Value;               /* Expression: 1
                                        * Referenced by: '<S72>/Ignition'
                                        */
  real_T OperationError_Value;         /* Expression: 4
                                        * Referenced by: '<S72>/Operation Error'
                                        */
  real_T OperationStateDriving_Value;  /* Expression: 4
                                        * Referenced by: '<S72>/Operation State Driving'
                                        */
  real_T Zero1_Value;                  /* Expression: 0
                                        * Referenced by: '<S72>/Zero1'
                                        */
  real_T Zero2_Value;                  /* Expression: 0
                                        * Referenced by: '<S72>/Zero2'
                                        */
  real_T Zero3_Value;                  /* Expression: 0
                                        * Referenced by: '<S72>/Zero3'
                                        */
  real_T Zero4_Value;                  /* Expression: 0
                                        * Referenced by: '<S72>/Zero4'
                                        */
  real_T Zero5_Value;                  /* Expression: 0
                                        * Referenced by: '<S72>/Zero5'
                                        */
  real_T DiscreteTimeIntegrator4_gainval_a;
                        /* Computed Parameter: DiscreteTimeIntegrator4_gainval_a
                         * Referenced by: '<S78>/Discrete-Time Integrator4'
                         */
  real_T DiscreteTimeIntegrator4_IC_o; /* Expression: 0
                                        * Referenced by: '<S78>/Discrete-Time Integrator4'
                                        */
  real_T DiscreteTimeIntegrator5_gainval_e;
                        /* Computed Parameter: DiscreteTimeIntegrator5_gainval_e
                         * Referenced by: '<S78>/Discrete-Time Integrator5'
                         */
  real_T DiscreteTimeIntegrator5_IC_j; /* Expression: 0
                                        * Referenced by: '<S78>/Discrete-Time Integrator5'
                                        */
  real_T DiscreteTimeIntegrator6_gainval_k;
                        /* Computed Parameter: DiscreteTimeIntegrator6_gainval_k
                         * Referenced by: '<S78>/Discrete-Time Integrator6'
                         */
  real_T DiscreteTimeIntegrator6_IC_f; /* Expression: 0
                                        * Referenced by: '<S78>/Discrete-Time Integrator6'
                                        */
  real_T DiscreteTimeIntegrator7_gainval_g;
                        /* Computed Parameter: DiscreteTimeIntegrator7_gainval_g
                         * Referenced by: '<S78>/Discrete-Time Integrator7'
                         */
  real_T DiscreteTimeIntegrator7_IC_o; /* Expression: 0
                                        * Referenced by: '<S78>/Discrete-Time Integrator7'
                                        */
  real_T Memory3_InitialCondition_j;   /* Expression: 0
                                        * Referenced by: '<S78>/Memory3'
                                        */
  real_T Saturation3_UpperSat_n;       /* Expression: 1
                                        * Referenced by: '<S78>/Saturation3'
                                        */
  real_T Saturation3_LowerSat_o;       /* Expression: -1
                                        * Referenced by: '<S78>/Saturation3'
                                        */
  real_T Gain5_Gain_a;                 /* Expression: -1
                                        * Referenced by: '<S78>/Gain5'
                                        */
  real_T Memory1_InitialCondition_a;   /* Expression: 0
                                        * Referenced by: '<S78>/Memory1'
                                        */
  real_T Saturation1_UpperSat_n;       /* Expression: 1
                                        * Referenced by: '<S78>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_lc;      /* Expression: -1
                                        * Referenced by: '<S78>/Saturation1'
                                        */
  real_T Gain6_Gain_p;                 /* Expression: -1
                                        * Referenced by: '<S78>/Gain6'
                                        */
  real_T Memory2_InitialCondition_k;   /* Expression: 0
                                        * Referenced by: '<S78>/Memory2'
                                        */
  real_T Saturation2_UpperSat_d;       /* Expression: 1
                                        * Referenced by: '<S78>/Saturation2'
                                        */
  real_T Saturation2_LowerSat_l;       /* Expression: -1
                                        * Referenced by: '<S78>/Saturation2'
                                        */
  real_T Gain7_Gain_f;                 /* Expression: -1
                                        * Referenced by: '<S78>/Gain7'
                                        */
  real_T Memory4_InitialCondition_h;   /* Expression: 0
                                        * Referenced by: '<S78>/Memory4'
                                        */
  real_T Saturation4_UpperSat_e;       /* Expression: 1
                                        * Referenced by: '<S78>/Saturation4'
                                        */
  real_T Saturation4_LowerSat_n;       /* Expression: -1
                                        * Referenced by: '<S78>/Saturation4'
                                        */
  real_T Gain8_Gain_i;                 /* Expression: -1
                                        * Referenced by: '<S78>/Gain8'
                                        */
  real_T Ignition_Value_l;             /* Expression: 1
                                        * Referenced by: '<S78>/Ignition'
                                        */
  real_T OperationError_Value_o;       /* Expression: 4
                                        * Referenced by: '<S78>/Operation Error'
                                        */
  real_T OperationStateDriving_Value_m;/* Expression: 4
                                        * Referenced by: '<S78>/Operation State Driving'
                                        */
  real_T dt_Value;                     /* Expression: 0.001
                                        * Referenced by: '<S98>/dt'
                                        */
  real_T Tau_Value;                    /* Expression: 0.01
                                        * Referenced by: '<S74>/Tau'
                                        */
  real_T one_Value;                    /* Expression: 1
                                        * Referenced by: '<S98>/one'
                                        */
  real_T Memory_InitialCondition_b;    /* Expression: 0
                                        * Referenced by: '<S98>/Memory'
                                        */
  real_T dt_Value_l;                   /* Expression: 0.001
                                        * Referenced by: '<S99>/dt'
                                        */
  real_T one_Value_o;                  /* Expression: 1
                                        * Referenced by: '<S99>/one'
                                        */
  real_T Memory_InitialCondition_pw;   /* Expression: 0
                                        * Referenced by: '<S99>/Memory'
                                        */
  real_T dt_Value_l0;                  /* Expression: 0.001
                                        * Referenced by: '<S100>/dt'
                                        */
  real_T one_Value_l;                  /* Expression: 1
                                        * Referenced by: '<S100>/one'
                                        */
  real_T Memory_InitialCondition_l;    /* Expression: 0
                                        * Referenced by: '<S100>/Memory'
                                        */
  real_T dt_Value_n;                   /* Expression: 0.001
                                        * Referenced by: '<S101>/dt'
                                        */
  real_T one_Value_p;                  /* Expression: 1
                                        * Referenced by: '<S101>/one'
                                        */
  real_T Memory_InitialCondition_bd;   /* Expression: 0
                                        * Referenced by: '<S101>/Memory'
                                        */
  real_T MasterSw_Value;               /* Expression: 1
                                        * Referenced by: '<S76>/MasterSw'
                                        */
  real_T ManualTrq_Nm_Value;           /* Expression: 0
                                        * Referenced by: '<S74>/ManualTrq_Nm'
                                        */
  real_T TorqueIn_Threshold;           /* Expression: 0
                                        * Referenced by: '<S74>/TorqueIn'
                                        */
  real_T SatTrq_UpperSat;              /* Expression: 1000
                                        * Referenced by: '<S74>/SatTrq'
                                        */
  real_T SatTrq_LowerSat;              /* Expression: -1000
                                        * Referenced by: '<S74>/SatTrq'
                                        */
  real_T Saturation_UpperSat_n;        /* Expression: inf
                                        * Referenced by: '<S334>/Saturation'
                                        */
  real_T Saturation_LowerSat_i;        /* Expression: 0
                                        * Referenced by: '<S334>/Saturation'
                                        */
  real_T Constant9_Value;              /* Expression: 0.3135
                                        * Referenced by: '<S80>/Constant9'
                                        */
  real_T Saturation_UpperSat_l;        /* Expression: inf
                                        * Referenced by: '<S80>/Saturation'
                                        */
  real_T Saturation_LowerSat_p0s;      /* Expression: 0
                                        * Referenced by: '<S80>/Saturation'
                                        */
  real_T Constant_Value_lx;            /* Expression: 0
                                        * Referenced by: '<S80>/Constant'
                                        */
  real_T Memory_InitialCondition_a;    /* Expression: 0
                                        * Referenced by: '<S80>/Memory'
                                        */
  real_T Zero1_Value_c;                /* Expression: 0.82
                                        * Referenced by: '<S86>/Zero1'
                                        */
  real_T Zero_Value;                   /* Expression: 1
                                        * Referenced by: '<S86>/Zero'
                                        */
  real_T OutputDamping_Gain;           /* Expression: 0
                                        * Referenced by: '<S80>/Output Damping'
                                        */
  real_T Switch_Threshold_b;           /* Expression: 0
                                        * Referenced by: '<S80>/Switch'
                                        */
  real_T Zero1_Value_h;                /* Expression: 0.74063
                                        * Referenced by: '<S85>/Zero1'
                                        */
  real_T Zero_Value_e;                 /* Expression: 1
                                        * Referenced by: '<S85>/Zero'
                                        */
  real_T Saturation_UpperSat_i;        /* Expression: inf
                                        * Referenced by: '<S335>/Saturation'
                                        */
  real_T Saturation_LowerSat_d;        /* Expression: 0
                                        * Referenced by: '<S335>/Saturation'
                                        */
  real_T Constant9_Value_i;            /* Expression: 0.3135
                                        * Referenced by: '<S81>/Constant9'
                                        */
  real_T Saturation_UpperSat_e2;       /* Expression: inf
                                        * Referenced by: '<S81>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: 0
                                        * Referenced by: '<S81>/Saturation'
                                        */
  real_T Constant_Value_bj;            /* Expression: 0
                                        * Referenced by: '<S81>/Constant'
                                        */
  real_T Memory_InitialCondition_j;    /* Expression: 0
                                        * Referenced by: '<S81>/Memory'
                                        */
  real_T Zero1_Value_g;                /* Expression: 0.82
                                        * Referenced by: '<S89>/Zero1'
                                        */
  real_T Zero_Value_p;                 /* Expression: 1
                                        * Referenced by: '<S89>/Zero'
                                        */
  real_T OutputDamping_Gain_o;         /* Expression: -1
                                        * Referenced by: '<S81>/Output Damping'
                                        */
  real_T Switch_Threshold_i;           /* Expression: 0
                                        * Referenced by: '<S81>/Switch'
                                        */
  real_T Zero1_Value_j;                /* Expression: 0.74063
                                        * Referenced by: '<S88>/Zero1'
                                        */
  real_T Zero_Value_i;                 /* Expression: 1
                                        * Referenced by: '<S88>/Zero'
                                        */
  real_T Saturation_UpperSat_ew;       /* Expression: inf
                                        * Referenced by: '<S336>/Saturation'
                                        */
  real_T Saturation_LowerSat_ejw;      /* Expression: 0
                                        * Referenced by: '<S336>/Saturation'
                                        */
  real_T Constant9_Value_d;            /* Expression: 0.3135
                                        * Referenced by: '<S82>/Constant9'
                                        */
  real_T Saturation_UpperSat_bk;       /* Expression: inf
                                        * Referenced by: '<S82>/Saturation'
                                        */
  real_T Saturation_LowerSat_du;       /* Expression: 0
                                        * Referenced by: '<S82>/Saturation'
                                        */
  real_T Constant_Value_i5;            /* Expression: 0
                                        * Referenced by: '<S82>/Constant'
                                        */
  real_T Memory_InitialCondition_n;    /* Expression: 0
                                        * Referenced by: '<S82>/Memory'
                                        */
  real_T OutputDamping_Gain_m;         /* Expression: 0
                                        * Referenced by: '<S82>/Output Damping'
                                        */
  real_T Zero1_Value_g5;               /* Expression: 0.82
                                        * Referenced by: '<S92>/Zero1'
                                        */
  real_T Zero_Value_f;                 /* Expression: 1
                                        * Referenced by: '<S92>/Zero'
                                        */
  real_T Switch_Threshold_c;           /* Expression: 0
                                        * Referenced by: '<S82>/Switch'
                                        */
  real_T Zero1_Value_ja;               /* Expression: 0.74063
                                        * Referenced by: '<S91>/Zero1'
                                        */
  real_T Zero_Value_j;                 /* Expression: 1
                                        * Referenced by: '<S91>/Zero'
                                        */
  real_T Saturation_UpperSat_ik;       /* Expression: inf
                                        * Referenced by: '<S337>/Saturation'
                                        */
  real_T Saturation_LowerSat_a;        /* Expression: 0
                                        * Referenced by: '<S337>/Saturation'
                                        */
  real_T Constant9_Value_j;            /* Expression: 0.3135
                                        * Referenced by: '<S83>/Constant9'
                                        */
  real_T Saturation_UpperSat_c;        /* Expression: inf
                                        * Referenced by: '<S83>/Saturation'
                                        */
  real_T Saturation_LowerSat_m;        /* Expression: 0
                                        * Referenced by: '<S83>/Saturation'
                                        */
  real_T Constant_Value_ds;            /* Expression: 0
                                        * Referenced by: '<S83>/Constant'
                                        */
  real_T Zero1_Value_cq;               /* Expression: 0.82
                                        * Referenced by: '<S95>/Zero1'
                                        */
  real_T Zero_Value_jk;                /* Expression: 1
                                        * Referenced by: '<S95>/Zero'
                                        */
  real_T Gain_Gain_j;                  /* Expression: -1
                                        * Referenced by: '<S83>/Gain'
                                        */
  real_T Memory_InitialCondition_e;    /* Expression: 0
                                        * Referenced by: '<S83>/Memory'
                                        */
  real_T Switch_Threshold_bu;          /* Expression: 0
                                        * Referenced by: '<S83>/Switch'
                                        */
  real_T Zero1_Value_n;                /* Expression: 0.74063
                                        * Referenced by: '<S94>/Zero1'
                                        */
  real_T Zero_Value_jw;                /* Expression: 1
                                        * Referenced by: '<S94>/Zero'
                                        */
  real_T Zero1_Value_a;                /* Expression: 0
                                        * Referenced by: '<S78>/Zero1'
                                        */
  real_T Zero2_Value_p;                /* Expression: 0
                                        * Referenced by: '<S78>/Zero2'
                                        */
  real_T Zero3_Value_j;                /* Expression: 0
                                        * Referenced by: '<S78>/Zero3'
                                        */
  real_T Zero4_Value_i;                /* Expression: 0
                                        * Referenced by: '<S78>/Zero4'
                                        */
  real_T Zero5_Value_l;                /* Expression: 0
                                        * Referenced by: '<S78>/Zero5'
                                        */
  real_T Constant_Value_ib;            /* Expression: 500
                                        * Referenced by: '<S78>/Constant'
                                        */
  real_T Constant_Value_eh;            /* Expression: 4.1
                                        * Referenced by: '<S79>/Constant'
                                        */
  real_T Gain10_Gain;                  /* Expression: 0.5
                                        * Referenced by: '<S79>/Gain10'
                                        */
  real_T Saturation1_UpperSat_nu;      /* Expression: 300
                                        * Referenced by: '<S73>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_f;       /* Expression: -300
                                        * Referenced by: '<S73>/Saturation1'
                                        */
  real_T Zero1_Value_jp;               /* Expression: 0
                                        * Referenced by: '<S75>/Zero1'
                                        */
  real_T Zero2_Value_g;                /* Expression: 0
                                        * Referenced by: '<S75>/Zero2'
                                        */
  real_T Zero5_Value_j;                /* Expression: 0
                                        * Referenced by: '<S75>/Zero5'
                                        */
  real_T LaunchSpdThr_rpm_Value;       /* Expression: 2
                                        * Referenced by: '<S102>/LaunchSpdThr_rpm'
                                        */
  real_T LaunchTrqThr_Nm_Value;        /* Expression: 20
                                        * Referenced by: '<S102>/LaunchTrqThr_Nm'
                                        */
  real_T SpdLatchThr_rpm_Value;        /* Expression: 1e-5
                                        * Referenced by: '<S102>/SpdLatchThr_rpm'
                                        */
  real_T rads2rpm_Gain;                /* Expression: 9.5492965964254
                                        * Referenced by: '<S76>/rads2rpm'
                                        */
  real_T SettleTime_s_Value;           /* Expression: 5
                                        * Referenced by: '<S102>/SettleTime_s'
                                        */
  real_T dStop_Value;                  /* Expression: 0
                                        * Referenced by: '<S102>/dStop'
                                        */
  real_T VehSpdThr_kph_Value;          /* Expression: 0.2
                                        * Referenced by: '<S102>/VehSpdThr_kph'
                                        */
  real_T BrkLatchThr_Nm_Value;         /* Expression: -50
                                        * Referenced by: '<S102>/BrkLatchThr_Nm'
                                        */
  real_T ModeOut_Threshold;            /* Expression: 0
                                        * Referenced by: '<S76>/ModeOut'
                                        */
  real_T RefLF_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S105>/RefLF_NmORrpm'
                                        */
  real_T RefRF_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S105>/RefRF_NmORrpm'
                                        */
  real_T RefLR_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S105>/RefLR_NmORrpm'
                                        */
  real_T RefRR_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S105>/RefRR_NmORrpm'
                                        */
  real_T SpeedOut_Threshold;           /* Expression: 0
                                        * Referenced by: '<S76>/SpeedOut'
                                        */
  real_T RateLimSpd_RisingLim;       /* Computed Parameter: RateLimSpd_RisingLim
                                      * Referenced by: '<S76>/RateLimSpd'
                                      */
  real_T RateLimSpd_FallingLim;     /* Computed Parameter: RateLimSpd_FallingLim
                                     * Referenced by: '<S76>/RateLimSpd'
                                     */
  real_T RateLimSpd_IC;                /* Expression: 0
                                        * Referenced by: '<S76>/RateLimSpd'
                                        */
  real_T SatSpd_UpperSat;              /* Expression: 700
                                        * Referenced by: '<S76>/SatSpd'
                                        */
  real_T SatSpd_LowerSat;              /* Expression: 0
                                        * Referenced by: '<S76>/SatSpd'
                                        */
  real_T Bit4_Gain;                    /* Expression: 16
                                        * Referenced by: '<S112>/Bit4'
                                        */
  real_T Bit3_Gain;                    /* Expression: 8
                                        * Referenced by: '<S112>/Bit3'
                                        */
  real_T Bit2_Gain;                    /* Expression: 4
                                        * Referenced by: '<S112>/Bit2'
                                        */
  real_T Bit1_Gain;                    /* Expression: 2
                                        * Referenced by: '<S112>/Bit1'
                                        */
  real_T Constant_Value_jx;            /* Expression: 0
                                        * Referenced by: '<S122>/Constant'
                                        */
  real_T Backlash_InitialOutput;       /* Expression: 0
                                        * Referenced by: '<S130>/Backlash'
                                        */
  real_T Constant_Value_a;             /* Expression: 0
                                        * Referenced by: '<S130>/Constant'
                                        */
  real_T index_Value;                  /* Expression: 1
                                        * Referenced by: '<S129>/index'
                                        */
  real_T Switch_Threshold_d;           /* Expression: 0
                                        * Referenced by: '<S129>/Switch'
                                        */
  real_T Switch1_Threshold_f;          /* Expression: 0
                                        * Referenced by: '<S129>/Switch1'
                                        */
  real_T DeadZone2_Start;              /* Expression: -5
                                        * Referenced by: '<S313>/Dead Zone2'
                                        */
  real_T DeadZone2_End;                /* Expression: 5
                                        * Referenced by: '<S313>/Dead Zone2'
                                        */
  real_T Integrator_IC_o;              /* Expression: 0
                                        * Referenced by: '<S348>/Integrator'
                                        */
  real_T Integrator_IC_d3;             /* Expression: 0
                                        * Referenced by: '<S373>/Integrator'
                                        */
  real_T Integrator_IC_b;              /* Expression: 0
                                        * Referenced by: '<S398>/Integrator'
                                        */
  real_T Integrator_IC_l;              /* Expression: 0
                                        * Referenced by: '<S423>/Integrator'
                                        */
  real_T DeadZone3_Start;              /* Expression: -10
                                        * Referenced by: '<S313>/Dead Zone3'
                                        */
  real_T DeadZone3_End;                /* Expression: 10
                                        * Referenced by: '<S313>/Dead Zone3'
                                        */
  real_T Integrator1_IC;               /* Expression: 0
                                        * Referenced by: '<S310>/Integrator1'
                                        */
  real_T Saturation_UpperSat_nn;       /* Expression: 10*9.81*2000
                                        * Referenced by: '<S128>/Saturation'
                                        */
  real_T Saturation_LowerSat_d5;       /* Expression: -10*9.81*2000
                                        * Referenced by: '<S128>/Saturation'
                                        */
  real_T Constant3_Value_o[4];         /* Expression: zeros(4,1)
                                        * Referenced by: '<S128>/Constant3'
                                        */
  real_T Integrator1_IC_k;             /* Expression: 0
                                        * Referenced by: '<S311>/Integrator1'
                                        */
  real_T Constant1_Value_o;            /* Expression: pi
                                        * Referenced by: '<S318>/Constant1'
                                        */
  real_T Constant3_Value_e[4];         /* Expression: ones(1,4).*0
                                        * Referenced by: '<S318>/Constant3'
                                        */
  real_T Constant2_Value_p[4];     /* Expression: [pi;0;pi;0].*0+[0;pi;0;pi].*.0
                                    * Referenced by: '<S318>/Constant2'
                                    */
  real_T InertialFrameCGtoAxleOffset_Value[12];
                                     /* Expression: [zeros(2,4);0.134*ones(1,4)]
                                      * Referenced by: '<S138>/Inertial Frame CG to Axle Offset'
                                      */
  real_T AxleNumber3_Value[2];       /* Expression: zeros(1,length(TrackNumVec))
                                      * Referenced by: '<S135>/Axle Number3'
                                      */
  real_T Constant1_Value_h;            /* Expression: zeros(1,NumAxl)
                                        * Referenced by: '<S141>/Constant1'
                                        */
  real_T Memory1_InitialCondition_p;   /* Expression: 0
                                        * Referenced by: '<S141>/Memory1'
                                        */
  real_T MeanWheelPosition_Gain;       /* Expression: 1/2
                                        * Referenced by: '<S141>/Mean Wheel Position'
                                        */
  real_T TrackNumber2_Value[2];        /* Expression: 1:length(TrackNumVec)
                                        * Referenced by: '<S135>/Track Number2'
                                        */
  real_T Constant1_Value_c;            /* Expression: pi
                                        * Referenced by: '<S443>/Constant1'
                                        */
  real_T Constant3_Value_oo[4];        /* Expression: ones(1,4).*0
                                        * Referenced by: '<S443>/Constant3'
                                        */
  real_T Constant2_Value_f[4];      /* Expression: [pi;0;pi;0].*0+[0;pi;0;pi].*0
                                     * Referenced by: '<S443>/Constant2'
                                     */
  real_T SteerRates_Value[4];          /* Expression: zeros(1,4)
                                        * Referenced by: '<S136>/SteerRates'
                                        */
  real_T Constant_Value_lp[4];         /* Expression: zeros(1,4)
                                        * Referenced by: '<S136>/Constant'
                                        */
  real_T ones2_Value[4];               /* Expression: ones(1,numWheels)
                                        * Referenced by: '<S316>/ones2'
                                        */
  real_T u_Value[4];                   /* Expression: [zeros(1,numWheels)]
                                        * Referenced by: '<S316>/0'
                                        */
  real_T ones_Value[92];               /* Expression: [ones(23,numWheels)]
                                        * Referenced by: '<S316>/ones'
                                        */
  real_T AxlesUsingAntiSway_Value;     /* Expression: find(AntiSwayEnByAxl==1)
                                        * Referenced by: '<S182>/Axles Using Anti-Sway'
                                        */
  real_T Gain_Gain_l;                  /* Expression: -1
                                        * Referenced by: '<S135>/Gain'
                                        */
  real_T AxleNumber1_Value;            /* Expression: 1:NumAxl
                                        * Referenced by: '<S135>/Axle Number1'
                                        */
  real_T TransferFcn5_A;               /* Computed Parameter: TransferFcn5_A
                                        * Referenced by: '<S120>/Transfer Fcn5'
                                        */
  real_T TransferFcn5_C;               /* Computed Parameter: TransferFcn5_C
                                        * Referenced by: '<S120>/Transfer Fcn5'
                                        */
  real_T TransferFcn6_A;               /* Computed Parameter: TransferFcn6_A
                                        * Referenced by: '<S120>/Transfer Fcn6'
                                        */
  real_T TransferFcn6_C;               /* Computed Parameter: TransferFcn6_C
                                        * Referenced by: '<S120>/Transfer Fcn6'
                                        */
  real_T TransferFcn7_A;               /* Computed Parameter: TransferFcn7_A
                                        * Referenced by: '<S120>/Transfer Fcn7'
                                        */
  real_T TransferFcn7_C;               /* Computed Parameter: TransferFcn7_C
                                        * Referenced by: '<S120>/Transfer Fcn7'
                                        */
  real_T TransferFcn8_A;               /* Computed Parameter: TransferFcn8_A
                                        * Referenced by: '<S120>/Transfer Fcn8'
                                        */
  real_T TransferFcn8_C;               /* Computed Parameter: TransferFcn8_C
                                        * Referenced by: '<S120>/Transfer Fcn8'
                                        */
  real_T Constant_Value_kk[3];         /* Expression: [0;0;0]
                                        * Referenced by: '<S216>/Constant'
                                        */
  real_T Constant1_Value_cs[3];        /* Expression: [0;0;0]
                                        * Referenced by: '<S216>/Constant1'
                                        */
  real_T Constant1_Value_j[9];         /* Expression: zeros(3,3)
                                        * Referenced by: '<S217>/Constant1'
                                        */
  real_T Crm_tableData[2];             /* Expression: [0 0]
                                        * Referenced by: '<S250>/Crm'
                                        */
  real_T Crm_bp01Data[2];              /* Expression: [-1 1]
                                        * Referenced by: '<S250>/Crm'
                                        */
  real_T u_Gain[3];                    /* Expression: 4.*ones(3,1)
                                        * Referenced by: '<S250>/4'
                                        */
  real_T u_Value_g;                    /* Expression: 0
                                        * Referenced by: '<S221>/0'
                                        */
  real_T Constant_Value_kb[12];        /* Expression: zeros(12,1)
                                        * Referenced by: '<S217>/Constant'
                                        */
  real_T Integrator_IC_fb[3];          /* Expression: [0 0 0]
                                        * Referenced by: '<S226>/Integrator'
                                        */
  real_T DeadZone_Start;               /* Expression: 0
                                        * Referenced by: '<S347>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_g;       /* Expression: 10
                                        * Referenced by: '<S347>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_d;       /* Expression: .01
                                        * Referenced by: '<S347>/Saturation1'
                                        */
  real_T Saturation_UpperSat_h4;       /* Expression: 400*pi*2
                                        * Referenced by: '<S347>/Saturation'
                                        */
  real_T Saturation_LowerSat_oj;       /* Expression: 2*pi
                                        * Referenced by: '<S347>/Saturation'
                                        */
  real_T DeadZone_Start_m;             /* Expression: 0
                                        * Referenced by: '<S348>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_j;       /* Expression: 10
                                        * Referenced by: '<S348>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_d1;      /* Expression: .01
                                        * Referenced by: '<S348>/Saturation1'
                                        */
  real_T Saturation_UpperSat_o;        /* Expression: 400*pi
                                        * Referenced by: '<S348>/Saturation'
                                        */
  real_T Saturation_LowerSat_dq;       /* Expression: 2*pi
                                        * Referenced by: '<S348>/Saturation'
                                        */
  real_T DeadZone_Start_l;             /* Expression: 0
                                        * Referenced by: '<S350>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_p;       /* Expression: 10
                                        * Referenced by: '<S350>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_m;       /* Expression: .01
                                        * Referenced by: '<S350>/Saturation1'
                                        */
  real_T Saturation_UpperSat_f;        /* Expression: 400*pi*2
                                        * Referenced by: '<S350>/Saturation'
                                        */
  real_T Saturation_LowerSat_d4;       /* Expression: 2*pi
                                        * Referenced by: '<S350>/Saturation'
                                        */
  real_T Constant_Value_jx1;           /* Expression: 0
                                        * Referenced by: '<S345>/Constant'
                                        */
  real_T Switch_Threshold_bx;          /* Expression: 0
                                        * Referenced by: '<S345>/Switch'
                                        */
  real_T DeadZone_Start_k;             /* Expression: 0
                                        * Referenced by: '<S372>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_k;       /* Expression: 10
                                        * Referenced by: '<S372>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dd;      /* Expression: .01
                                        * Referenced by: '<S372>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ig;       /* Expression: 400*pi*2
                                        * Referenced by: '<S372>/Saturation'
                                        */
  real_T Saturation_LowerSat_f1;       /* Expression: 2*pi
                                        * Referenced by: '<S372>/Saturation'
                                        */
  real_T DeadZone_Start_my;            /* Expression: 0
                                        * Referenced by: '<S373>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_d;       /* Expression: 10
                                        * Referenced by: '<S373>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_n;       /* Expression: .01
                                        * Referenced by: '<S373>/Saturation1'
                                        */
  real_T Saturation_UpperSat_aq;       /* Expression: 400*pi
                                        * Referenced by: '<S373>/Saturation'
                                        */
  real_T Saturation_LowerSat_g;        /* Expression: 2*pi
                                        * Referenced by: '<S373>/Saturation'
                                        */
  real_T DeadZone_Start_g;             /* Expression: 0
                                        * Referenced by: '<S375>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_c;       /* Expression: 10
                                        * Referenced by: '<S375>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dh;      /* Expression: .01
                                        * Referenced by: '<S375>/Saturation1'
                                        */
  real_T Saturation_UpperSat_hc;       /* Expression: 400*pi*2
                                        * Referenced by: '<S375>/Saturation'
                                        */
  real_T Saturation_LowerSat_l;        /* Expression: 2*pi
                                        * Referenced by: '<S375>/Saturation'
                                        */
  real_T Constant_Value_a2;            /* Expression: 0
                                        * Referenced by: '<S370>/Constant'
                                        */
  real_T Switch_Threshold_bq;          /* Expression: 0
                                        * Referenced by: '<S370>/Switch'
                                        */
  real_T DeadZone_Start_d;             /* Expression: 0
                                        * Referenced by: '<S397>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_pk;      /* Expression: 10
                                        * Referenced by: '<S397>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_h;       /* Expression: .01
                                        * Referenced by: '<S397>/Saturation1'
                                        */
  real_T Saturation_UpperSat_co;       /* Expression: 400*pi*2
                                        * Referenced by: '<S397>/Saturation'
                                        */
  real_T Saturation_LowerSat_d1;       /* Expression: 2*pi
                                        * Referenced by: '<S397>/Saturation'
                                        */
  real_T DeadZone_Start_h;             /* Expression: 0
                                        * Referenced by: '<S398>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_b;       /* Expression: 10
                                        * Referenced by: '<S398>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dq;      /* Expression: .01
                                        * Referenced by: '<S398>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ak;       /* Expression: 400*pi
                                        * Referenced by: '<S398>/Saturation'
                                        */
  real_T Saturation_LowerSat_jc;       /* Expression: 2*pi
                                        * Referenced by: '<S398>/Saturation'
                                        */
  real_T DeadZone_Start_c;             /* Expression: 0
                                        * Referenced by: '<S400>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_ne;      /* Expression: 10
                                        * Referenced by: '<S400>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_b;       /* Expression: .01
                                        * Referenced by: '<S400>/Saturation1'
                                        */
  real_T Saturation_UpperSat_p;        /* Expression: 400*pi*2
                                        * Referenced by: '<S400>/Saturation'
                                        */
  real_T Saturation_LowerSat_jx;       /* Expression: 2*pi
                                        * Referenced by: '<S400>/Saturation'
                                        */
  real_T Constant_Value_ex;            /* Expression: 0
                                        * Referenced by: '<S395>/Constant'
                                        */
  real_T Switch_Threshold_a;           /* Expression: 0
                                        * Referenced by: '<S395>/Switch'
                                        */
  real_T DeadZone_Start_e;             /* Expression: 0
                                        * Referenced by: '<S422>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_kn;      /* Expression: 10
                                        * Referenced by: '<S422>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_na;      /* Expression: .01
                                        * Referenced by: '<S422>/Saturation1'
                                        */
  real_T Saturation_UpperSat_nj;       /* Expression: 400*pi*2
                                        * Referenced by: '<S422>/Saturation'
                                        */
  real_T Saturation_LowerSat_pi;       /* Expression: 2*pi
                                        * Referenced by: '<S422>/Saturation'
                                        */
  real_T DeadZone_Start_a;             /* Expression: 0
                                        * Referenced by: '<S423>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_a1;      /* Expression: 10
                                        * Referenced by: '<S423>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_p;       /* Expression: .01
                                        * Referenced by: '<S423>/Saturation1'
                                        */
  real_T Saturation_UpperSat_dh;       /* Expression: 400*pi
                                        * Referenced by: '<S423>/Saturation'
                                        */
  real_T Saturation_LowerSat_eje;      /* Expression: 2*pi
                                        * Referenced by: '<S423>/Saturation'
                                        */
  real_T DeadZone_Start_b;             /* Expression: 0
                                        * Referenced by: '<S425>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_fc;      /* Expression: 10
                                        * Referenced by: '<S425>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dn;      /* Expression: .01
                                        * Referenced by: '<S425>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ep;       /* Expression: 400*pi*2
                                        * Referenced by: '<S425>/Saturation'
                                        */
  real_T Saturation_LowerSat_gb;       /* Expression: 2*pi
                                        * Referenced by: '<S425>/Saturation'
                                        */
  real_T Constant_Value_im;            /* Expression: 0
                                        * Referenced by: '<S420>/Constant'
                                        */
  real_T Switch_Threshold_l;           /* Expression: 0
                                        * Referenced by: '<S420>/Switch'
                                        */
  real_T Constant_Value_dr;            /* Expression: 0
                                        * Referenced by: '<S334>/Constant'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0
                                        * Referenced by: '<S334>/Switch'
                                        */
  real_T Constant_Value_ic;            /* Expression: 0
                                        * Referenced by: '<S335>/Constant'
                                        */
  real_T Switch_Threshold_e;           /* Expression: 0
                                        * Referenced by: '<S335>/Switch'
                                        */
  real_T Constant_Value_lt;            /* Expression: 0
                                        * Referenced by: '<S336>/Constant'
                                        */
  real_T Switch_Threshold_ci;          /* Expression: 0
                                        * Referenced by: '<S336>/Switch'
                                        */
  real_T Constant_Value_jw;            /* Expression: 0
                                        * Referenced by: '<S337>/Constant'
                                        */
  real_T Switch_Threshold_j;           /* Expression: 0
                                        * Referenced by: '<S337>/Switch'
                                        */
  real_T Ackermansteer_tableData[189];
  /* Expression: [-9.40000000000000	-9.30000000000000	-9.20000000000000	-9.10000000000000	-9	-8.90000000000000	-8.80000000000000	-8.70000000000000	-8.60000000000000	-8.50000000000000	-8.40000000000000	-8.30000000000000	-8.20000000000000	-8.10000000000000	-8	-7.90000000000000	-7.80000000000000	-7.70000000000000	-7.60000000000000	-7.50000000000000	-7.40000000000000	-7.30000000000000	-7.20000000000000	-7.10000000000000	-7	-6.90000000000000	-6.80000000000000	-6.70000000000000	-6.60000000000000	-6.50000000000000	-6.40000000000000	-6.30000000000000	-6.20000000000000	-6.10000000000000	-6	-5.90000000000000	-5.80000000000000	-5.70000000000000	-5.60000000000000	-5.50000000000000	-5.40000000000000	-5.30000000000000	-5.20000000000000	-5.10000000000000	-5	-4.90000000000000	-4.80000000000000	-4.70000000000000	-4.60000000000000	-4.50000000000000	-4.40000000000000	-4.30000000000000	-4.20000000000000	-4.10000000000000	-4	-3.90000000000000	-3.80000000000000	-3.70000000000000	-3.60000000000000	-3.50000000000000	-3.40000000000000	-3.30000000000000	-3.20000000000000	-3.10000000000000	-3	-2.90000000000000	-2.80000000000000	-2.70000000000000	-2.60000000000000	-2.50000000000000	-2.40000000000000	-2.30000000000000	-2.20000000000000	-2.10000000000000	-2	-1.90000000000000	-1.80000000000000	-1.70000000000000	-1.60000000000000	-1.50000000000000	-1.40000000000000	-1.30000000000000	-1.20000000000000	-1.10000000000000	-1	-0.900000000000000	-0.800000000000000	-0.700000000000000	-0.600000000000000	-0.500000000000000	-0.400000000000000	-0.300000000000000	-0.200000000000000	-0.100000000000000	0	0.100000000000000	0.200000000000000	0.300000000000000	0.400000000000000	0.500000000000000	0.600000000000000	0.700000000000000	0.800000000000000	0.900000000000000	1	1.10000000000000	1.20000000000000	1.30000000000000	1.40000000000000	1.50000000000000	1.60000000000000	1.70000000000000	1.80000000000000	1.90000000000000	2	2.10000000000000	2.20000000000000	2.30000000000000	2.40000000000000	2.50000000000000	2.60000000000000	2.70000000000000	2.80000000000000	2.90000000000000	3	3.10000000000000	3.20000000000000	3.30000000000000	3.40000000000000	3.50000000000000	3.60000000000000	3.70000000000000	3.80000000000000	3.90000000000000	4	4.10000000000000	4.20000000000000	4.30000000000000	4.40000000000000	4.50000000000000	4.60000000000000	4.70000000000000	4.80000000000000	4.90000000000000	5	5.10000000000000	5.20000000000000	5.30000000000000	5.40000000000000	5.50000000000000	5.60000000000000	5.70000000000000	5.80000000000000	5.90000000000000	6	6.10000000000000	6.20000000000000	6.30000000000000	6.40000000000000	6.50000000000000	6.60000000000000	6.70000000000000	6.80000000000000	6.90000000000000	7	7.10000000000000	7.20000000000000	7.30000000000000	7.40000000000000	7.50000000000000	7.60000000000000	7.70000000000000	7.80000000000000	7.90000000000000	8	8.10000000000000	8.20000000000000	8.30000000000000	8.40000000000000	8.50000000000000	8.60000000000000	8.70000000000000	8.80000000000000	8.90000000000000	9	9.10000000000000	9.20000000000000	9.30000000000000	9.40000000000000]
   * Referenced by: '<S1>/Ackerman steer'
   */
  real_T Ackermansteer_bp01Data[189];
  /* Expression: [-0.446316297916244	-0.442150786830951	-0.437977161945005	-0.433795286969319	-0.429605025209166	-0.425406239561593	-0.421198792513030	-0.416982546137136	-0.412757362092861	-0.408523101622746	-0.404279625551461	-0.400026794284603	-0.395764467807732	-0.391492505685692	-0.387210767062193	-0.382919110659678	-0.378617394779480	-0.374305477302279	-0.369983215688866	-0.365650466981227	-0.361307087803951	-0.356952934365975	-0.352587862462675	-0.348211727478321	-0.343824384388886	-0.339425687765245	-0.335015491776758	-0.330593650195247	-0.326160016399396	-0.321714443379563	-0.317256783743034	-0.312786889719717	-0.308304613168298	-0.303809805582871	-0.299302318100048	-0.294782001506567	-0.290248706247410	-0.285702282434450	-0.281142579855627	-0.276569447984685	-0.271982735991464	-0.267382292752788	-0.262767966863931	-0.258139606650706	-0.253497060182174	-0.248840175283994	-0.244168799552432	-0.239482780369037	-0.234781964916014	-0.230066200192295	-0.225335333030331	-0.220589210113630	-0.215827677995039	-0.211050583115807	-0.206257771825433	-0.201449090402323	-0.196624385075265	-0.191783502045761	-0.186926287511198	-0.182052587688919	-0.177162248841170	-0.172255117300976	-0.167331039498945	-0.162389861991014	-0.157431431487183	-0.152455594881217	-0.147462199281363	-0.142451092042095	-0.137422120796894	-0.132375133492088	-0.127309978421782	-0.122226504263869	-0.117124560117169	-0.112003995539695	-0.106864660588066	-0.101706405858090	-0.0965290825265349	-0.0913325423940914	-0.0861166379295599	-0.0808812223152671	-0.0756261494937309	-0.0703512742155897	-0.0650564520888085	-0.0597415396291770	-0.0544063943121130	-0.0490508746257825	-0.0436748401255510	-0.0382781514897739	-0.0328606705769398	-0.0274222604841732	-0.0219627856071077	-0.0164821117011354	-0.0109801059440403	-0.00545663700002112	0	0.00547767577609933	0.0110656090794470	0.0166755107772618	0.0223075038233658	0.0279617092740365	0.0336382462142580	0.0393372316825870	0.0450587805946413	0.0508030056652199	0.0565700173290652	0.0623599236602815	0.0681728302904245	0.0740088403252790	0.0798680542603446	0.0857505698950509	0.0916564822457268	0.0975858834573503	0.103538862714108	0.109515506148794	0.115515896751090	0.121540114274751	0.127588235143750	0.133660332357415	0.139756475394610	0.145876730117007	0.152021158671496	0.158189819391799	0.164382766699339	0.170600051003425	0.176841718600822	0.183107811574771	0.189398367693541	0.195713420308563	0.202052998252265	0.208417125735647	0.214805822245717	0.221219102442855	0.227656976058216	0.234119447791246	0.240606517207442	0.247118178636427	0.253654421070478	0.260215228063589	0.266800577631217	0.273410442150791	0.280044788263140	0.286703576774936	0.293386762562311	0.300094294475738	0.306826115246357	0.313582161393837	0.320362363135953	0.327166644299990	0.333994922236136	0.340847107733006	0.347723104935443	0.354622811264752	0.361546117341510	0.368492906911125	0.375463056772267	0.382456436708368	0.389472909422305	0.396512330474460	0.403574548224288	0.410659403775562	0.417766730925451	0.424896356117582	0.432048098399246	0.439221769382891	0.446417173212069	0.453634106531970	0.460872358464695	0.468131710589418	0.475411936927561	0.482712803933137	0.490034070488372	0.497375487904758	0.504736799929627	0.512117742758400	0.519518045052588	0.526937427963674	0.534375605162959	0.541832282877473	0.549307159932028	0.556799927797496	0.564310270645373	0.571837865408689	0.579382381849329	0.586943482631776	0.594520823403350	0.602114052880920	0.609722812944144	0.617346738735206	0.624985458765058]
   * Referenced by: '<S1>/Ackerman steer'
   */
  real_T Gain_Gain_e0;                 /* Expression: 0.01
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Gain1_Gain_l;                 /* Expression: 0.01
                                        * Referenced by: '<Root>/Gain1'
                                        */
  uint32_T R_maxIndex[2];              /* Computed Parameter: R_maxIndex
                                        * Referenced by: '<S25>/R'
                                        */
  uint32_T uDLookupTable_maxIndex[2];
                                   /* Computed Parameter: uDLookupTable_maxIndex
                                    * Referenced by: '<S41>/2-D Lookup Table'
                                    */
  uint32_T EffMap_maxIndex[2];         /* Computed Parameter: EffMap_maxIndex
                                        * Referenced by: '<S60>/Eff Map'
                                        */
  uint16_T ProtocolVer_Value;          /* Computed Parameter: ProtocolVer_Value
                                        * Referenced by: '<S104>/ProtocolVer'
                                        */
  boolean_T Memory_InitialCondition_jw;/* Expression: false
                                        * Referenced by: '<S141>/Memory'
                                        */
  boolean_T Constant_Value_ig;         /* Expression: true
                                        * Referenced by: '<S141>/Constant'
                                        */
  uint8_T Bit6_Gain;                   /* Computed Parameter: Bit6_Gain
                                        * Referenced by: '<S112>/Bit6'
                                        */
  uint8_T Bit5_Gain;                   /* Computed Parameter: Bit5_Gain
                                        * Referenced by: '<S112>/Bit5'
                                        */
  uint8_T Bit0_Gain;                   /* Computed Parameter: Bit0_Gain
                                        * Referenced by: '<S112>/Bit0'
                                        */
  uint8_T SpTqLF_Value;                /* Computed Parameter: SpTqLF_Value
                                        * Referenced by: '<S103>/SpTqLF'
                                        */
  uint8_T SpTqLR_Value;                /* Computed Parameter: SpTqLR_Value
                                        * Referenced by: '<S103>/SpTqLR'
                                        */
  uint8_T SpTqRF_Value;                /* Computed Parameter: SpTqRF_Value
                                        * Referenced by: '<S103>/SpTqRF'
                                        */
  uint8_T SpTqRR_Value;                /* Computed Parameter: SpTqRR_Value
                                        * Referenced by: '<S103>/SpTqRR'
                                        */
  uint8_T ZeroTorque_Value;            /* Computed Parameter: ZeroTorque_Value
                                        * Referenced by: '<S76>/ZeroTorque'
                                        */
  uint8_T Constant1_Value_o1;          /* Computed Parameter: Constant1_Value_o1
                                        * Referenced by: '<S104>/Constant1'
                                        */
  uint8_T Constant2_Value_j;           /* Computed Parameter: Constant2_Value_j
                                        * Referenced by: '<S104>/Constant2'
                                        */
  uint8_T Constant3_Value_m;           /* Computed Parameter: Constant3_Value_m
                                        * Referenced by: '<S104>/Constant3'
                                        */
  uint8_T ID1_Value;                   /* Computed Parameter: ID1_Value
                                        * Referenced by: '<S104>/ID1'
                                        */
  uint8_T ID2_Value;                   /* Computed Parameter: ID2_Value
                                        * Referenced by: '<S104>/ID2'
                                        */
  uint8_T SystemActive_Value;          /* Computed Parameter: SystemActive_Value
                                        * Referenced by: '<S76>/SystemActive'
                                        */
  uint8_T Bit7_Gain;                   /* Computed Parameter: Bit7_Gain
                                        * Referenced by: '<S112>/Bit7'
                                        */
  uint8_T SystemCtrlBits2_Value;    /* Computed Parameter: SystemCtrlBits2_Value
                                     * Referenced by: '<S104>/SystemCtrlBits2'
                                     */
  uint8_T FixPtConstant_Value;        /* Computed Parameter: FixPtConstant_Value
                                       * Referenced by: '<S118>/FixPt Constant'
                                       */
  uint8_T Output_InitialCondition;/* Computed Parameter: Output_InitialCondition
                                   * Referenced by: '<S113>/Output'
                                   */
  uint8_T Constant_Value_ej;           /* Computed Parameter: Constant_Value_ej
                                        * Referenced by: '<S119>/Constant'
                                        */
  uint8_T Constant_Value_f;            /* Computed Parameter: Constant_Value_f
                                        * Referenced by: '<S104>/Constant'
                                        */
  uint8_T ManualSwitch6_CurrentSetting;
                             /* Computed Parameter: ManualSwitch6_CurrentSetting
                              * Referenced by: '<S128>/Manual Switch6'
                              */
  P_LockUp_CAVE_MachE_sil_test_T sf_LockUp_c;/* '<S421>/LockUp' */
  P_LockUp_CAVE_MachE_sil_test_T sf_LockUp_h;/* '<S396>/LockUp' */
  P_LockUp_CAVE_MachE_sil_test_T sf_LockUp_n;/* '<S371>/LockUp' */
  P_LockUp_CAVE_MachE_sil_test_T sf_LockUp;/* '<S346>/LockUp' */
  P_CoreSubsys_CAVE_MachE_sil_test_a_T CoreSubsys_d;
  /* '<S140>/For each track and axle combination calculate suspension forces and moments' */
  P_CoreSubsys_CAVE_MachE_sil_test_e_T CoreSubsys_n;
                                     /* '<S182>/For Each Axle With Anti-Sway' */
  P_CoreSubsys_CAVE_MachE_sil_test_d_T CoreSubsys_p;
  /* '<S135>/For each track and axle combination calculate suspension forces and moments' */
  P_CoreSubsys_CAVE_MachE_sil_test_g_T CoreSubsys_m;
         /* '<S135>/For each axle calculate axle cg positions and velocities' */
  P_CoreSubsys_CAVE_MachE_sil_test_T CoreSubsys;
  /* '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */
};

/* Real-time Model Data Structure */
struct tag_RTM_CAVE_MachE_sil_test_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_CAVE_MachE_sil_test_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[93];
  real_T odeF[4][93];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_P;

/* Block signals (default storage) */
extern B_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_B;

/* Continuous states (default storage) */
extern X_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_X;

/* Block states (default storage) */
extern DW_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_DW;

/* Model entry point functions */
extern void CAVE_MachE_sil_test_initialize(void);
extern void CAVE_MachE_sil_test_output(void);
extern void CAVE_MachE_sil_test_update(void);
extern void CAVE_MachE_sil_test_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CAVE_MachE_sil_test_T *const CAVE_MachE_sil_test_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'CAVE_MachE_sil_test'
 * '<S1>'   : 'CAVE_MachE_sil_test/Vehicle Under Test'
 * '<S2>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Environment'
 * '<S3>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Monitor'
 * '<S4>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle'
 * '<S5>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Environment/Ground Feedback'
 * '<S6>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Environment/Subsystem'
 * '<S7>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Environment/Subsystem1'
 * '<S8>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Environment/Ground Feedback/Constant'
 * '<S9>'   : 'CAVE_MachE_sil_test/Vehicle Under Test/Monitor/Calculate Powertrain Energy'
 * '<S10>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery'
 * '<S11>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Front Wheel Drive'
 * '<S12>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor'
 * '<S13>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller '
 * '<S14>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface'
 * '<S15>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics'
 * '<S16>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery'
 * '<S17>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Subsystem1'
 * '<S18>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1'
 * '<S19>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal'
 * '<S20>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus'
 * '<S21>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Output Passthrough'
 * '<S22>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery'
 * '<S23>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/Charge Model'
 * '<S24>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/State of Charge Capacity'
 * '<S25>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/Voltage and Power Calculation'
 * '<S26>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator'
 * '<S27>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S28>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrStored Input'
 * '<S29>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S30>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff'
 * '<S31>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus'
 * '<S32>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator'
 * '<S33>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S34>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator/PwrStored Input'
 * '<S35>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S36>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor'
 * '<S37>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2'
 * '<S38>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Electrical Current'
 * '<S39>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power '
 * '<S40>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units'
 * '<S41>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Tabular Power Loss Data'
 * '<S42>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem'
 * '<S43>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem1'
 * '<S44>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator'
 * '<S45>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S46>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrStored Input'
 * '<S47>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S48>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command'
 * '<S49>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Subsystem1'
 * '<S50>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Subsystem4'
 * '<S51>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Accel Pedal to Traction Wheel Torque Request1'
 * '<S52>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Battery Management System'
 * '<S53>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management'
 * '<S54>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Series Regen Braking'
 * '<S55>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Accel Pedal to Traction Wheel Torque Request1/Max Motor Torque'
 * '<S56>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Compare To Constant'
 * '<S57>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management'
 * '<S58>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits'
 * '<S59>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Compare To Constant'
 * '<S60>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Mech to Elec Power Estimate'
 * '<S61>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit'
 * '<S62>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly'
 * '<S63>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Compare To Zero'
 * '<S64>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Compare To Zero1'
 * '<S65>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Saturation Dynamic1'
 * '<S66>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit/Max Motor Torque'
 * '<S67>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit/Saturation Dynamic'
 * '<S68>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly/Compare To Constant'
 * '<S69>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly/Compare To Constant1'
 * '<S70>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Series Regen Braking/Max Motor Torque'
 * '<S71>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/DummyPT'
 * '<S72>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Format CM Powertrain Signals'
 * '<S73>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics'
 * '<S74>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess'
 * '<S75>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Subsystem2'
 * '<S76>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid'
 * '<S77>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/WheelSpdCmd'
 * '<S78>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal'
 * '<S79>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Diff Trq'
 * '<S80>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem'
 * '<S81>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2'
 * '<S82>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3'
 * '<S83>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4'
 * '<S84>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem/MATLAB Function'
 * '<S85>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem/WheelInertia'
 * '<S86>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem/WheelInertia1'
 * '<S87>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2/MATLAB Function'
 * '<S88>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2/WheelInertia'
 * '<S89>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2/WheelInertia1'
 * '<S90>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3/MATLAB Function'
 * '<S91>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3/WheelInertia'
 * '<S92>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3/WheelInertia1'
 * '<S93>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4/MATLAB Function'
 * '<S94>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4/WheelInertia'
 * '<S95>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4/WheelInertia1'
 * '<S96>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/Compare To Constant'
 * '<S97>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/Compare To Constant1'
 * '<S98>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau'
 * '<S99>'  : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau1'
 * '<S100>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau2'
 * '<S101>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau3'
 * '<S102>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid'
 * '<S103>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Mode Override'
 * '<S104>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx'
 * '<S105>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/TrqSpd Override'
 * '<S106>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid/Band-Aid'
 * '<S107>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater'
 * '<S108>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater1'
 * '<S109>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater2'
 * '<S110>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater3'
 * '<S111>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/MATLAB Function'
 * '<S112>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Packer'
 * '<S113>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/WatchdogCnt'
 * '<S114>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater/Defloater'
 * '<S115>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater1/Defloater'
 * '<S116>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater2/Defloater'
 * '<S117>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater3/Defloater'
 * '<S118>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/WatchdogCnt/Increment Real World'
 * '<S119>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/WatchdogCnt/Wrap To Zero'
 * '<S120>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF'
 * '<S121>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Brake Pressure'
 * '<S122>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering'
 * '<S123>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Subsystem4'
 * '<S124>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension'
 * '<S125>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle'
 * '<S126>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle Routing'
 * '<S127>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheel Routing'
 * '<S128>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires'
 * '<S129>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering'
 * '<S130>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput'
 * '<S131>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput/ParalConstRatio'
 * '<S132>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput/ParalConstRatio/Parallel'
 * '<S133>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Subsystem4/Subsystem'
 * '<S134>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension'
 * '<S135>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring'
 * '<S136>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Steer Rate Adapter'
 * '<S137>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Steering Adapter'
 * '<S138>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Vehicle Adapter'
 * '<S139>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Wheel  Adapter'
 * '<S140>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson'
 * '<S141>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/Add Axle Offsets'
 * '<S142>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle and track calculate suspension and wheel positions and velocities'
 * '<S143>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities'
 * '<S144>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments'
 * '<S145>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/Wheel Carrier to Axle Interface Compliance'
 * '<S146>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle and track calculate suspension and wheel positions and velocities/Select DCM'
 * '<S147>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics'
 * '<S148>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics'
 * '<S149>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Suspension Forces and Moments'
 * '<S150>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Track location transforms'
 * '<S151>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments'
 * '<S152>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics/Select Axle Mass By Axle'
 * '<S153>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics/Select X Axis Axle Mass Moment of Inertia By Axle'
 * '<S154>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Track location transforms/DCM Transpose'
 * '<S155>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments/Select Axle Mass By Axle'
 * '<S156>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments/Select Y Axis Axle Mass Moment of Inertia By Axle'
 * '<S157>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension'
 * '<S158>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Steering Delta Select'
 * '<S159>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations'
 * '<S160>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Vehicle Moments From X and Y Forces'
 * '<S161>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic'
 * '<S162>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled'
 * '<S163>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Camber Height Slope'
 * '<S164>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Camber Steering Center'
 * '<S165>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Caster Height Slope'
 * '<S166>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Caster Steering Center'
 * '<S167>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Roll Steer Slope'
 * '<S168>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Toe Steering Center'
 * '<S169>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension'
 * '<S170>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination'
 * '<S171>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope'
 * '<S172>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force'
 * '<S173>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select C By Axle'
 * '<S174>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select F0 By Axle'
 * '<S175>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select K By Axle'
 * '<S176>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select Max Travel By Axle'
 * '<S177>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Max stop reached'
 * '<S178>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Min stop reached'
 * '<S179>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Disabled'
 * '<S180>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force'
 * '<S181>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments'
 * '<S182>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force'
 * '<S183>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway'
 * '<S184>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta'
 * '<S185>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Arm Neutral Angle By Axle'
 * '<S186>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Arm Radius By Axle'
 * '<S187>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Bar Torsion Spring Constant By Axle'
 * '<S188>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension'
 * '<S189>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Steering Delta Select'
 * '<S190>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations'
 * '<S191>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Vehicle Moments From X and Y Forces'
 * '<S192>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic'
 * '<S193>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled'
 * '<S194>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Height Slope'
 * '<S195>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Steering Center'
 * '<S196>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Steering Slope'
 * '<S197>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Height Slope'
 * '<S198>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Steering Center'
 * '<S199>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Steering Slope'
 * '<S200>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Roll Steer Slope'
 * '<S201>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Toe Steering Center'
 * '<S202>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Toe Steering Slope'
 * '<S203>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension'
 * '<S204>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination'
 * '<S205>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope'
 * '<S206>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force'
 * '<S207>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select C By Axle'
 * '<S208>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select F0 By Axle'
 * '<S209>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select K By Axle'
 * '<S210>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select Max Travel By Axle'
 * '<S211>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Max stop reached'
 * '<S212>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Min stop reached'
 * '<S213>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Enabled'
 * '<S214>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Enabled/Steering Height Slope By Steered Axle'
 * '<S215>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem'
 * '<S216>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF'
 * '<S217>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1'
 * '<S218>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1'
 * '<S219>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Aero Drag'
 * '<S220>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Forces'
 * '<S221>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Gravity'
 * '<S222>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Moment Calc'
 * '<S223>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Moments'
 * '<S224>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Power'
 * '<S225>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing'
 * '<S226>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection'
 * '<S227>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Susp2Chassis'
 * '<S228>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Wheel to CG'
 * '<S229>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/vehdyncginert'
 * '<S230>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles'
 * '<S231>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot'
 * '<S232>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Determine Force,  Mass & Inertia'
 * '<S233>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Vbxw'
 * '<S234>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Velocity Conversion'
 * '<S235>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Velocity Conversion1'
 * '<S236>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Velocity Conversion2'
 * '<S237>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/transform to Inertial axes '
 * '<S238>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix'
 * '<S239>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles/phidot thetadot psidot'
 * '<S240>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S241>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/3x3 Cross Product'
 * '<S242>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/I x w'
 * '<S243>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/I x w1'
 * '<S244>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/3x3 Cross Product/Subsystem'
 * '<S245>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/3x3 Cross Product/Subsystem1'
 * '<S246>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Determine Force,  Mass & Inertia/Mass input//output  momentum'
 * '<S247>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Determine Force,  Mass & Inertia/Mass input//output  momentum/For Each Subsystem'
 * '<S248>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Vbxw/Subsystem'
 * '<S249>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Vbxw/Subsystem1'
 * '<S250>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Aero Drag/Drag Force'
 * '<S251>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left'
 * '<S252>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right'
 * '<S253>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric'
 * '<S254>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left'
 * '<S255>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right'
 * '<S256>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement'
 * '<S257>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S258>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S259>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S260>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S261>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S262>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S263>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S264>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement'
 * '<S265>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S266>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S267>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S268>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S269>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S270>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S271>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S272>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta'
 * '<S273>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip'
 * '<S274>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Rotation Angles to Direction Cosine Matrix'
 * '<S275>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/transform to Inertial axes'
 * '<S276>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/transform to Inertial axes1'
 * '<S277>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR'
 * '<S278>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly'
 * '<S279>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly/Compare To Constant'
 * '<S280>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly/Compare To Constant1'
 * '<S281>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S282>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR/Subsystem'
 * '<S283>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR/Subsystem1'
 * '<S284>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement'
 * '<S285>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S286>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S287>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S288>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S289>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S290>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S291>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S292>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement'
 * '<S293>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S294>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S295>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S296>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S297>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S298>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S299>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S300>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Angle Wrap'
 * '<S301>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip'
 * '<S302>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/phidot thetadot psidot'
 * '<S303>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/wxR'
 * '<S304>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Angle Wrap/None'
 * '<S305>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip/div0protect - abs poly'
 * '<S306>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip/div0protect - abs poly/Compare To Constant'
 * '<S307>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip/div0protect - abs poly/Compare To Constant1'
 * '<S308>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/wxR/Subsystem'
 * '<S309>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/wxR/Subsystem1'
 * '<S310>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/Cont LPF'
 * '<S311>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/Cont LPF1'
 * '<S312>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS'
 * '<S313>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh'
 * '<S314>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires'
 * '<S315>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire'
 * '<S316>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/scale factors with friction'
 * '<S317>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Routiong'
 * '<S318>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel Angles'
 * '<S319>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform'
 * '<S320>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix'
 * '<S321>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S322>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires'
 * '<S323>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1'
 * '<S324>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2'
 * '<S325>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3'
 * '<S326>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4'
 * '<S327>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fx'
 * '<S328>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fy'
 * '<S329>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fz'
 * '<S330>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Mx'
 * '<S331>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/My'
 * '<S332>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Mz'
 * '<S333>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Re'
 * '<S334>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response'
 * '<S335>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response1'
 * '<S336>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response2'
 * '<S337>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response3'
 * '<S338>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/alpha'
 * '<S339>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/kappa'
 * '<S340>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/omega'
 * '<S341>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/z'
 * '<S342>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/zdot'
 * '<S343>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Bus Routing'
 * '<S344>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input'
 * '<S345>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Vertical Wheel and Unsprung Mass Response'
 * '<S346>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module'
 * '<S347>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Fx Relaxation'
 * '<S348>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Fy Relaxation'
 * '<S349>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Magic Tire Const Input'
 * '<S350>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/My Relaxation'
 * '<S351>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes'
 * '<S352>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Friction Model'
 * '<S353>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp'
 * '<S354>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes/Disk Brake'
 * '<S355>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S356>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/Locked'
 * '<S357>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/Slipping'
 * '<S358>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup'
 * '<S359>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectSlip'
 * '<S360>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S361>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S362>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S363>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S364>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S365>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S366>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S367>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S368>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Bus Routing'
 * '<S369>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input'
 * '<S370>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Vertical Wheel and Unsprung Mass Response'
 * '<S371>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module'
 * '<S372>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Fx Relaxation'
 * '<S373>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Fy Relaxation'
 * '<S374>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Magic Tire Const Input'
 * '<S375>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/My Relaxation'
 * '<S376>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes'
 * '<S377>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Friction Model'
 * '<S378>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp'
 * '<S379>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes/Disk Brake'
 * '<S380>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S381>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/Locked'
 * '<S382>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/Slipping'
 * '<S383>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup'
 * '<S384>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectSlip'
 * '<S385>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S386>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S387>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S388>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S389>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S390>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S391>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S392>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S393>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Bus Routing'
 * '<S394>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input'
 * '<S395>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Vertical Wheel and Unsprung Mass Response'
 * '<S396>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module'
 * '<S397>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Fx Relaxation'
 * '<S398>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Fy Relaxation'
 * '<S399>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Magic Tire Const Input'
 * '<S400>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/My Relaxation'
 * '<S401>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes'
 * '<S402>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Friction Model'
 * '<S403>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp'
 * '<S404>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes/Disk Brake'
 * '<S405>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S406>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/Locked'
 * '<S407>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/Slipping'
 * '<S408>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup'
 * '<S409>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectSlip'
 * '<S410>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S411>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S412>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S413>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S414>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S415>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S416>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S417>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S418>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Bus Routing'
 * '<S419>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input'
 * '<S420>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Vertical Wheel and Unsprung Mass Response'
 * '<S421>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module'
 * '<S422>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Fx Relaxation'
 * '<S423>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Fy Relaxation'
 * '<S424>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Magic Tire Const Input'
 * '<S425>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/My Relaxation'
 * '<S426>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes'
 * '<S427>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Friction Model'
 * '<S428>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp'
 * '<S429>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes/Disk Brake'
 * '<S430>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S431>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/Locked'
 * '<S432>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/Slipping'
 * '<S433>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup'
 * '<S434>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectSlip'
 * '<S435>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S436>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S437>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S438>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S439>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S440>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S441>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S442>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S443>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel Angles'
 * '<S444>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform'
 * '<S445>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix'
 * '<S446>' : 'CAVE_MachE_sil_test/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 */
#endif                                 /* RTW_HEADER_CAVE_MachE_sil_test_h_ */
