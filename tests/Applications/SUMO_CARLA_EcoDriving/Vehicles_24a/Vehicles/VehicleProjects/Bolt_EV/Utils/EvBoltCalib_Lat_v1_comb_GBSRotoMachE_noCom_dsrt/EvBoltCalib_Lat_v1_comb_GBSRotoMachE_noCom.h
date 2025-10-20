/*
 * EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom.h
 *
 * Code generation for model "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom".
 *
 * Model version              : 1.29
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon Jul 10 17:21:16 2023
 *
 * Target selection: dsrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_h_
#define RTW_HEADER_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_h_
#include <string.h>
#include <math.h>
#ifndef EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_COMMON_INCLUDES_
# define EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsmpbap.h"
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsmpb_bd.h"
#endif         /* EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_COMMON_INCLUDES_ */

#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_types.h"

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

/* Block signals for system '<S198>/Floater' */
typedef struct {
  real32_T float32;                    /* '<S198>/Floater' */
} B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block signals for system '<S249>/MATLAB Function' */
typedef struct {
  real_T switchFlag;                   /* '<S249>/MATLAB Function' */
} B_MATLABFunction_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block signals for system '<S276>/Defloater' */
typedef struct {
  uint8_T byte1;                       /* '<S276>/Defloater' */
  uint8_T byte2;                       /* '<S276>/Defloater' */
  uint8_T byte3;                       /* '<S276>/Defloater' */
  uint8_T byte4;                       /* '<S276>/Defloater' */
} B_Defloater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block signals for system '<S304>/For each axle and track calculate suspension and wheel positions and velocities' */
typedef struct {
  real_T Selector6;                    /* '<S311>/Selector6' */
  real_T TrigonometricFunction;        /* '<S311>/Trigonometric Function' */
  real_T Gain;                         /* '<S311>/Gain' */
  real_T Sum;                          /* '<S315>/Sum' */
  real_T DCMStaringRow;                /* '<S315>/DCM Staring Row' */
  real_T Sum1;                         /* '<S315>/Sum1' */
  real_T SelectDCM[9];                 /* '<S315>/Select DCM' */
  real_T MathFunction[9];              /* '<S311>/Math Function' */
  real_T Selector1[3];                 /* '<S311>/Selector1' */
  real_T MatrixMultiply1[3];           /* '<S311>/Matrix Multiply1' */
  real_T Translationeffectonpositions[3];/* '<S311>/Selector2' */
  real_T Sum1_l[3];                    /* '<S311>/Sum1' */
  real_T Selector5[3];                 /* '<S311>/Selector5' */
  real_T TrigonometricFunction1;       /* '<S311>/Trigonometric Function1' */
  real_T MatrixConcatenate4[3];        /* '<S311>/Matrix Concatenate4' */
  real_T Product2[3];                  /* '<S311>/Product2' */
  real_T Sum3[3];                      /* '<S311>/Sum3' */
  real_T Selector[3];                  /* '<S311>/Selector' */
  real_T Rotationeffectonpositions[3]; /* '<S311>/Matrix Multiply3' */
  real_T Sum4[3];                      /* '<S311>/Sum4' */
  real_T Product1[3];                  /* '<S311>/Product1' */
  real_T Sum2[3];                      /* '<S311>/Sum2' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block signals for system '<S304>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T Reshape[3];                   /* '<S323>/Reshape' */
  real_T u;                            /* '<S323>/ ' */
  real_T TrigonometricFunction;        /* '<S323>/Trigonometric Function' */
  real_T TrigonometricFunction1;       /* '<S323>/Trigonometric Function1' */
  real_T MatrixConcatenate4[3];        /* '<S323>/Matrix Concatenate4' */
  real_T MatrixConcatenate5[3];        /* '<S323>/Matrix Concatenate5' */
  real_T MatrixConcatenate3[9];        /* '<S323>/Matrix Concatenate3' */
  real_T cgcoordinates[3];             /* '<S319>/cg coordinates' */
  real_T MathFunction1[9];             /* '<S319>/Math Function1' */
  real_T zdot;                         /* '<S317>/Vz' */
  real_T TmpSignalConversionAtMatrixMultiply2Inport2[3];/* '<S319>/cgV in' */
  real_T MatrixMultiply2[3];           /* '<S319>/Matrix Multiply2' */
  real_T p;                            /* '<S317>/Vy1' */
  real_T yaxistrackcoordinates[2];     /* '<S320>/y axis track coordinates' */
  real_T MathFunction[2];              /* '<S320>/Math Function' */
  real_T yaxistrackcoordinates_o[2];   /* '<S318>/y axis track coordinates' */
  real_T Product1;                     /* '<S317>/Product1' */
  real_T Sum;                          /* '<S317>/Sum' */
  real_T ydot;                         /* '<S317>/Vy' */
  real_T ydotp;                        /* '<S317>/Vyp' */
  real_T DataTypeConversion[2];        /* '<S320>/Data Type Conversion' */
  real_T Product[2];                   /* '<S320>/Product' */
  real_T DotProduct1;                  /* '<S320>/Dot Product1' */
  real_T DataTypeConversion_j;         /* '<S324>/Data Type Conversion' */
  real_T Product_g;                    /* '<S324>/Product' */
  real_T SumofElements;                /* '<S324>/Sum of Elements' */
  real_T Product2[2];                  /* '<S320>/Product2' */
  real_T DataTypeConversion_a;         /* '<S325>/Data Type Conversion' */
  real_T Product_k;                    /* '<S325>/Product' */
  real_T SumofElements_b;              /* '<S325>/Sum of Elements' */
  real_T Sum_e[2];                     /* '<S320>/Sum' */
  real_T Product3[2];                  /* '<S320>/Product3' */
  real_T Product4[2];                  /* '<S320>/Product4' */
  real_T xaxiswheelmoments[2];         /* '<S320>/x axis wheel moments' */
  real_T DotProduct2;                  /* '<S320>/Dot Product2' */
  real_T Sum1;                         /* '<S320>/Sum1' */
  real_T DataTypeConversion_k[2];      /* '<S318>/Data Type Conversion' */
  real_T Product_b[2];                 /* '<S318>/Product' */
  real_T DotProduct1_h;                /* '<S318>/Dot Product1' */
  real_T SuspensionMomentDirectionOnSolidAxle;
                        /* '<S318>/Suspension Moment Direction On Solid Axle' */
  real_T Sum1_n;                       /* '<S316>/Sum1' */
  real_T DataTypeConversion_e;         /* '<S322>/Data Type Conversion' */
  real_T Product_p;                    /* '<S322>/Product' */
  real_T SumofElements_h;              /* '<S322>/Sum of Elements' */
  real_T pdot;                         /* '<S317>/Divide' */
  real_T DotProduct;                   /* '<S320>/Dot Product' */
  real_T DotProduct_o;                 /* '<S318>/Dot Product' */
  real_T SuspensionForceDirectionOnSolidAxle;
                         /* '<S318>/Suspension Force Direction On Solid Axle' */
  real_T Sum_f;                        /* '<S316>/Sum' */
  real_T DataTypeConversion_g;         /* '<S321>/Data Type Conversion' */
  real_T Product_j;                    /* '<S321>/Product' */
  real_T SumofElements_h5;             /* '<S321>/Sum of Elements' */
  real_T Divide1;                      /* '<S317>/Divide1' */
  real_T Sum2;                         /* '<S317>/Sum2' */
  real_T zddot;                        /* '<S317>/Sum1' */
  boolean_T RelationalOperator[2];     /* '<S320>/Relational Operator' */
  boolean_T RelationalOperator_j;      /* '<S324>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S325>/Relational Operator' */
  boolean_T RelationalOperator_i[2];   /* '<S318>/Relational Operator' */
  boolean_T RelationalOperator_d;      /* '<S322>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S321>/Relational Operator' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_l_T;

/* Continuous states for system '<S304>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T _CSTATE;                      /* '<S323>/ ' */
  real_T cgcoordinates_CSTATE[3];      /* '<S319>/cg coordinates' */
  real_T Vz_CSTATE;                    /* '<S317>/Vz' */
  real_T Vy1_CSTATE;                   /* '<S317>/Vy1' */
  real_T Vy_CSTATE;                    /* '<S317>/Vy' */
} X_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_o_T;

/* State derivatives for system '<S304>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T _CSTATE;                      /* '<S323>/ ' */
  real_T cgcoordinates_CSTATE[3];      /* '<S319>/cg coordinates' */
  real_T Vz_CSTATE;                    /* '<S317>/Vz' */
  real_T Vy1_CSTATE;                   /* '<S317>/Vy1' */
  real_T Vy_CSTATE;                    /* '<S317>/Vy' */
} XDot_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_k_T;

/* State Disabled for system '<S304>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  boolean_T _CSTATE;                   /* '<S323>/ ' */
  boolean_T cgcoordinates_CSTATE[3];   /* '<S319>/cg coordinates' */
  boolean_T Vz_CSTATE;                 /* '<S317>/Vz' */
  boolean_T Vy1_CSTATE;                /* '<S317>/Vy1' */
  boolean_T Vy_CSTATE;                 /* '<S317>/Vy' */
} XDis_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_e_T;

/* Block signals for system '<S341>/Min stop reached' */
typedef struct {
  real_T Sum1;                         /* '<S347>/Sum1' */
  real_T Gain5;                        /* '<S347>/Gain5' */
  real_T Gain4;                        /* '<S347>/Gain4' */
  real_T Product3;                     /* '<S347>/Product3' */
  real_T Abs1;                         /* '<S347>/Abs1' */
  real_T Saturation;                   /* '<S347>/Saturation' */
  real_T TrigonometricFunction;        /* '<S347>/Trigonometric Function' */
  real_T Gain;                         /* '<S347>/Gain' */
  real_T Sum2;                         /* '<S347>/Sum2' */
  real_T LowerHardStopBlendMult;       /* '<S347>/Lower Hard Stop Blend Mult' */
  real_T MathFunction;                 /* '<S347>/Math Function' */
  real_T Product2;                     /* '<S347>/Product2' */
  real_T Product;                      /* '<S347>/Product' */
  real_T Product1;                     /* '<S347>/Product1' */
  real_T Sum;                          /* '<S347>/Sum' */
  real_T Product4;                     /* '<S347>/Product4' */
} B_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block signals for system '<S341>/Max stop reached' */
typedef struct {
  real_T Sum1;                         /* '<S346>/Sum1' */
  real_T Gain5;                        /* '<S346>/Gain5' */
  real_T Gain4;                        /* '<S346>/Gain4' */
  real_T Product3;                     /* '<S346>/Product3' */
  real_T Abs1;                         /* '<S346>/Abs1' */
  real_T Saturation;                   /* '<S346>/Saturation' */
  real_T TrigonometricFunction;        /* '<S346>/Trigonometric Function' */
  real_T Gain;                         /* '<S346>/Gain' */
  real_T MathFunction;                 /* '<S346>/Math Function' */
  real_T Product2;                     /* '<S346>/Product2' */
  real_T Product;                      /* '<S346>/Product' */
  real_T Product1;                     /* '<S346>/Product1' */
  real_T Sum;                          /* '<S346>/Sum' */
  real_T Sum2;                         /* '<S346>/Sum2' */
  real_T UpperHardStopBlendMult;       /* '<S346>/Upper Hard Stop Blend Mult' */
  real_T Product4;                     /* '<S346>/Product4' */
} B_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block signals for system '<S304>/For each track and axle combination calculate suspension forces and moments' */
typedef struct {
  real_T DataTypeConversion;           /* '<S333>/Data Type Conversion' */
  real_T Product;                      /* '<S333>/Product' */
  real_T SumofElements;                /* '<S333>/Sum of Elements' */
  real_T DataTypeConversion_m;         /* '<S332>/Data Type Conversion' */
  real_T Product_d;                    /* '<S332>/Product' */
  real_T SumofElements_f;              /* '<S332>/Sum of Elements' */
  real_T TmpSignalConversionAtSelector3Inport1[2];/* '<S327>/Mux' */
  real_T Selector1;                    /* '<S326>/Selector1' */
  real_T Sum2;                         /* '<S326>/Sum2' */
  real_T Selector5;                    /* '<S327>/Selector5' */
  real_T Product_g;                    /* '<S327>/Product' */
  real_T Selector3;                    /* '<S327>/Selector3' */
  real_T Abs;                          /* '<S338>/Abs' */
  real_T Product_dj;                   /* '<S338>/Product' */
  real_T Selector2[2];                 /* '<S326>/Selector2' */
  real_T Selector[2];                  /* '<S326>/Selector' */
  real_T Add;                          /* '<S338>/Add' */
  real_T DataTypeConversion_c;         /* '<S344>/Data Type Conversion' */
  real_T Product_b;                    /* '<S344>/Product' */
  real_T SumofElements_g;              /* '<S344>/Sum of Elements' */
  real_T DataTypeConversion_l;         /* '<S343>/Data Type Conversion' */
  real_T Product_h;                    /* '<S343>/Product' */
  real_T SumofElements_m;              /* '<S343>/Sum of Elements' */
  real_T Product4;                     /* '<S339>/Product4' */
  real_T Add4;                         /* '<S339>/Add4' */
  real_T HeightSignConvention;         /* '<S339>/Height Sign Convention' */
  real_T Product3;                     /* '<S331>/Product3' */
  real_T Sum2_b;                       /* '<S331>/Sum2' */
  real_T DataTypeConversion_d;         /* '<S335>/Data Type Conversion' */
  real_T Product_l;                    /* '<S335>/Product' */
  real_T SumofElements_mz;             /* '<S335>/Sum of Elements' */
  real_T DataTypeConversion_k;         /* '<S334>/Data Type Conversion' */
  real_T Product_bk;                   /* '<S334>/Product' */
  real_T SumofElements_b;              /* '<S334>/Sum of Elements' */
  real_T Product5;                     /* '<S331>/Product5' */
  real_T Sum1;                         /* '<S331>/Sum1' */
  real_T DataTypeConversion_a;         /* '<S337>/Data Type Conversion' */
  real_T Product_j;                    /* '<S337>/Product' */
  real_T SumofElements_d;              /* '<S337>/Sum of Elements' */
  real_T DataTypeConversion_p;         /* '<S336>/Data Type Conversion' */
  real_T Product_l2;                   /* '<S336>/Product' */
  real_T SumofElements_i;              /* '<S336>/Sum of Elements' */
  real_T Product1;                     /* '<S331>/Product1' */
  real_T Sum;                          /* '<S331>/Sum' */
  real_T Product3_d;                   /* '<S339>/Product3' */
  real_T DataTypeConversion_f;         /* '<S342>/Data Type Conversion' */
  real_T Product_c;                    /* '<S342>/Product' */
  real_T SumofElements_e;              /* '<S342>/Sum of Elements' */
  real_T Add2;                         /* '<S338>/Add2' */
  real_T Product5_d;                   /* '<S339>/Product5' */
  real_T Add1;                         /* '<S339>/Add1' */
  real_T Product1_a;                   /* '<S339>/Product1' */
  real_T Sign1;                        /* '<S339>/Sign1' */
  real_T Product2;                     /* '<S339>/Product2' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[6];/* '<S313>/Suspension' */
  real_T DataTypeConversion_fh;        /* '<S345>/Data Type Conversion' */
  real_T Product_i;                    /* '<S345>/Product' */
  real_T SumofElements_fu;             /* '<S345>/Sum of Elements' */
  real_T Sum_a;                        /* '<S341>/Sum' */
  real_T Sum_e;                        /* '<S339>/Sum' */
  real_T VehicleForceSign;             /* '<S338>/Vehicle Force Sign' */
  real_T Selector1_g;                  /* '<S329>/Selector1' */
  real_T Selector_d;                   /* '<S329>/Selector' */
  real_T VehicleHeight;                /* '<S338>/Sign convention' */
  real_T Sum_h;                        /* '<S329>/Sum' */
  real_T Product_l5;                   /* '<S329>/Product' */
  real_T UnaryMinus;                   /* '<S329>/Unary Minus' */
  real_T Selector2_g;                  /* '<S329>/Selector2' */
  real_T Product1_p;                   /* '<S329>/Product1' */
  real_T Reshape[3];                   /* '<S329>/Reshape' */
  real_T Selector3_o[3];               /* '<S329>/Selector3' */
  real_T Sum1_n[3];                    /* '<S329>/Sum1' */
  real_T Sum3;                         /* '<S331>/Sum3' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[3];/* '<S313>/Suspension' */
  real_T Reshape19[2];                 /* '<S313>/Reshape19' */
  real_T Selector3_e;                  /* '<S326>/Selector3' */
  boolean_T RelationalOperator;        /* '<S333>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S332>/Relational Operator' */
  boolean_T RelationalOperator_j;      /* '<S344>/Relational Operator' */
  boolean_T RelationalOperator_d;      /* '<S343>/Relational Operator' */
  boolean_T RelationalOperator_l;      /* '<S335>/Relational Operator' */
  boolean_T RelationalOperator_b;      /* '<S334>/Relational Operator' */
  boolean_T RelationalOperator_c;      /* '<S337>/Relational Operator' */
  boolean_T RelationalOperator_a;      /* '<S336>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S342>/Relational Operator' */
  boolean_T RelationalOperator_e;      /* '<S345>/Relational Operator' */
  B_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Maxstopreached;/* '<S341>/Max stop reached' */
  B_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Minstopreached;/* '<S341>/Min stop reached' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_l0_T;

/* Block signals for system '<S351>/For Each Axle With Anti-Sway' */
typedef struct {
  real_T DataTypeConversion;           /* '<S355>/Data Type Conversion' */
  real_T Product;                      /* '<S355>/Product' */
  real_T SumofElements;                /* '<S355>/Sum of Elements' */
  real_T DataTypeConversion_b;         /* '<S354>/Data Type Conversion' */
  real_T Product_o;                    /* '<S354>/Product' */
  real_T SumofElements_a;              /* '<S354>/Sum of Elements' */
  real_T TrigonometricFunction;        /* '<S353>/Trigonometric Function' */
  real_T Z0;                           /* '<S353>/Product' */
  real_T Selector5[2];                 /* '<S352>/Selector5' */
  real_T Selector1;                    /* '<S352>/Selector1' */
  real_T Sum2[2];                      /* '<S352>/Sum2' */
  real_T Selector3[2];                 /* '<S352>/Selector3' */
  real_T Selector6[2];                 /* '<S352>/Selector6' */
  real_T Selector4[2];                 /* '<S352>/Selector4' */
  real_T Sum3;                         /* '<S353>/Sum3' */
  real_T Sum6;                         /* '<S353>/Sum6' */
  real_T Sum[2];                       /* '<S353>/Sum' */
  real_T Product1[2];                  /* '<S353>/Product1' */
  real_T AngleTangentLimit[2];         /* '<S353>/Angle Tangent Limit' */
  real_T DataTypeConversion_bj;        /* '<S356>/Data Type Conversion' */
  real_T Product_p;                    /* '<S356>/Product' */
  real_T SumofElements_e;              /* '<S356>/Sum of Elements' */
  real_T TrigonometricFunction1[2];    /* '<S353>/Trigonometric Function1' */
  real_T Sum1[2];                      /* '<S353>/Sum1' */
  real_T deltaTheta;                   /* '<S353>/Sum2' */
  real_T antiswaybartorque;            /* '<S353>/Product4' */
  real_T Product2;                     /* '<S353>/Product2' */
  real_T TrigonometricFunction2[2];    /* '<S353>/Trigonometric Function2' */
  real_T Product3[2];                  /* '<S353>/Product3' */
  real_T Selector[2];                  /* '<S352>/Selector' */
  real_T Sum4[2];                      /* '<S353>/Sum4' */
  real_T Selector2[2];                 /* '<S352>/Selector2' */
  real_T Sum5[2];                      /* '<S353>/Sum5' */
  boolean_T RelationalOperator;        /* '<S355>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S354>/Relational Operator' */
  boolean_T RelationalOperator_o;      /* '<S356>/Relational Operator' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_o_T;

/* Block signals for system '<S309>/For each track and axle combination calculate suspension forces and moments' */
typedef struct {
  real_T TmpSignalConversionAtSelector3Inport1[3];/* '<S358>/Mux' */
  real_T Selector1;                    /* '<S357>/Selector1' */
  real_T Sum2;                         /* '<S357>/Sum2' */
  real_T Selector5;                    /* '<S358>/Selector5' */
  real_T Product;                      /* '<S358>/Product' */
  real_T Selector3;                    /* '<S358>/Selector3' */
  real_T Abs2;                         /* '<S362>/Abs2' */
  real_T DataTypeConversion;           /* '<S365>/Data Type Conversion' */
  real_T Product_n;                    /* '<S365>/Product' */
  real_T SumofElements;                /* '<S365>/Sum of Elements' */
  real_T Product2;                     /* '<S362>/Product2' */
  real_T DataTypeConversion_p;         /* '<S364>/Data Type Conversion' */
  real_T Product_h;                    /* '<S364>/Product' */
  real_T SumofElements_d;              /* '<S364>/Sum of Elements' */
  real_T DataTypeConversion_p5;        /* '<S363>/Data Type Conversion' */
  real_T Product_d;                    /* '<S363>/Product' */
  real_T SumofElements_i;              /* '<S363>/Sum of Elements' */
  real_T DataTypeConversion_a;         /* '<S383>/Data Type Conversion' */
  real_T Product_m;                    /* '<S383>/Product' */
  real_T SumofElements_k;              /* '<S383>/Sum of Elements' */
  real_T Abs;                          /* '<S372>/Abs' */
  real_T Product_hq;                   /* '<S372>/Product' */
  real_T Selector2[2];                 /* '<S357>/Selector2' */
  real_T Selector[2];                  /* '<S357>/Selector' */
  real_T Add;                          /* '<S372>/Add' */
  real_T DataTypeConversion_c;         /* '<S378>/Data Type Conversion' */
  real_T Product_k;                    /* '<S378>/Product' */
  real_T SumofElements_g;              /* '<S378>/Sum of Elements' */
  real_T DataTypeConversion_o;         /* '<S377>/Data Type Conversion' */
  real_T Product_c;                    /* '<S377>/Product' */
  real_T SumofElements_o;              /* '<S377>/Sum of Elements' */
  real_T Product4;                     /* '<S373>/Product4' */
  real_T Add4;                         /* '<S373>/Add4' */
  real_T HeightSignConvention;         /* '<S373>/Height Sign Convention' */
  real_T Product3;                     /* '<S362>/Product3' */
  real_T Sum2_b;                       /* '<S362>/Sum2' */
  real_T Abs1;                         /* '<S362>/Abs1' */
  real_T DataTypeConversion_m;         /* '<S368>/Data Type Conversion' */
  real_T Product_b;                    /* '<S368>/Product' */
  real_T SumofElements_o0;             /* '<S368>/Sum of Elements' */
  real_T Product4_j;                   /* '<S362>/Product4' */
  real_T DataTypeConversion_ck;        /* '<S367>/Data Type Conversion' */
  real_T Product_l;                    /* '<S367>/Product' */
  real_T SumofElements_j;              /* '<S367>/Sum of Elements' */
  real_T DataTypeConversion_b;         /* '<S366>/Data Type Conversion' */
  real_T Product_g;                    /* '<S366>/Product' */
  real_T SumofElements_c;              /* '<S366>/Sum of Elements' */
  real_T Product5;                     /* '<S362>/Product5' */
  real_T Sum1;                         /* '<S362>/Sum1' */
  real_T Abs_d;                        /* '<S362>/Abs' */
  real_T DataTypeConversion_d;         /* '<S371>/Data Type Conversion' */
  real_T Product_i;                    /* '<S371>/Product' */
  real_T SumofElements_oi;             /* '<S371>/Sum of Elements' */
  real_T Product_lx;                   /* '<S362>/Product' */
  real_T DataTypeConversion_f;         /* '<S370>/Data Type Conversion' */
  real_T Product_e;                    /* '<S370>/Product' */
  real_T SumofElements_f;              /* '<S370>/Sum of Elements' */
  real_T DataTypeConversion_pi;        /* '<S369>/Data Type Conversion' */
  real_T Product_o;                    /* '<S369>/Product' */
  real_T SumofElements_h;              /* '<S369>/Sum of Elements' */
  real_T Product1;                     /* '<S362>/Product1' */
  real_T Sum;                          /* '<S362>/Sum' */
  real_T Product3_n;                   /* '<S373>/Product3' */
  real_T DataTypeConversion_p4;        /* '<S376>/Data Type Conversion' */
  real_T Product_l4;                   /* '<S376>/Product' */
  real_T SumofElements_p;              /* '<S376>/Sum of Elements' */
  real_T Add2;                         /* '<S372>/Add2' */
  real_T Product5_c;                   /* '<S373>/Product5' */
  real_T Add1;                         /* '<S373>/Add1' */
  real_T Product1_o;                   /* '<S373>/Product1' */
  real_T Sign1;                        /* '<S373>/Sign1' */
  real_T Product2_d;                   /* '<S373>/Product2' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[6];/* '<S350>/Suspension' */
  real_T DataTypeConversion_g;         /* '<S379>/Data Type Conversion' */
  real_T Product_gk;                   /* '<S379>/Product' */
  real_T SumofElements_dv;             /* '<S379>/Sum of Elements' */
  real_T Sum_l;                        /* '<S375>/Sum' */
  real_T Sum_p;                        /* '<S373>/Sum' */
  real_T VehicleForceSign;             /* '<S372>/Vehicle Force Sign' */
  real_T Selector1_c;                  /* '<S360>/Selector1' */
  real_T Selector_h;                   /* '<S360>/Selector' */
  real_T VehicleHeight;                /* '<S372>/Sign convention' */
  real_T Sum_f;                        /* '<S360>/Sum' */
  real_T Product_j;                    /* '<S360>/Product' */
  real_T UnaryMinus;                   /* '<S360>/Unary Minus' */
  real_T Selector2_g;                  /* '<S360>/Selector2' */
  real_T Product1_l;                   /* '<S360>/Product1' */
  real_T Reshape[3];                   /* '<S360>/Reshape' */
  real_T Selector3_k[3];               /* '<S360>/Selector3' */
  real_T Sum1_f[3];                    /* '<S360>/Sum1' */
  real_T Sum3;                         /* '<S362>/Sum3' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[3];/* '<S350>/Suspension' */
  real_T Selector3_i;                  /* '<S357>/Selector3' */
  boolean_T RelationalOperator;        /* '<S365>/Relational Operator' */
  boolean_T RelationalOperator_l;      /* '<S364>/Relational Operator' */
  boolean_T RelationalOperator_o;      /* '<S363>/Relational Operator' */
  boolean_T RelationalOperator_a;      /* '<S383>/Relational Operator' */
  boolean_T RelationalOperator_c;      /* '<S378>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S377>/Relational Operator' */
  boolean_T RelationalOperator_ag;     /* '<S368>/Relational Operator' */
  boolean_T RelationalOperator_l5;     /* '<S367>/Relational Operator' */
  boolean_T RelationalOperator_e;      /* '<S366>/Relational Operator' */
  boolean_T RelationalOperator_n;      /* '<S371>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S370>/Relational Operator' */
  boolean_T RelationalOperator_m;      /* '<S369>/Relational Operator' */
  boolean_T RelationalOperator_mm;     /* '<S376>/Relational Operator' */
  boolean_T RelationalOperator_nm;     /* '<S379>/Relational Operator' */
  B_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Maxstopreached;/* '<S375>/Max stop reached' */
  B_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Minstopreached;/* '<S375>/Min stop reached' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_n_T;

/* Block signals for system '<S415>/For Each Subsystem' */
typedef struct {
  real_T Product[3];                   /* '<S416>/Product' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_b_T;

/* Block signals for system '<S482>/Wheel to Body Transform' */
typedef struct {
  real_T TmpSignalConversionAtsincosInport1[3];/* '<S489>/In1' */
  real_T sincos_o1[3];                 /* '<S489>/sincos' */
  real_T sincos_o2[3];                 /* '<S489>/sincos' */
  real_T VectorConcatenate[9];         /* '<S490>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor[9];
                                /* '<S490>/Reshape (9) to [3x3] column-major' */
  real_T Divide1[3];                   /* '<S488>/Divide1' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_c_T;

/* Block signals for system '<S513>/Magic Tire Const Input' */
typedef struct {
  real_T Fx;                           /* '<S513>/Magic Tire Const Input' */
  real_T Fy;                           /* '<S513>/Magic Tire Const Input' */
  real_T FzTire;                       /* '<S513>/Magic Tire Const Input' */
  real_T My;                           /* '<S513>/Magic Tire Const Input' */
  real_T Re;                           /* '<S513>/Magic Tire Const Input' */
  real_T sig_x;                        /* '<S513>/Magic Tire Const Input' */
  real_T sig_y;                        /* '<S513>/Magic Tire Const Input' */
  real_T a;                            /* '<S513>/Magic Tire Const Input' */
  real_T b;                            /* '<S513>/Magic Tire Const Input' */
} B_MagicTireConstInput_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_no_T;

/* Block signals for system '<S515>/LockUp' */
typedef struct {
  real_T Tout;                         /* '<S515>/LockUp' */
  real_T Tfmaxs;                       /* '<S515>/LockUp' */
  real_T Tout_l;                       /* '<S515>/LockUp' */
  real_T Tfmaxs_b;                     /* '<S515>/LockUp' */
  real_T Omega;                        /* '<S515>/LockUp' */
  real_T Omegadot;                     /* '<S515>/LockUp' */
  real_T ReactionTorque;               /* '<S515>/LockUp' */
  real_T Myb;                          /* '<S515>/LockUp' */
  real_T omegawheel;                   /* '<S526>/omega wheel' */
  real_T u;                            /* '<S526>/-4' */
  real_T TrigonometricFunction;        /* '<S526>/Trigonometric Function' */
  real_T MaxDynamicFrictionTorque1;  /* '<S526>/Max Dynamic Friction Torque1' */
  real_T OutputDamping;                /* '<S526>/Output Damping' */
  real_T OutputSum;                    /* '<S526>/Output Sum' */
  real_T OutputInertia;                /* '<S526>/Output Inertia' */
  real_T OutputDamping_j;              /* '<S533>/Output Damping' */
  real_T Sum2;                         /* '<S533>/Sum2' */
  real_T Sum1;                         /* '<S533>/Sum1' */
  real_T Abs;                          /* '<S530>/Abs' */
  real_T UnaryMinus;                   /* '<S534>/Unary Minus' */
  real_T Abs_e;                        /* '<S535>/Abs' */
  real_T Abs_l;                        /* '<S536>/Abs' */
  boolean_T RelationalOperator;        /* '<S530>/Relational Operator' */
  boolean_T RelationalOperator_c;      /* '<S535>/Relational Operator' */
  boolean_T UnitDelay;                 /* '<S532>/Unit Delay' */
  boolean_T CombinatorialLogic;        /* '<S532>/Combinatorial  Logic' */
  boolean_T RelationalOperator_d;      /* '<S536>/Relational Operator' */
} B_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block states (default storage) for system '<S515>/LockUp' */
typedef struct {
  real_T lastMajorTime;                /* '<S515>/LockUp' */
  boolean_T UnitDelay_DSTATE;          /* '<S532>/Unit Delay' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c6_autolibshared;  /* '<S515>/LockUp' */
  uint8_T is_c6_autolibshared;         /* '<S515>/LockUp' */
} DW_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Continuous states for system '<S515>/LockUp' */
typedef struct {
  real_T omegaWheel_l3;                /* '<S526>/omega wheel' */
} X_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* State derivatives for system '<S515>/LockUp' */
typedef struct {
  real_T omegaWheel_l3;                /* '<S526>/omega wheel' */
} XDot_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* State Disabled for system '<S515>/LockUp' */
typedef struct {
  boolean_T omegaWheel_l3;             /* '<S526>/omega wheel' */
} XDis_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block signals for system '<S484>/Wheel to Body Transform' */
typedef struct {
  real_T TmpSignalConversionAtsincosInport1[3];/* '<S614>/In1' */
  real_T sincos_o1[3];                 /* '<S614>/sincos' */
  real_T sincos_o2[3];                 /* '<S614>/sincos' */
  real_T VectorConcatenate[9];         /* '<S615>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor[9];
                                /* '<S615>/Reshape (9) to [3x3] column-major' */
  real_T Divide1[3];                   /* '<S613>/Divide1' */
} B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_ca_T;

/* Block signals (default storage) */
typedef struct {
  int64m_T Gain;                       /* '<S101>/Gain' */
  int64m_T BrakePedal_CmdFinal_pct;    /* '<S98>/Gain1' */
  real_T ubvbwb[3];                    /* '<S387>/ub,vb,wb' */
  real_T UnitConversion[3];            /* '<S403>/Unit Conversion' */
  real_T VehSpdKph;                    /* '<S81>/Gain1' */
  real_T Reshape1;                     /* '<S310>/Reshape1' */
  real_T Memory1[2];                   /* '<S310>/Memory1' */
  real_T IntegratorSecondOrder_o1;     /* '<S514>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2;     /* '<S514>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o1_a;   /* '<S539>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_n;   /* '<S539>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o1_o;   /* '<S564>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_p;   /* '<S564>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o1_h;   /* '<S589>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_h;   /* '<S589>/Integrator, Second-Order' */
  real_T VectorConcatenate[4];         /* '<S510>/Vector Concatenate' */
  real_T z[4];                         /* '<S486>/Unary Minus1' */
  real_T z_k[4];                       /* '<S308>/Reshape' */
  real_T Reshape3[2];                  /* '<S304>/Reshape3' */
  real_T Switch[2];                    /* '<S310>/Switch' */
  real_T Reshape21[2];                 /* '<S310>/Reshape21' */
  real_T SumofElements;                /* '<S310>/Sum of Elements' */
  real_T MatrixConcatenate[3];         /* '<S310>/Matrix Concatenate' */
  real_T Reshape8[3];                  /* '<S304>/Reshape8' */
  real_T Sum[3];                       /* '<S310>/Sum' */
  real_T Reshape9[3];                  /* '<S304>/Reshape9' */
  real_T Reshape7[6];                  /* '<S304>/Reshape7' */
  real_T Selector[2];                  /* '<S304>/Selector' */
  real_T Reshape6[6];                  /* '<S304>/Reshape6' */
  real_T Selector1[2];                 /* '<S304>/Selector1' */
  real_T MatrixConcatenate1[4];        /* '<S304>/Matrix Concatenate1' */
  real_T xeyeze[3];                    /* '<S387>/xe,ye,ze' */
  real_T phithetapsi[3];               /* '<S399>/phi theta psi' */
  real_T TmpSignalConversionAtsincosInport1[3];/* '<S407>/In1' */
  real_T sincos_o1[3];                 /* '<S426>/sincos' */
  real_T sincos_o2[3];                 /* '<S426>/sincos' */
  real_T VectorConcatenate_l[9];       /* '<S430>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor[9];
                                /* '<S430>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1[9];                /* '<S425>/Transpose1' */
  real_T Selector1_p[3];               /* '<S420>/Selector1' */
  real_T Reshape1_n[3];                /* '<S428>/Reshape1' */
  real_T Product[3];                   /* '<S428>/Product' */
  real_T Reshape2[3];                  /* '<S428>/Reshape2' */
  real_T Add[3];                       /* '<S425>/Add' */
  real_T sincos_o1_h[3];               /* '<S434>/sincos' */
  real_T sincos_o2_g[3];               /* '<S434>/sincos' */
  real_T VectorConcatenate_p[9];       /* '<S438>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_m[9];
                                /* '<S438>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_p[9];              /* '<S433>/Transpose1' */
  real_T Selector1_h[3];               /* '<S421>/Selector1' */
  real_T Reshape1_k[3];                /* '<S436>/Reshape1' */
  real_T Product_a[3];                 /* '<S436>/Product' */
  real_T Reshape2_i[3];                /* '<S436>/Reshape2' */
  real_T Add_k[3];                     /* '<S433>/Add' */
  real_T sincos_o1_l[3];               /* '<S454>/sincos' */
  real_T sincos_o2_m[3];               /* '<S454>/sincos' */
  real_T VectorConcatenate_o[9];       /* '<S458>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_mx[9];
                                /* '<S458>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_f[9];              /* '<S453>/Transpose1' */
  real_T Selector1_c[3];               /* '<S423>/Selector1' */
  real_T Reshape1_b[3];                /* '<S456>/Reshape1' */
  real_T Product_aw[3];                /* '<S456>/Product' */
  real_T Reshape2_n[3];                /* '<S456>/Reshape2' */
  real_T Add_c[3];                     /* '<S453>/Add' */
  real_T sincos_o1_c[3];               /* '<S462>/sincos' */
  real_T sincos_o2_gs[3];              /* '<S462>/sincos' */
  real_T VectorConcatenate_n[9];       /* '<S466>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_k[9];
                                /* '<S466>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_fn[9];             /* '<S461>/Transpose1' */
  real_T Selector1_l[3];               /* '<S424>/Selector1' */
  real_T Reshape1_kj[3];               /* '<S464>/Reshape1' */
  real_T Product_e[3];                 /* '<S464>/Product' */
  real_T Reshape2_e[3];                /* '<S464>/Reshape2' */
  real_T Add_f[3];                     /* '<S461>/Add' */
  real_T Reshape[12];                  /* '<S307>/Reshape' */
  real_T P[12];                        /* '<S307>/Sum' */
  real_T Selector1_f[6];               /* '<S303>/Selector1' */
  real_T Selector3[2];                 /* '<S304>/Selector3' */
  real_T Reshape12[2];                 /* '<S304>/Reshape12' */
  real_T pqr[3];                       /* '<S387>/p,q,r ' */
  real_T jxk;                          /* '<S431>/j x k' */
  real_T kxi;                          /* '<S431>/k x i' */
  real_T ixj;                          /* '<S431>/i x j' */
  real_T kxj;                          /* '<S432>/k x j' */
  real_T ixk;                          /* '<S432>/i x k' */
  real_T jxi;                          /* '<S432>/j x i' */
  real_T Sum_j[3];                     /* '<S429>/Sum' */
  real_T Add1[3];                      /* '<S425>/Add1' */
  real_T jxk_f;                        /* '<S439>/j x k' */
  real_T kxi_g;                        /* '<S439>/k x i' */
  real_T ixj_o;                        /* '<S439>/i x j' */
  real_T kxj_b;                        /* '<S440>/k x j' */
  real_T ixk_k;                        /* '<S440>/i x k' */
  real_T jxi_b;                        /* '<S440>/j x i' */
  real_T Sum_k[3];                     /* '<S437>/Sum' */
  real_T Add1_m[3];                    /* '<S433>/Add1' */
  real_T jxk_g;                        /* '<S459>/j x k' */
  real_T kxi_k;                        /* '<S459>/k x i' */
  real_T ixj_b;                        /* '<S459>/i x j' */
  real_T kxj_d;                        /* '<S460>/k x j' */
  real_T ixk_c;                        /* '<S460>/i x k' */
  real_T jxi_j;                        /* '<S460>/j x i' */
  real_T Sum_h[3];                     /* '<S457>/Sum' */
  real_T Add1_b[3];                    /* '<S453>/Add1' */
  real_T jxk_gi;                       /* '<S467>/j x k' */
  real_T kxi_o;                        /* '<S467>/k x i' */
  real_T ixj_e;                        /* '<S467>/i x j' */
  real_T kxj_e;                        /* '<S468>/k x j' */
  real_T ixk_b;                        /* '<S468>/i x k' */
  real_T jxi_o;                        /* '<S468>/j x i' */
  real_T Sum_p[3];                     /* '<S465>/Sum' */
  real_T Add1_p[3];                    /* '<S461>/Add1' */
  real_T V[12];                        /* '<S307>/Reshape2' */
  real_T Selector18[6];                /* '<S303>/Selector18' */
  real_T Selector4[2];                 /* '<S304>/Selector4' */
  real_T Reshape13[2];                 /* '<S304>/Reshape13' */
  real_T MatrixConcatenate3[4];        /* '<S304>/Matrix Concatenate3' */
  real_T Integrator;                   /* '<S516>/Integrator' */
  real_T Integrator_l;                 /* '<S541>/Integrator' */
  real_T Integrator_f;                 /* '<S566>/Integrator' */
  real_T Integrator_b;                 /* '<S591>/Integrator' */
  real_T VectorConcatenate_nb[4];      /* '<S496>/Vector Concatenate' */
  real_T Integrator_fu;                /* '<S517>/Integrator' */
  real_T Integrator_n;                 /* '<S542>/Integrator' */
  real_T Integrator_e;                 /* '<S567>/Integrator' */
  real_T Integrator_d;                 /* '<S592>/Integrator' */
  real_T VectorConcatenate_pf[4];      /* '<S497>/Vector Concatenate' */
  real_T Integrator1[4];               /* '<S479>/Integrator1' */
  real_T Saturation[4];                /* '<S297>/Saturation' */
  real_T VectorConcatenate8[12];       /* '<S482>/Vector Concatenate8' */
  real_T MathFunction[12];             /* '<S482>/Math Function' */
  real_T Integrator1_d[12];            /* '<S480>/Integrator1' */
  real_T CamberAngles[4];              /* '<S297>/Selector3' */
  real_T CamberAngles_l[4];            /* '<S297>/Manual Switch6' */
  real_T Add2[4];                      /* '<S487>/Add2' */
  real_T Reshape1_i[4];                /* '<S487>/Reshape1' */
  real_T Reshape2_np[4];               /* '<S487>/Reshape2' */
  real_T WheelAngles[4];               /* '<S297>/Selector2' */
  real_T Add1_f[4];                    /* '<S487>/Add1' */
  real_T Reshape_e[4];                 /* '<S487>/Reshape' */
  real_T VectorConcatenate3[12];       /* '<S487>/Vector Concatenate3' */
  real_T Reshape6_m[4];                /* '<S308>/Reshape6' */
  real_T Reshape10[2];                 /* '<S304>/Reshape10' */
  real_T Reshape7_g[4];                /* '<S308>/Reshape7' */
  real_T Reshape11[2];                 /* '<S304>/Reshape11' */
  real_T Memory;                       /* '<S144>/Memory' */
  real_T Gain3;                        /* '<S144>/Gain3' */
  real_T T_FL;                         /* '<S144>/Gain4' */
  real_T VectorConcatenate_lv[4];      /* '<S144>/Vector Concatenate' */
  real_T Reshape4[4];                  /* '<S484>/Reshape4' */
  real_T Gain4[4];                     /* '<S484>/Gain4' */
  real_T Integrator_k;                 /* '<S519>/Integrator' */
  real_T VectorConcatenate_g[4];       /* '<S289>/Vector Concatenate' */
  real_T Sum6;                         /* '<S514>/Sum6' */
  real_T Saturation_h;                 /* '<S514>/Saturation' */
  real_T Add2_d;                       /* '<S515>/Add2' */
  real_T Product3;                     /* '<S515>/Product3' */
  real_T Add1_d;                       /* '<S515>/Add1' */
  real_T DecelCmd;                     /* '<S2>/Switch1' */
  real_T Saturation_f;                 /* '<S168>/Saturation' */
  real_T BrkTrqReqTotal;               /* '<S168>/Gain1' */
  real_T Memory1_b;                    /* '<S144>/Memory1' */
  real_T Saturation1;                  /* '<S184>/Saturation1' */
  real_T Divide;                       /* '<S184>/Divide' */
  real_T Saturation_n;                 /* '<S184>/Saturation' */
  real_T MotTrqMaxWhls;                /* '<S168>/MotTrqReflectedToWheels' */
  real_T min;                          /* '<S168>/MinMax' */
  real_T RegenBrakingCutoff;           /* '<S168>/RegenBrakingCutoff' */
  real_T Switch_l;                     /* '<S128>/Switch' */
  real_T IntegratorLimited;            /* '<S128>/Integrator Limited' */
  real_T Divide_a;                     /* '<S129>/Divide' */
  real_T ChrgLmt;                      /* '<S168>/ChrgLmt' */
  real_T RegenFactor;                  /* '<S168>/Product1' */
  real_T MotTrqRegenWhl;               /* '<S168>/Product3' */
  real_T Subtract;                     /* '<S168>/Subtract' */
  real_T BrkTrqReqTotal_o;             /* '<S168>/Gain2' */
  real_T Saturation1_g;                /* '<S168>/Saturation1' */
  real_T Gain_j;                       /* '<S290>/Gain' */
  real_T Gain2;                        /* '<S290>/Gain2' */
  real_T Gain3_n;                      /* '<S290>/Gain3' */
  real_T Gain1;                        /* '<S290>/Gain1' */
  real_T Gain4_f;                      /* '<S290>/Gain4' */
  real_T VectorConcatenate2[4];        /* '<S290>/Vector Concatenate2' */
  real_T Reshape3_b[4];                /* '<S484>/Reshape3' */
  real_T TorqueConversion1;            /* '<S524>/Torque Conversion1' */
  real_T product;                      /* '<S524>/product' */
  real_T DisallowNegativeBrakeTorque;
                                   /* '<S524>/Disallow Negative Brake Torque' */
  real_T TorqueConversion;             /* '<S524>/Torque Conversion' */
  real_T Ratioofstatictokinetic;       /* '<S521>/Ratio of static to kinetic' */
  real_T Selector11[6];                /* '<S303>/Selector11' */
  real_T Selector4_b[2];               /* '<S309>/Selector4' */
  real_T Reshape15[2];                 /* '<S309>/Reshape15' */
  real_T Selector5[2];                 /* '<S309>/Selector5' */
  real_T Reshape13_j[2];               /* '<S309>/Reshape13' */
  real_T VectorConcatenate_m[4];       /* '<S511>/Vector Concatenate' */
  real_T zdot[4];                      /* '<S486>/Unary Minus2' */
  real_T zdot_o[4];                    /* '<S308>/Reshape2' */
  real_T Reshape8_f[2];                /* '<S309>/Reshape8' */
  real_T MatrixConcatenate6[6];        /* '<S309>/Matrix Concatenate6' */
  real_T Selector7[2];                 /* '<S304>/Selector7' */
  real_T Selector8[2];                 /* '<S304>/Selector8' */
  real_T Reshape15_o[2];               /* '<S304>/Reshape15' */
  real_T MatrixConcatenate5[6];        /* '<S304>/Matrix Concatenate5' */
  real_T Reshape5[6];                  /* '<S304>/Reshape5' */
  real_T Sum2[6];                      /* '<S304>/Sum2' */
  real_T xdot[4];                      /* '<S303>/Matrix Concatenate1' */
  real_T ydot[4];                      /* '<S303>/Matrix Concatenate' */
  real_T Reshape1_o[4];                /* '<S486>/Reshape1' */
  real_T MatrixConcatenate_j[12];      /* '<S297>/Matrix Concatenate' */
  real_T Add2_f[4];                    /* '<S612>/Add2' */
  real_T Reshape1_ic[4];               /* '<S612>/Reshape1' */
  real_T Reshape2_p[4];                /* '<S612>/Reshape2' */
  real_T Add1_h[4];                    /* '<S612>/Add1' */
  real_T Reshape_p[4];                 /* '<S612>/Reshape' */
  real_T VectorConcatenate3_p[12];     /* '<S612>/Vector Concatenate3' */
  real_T Reshape2_b[4];                /* '<S305>/Reshape2' */
  real_T MatrixConcatenate1_a[12];     /* '<S305>/Matrix Concatenate1' */
  real_T Reshape3_n[4];                /* '<S305>/Reshape3' */
  real_T Reshape4_g[4];                /* '<S305>/Reshape4' */
  real_T MatrixConcatenate_f[12];      /* '<S305>/Matrix Concatenate' */
  real_T AngVel[12];                   /* '<S305>/Add' */
  real_T Selector1_d[4];               /* '<S297>/Selector1' */
  real_T Reshape1_e[4];                /* '<S297>/Reshape1' */
  real_T UnaryMinus[4];                /* '<S484>/Unary Minus' */
  real_T VectorConcatenate1[4];        /* '<S289>/Vector Concatenate1' */
  real_T Reshape_i[4];                 /* '<S485>/Reshape' */
  real_T Reshape1_j[4];                /* '<S485>/Reshape1' */
  real_T VectorConcatenate_pm[108];    /* '<S485>/Vector Concatenate' */
  real_T Selector9[27];                /* '<S491>/Selector9' */
  real_T Integrator_fo;                /* '<S544>/Integrator' */
  real_T Sum6_p;                       /* '<S539>/Sum6' */
  real_T Saturation_d;                 /* '<S539>/Saturation' */
  real_T Add2_l;                       /* '<S540>/Add2' */
  real_T Product3_h;                   /* '<S540>/Product3' */
  real_T Add1_g;                       /* '<S540>/Add1' */
  real_T TorqueConversion1_e;          /* '<S549>/Torque Conversion1' */
  real_T product_l;                    /* '<S549>/product' */
  real_T DisallowNegativeBrakeTorque_c;
                                   /* '<S549>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_p;           /* '<S549>/Torque Conversion' */
  real_T Ratioofstatictokinetic_n;     /* '<S546>/Ratio of static to kinetic' */
  real_T Selector19[27];               /* '<S491>/Selector19' */
  real_T Integrator_a;                 /* '<S569>/Integrator' */
  real_T Sum6_f;                       /* '<S564>/Sum6' */
  real_T Saturation_j;                 /* '<S564>/Saturation' */
  real_T Add2_e;                       /* '<S565>/Add2' */
  real_T Product3_l;                   /* '<S565>/Product3' */
  real_T Add1_k;                       /* '<S565>/Add1' */
  real_T TorqueConversion1_b;          /* '<S574>/Torque Conversion1' */
  real_T product_c;                    /* '<S574>/product' */
  real_T DisallowNegativeBrakeTorque_d;
                                   /* '<S574>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_e;           /* '<S574>/Torque Conversion' */
  real_T Ratioofstatictokinetic_c;     /* '<S571>/Ratio of static to kinetic' */
  real_T Selector29[27];               /* '<S491>/Selector29' */
  real_T Integrator_p;                 /* '<S594>/Integrator' */
  real_T Sum6_o;                       /* '<S589>/Sum6' */
  real_T Saturation_nz;                /* '<S589>/Saturation' */
  real_T Add2_dd;                      /* '<S590>/Add2' */
  real_T Product3_i;                   /* '<S590>/Product3' */
  real_T Add1_ge;                      /* '<S590>/Add1' */
  real_T TorqueConversion1_g;          /* '<S599>/Torque Conversion1' */
  real_T product_m;                    /* '<S599>/product' */
  real_T DisallowNegativeBrakeTorque_a;
                                   /* '<S599>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_n;           /* '<S599>/Torque Conversion' */
  real_T Ratioofstatictokinetic_b;     /* '<S596>/Ratio of static to kinetic' */
  real_T Selector39[27];               /* '<S491>/Selector39' */
  real_T VectorConcatenate_md[4];      /* '<S502>/Vector Concatenate' */
  real_T Re[4];                        /* '<S308>/Reshape1' */
  real_T Reshape4_j[2];                /* '<S304>/Reshape4' */
  real_T VectorConcatenate_j[4];       /* '<S499>/Vector Concatenate' */
  real_T Reshape3_c[4];                /* '<S308>/Reshape3' */
  real_T VectorConcatenate_g3[4];      /* '<S500>/Vector Concatenate' */
  real_T Reshape4_e[4];                /* '<S308>/Reshape4' */
  real_T VectorConcatenate_b[4];       /* '<S501>/Vector Concatenate' */
  real_T Reshape5_j[4];                /* '<S308>/Reshape5' */
  real_T M[12];                        /* '<S308>/Matrix Concatenate' */
  real_T Selector17[6];                /* '<S303>/Selector17' */
  real_T UnitConversion_k[3];          /* '<S405>/Unit Conversion' */
  real_T PowertrainData_10_HS1Time;    /* '<S1>/Data Inport S-Fcn' */
  real_T PowertrainData_10_HS1DeltaTime;/* '<S1>/Data Inport S-Fcn' */
  real_T WheelSpeed_HS1Time;           /* '<S1>/Data Inport S-Fcn' */
  real_T WheelSpeed_HS1DeltaTime;      /* '<S1>/Data Inport S-Fcn' */
  real_T WhlRr_W_MeasValue;            /* '<S1>/Data Inport S-Fcn' */
  real_T WhlRl_W_MeasValue;            /* '<S1>/Data Inport S-Fcn' */
  real_T WhlFr_W_MeasValue;            /* '<S1>/Data Inport S-Fcn' */
  real_T WhlFl_W_MeasValue;            /* '<S1>/Data Inport S-Fcn' */
  real_T BrakeSysFeatures_HS1Time;     /* '<S1>/Data Inport S-Fcn' */
  real_T BrakeSysFeatures_HS1DeltaTime;/* '<S1>/Data Inport S-Fcn' */
  real_T Veh_V_ActlBrkValue;           /* '<S1>/Data Inport S-Fcn' */
  real_T EngVehicleSpThrottle_HS1Time; /* '<S1>/Data Inport S-Fcn' */
  real_T EngVehicleSpThrottle_HS1DeltaTime;/* '<S1>/Data Inport S-Fcn' */
  real_T EngAout3_N_ActlValue;         /* '<S1>/Data Inport S-Fcn' */
  real_T ApedPos_Pc_ActlArbValue;      /* '<S1>/Data Inport S-Fcn' */
  real_T BrakeSnData_4_HS1Time;        /* '<S1>/Data Inport S-Fcn' */
  real_T BrakeSnData_4_HS1DeltaTime;   /* '<S1>/Data Inport S-Fcn' */
  real_T BattTrac_U2_ActlValue;        /* '<S1>/Data Inport S-Fcn' */
  real_T BattTrac_I2_ActlValue;        /* '<S1>/Data Inport S-Fcn' */
  real_T VectorConcatenate_p1[4];      /* '<S2>/Vector Concatenate' */
  real_T EnrgyTrans;                   /* '<S86>/Integrator' */
  real_T UnitConversion_c;             /* '<S86>/Unit Conversion' */
  real_T Constant;                     /* '<S2>/Constant' */
  real_T GearCmd;                      /* '<S2>/Constant2' */
  real_T CltchCmd;                     /* '<S2>/Constant3' */
  real_T lgSw;                         /* '<S2>/Constant4' */
  real_T Switch2;                      /* '<S2>/Switch2' */
  real_T SteeringCmd;                  /* '<S2>/EV Bolt Steer' */
  real_T Constant_i;                   /* '<S83>/Constant' */
  real_T Constant1;                    /* '<S83>/Constant1' */
  real_T VectorConcatenate_jq[3];      /* '<S83>/Vector Concatenate' */
  real_T AccelCmd;                     /* '<S2>/Step' */
  real_T AccelCmd_b;                   /* '<S2>/Switch' */
  real_T MultiportSwitch2;             /* '<S87>/Multiport Switch2' */
  real_T BrakePedalCmd_pct;            /* '<S87>/BR_limit' */
  real_T BrakePressed_flag;            /* '<S81>/Data Type Conversion1' */
  real_T omegawheel;                   /* '<S249>/omega wheel' */
  real_T omegawheel_m;                 /* '<S250>/omega wheel' */
  real_T omegawheel_c;                 /* '<S251>/omega wheel' */
  real_T omegawheel_k;                 /* '<S252>/omega wheel' */
  real_T Saturation_n3[4];             /* '<S192>/Saturation' */
  real_T rads2rpm[4];                  /* '<S195>/rads2rpm' */
  real_T MinMax;                       /* '<S267>/MinMax' */
  real_T Divide1;                      /* '<S267>/Divide1' */
  real_T Product_d;                    /* '<S267>/Product' */
  real_T Sum3;                         /* '<S267>/Sum3' */
  real_T Memory_g;                     /* '<S267>/Memory' */
  real_T Product2;                     /* '<S267>/Product2' */
  real_T Add_o;                        /* '<S267>/Add' */
  real_T LF;                           /* '<S193>/Data Type Conversion1' */
  real_T MinMax_l;                     /* '<S268>/MinMax' */
  real_T Divide1_f;                    /* '<S268>/Divide1' */
  real_T Product_i;                    /* '<S268>/Product' */
  real_T Sum3_f;                       /* '<S268>/Sum3' */
  real_T Memory_o;                     /* '<S268>/Memory' */
  real_T Product2_i;                   /* '<S268>/Product2' */
  real_T Add_l;                        /* '<S268>/Add' */
  real_T RF;                           /* '<S193>/Data Type Conversion2' */
  real_T MinMax_d;                     /* '<S269>/MinMax' */
  real_T Divide1_d;                    /* '<S269>/Divide1' */
  real_T Product_n;                    /* '<S269>/Product' */
  real_T Sum3_j;                       /* '<S269>/Sum3' */
  real_T Memory_d;                     /* '<S269>/Memory' */
  real_T Product2_l;                   /* '<S269>/Product2' */
  real_T Add_e;                        /* '<S269>/Add' */
  real_T LR;                           /* '<S193>/Data Type Conversion3' */
  real_T MinMax_i;                     /* '<S270>/MinMax' */
  real_T Divide1_n;                    /* '<S270>/Divide1' */
  real_T Product_h;                    /* '<S270>/Product' */
  real_T Sum3_n;                       /* '<S270>/Sum3' */
  real_T Memory_p;                     /* '<S270>/Memory' */
  real_T Product2_o;                   /* '<S270>/Product2' */
  real_T Add_h;                        /* '<S270>/Add' */
  real_T RR;                           /* '<S193>/Data Type Conversion4' */
  real_T SpeedOut[4];                  /* '<S195>/SpeedOut' */
  real_T RateLimSpd[4];                /* '<S195>/RateLimSpd' */
  real_T Spd[4];                       /* '<S195>/SatSpd' */
  real_T ModeSwitch[4];                /* '<S195>/ModeSwitch' */
  real_T ModeOut[4];                   /* '<S195>/ModeOut' */
  real_T DataTypeConversion1;          /* '<S100>/Data Type Conversion1' */
  real_T Memory2;                      /* '<S108>/Memory2' */
  real_T Integrator_j;                 /* '<S113>/Integrator' */
  real_T Memory_c;                     /* '<S117>/Memory' */
  real_T Sum3_a;                       /* '<S117>/Sum3' */
  real_T Memory_l;                     /* '<S118>/Memory' */
  real_T Sum3_o;                       /* '<S118>/Sum3' */
  real_T StatusWord;                   /* '<S101>/Data Type Conversion' */
  real_T ActualPosition_cnt;           /* '<S101>/Data Type Conversion1' */
  real_T SineWave;                     /* '<S98>/Sine Wave' */
  real_T RateLimiter;                  /* '<S98>/Rate Limiter' */
  real_T Switch_a;                     /* '<S98>/Switch' */
  real_T bped2br_N;                    /* '<S87>/bped2br_N' */
  real_T Switch2_c;                    /* '<S98>/Switch2' */
  real_T BrakePedal_TargetlPosition_pct;/* '<S98>/Gain' */
  real_T Memory_j;                     /* '<S101>/Memory' */
  real_T Memory2_j;                    /* '<S101>/Memory2' */
  real_T Memory1_p;                    /* '<S101>/Memory1' */
  real_T Sum_hq;                       /* '<S101>/Sum' */
  real_T Product_iq;                   /* '<S101>/Product' */
  real_T TargetPosition_cnt;           /* '<S101>/Sum1' */
  real_T Switch1;                      /* '<S98>/Switch1' */
  real_T TargetPositionFinal_cnt;      /* '<S101>/Gain2' */
  real_T Sum2_o;                       /* '<S101>/Sum2' */
  real_T ActualPosition_pct;           /* '<S101>/Product1' */
  real_T RateLimiter_d;                /* '<S99>/Rate Limiter' */
  real_T MultiportSwitch1;             /* '<S87>/Multiport Switch1' */
  real_T AccelPedalCmd_pct;            /* '<S87>/AR_limit' */
  real_T APP_CmdFinal_pct;             /* '<S99>/Switch2' */
  real_T APP1Tbl;                      /* '<S99>/APP1Tbl' */
  real_T Gain3_k;                      /* '<S99>/Gain3' */
  real_T Product2_lr;                  /* '<S99>/Product2' */
  real_T APP1_CmdFinal_V;              /* '<S99>/Gain1' */
  real_T APP2Tbl;                      /* '<S99>/APP2Tbl' */
  real_T Gain4_ff;                     /* '<S99>/Gain4' */
  real_T Product3_b;                   /* '<S99>/Product3' */
  real_T APP2_CmdFinal_V;              /* '<S99>/Gain' */
  real_T AccelPedal_CmdFinal_pct;      /* '<S99>/Gain2' */
  real_T Curr;                         /* '<S92>/Memory' */
  real_T Gain1_i;                      /* '<S128>/Gain1' */
  real_T Em;                           /* '<S130>/Em' */
  real_T R;                            /* '<S130>/R' */
  real_T Gain2_i;                      /* '<S130>/Gain2' */
  real_T Product_io;                   /* '<S130>/Product' */
  real_T Subtract_h;                   /* '<S130>/Subtract' */
  real_T Gain1_c;                      /* '<S130>/Gain1' */
  real_T Product1;                     /* '<S130>/Product1' */
  real_T Gain3_o;                      /* '<S130>/Gain3' */
  real_T Gain4_i;                      /* '<S130>/Gain4' */
  real_T Product_b;                    /* '<S125>/Product' */
  real_T Gain_d;                       /* '<S125>/Gain' */
  real_T Gain1_e;                      /* '<S125>/Gain1' */
  real_T Add_j;                        /* '<S125>/Add' */
  real_T mph2ms;                       /* '<S96>/mph2m//s' */
  real_T utireRadius;                  /* '<S96>/1//tireRadius' */
  real_T MultiportSwitch1_c[4];        /* '<S96>/Multiport Switch1' */
  real_T SteerAngleCAN;                /* '<S81>/Data Type Conversion' */
  real_T AxlTrqLump;                   /* '<S144>/Add' */
  real_T VectorConcatenate_h[4];       /* '<S509>/Vector Concatenate' */
  real_T VectorConcatenate_c[4];       /* '<S81>/Vector Concatenate' */
  real_T MultiportSwitch[4];           /* '<S81>/Multiport Switch' */
  real_T Add1_j;                       /* '<S144>/Add1' */
  real_T Gain1_k;                      /* '<S144>/Gain1' */
  real_T Spd_e;                        /* '<S144>/Gain2' */
  real_T Abs;                          /* '<S153>/Abs' */
  real_T Divide_n;                     /* '<S153>/Divide' */
  real_T Integrator_h;                 /* '<S153>/Integrator' */
  real_T Gain1_d;                      /* '<S153>/Gain1' */
  real_T Merge;                        /* '<S153>/Merge' */
  real_T TransferredPower;             /* '<S145>/Transferred Power' */
  real_T PwrMechLoss;                  /* '<S145>/Constant' */
  real_T PwrDampLoss;                  /* '<S145>/Constant1' */
  real_T PwrStoredShft;                /* '<S145>/Constant2' */
  real_T VectorConcatenate_e[3];       /* '<S145>/Vector Concatenate' */
  real_T Product_o;                    /* '<S155>/Product' */
  real_T uDLookupTable;                /* '<S155>/2-D Lookup Table' */
  real_T Add_d;                        /* '<S152>/Add' */
  real_T Saturation_g;                 /* '<S152>/Saturation' */
  real_T Divide_f;                     /* '<S152>/Divide' */
  real_T Sum_m;                        /* '<S153>/Sum' */
  real_T Add_p;                        /* '<S154>/Add' */
  real_T Gain_k;                       /* '<S154>/Gain' */
  real_T Gain1_l;                      /* '<S154>/Gain1' */
  real_T Subtract_i;                   /* '<S154>/Subtract' */
  real_T Saturation1_a;                /* '<S169>/Saturation1' */
  real_T Divide_l;                     /* '<S169>/Divide' */
  real_T Saturation_h4;                /* '<S169>/Saturation' */
  real_T Product1_g;                   /* '<S165>/Product1' */
  real_T ChrgLmt_c;                    /* '<S166>/ChrgLmt' */
  real_T DischrgLmt;                   /* '<S166>/DischrgLmt' */
  real_T Product_f;                    /* '<S166>/Product' */
  real_T Product1_h;                   /* '<S166>/Product1' */
  real_T WhlTrqReflectedToMot;         /* '<S168>/WhlTrqReflectedToMot' */
  real_T MotTrqCmdRegen;               /* '<S168>/Gain' */
  real_T AccelDecelSwitch;             /* '<S167>/Accel Decel Switch' */
  real_T Abs_l;                        /* '<S171>/Abs' */
  real_T Product3_f;                   /* '<S174>/Product3' */
  real_T rads_to_rpm;                  /* '<S174>/rads_to_rpm' */
  real_T Abs_f;                        /* '<S174>/Abs' */
  real_T Abs1;                         /* '<S174>/Abs1' */
  real_T EffMap;                       /* '<S174>/Eff Map' */
  real_T Gain1_h;                      /* '<S174>/Gain1' */
  real_T Product_o4;                   /* '<S174>/Product' */
  real_T Switch2_d;                    /* '<S174>/Switch2' */
  real_T MathFunction_p;               /* '<S174>/Math Function' */
  real_T Product4;                     /* '<S174>/Product4' */
  real_T Subtract_e;                   /* '<S172>/Subtract' */
  real_T Subtract1;                    /* '<S172>/Subtract1' */
  real_T Switch_b;                     /* '<S179>/Switch' */
  real_T Switch2_h;                    /* '<S179>/Switch2' */
  real_T ElecToMechPwr;                /* '<S171>/ElecToMechPwr' */
  real_T UnaryMinus_e;                 /* '<S176>/Unary Minus' */
  real_T Switch1_f;                    /* '<S176>/Switch1' */
  real_T Fcn;                          /* '<S176>/Fcn' */
  real_T Product_dc;                   /* '<S176>/Product' */
  real_T Switch_bq;                    /* '<S176>/Switch' */
  real_T MechPwrToTrq;                 /* '<S171>/MechPwrToTrq' */
  real_T Switch_n;                     /* '<S171>/Switch' */
  real_T Switch1_n;                    /* '<S171>/Switch1' */
  real_T Saturation1_d;                /* '<S180>/Saturation1' */
  real_T Divide_o;                     /* '<S180>/Divide' */
  real_T Saturation_l;                 /* '<S180>/Saturation' */
  real_T Gain_j4;                      /* '<S175>/Gain' */
  real_T Switch_bu;                    /* '<S181>/Switch' */
  real_T Switch2_k;                    /* '<S181>/Switch2' */
  real_T LinkSpeed;                    /* '<S185>/Data Inport S-Fcn' */
  real_T Constant_j;                   /* '<S96>/Constant' */
  real_T APPTorque;                    /* '<S189>/APPToTorque' */
  real_T APPToTorque1;                 /* '<S189>/APPToTorque1' */
  real_T BPPTorque;                    /* '<S189>/Flip' */
  real_T TorqueTotal;                  /* '<S189>/Sum' */
  real_T Speed_mps[4];                 /* '<S189>/WheelRadius' */
  real_T Speed_kph[4];                 /* '<S189>/mps2kph' */
  real_T Speed_mph[4];                 /* '<S189>/kph2mph' */
  real_T Gear[4];                      /* '<S189>/NissanTrans' */
  real_T TransTorqueOut[4];            /* '<S189>/Gear' */
  real_T WheelTorqueOut[4];            /* '<S189>/FDR' */
  real_T Memory_m[4];                  /* '<S189>/Memory' */
  real_T PTWFLrot;                     /* '<S190>/Discrete-Time Integrator4' */
  real_T PTWFRrot;                     /* '<S190>/Discrete-Time Integrator5' */
  real_T PTWRLrot;                     /* '<S190>/Discrete-Time Integrator6' */
  real_T PtTWRRrot;                    /* '<S190>/Discrete-Time Integrator7' */
  real_T Memory3;                      /* '<S190>/Memory3' */
  real_T Saturation3;                  /* '<S190>/Saturation3' */
  real_T Product3_j;                   /* '<S190>/Product3' */
  real_T Gain5;                        /* '<S190>/Gain5' */
  real_T Memory1_a;                    /* '<S190>/Memory1' */
  real_T Saturation1_i;                /* '<S190>/Saturation1' */
  real_T Product1_i;                   /* '<S190>/Product1' */
  real_T Gain6;                        /* '<S190>/Gain6' */
  real_T Memory2_o;                    /* '<S190>/Memory2' */
  real_T Saturation2;                  /* '<S190>/Saturation2' */
  real_T Product2_f;                   /* '<S190>/Product2' */
  real_T Gain7;                        /* '<S190>/Gain7' */
  real_T Memory4;                      /* '<S190>/Memory4' */
  real_T Saturation4;                  /* '<S190>/Saturation4' */
  real_T Product4_d;                   /* '<S190>/Product4' */
  real_T Gain8;                        /* '<S190>/Gain8' */
  real_T PTIgnition;                   /* '<S190>/Ignition' */
  real_T OperationError;               /* '<S190>/Operation Error' */
  real_T OperationStateDriving;        /* '<S190>/Operation State Driving' */
  real_T Zero1;                        /* '<S190>/Zero1' */
  real_T Zero2;                        /* '<S190>/Zero2' */
  real_T Zero3;                        /* '<S190>/Zero3' */
  real_T Zero4;                        /* '<S190>/Zero4' */
  real_T Zero5;                        /* '<S190>/Zero5' */
  real_T PTWFLrot_e;                   /* '<S247>/Discrete-Time Integrator4' */
  real_T PTWFRrot_j;                   /* '<S247>/Discrete-Time Integrator5' */
  real_T PTWRLrot_l;                   /* '<S247>/Discrete-Time Integrator6' */
  real_T PtTWRRrot_o;                  /* '<S247>/Discrete-Time Integrator7' */
  real_T Memory3_i;                    /* '<S247>/Memory3' */
  real_T Saturation3_g;                /* '<S247>/Saturation3' */
  real_T BrakeTrqFL;                   /* '<S247>/Product3' */
  real_T Gain5_j;                      /* '<S247>/Gain5' */
  real_T Memory1_o;                    /* '<S247>/Memory1' */
  real_T Saturation1_av;               /* '<S247>/Saturation1' */
  real_T BrakeTrqFR;                   /* '<S247>/Product1' */
  real_T Gain6_p;                      /* '<S247>/Gain6' */
  real_T Memory2_f;                    /* '<S247>/Memory2' */
  real_T Saturation2_m;                /* '<S247>/Saturation2' */
  real_T BrakeTrqRL;                   /* '<S247>/Product2' */
  real_T Gain7_g;                      /* '<S247>/Gain7' */
  real_T Memory4_f;                    /* '<S247>/Memory4' */
  real_T Saturation4_g;                /* '<S247>/Saturation4' */
  real_T BrakeTrqRR;                   /* '<S247>/Product4' */
  real_T Gain8_a;                      /* '<S247>/Gain8' */
  real_T PTIgnition_b;                 /* '<S247>/Ignition' */
  real_T OperationError_j;             /* '<S247>/Operation Error' */
  real_T OperationStateDriving_g;      /* '<S247>/Operation State Driving' */
  real_T TorqueIn[4];                  /* '<S193>/TorqueIn' */
  real_T SatTrq[4];                    /* '<S193>/SatTrq' */
  real_T Trq[4];                       /* '<S193>/TorqueIn ' */
  real_T MultiportSwitch_k[4];         /* '<S96>/Multiport Switch' */
  real_T IntegratorSecondOrder_o1_d;   /* '<S503>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_b;   /* '<S503>/Integrator, Second-Order' */
  real_T Sum6_a;                       /* '<S503>/Sum6' */
  real_T Saturation_m;                 /* '<S503>/Saturation' */
  real_T Add2_c;                       /* '<S249>/Add2' */
  real_T Saturation_e;                 /* '<S249>/Saturation' */
  real_T Product3_e;                   /* '<S249>/Product3' */
  real_T Add1_gej;                     /* '<S249>/Add1' */
  real_T Memory_p1;                    /* '<S249>/Memory' */
  real_T OutputDamping;                /* '<S249>/Output Damping' */
  real_T Switch_bk;                    /* '<S255>/Switch' */
  real_T Divide_m;                     /* '<S255>/Divide' */
  real_T Sum_i;                        /* '<S249>/Sum' */
  real_T Sum3_e;                       /* '<S249>/Sum3' */
  real_T Switch_aq;                    /* '<S249>/Switch' */
  real_T Switch_c;                     /* '<S254>/Switch' */
  real_T Divide_i;                     /* '<S254>/Divide' */
  real_T rotvFL;                       /* '<S249>/Discrete-Time Integrator' */
  real_T IntegratorSecondOrder_o1_n;   /* '<S504>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_g;   /* '<S504>/Integrator, Second-Order' */
  real_T Sum6_k;                       /* '<S504>/Sum6' */
  real_T Saturation_b;                 /* '<S504>/Saturation' */
  real_T Add2_fm;                      /* '<S250>/Add2' */
  real_T Saturation_bu;                /* '<S250>/Saturation' */
  real_T Product3_k;                   /* '<S250>/Product3' */
  real_T Add1_h5;                      /* '<S250>/Add1' */
  real_T Memory_gb;                    /* '<S250>/Memory' */
  real_T Switch_bc;                    /* '<S258>/Switch' */
  real_T Divide_n4;                    /* '<S258>/Divide' */
  real_T OutputDamping_b;              /* '<S250>/Output Damping' */
  real_T Sum3_na;                      /* '<S250>/Sum3' */
  real_T Switch_ag;                    /* '<S250>/Switch' */
  real_T Switch_d;                     /* '<S257>/Switch' */
  real_T Divide_c;                     /* '<S257>/Divide' */
  real_T rotvFL_f;                     /* '<S250>/Discrete-Time Integrator' */
  real_T IntegratorSecondOrder_o1_o4;  /* '<S505>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_d;   /* '<S505>/Integrator, Second-Order' */
  real_T Sum6_k0;                      /* '<S505>/Sum6' */
  real_T Saturation_i;                 /* '<S505>/Saturation' */
  real_T Add2_k;                       /* '<S251>/Add2' */
  real_T Saturation_p;                 /* '<S251>/Saturation' */
  real_T Product3_l3;                  /* '<S251>/Product3' */
  real_T Add1_dm;                      /* '<S251>/Add1' */
  real_T OutputDamping_g;              /* '<S251>/Output Damping' */
  real_T Switch_j;                     /* '<S261>/Switch' */
  real_T Divide_p;                     /* '<S261>/Divide' */
  real_T Sum_p1;                       /* '<S251>/Sum' */
  real_T Sum3_l;                       /* '<S251>/Sum3' */
  real_T Switch_c4;                    /* '<S251>/Switch' */
  real_T Switch_h;                     /* '<S260>/Switch' */
  real_T Divide_k;                     /* '<S260>/Divide' */
  real_T rotvFL_a;                     /* '<S251>/Discrete-Time Integrator' */
  real_T Memory_b;                     /* '<S251>/Memory' */
  real_T IntegratorSecondOrder_o1_p;   /* '<S506>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_hy;  /* '<S506>/Integrator, Second-Order' */
  real_T Sum6_fh;                      /* '<S506>/Sum6' */
  real_T Saturation_o;                 /* '<S506>/Saturation' */
  real_T Add2_ci;                      /* '<S252>/Add2' */
  real_T Saturation_mk;                /* '<S252>/Saturation' */
  real_T Product3_l3n;                 /* '<S252>/Product3' */
  real_T Add1_pn;                      /* '<S252>/Add1' */
  real_T Switch_o;                     /* '<S264>/Switch' */
  real_T Divide_cn;                    /* '<S264>/Divide' */
  real_T Gain_a;                       /* '<S252>/Gain' */
  real_T Sum3_i;                       /* '<S252>/Sum3' */
  real_T Switch_m;                     /* '<S252>/Switch' */
  real_T Switch_hv;                    /* '<S263>/Switch' */
  real_T Divide_c2;                    /* '<S263>/Divide' */
  real_T rotvFL_o;                     /* '<S252>/Discrete-Time Integrator' */
  real_T Memory_mu;                    /* '<S252>/Memory' */
  real_T Zero1_c;                      /* '<S247>/Zero1' */
  real_T Zero2_p;                      /* '<S247>/Zero2' */
  real_T Zero3_j;                      /* '<S247>/Zero3' */
  real_T Zero4_m;                      /* '<S247>/Zero4' */
  real_T Zero5_a;                      /* '<S247>/Zero5' */
  real_T DiffTrq;                      /* '<S248>/Product' */
  real_T Gain10;                       /* '<S248>/Gain10' */
  real_T Saturation1_b;                /* '<S192>/Saturation1' */
  real_T BrakeIFTrq_Reg_trgRR;         /* '<S194>/Zero1' */
  real_T BrakeIFTrq_DriveSrc_trgd3;    /* '<S194>/Zero2' */
  real_T BrakeIFTrq_PBRR;              /* '<S194>/Zero5' */
  real_T BrkLatchThr_Nm;               /* '<S271>/BrkLatchThr_Nm' */
  real_T SpTqRF;                       /* '<S281>/Bit2' */
  real_T Backlash;                     /* '<S299>/Backlash' */
  real_T PwrLoss;                      /* '<S299>/Constant' */
  real_T InstStrgRatio;                /* '<S300>/Constant' */
  real_T Saturation_oi;                /* '<S299>/Saturation' */
  real_T Gain_e;                       /* '<S300>/Gain' */
  real_T Gain1_g;                      /* '<S300>/Gain1' */
  real_T Add_b;                        /* '<S301>/Add' */
  real_T TrqIn;                        /* '<S300>/Gain2' */
  real_T TrqIn_p;                      /* '<S299>/Unary Minus' */
  real_T UnaryMinus1;                  /* '<S298>/Unary Minus1' */
  real_T UnaryMinus_k;                 /* '<S298>/Unary Minus' */
  real_T VectorConcatenate2_b[4];      /* '<S291>/Vector Concatenate2' */
  real_T Reshape3_i[2];                /* '<S309>/Reshape3' */
  real_T Reshape4_e0[2];               /* '<S309>/Reshape4' */
  real_T MatrixConcatenate4[4];        /* '<S309>/Matrix Concatenate4' */
  real_T Reshape9_d[2];                /* '<S309>/Reshape9' */
  real_T MatrixConcatenate2[4];        /* '<S309>/Matrix Concatenate2' */
  real_T Selector10[6];                /* '<S303>/Selector10' */
  real_T Selector_d[2];                /* '<S309>/Selector' */
  real_T Reshape7_p[2];                /* '<S309>/Reshape7' */
  real_T Selector1_fo[2];              /* '<S309>/Selector1' */
  real_T Reshape6_l[2];                /* '<S309>/Reshape6' */
  real_T MatrixConcatenate3_o[4];      /* '<S309>/Matrix Concatenate3' */
  real_T Reshape1_a[2];                /* '<S306>/Reshape1' */
  real_T Reshape17[2];                 /* '<S309>/Reshape17' */
  real_T Selector9_l[6];               /* '<S303>/Selector9' */
  real_T Reshape18[6];                 /* '<S309>/Reshape18' */
  real_T Reshape19[2];                 /* '<S309>/Reshape19' */
  real_T AssignWhlFz[2];               /* '<S351>/Assign WhlFz' */
  real_T Reshape2_m[2];                /* '<S309>/Reshape2' */
  real_T MatrixConcatenate1_p[6];      /* '<S309>/Matrix Concatenate1' */
  real_T MatrixConcatenate2_h[4];      /* '<S304>/Matrix Concatenate2' */
  real_T Sum_n[2];                     /* '<S314>/Sum' */
  real_T CarriertoAxleCompliance[2];   /* '<S314>/Carrier to Axle Compliance' */
  real_T Reshape2_ms[2];               /* '<S304>/Reshape2' */
  real_T Sum2_h[2];                    /* '<S314>/Sum2' */
  real_T CarriertoAxleDamping[2];      /* '<S314>/Carrier to Axle Damping' */
  real_T Sum1[2];                      /* '<S314>/Sum1' */
  real_T Gain_g[2];                    /* '<S304>/Gain' */
  real_T MatrixConcatenate6_c[6];      /* '<S304>/Matrix Concatenate6' */
  real_T Reshape14[6];                 /* '<S309>/Reshape14' */
  real_T Reshape18_p[6];               /* '<S304>/Reshape18' */
  real_T AssignVehFz[2];               /* '<S351>/Assign VehFz' */
  real_T Reshape1_g[2];                /* '<S309>/Reshape1' */
  real_T MatrixConcatenate_fi[6];      /* '<S309>/Matrix Concatenate' */
  real_T Reshape1_h[2];                /* '<S304>/Reshape1' */
  real_T MatrixConcatenate_o[6];       /* '<S304>/Matrix Concatenate' */
  real_T Ang[12];                      /* '<S303>/Matrix Concatenate2' */
  real_T F[12];                        /* '<S303>/Matrix Concatenate4' */
  real_T M_i[12];                      /* '<S303>/Matrix Concatenate5' */
  real_T F_k[12];                      /* '<S303>/Matrix Concatenate6' */
  real_T Reshape_ps[12];               /* '<S304>/Reshape' */
  real_T Camberselect[2];              /* '<S304>/Camber select' */
  real_T Casterselect[2];              /* '<S304>/Caster select' */
  real_T Energyselect[2];              /* '<S304>/Energy select' */
  real_T Heightselect[2];              /* '<S304>/Height select' */
  real_T Powerselect[2];               /* '<S304>/Power select' */
  real_T Reshape16[6];                 /* '<S304>/Reshape16' */
  real_T Reshape17_m[6];               /* '<S304>/Reshape17' */
  real_T Reshape19_o[6];               /* '<S304>/Reshape19' */
  real_T Reshape20[6];                 /* '<S304>/Reshape20' */
  real_T Toeselect[2];                 /* '<S304>/Toe select' */
  real_T Reshape_m[12];                /* '<S309>/Reshape' */
  real_T Camberselect_m[2];            /* '<S309>/Camber select' */
  real_T Casterselect_l[2];            /* '<S309>/Caster select' */
  real_T Energyselect_p[2];            /* '<S309>/Energy select' */
  real_T Heightselect_l[2];            /* '<S309>/Height select' */
  real_T Powerselect_n[2];             /* '<S309>/Power select' */
  real_T Reshape10_g[6];               /* '<S309>/Reshape10' */
  real_T Reshape11_g[6];               /* '<S309>/Reshape11' */
  real_T Reshape12_b[6];               /* '<S309>/Reshape12' */
  real_T Reshape16_i[6];               /* '<S309>/Reshape16' */
  real_T Toeselect_h[2];               /* '<S309>/Toe select' */
  real_T sincos_o1_a[3];               /* '<S407>/sincos' */
  real_T sincos_o2_p[3];               /* '<S407>/sincos' */
  real_T VectorConcatenate_b5[9];      /* '<S409>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_i[9];
                                /* '<S409>/Reshape (9) to [3x3] column-major' */
  real_T sincos_o1_o[3];               /* '<S408>/sincos' */
  real_T sincos_o2_gn[3];              /* '<S408>/sincos' */
  real_T phidot;                       /* '<S408>/phidot' */
  real_T thetadot;                     /* '<S408>/thetadot' */
  real_T psidot;                       /* '<S408>/psidot' */
  real_T TmpSignalConversionAtphithetapsiInport1[3];/* '<S399>/phidot thetadot psidot' */
  real_T MatrixConcatenation[18];      /* '<S401>/Matrix Concatenation' */
  real_T Selector_f[9];                /* '<S400>/Selector' */
  real_T Reshape1_ar[3];               /* '<S411>/Reshape1' */
  real_T Product_g[3];                 /* '<S411>/Product' */
  real_T Reshape2_f[3];                /* '<S411>/Reshape2' */
  real_T ixj_p;                        /* '<S413>/i x j' */
  real_T jxk_j;                        /* '<S413>/j x k' */
  real_T kxi_e;                        /* '<S413>/k x i' */
  real_T ixk_k3;                       /* '<S414>/i x k' */
  real_T jxi_b1;                       /* '<S414>/j x i' */
  real_T kxj_f;                        /* '<S414>/k x j' */
  real_T Sum_a[3];                     /* '<S410>/Sum' */
  real_T Selector1_g[9];               /* '<S400>/Selector1' */
  real_T Reshape1_f[3];                /* '<S412>/Reshape1' */
  real_T Product_j[3];                 /* '<S412>/Product' */
  real_T Reshape2_g[3];                /* '<S412>/Reshape2' */
  real_T Reshape_c[12];                /* '<S396>/Reshape' */
  real_T Selector3_g[4];               /* '<S396>/Selector3' */
  real_T Reshape3_j[4];                /* '<S396>/Reshape3' */
  real_T Product2_h[4];                /* '<S391>/Product2' */
  real_T SumofElements1;               /* '<S391>/Sum of Elements1' */
  real_T Selector2[4];                 /* '<S396>/Selector2' */
  real_T Reshape2_pd[4];               /* '<S396>/Reshape2' */
  real_T Product1_k[4];                /* '<S391>/Product1' */
  real_T SumofElements15;              /* '<S391>/Sum of Elements15' */
  real_T SumofElements2;               /* '<S391>/Sum of Elements2' */
  real_T Product4_i;                   /* '<S391>/Product4' */
  real_T SumofElements4;               /* '<S391>/Sum of Elements4' */
  real_T Product5;                     /* '<S391>/Product5' */
  real_T SumofElements3;               /* '<S391>/Sum of Elements3' */
  real_T Selector1_hl[4];              /* '<S396>/Selector1' */
  real_T Reshape1_b3[4];               /* '<S396>/Reshape1' */
  real_T Product_m[4];                 /* '<S391>/Product' */
  real_T SumofElements16;              /* '<S391>/Sum of Elements16' */
  real_T SumofElements10;              /* '<S391>/Sum of Elements10' */
  real_T Product6;                     /* '<S391>/Product6' */
  real_T SumofElements11;              /* '<S391>/Sum of Elements11' */
  real_T Product7;                     /* '<S391>/Product7' */
  real_T SumofElements8;               /* '<S391>/Sum of Elements8' */
  real_T Product3_o[4];                /* '<S391>/Product3' */
  real_T SumofElements7;               /* '<S391>/Sum of Elements7' */
  real_T VectorConcatenate_f[3];       /* '<S386>/Vector Concatenate' */
  real_T Selector1_a[4];               /* '<S397>/Selector1' */
  real_T Selector2_n[4];               /* '<S397>/Selector2' */
  real_T Selector3_i[4];               /* '<S397>/Selector3' */
  real_T VectorConcatenate_nl[3];      /* '<S397>/Vector Concatenate' */
  real_T Reshape1_p[3];                /* '<S388>/Reshape1' */
  real_T InertialtoBody[3];            /* '<S388>/Inertial to Body' */
  real_T Add1_n[3];                    /* '<S419>/Add1' */
  real_T Product_k[3];                 /* '<S419>/Product' */
  real_T SumofElements_l;              /* '<S419>/Sum of Elements' */
  real_T Sqrt;                         /* '<S419>/Sqrt' */
  real_T Product2_m;                   /* '<S419>/Product2' */
  real_T TrigonometricFunction;        /* '<S419>/Trigonometric Function' */
  real_T u[3];                         /* '<S419>/4' */
  real_T Tanh[3];                      /* '<S419>/Tanh' */
  real_T VectorConcatenate_fs[6];      /* '<S419>/Vector Concatenate' */
  real_T Product1_g3[6];               /* '<S419>/Product1' */
  real_T uAPabsRT[6];                  /* '<S419>/.5.*A.*Pabs.//R.//T' */
  real_T Product4_p[3];                /* '<S419>/Product4' */
  real_T UnaryMinus1_g[3];             /* '<S386>/Unary Minus1' */
  real_T Add_n[3];                     /* '<S386>/Add' */
  real_T Sum2_a[3];                    /* '<S400>/Sum2' */
  real_T Reshape1_m[3];                /* '<S400>/Reshape1' */
  real_T Selector2_k[9];               /* '<S400>/Selector2' */
  real_T Product2_d[3];                /* '<S400>/Product2' */
  real_T Reshape_o[3];                 /* '<S400>/Reshape' */
  real_T SumofElements_e[3];           /* '<S415>/Sum of Elements' */
  real_T Reshape_h[3];                 /* '<S401>/Reshape' */
  real_T Inertialgravityvector[3];     /* '<S390>/Vector Concatenate' */
  real_T Fg_I[3];                      /* '<S390>/Product' */
  real_T Reshape_ox[3];                /* '<S390>/Reshape' */
  real_T Fg_B[3];                      /* '<S390>/Inertial to Body' */
  real_T Product3_kb[3];               /* '<S419>/Product3' */
  real_T VectorConcatenate_pf5[3];     /* '<S396>/Vector Concatenate' */
  real_T Sum_o[3];                     /* '<S386>/Sum' */
  real_T Sum_op[3];                    /* '<S401>/Sum' */
  real_T Product_fl[3];                /* '<S387>/Product' */
  real_T jxk_h;                        /* '<S417>/j x k' */
  real_T kxi_f;                        /* '<S417>/k x i' */
  real_T ixj_ol;                       /* '<S417>/i x j' */
  real_T kxj_k;                        /* '<S418>/k x j' */
  real_T ixk_d;                        /* '<S418>/i x k' */
  real_T jxi_jh;                       /* '<S418>/j x i' */
  real_T Sum_kr[3];                    /* '<S402>/Sum' */
  real_T Sum_d[3];                     /* '<S387>/Sum' */
  real_T Transpose[9];                 /* '<S387>/Transpose' */
  real_T Reshape1_d[3];                /* '<S406>/Reshape1' */
  real_T Product_aa[3];                /* '<S406>/Product' */
  real_T Reshape2_me[3];               /* '<S406>/Reshape2' */
  real_T UnitConversion_j[3];          /* '<S404>/Unit Conversion' */
  real_T Constant_o[12];               /* '<S386>/Constant' */
  real_T VectorConcatenate_a[6];       /* '<S393>/Vector Concatenate' */
  real_T Abs_f5[6];                    /* '<S393>/Abs' */
  real_T UnaryMinus_o[3];              /* '<S386>/Unary Minus' */
  real_T VectorConcatenate1_j[6];      /* '<S393>/Vector Concatenate1' */
  real_T Abs1_b[6];                    /* '<S393>/Abs1' */
  real_T SumofElements_g;              /* '<S393>/Sum of Elements' */
  real_T SumofElements1_d;             /* '<S393>/Sum of Elements1' */
  real_T Reshape1_k0[3];               /* '<S427>/Reshape1' */
  real_T Product_c[3];                 /* '<S427>/Product' */
  real_T Reshape2_a[3];                /* '<S427>/Reshape2' */
  real_T V_wb[3];                      /* '<S425>/Add4' */
  real_T Reshape1_dx[3];               /* '<S435>/Reshape1' */
  real_T Product_cv[3];                /* '<S435>/Product' */
  real_T Reshape2_c[3];                /* '<S435>/Reshape2' */
  real_T V_wb_k[3];                    /* '<S433>/Add4' */
  real_T sincos_o1_hn[3];              /* '<S443>/sincos' */
  real_T sincos_o2_d[3];               /* '<S443>/sincos' */
  real_T VectorConcatenate_am[9];      /* '<S450>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_e[9];
                                /* '<S450>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_e[9];              /* '<S441>/Transpose1' */
  real_T VectorConcatenate_i[3];       /* '<S422>/Vector Concatenate' */
  real_T Subtract_g[3];                /* '<S422>/Subtract' */
  real_T Reshape1_iq[3];               /* '<S445>/Reshape1' */
  real_T Product_kf[3];                /* '<S445>/Product' */
  real_T Reshape2_j[3];                /* '<S445>/Reshape2' */
  real_T Add_ho[3];                    /* '<S441>/Add' */
  real_T jxk_c;                        /* '<S451>/j x k' */
  real_T kxi_o2;                       /* '<S451>/k x i' */
  real_T ixj_f;                        /* '<S451>/i x j' */
  real_T kxj_j;                        /* '<S452>/k x j' */
  real_T ixk_m;                        /* '<S452>/i x k' */
  real_T jxi_f;                        /* '<S452>/j x i' */
  real_T Sum_l[3];                     /* '<S446>/Sum' */
  real_T Add1_o[3];                    /* '<S441>/Add1' */
  real_T Reshape1_ff[3];               /* '<S444>/Reshape1' */
  real_T Product_id[3];                /* '<S444>/Product' */
  real_T Reshape2_ca[3];               /* '<S444>/Reshape2' */
  real_T V_wb_o[3];                    /* '<S441>/Add4' */
  real_T Fcn_i;                        /* '<S447>/Fcn' */
  real_T Abs_la;                       /* '<S447>/Abs' */
  real_T Switch_dt;                    /* '<S447>/Switch' */
  real_T Divide_f2;                    /* '<S442>/Divide' */
  real_T Beta;                         /* '<S442>/Trigonometric Function' */
  real_T Reshape1_kx[3];               /* '<S455>/Reshape1' */
  real_T Product_d4[3];                /* '<S455>/Product' */
  real_T Reshape2_js[3];               /* '<S455>/Reshape2' */
  real_T V_wb_n[3];                    /* '<S453>/Add4' */
  real_T Reshape1_bj[3];               /* '<S463>/Reshape1' */
  real_T Product_dl[3];                /* '<S463>/Product' */
  real_T Reshape2_n1[3];               /* '<S463>/Reshape2' */
  real_T V_wb_j[3];                    /* '<S461>/Add4' */
  real_T sincos_o1_m[3];               /* '<S471>/sincos' */
  real_T sincos_o2_md[3];              /* '<S471>/sincos' */
  real_T thetadot_e;                   /* '<S471>/thetadot' */
  real_T jxk_e;                        /* '<S477>/j x k' */
  real_T psidot_g;                     /* '<S471>/psidot' */
  real_T kxi_ki;                       /* '<S477>/k x i' */
  real_T phidot_m;                     /* '<S471>/phidot' */
  real_T ixj_d;                        /* '<S477>/i x j' */
  real_T kxj_g;                        /* '<S478>/k x j' */
  real_T ixk_k2;                       /* '<S478>/i x k' */
  real_T jxi_d;                        /* '<S478>/j x i' */
  real_T Sum_e[3];                     /* '<S472>/Sum' */
  real_T Add_hc[3];                    /* '<S395>/Add' */
  real_T Fcn_c;                        /* '<S474>/Fcn' */
  real_T Abs_h;                        /* '<S474>/Abs' */
  real_T Switch_nu;                    /* '<S474>/Switch' */
  real_T Divide_g;                     /* '<S470>/Divide' */
  real_T Beta_j;                       /* '<S470>/Trigonometric Function' */
  real_T Integrator_i[3];              /* '<S395>/Integrator' */
  real_T UnitConversion1[3];           /* '<S395>/Unit Conversion1' */
  real_T UnitConversion3[3];           /* '<S395>/Unit Conversion3' */
  real_T VectorConcatenate2_k[12];     /* '<S396>/Vector Concatenate2' */
  real_T VectorConcatenate4[4];        /* '<S296>/Vector Concatenate4' */
  real_T VectorConcatenate3_a[4];      /* '<S296>/Vector Concatenate3' */
  real_T Subtract_p[4];                /* '<S296>/Subtract' */
  real_T VectorConcatenate1_l[4];      /* '<S296>/Vector Concatenate1' */
  real_T VectorConcatenate2_bt[4];     /* '<S296>/Vector Concatenate2' */
  real_T Selector_b[4];                /* '<S297>/Selector' */
  real_T Reshape2_o[4];                /* '<S297>/Reshape2' */
  real_T Sum_dw[4];                    /* '<S479>/Sum' */
  real_T Divide_ah[4];                 /* '<S479>/Divide' */
  real_T Sum_ja[12];                   /* '<S480>/Sum' */
  real_T Divide_d[12];                 /* '<S480>/Divide' */
  real_T Reshape6_ll[4];               /* '<S482>/Reshape6' */
  real_T Reshape_h0[4];                /* '<S486>/Reshape' */
  real_T VectorConcatenate5[4];        /* '<S482>/Vector Concatenate5' */
  real_T VectorConcatenate6[4];        /* '<S482>/Vector Concatenate6' */
  real_T SignalCopy;                   /* '<S512>/Signal Copy' */
  real_T Product2_mt;                  /* '<S516>/Product2' */
  real_T Add1_c;                       /* '<S516>/Add1' */
  real_T Abs_e;                        /* '<S516>/Abs' */
  real_T Add_jj;                       /* '<S516>/Add' */
  real_T DeadZone;                     /* '<S516>/Dead Zone' */
  real_T Saturation1_h;                /* '<S516>/Saturation1' */
  real_T Product3_es;                  /* '<S516>/Product3' */
  real_T Saturation_lr;                /* '<S516>/Saturation' */
  real_T Fdot;                         /* '<S516>/Product1' */
  real_T Product2_j;                   /* '<S517>/Product2' */
  real_T Add1_k4;                      /* '<S517>/Add1' */
  real_T Abs_g;                        /* '<S517>/Abs' */
  real_T Add_lq;                       /* '<S517>/Add' */
  real_T DeadZone_h;                   /* '<S517>/Dead Zone' */
  real_T Saturation1_c;                /* '<S517>/Saturation1' */
  real_T Product3_fx;                  /* '<S517>/Product3' */
  real_T Saturation_g5;                /* '<S517>/Saturation' */
  real_T Fdot_a;                       /* '<S517>/Product1' */
  real_T Product2_ht;                  /* '<S519>/Product2' */
  real_T Add1_a;                       /* '<S519>/Add1' */
  real_T Abs_n;                        /* '<S519>/Abs' */
  real_T Add_m;                        /* '<S519>/Add' */
  real_T DeadZone_j;                   /* '<S519>/Dead Zone' */
  real_T Saturation1_bf;               /* '<S519>/Saturation1' */
  real_T Product3_ix;                  /* '<S519>/Product3' */
  real_T Saturation_dk;                /* '<S519>/Saturation' */
  real_T Fdot_h;                       /* '<S519>/Product1' */
  real_T Switch_au;                    /* '<S514>/Switch' */
  real_T Gain2_m;                      /* '<S514>/Gain2' */
  real_T Sum2_g;                       /* '<S514>/Sum2' */
  real_T Gain1_ge;                     /* '<S514>/Gain1' */
  real_T SignalCopy_e;                 /* '<S537>/Signal Copy' */
  real_T Product2_k;                   /* '<S541>/Product2' */
  real_T Add1_k1;                      /* '<S541>/Add1' */
  real_T Abs_k;                        /* '<S541>/Abs' */
  real_T Add_bv;                       /* '<S541>/Add' */
  real_T DeadZone_f;                   /* '<S541>/Dead Zone' */
  real_T Saturation1_f;                /* '<S541>/Saturation1' */
  real_T Product3_oh;                  /* '<S541>/Product3' */
  real_T Saturation_jp;                /* '<S541>/Saturation' */
  real_T Fdot_f;                       /* '<S541>/Product1' */
  real_T Product2_ok;                  /* '<S542>/Product2' */
  real_T Add1_nb;                      /* '<S542>/Add1' */
  real_T Abs_o;                        /* '<S542>/Abs' */
  real_T Add_mi;                       /* '<S542>/Add' */
  real_T DeadZone_b;                   /* '<S542>/Dead Zone' */
  real_T Saturation1_b1;               /* '<S542>/Saturation1' */
  real_T Product3_a;                   /* '<S542>/Product3' */
  real_T Saturation_pn;                /* '<S542>/Saturation' */
  real_T Fdot_l;                       /* '<S542>/Product1' */
  real_T Product2_kj;                  /* '<S544>/Product2' */
  real_T Add1_bl;                      /* '<S544>/Add1' */
  real_T Abs_a;                        /* '<S544>/Abs' */
  real_T Add_fx;                       /* '<S544>/Add' */
  real_T DeadZone_p;                   /* '<S544>/Dead Zone' */
  real_T Saturation1_k;                /* '<S544>/Saturation1' */
  real_T Product3_j1;                  /* '<S544>/Product3' */
  real_T Saturation_fp;                /* '<S544>/Saturation' */
  real_T Fdot_o;                       /* '<S544>/Product1' */
  real_T Switch_o0;                    /* '<S539>/Switch' */
  real_T Gain2_o;                      /* '<S539>/Gain2' */
  real_T Sum2_d;                       /* '<S539>/Sum2' */
  real_T Gain1_p;                      /* '<S539>/Gain1' */
  real_T SignalCopy_g;                 /* '<S562>/Signal Copy' */
  real_T Product2_ji;                  /* '<S566>/Product2' */
  real_T Add1_oe;                      /* '<S566>/Add1' */
  real_T Abs_kh;                       /* '<S566>/Abs' */
  real_T Add_du;                       /* '<S566>/Add' */
  real_T DeadZone_d;                   /* '<S566>/Dead Zone' */
  real_T Saturation1_dh;               /* '<S566>/Saturation1' */
  real_T Product3_oq;                  /* '<S566>/Product3' */
  real_T Saturation_gr;                /* '<S566>/Saturation' */
  real_T Fdot_d;                       /* '<S566>/Product1' */
  real_T Product2_ii;                  /* '<S567>/Product2' */
  real_T Add1_gr;                      /* '<S567>/Add1' */
  real_T Abs_hy;                       /* '<S567>/Abs' */
  real_T Add_dy;                       /* '<S567>/Add' */
  real_T DeadZone_e;                   /* '<S567>/Dead Zone' */
  real_T Saturation1_ke;               /* '<S567>/Saturation1' */
  real_T Product3_ol;                  /* '<S567>/Product3' */
  real_T Saturation_k;                 /* '<S567>/Saturation' */
  real_T Fdot_b;                       /* '<S567>/Product1' */
  real_T Product2_fs;                  /* '<S569>/Product2' */
  real_T Add1_l;                       /* '<S569>/Add1' */
  real_T Abs_fr;                       /* '<S569>/Abs' */
  real_T Add_ot;                       /* '<S569>/Add' */
  real_T DeadZone_a;                   /* '<S569>/Dead Zone' */
  real_T Saturation1_hj;               /* '<S569>/Saturation1' */
  real_T Product3_lt;                  /* '<S569>/Product3' */
  real_T Saturation_gl;                /* '<S569>/Saturation' */
  real_T Fdot_c;                       /* '<S569>/Product1' */
  real_T Switch_bz;                    /* '<S564>/Switch' */
  real_T Gain2_h;                      /* '<S564>/Gain2' */
  real_T Sum2_dp;                      /* '<S564>/Sum2' */
  real_T Gain1_o;                      /* '<S564>/Gain1' */
  real_T SignalCopy_k;                 /* '<S587>/Signal Copy' */
  real_T Product2_b;                   /* '<S591>/Product2' */
  real_T Add1_e;                       /* '<S591>/Add1' */
  real_T Abs_j;                        /* '<S591>/Abs' */
  real_T Add_lr;                       /* '<S591>/Add' */
  real_T DeadZone_l;                   /* '<S591>/Dead Zone' */
  real_T Saturation1_kd;               /* '<S591>/Saturation1' */
  real_T Product3_ip;                  /* '<S591>/Product3' */
  real_T Saturation_e4;                /* '<S591>/Saturation' */
  real_T Fdot_m;                       /* '<S591>/Product1' */
  real_T Product2_ip;                  /* '<S592>/Product2' */
  real_T Add1_ok;                      /* '<S592>/Add1' */
  real_T Abs_az;                       /* '<S592>/Abs' */
  real_T Add_j5;                       /* '<S592>/Add' */
  real_T DeadZone_bq;                  /* '<S592>/Dead Zone' */
  real_T Saturation1_o;                /* '<S592>/Saturation1' */
  real_T Product3_e2;                  /* '<S592>/Product3' */
  real_T Saturation_fl;                /* '<S592>/Saturation' */
  real_T Fdot_g;                       /* '<S592>/Product1' */
  real_T Product2_ol;                  /* '<S594>/Product2' */
  real_T Add1_lr;                      /* '<S594>/Add1' */
  real_T Abs_fn;                       /* '<S594>/Abs' */
  real_T Add_fb;                       /* '<S594>/Add' */
  real_T DeadZone_i;                   /* '<S594>/Dead Zone' */
  real_T Saturation1_l;                /* '<S594>/Saturation1' */
  real_T Product3_d;                   /* '<S594>/Product3' */
  real_T Saturation_pu;                /* '<S594>/Saturation' */
  real_T Fdot_bw;                      /* '<S594>/Product1' */
  real_T Switch_i;                     /* '<S589>/Switch' */
  real_T Gain2_c;                      /* '<S589>/Gain2' */
  real_T Sum2_p;                       /* '<S589>/Sum2' */
  real_T Gain1_b;                      /* '<S589>/Gain1' */
  real_T VectorConcatenate_ag[4];      /* '<S498>/Vector Concatenate' */
  real_T Switch_cu;                    /* '<S503>/Switch' */
  real_T Gain2_b;                      /* '<S503>/Gain2' */
  real_T Sum2_e;                       /* '<S503>/Sum2' */
  real_T Gain1_m;                      /* '<S503>/Gain1' */
  real_T Switch_e;                     /* '<S504>/Switch' */
  real_T Gain2_g;                      /* '<S504>/Gain2' */
  real_T Sum2_dd;                      /* '<S504>/Sum2' */
  real_T Gain1_n;                      /* '<S504>/Gain1' */
  real_T Switch_op;                    /* '<S505>/Switch' */
  real_T Gain2_k;                      /* '<S505>/Gain2' */
  real_T Sum2_f;                       /* '<S505>/Sum2' */
  real_T Gain1_er;                     /* '<S505>/Gain1' */
  real_T Switch_f;                     /* '<S506>/Switch' */
  real_T Gain2_d;                      /* '<S506>/Gain2' */
  real_T Sum2_hm;                      /* '<S506>/Sum2' */
  real_T Gain1_n4;                     /* '<S506>/Gain1' */
  real_T VectorConcatenate_bc[4];      /* '<S507>/Vector Concatenate' */
  real_T VectorConcatenate_ol[4];      /* '<S508>/Vector Concatenate' */
  real_T SteeringCmd_c;                /* '<S2>/Ackerman steer' */
  real_T ImpAsg_InsertedFor_zdotWheel_at_inport_0[4];/* '<S613>/Demux1' */
  real_T ImpAsg_InsertedFor_ydotWheel_at_inport_0[4];/* '<S613>/Demux1' */
  real_T ImpAsg_InsertedFor_xdotWheel_at_inport_0[4];/* '<S613>/Demux1' */
  real_T ImpAsg_InsertedFor_Fz_at_inport_0[4];/* '<S488>/Demux1' */
  real_T ImpAsg_InsertedFor_Fy_at_inport_0[4];/* '<S488>/Demux1' */
  real_T ImpAsg_InsertedFor_Fx_at_inport_0[4];/* '<S488>/Demux1' */
  real_T Mbar;                         /* '<S386>/vehdyncginert' */
  real_T Ibar[9];                      /* '<S386>/vehdyncginert' */
  real_T Rbar[3];                      /* '<S386>/vehdyncginert' */
  real_T Xbar[3];                      /* '<S386>/vehdyncginert' */
  real_T Wbar[4];                      /* '<S386>/vehdyncginert' */
  real_T HPbar[12];                    /* '<S386>/vehdyncginert' */
  real_T ImpAsg_InsertedFor_F_at_inport_0[3];/* '<S416>/Product' */
  real_T ImpAsg_InsertedFor_WhlFz_at_inport_0[2];/* '<S350>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlAng_at_inport_0[6];/* '<S350>/Suspension' */
  real_T ImpAsg_InsertedFor_VehM_at_inport_0[6];/* '<S350>/Suspension' */
  real_T ImpAsg_InsertedFor_VehFz_at_inport_0[2];/* '<S350>/Suspension' */
  real_T ImpAsg_InsertedFor_Info_at_inport_0[12];/* '<S350>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlFzAs_at_inport_0[2];/* '<S352>/Mux' */
  real_T ImpAsg_InsertedFor_VehFzAs_at_inport_0[2];/* '<S352>/Mux1' */
  real_T ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[2];/* '<S352>/Sum2' */
  real_T ImpAsg_InsertedFor_WhlFz_at_inport_0_l[2];/* '<S313>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlAng_at_inport_0_p[6];/* '<S313>/Suspension' */
  real_T ImpAsg_InsertedFor_VehM_at_inport_0_b[6];/* '<S313>/Suspension' */
  real_T ImpAsg_InsertedFor_VehFz_at_inport_0_j[2];/* '<S313>/Suspension' */
  real_T ImpAsg_InsertedFor_Info_at_inport_0_i[12];/* '<S313>/Suspension' */
  real_T ImpAsg_InsertedFor_p_at_inport_0;
                                     /* '<S312>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_cgV_at_inport_0[3];
                                     /* '<S312>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_cgP_at_inport_0[3];
                                     /* '<S312>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_DCM_at_inport_0[9];
                                     /* '<S312>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_WhlV_at_inport_0[6];/* '<S311>/Sum2' */
  real_T ImpAsg_InsertedFor_WhlP_at_inport_0[6];/* '<S311>/Sum4' */
  real_T ImpAsg_InsertedFor_SuspV_at_inport_0[6];/* '<S311>/Sum3' */
  real_T ImpAsg_InsertedFor_SuspP_at_inport_0[6];/* '<S311>/Sum1' */
  real_T TmpSignalConversionAtSFunctionInport6[4];/* '<S271>/Band-Aid' */
  real_T OutMode;                      /* '<S271>/Band-Aid' */
  real_T OutSpd[4];                    /* '<S271>/Band-Aid' */
  real_T State;                        /* '<S271>/Band-Aid' */
  real_T Divide_l4;                    /* '<S157>/Divide' */
  real_T Divide1_fk;                   /* '<S157>/Divide1' */
  real_T Interpolatedzerocrossing;     /* '<S157>/Interpolated zero-crossing' */
  real_T Divide2;                      /* '<S157>/Divide2' */
  real_T Clock1;                       /* '<S136>/Clock1' */
  real_T Memory4_g;                    /* '<S136>/Memory4' */
  real_T DataTypeConversion;           /* '<S136>/Data Type Conversion' */
  real_T Clock2;                       /* '<S136>/Clock2' */
  real_T Switch_na;                    /* '<S136>/Switch' */
  real_T tCycle;                       /* '<S136>/Add1' */
  real_T Add_di;                       /* '<S89>/Add' */
  real_T CoastDown;                    /* '<S89>/CoastDown' */
  real_T Vehicle_Spd_kph;              /* '<S89>/Data Type Conversion11' */
  real_T Drive;                        /* '<S135>/Constant' */
  real_T Memory1_m;                    /* '<S135>/Memory1' */
  real_T Memory2_k;                    /* '<S135>/Memory2' */
  real_T RefSpdkmhr;                   /* '<S135>/SpdSat' */
  real_T Memory1_n;                    /* '<S137>/Memory1' */
  real_T Memory_f;                     /* '<S137>/Memory' */
  real_T Add_pf;                       /* '<S137>/Add' */
  real_T Feedforward;                  /* '<S137>/Feedforward' */
  real_T Driver_FdfwdTerm_pct;         /* '<S137>/FeedforwardGain' */
  real_T Driver_IntTerm_pct;           /* '<S137>/Limits [-50,50]' */
  real_T VS_Flt;                       /* '<S137>/Lowpass Filter' */
  real_T SpdErr;                       /* '<S137>/Sum' */
  real_T Kp;                           /* '<S137>/Kp' */
  real_T ProportionalGainScheduling; /* '<S137>/Proportional Gain Scheduling' */
  real_T Driver_PropTerm_pct;          /* '<S137>/Product1' */
  real_T Driver_TotCmdPreLim_pct;      /* '<S137>/Sum1' */
  real_T Gain_ey;                      /* '<S137>/Gain' */
  real_T Ki;                           /* '<S137>/Ki' */
  real_T BrakePedalPosPre_pct;         /* '<S137>/Switch2' */
  real_T AcceleratorPedalPosPre_pct;   /* '<S137>/Switch1' */
  real_T AcceleratorPedalPos_pct;      /* '<S137>/Switch3' */
  real_T BrakePedalPos_pct;            /* '<S137>/Switch4' */
  real_T RefSpdmph;                    /* '<S135>/km to miles' */
  real_T Vehicle_Spd_mph;              /* '<S89>/Gain' */
  real_T RateLimiter_h;                /* '<S89>/Rate Limiter' */
  real_T pMCT;                         /* '<S89>/pMCT' */
  real_T US06;                         /* '<S89>/US06' */
  real_T MCT;                          /* '<S89>/MCT' */
  real_T limitedMCT;                   /* '<S89>/limitedMCT' */
  real_T MultiportSwitch_b;            /* '<S89>/Multiport Switch' */
  real_T RefSpdkmhr_i;                 /* '<S89>/Multiport Switch1' */
  real_T Vehicle_SpdDmd_mph;           /* '<S89>/Gain1' */
  real_T Sum3_iw;                      /* '<S116>/Sum3' */
  real_T Sum3_p;                       /* '<S115>/Sum3' */
  real_T ControlWord;                  /* '<S101>/MOOG State Machine' */
  real_T PositionCmd;                  /* '<S101>/MOOG State Machine' */
  real_T Actuator_Ready;               /* '<S101>/MOOG State Machine' */
  real_T FaultCntr;                    /* '<S101>/MOOG State Machine' */
  real_T MoogSMState;                  /* '<S101>/MOOG State Machine' */
  real_T InitComplete;                 /* '<S101>/MOOG State Machine' */
  real_T MoogOpMode;                   /* '<S101>/MOOG State Machine' */
  real_T LearningComplete;             /* '<S101>/MOOG State Machine' */
  real_T MaxStop;                      /* '<S101>/MOOG State Machine' */
  real_T MinStop;                      /* '<S101>/MOOG State Machine' */
  uint32_T PowertrainData_10_HS1Counter;/* '<S1>/Data Inport S-Fcn' */
  uint32_T WheelSpeed_HS1Counter;      /* '<S1>/Data Inport S-Fcn' */
  uint32_T BrakeSysFeatures_HS1Counter;/* '<S1>/Data Inport S-Fcn' */
  uint32_T EngVehicleSpThrottle_HS1Counter;/* '<S1>/Data Inport S-Fcn' */
  uint32_T BrakeSnData_4_HS1Counter;   /* '<S1>/Data Inport S-Fcn' */
  uint32_T TPDO3Counter;               /* '<S111>/Data Inport S-Fcn' */
  uint32_T Reserved;                   /* '<S196>/uint32 (unsigned 32)1' */
  uint32_T Reserved_m;                 /* '<S196>/uint32 (unsigned 32)2' */
  uint32_T Reserved_i;                 /* '<S196>/uint32 (unsigned 32)3' */
  uint32_T Reserved_j;                 /* '<S196>/uint32 (unsigned 32)4' */
  real32_T DataTypeConversion_c;       /* '<S267>/Data Type Conversion' */
  real32_T DataTypeConversion_o;       /* '<S268>/Data Type Conversion' */
  real32_T DataTypeConversion_cd;      /* '<S269>/Data Type Conversion' */
  real32_T DataTypeConversion_a;       /* '<S270>/Data Type Conversion' */
  int32_T ActualPositionValue;         /* '<S105>/Data Inport S-Fcn' */
  int32_T TargetPosition_Cnt_Final;    /* '<S98>/Data Type Conversion2' */
  uint16_T BrkTot_Tq_RqArbValue;       /* '<S1>/Data Inport S-Fcn' */
  uint16_T BrkTot_Tq_ActlValue;        /* '<S1>/Data Inport S-Fcn' */
  uint16_T ReceivedBytes;              /* '<S186>/Data Inport S-Fcn' */
  uint16_T SourcePort;                 /* '<S186>/Data Inport S-Fcn' */
  uint16_T SentBytes;                  /* '<S187>/Data Inport S-Fcn' */
  uint16_T SystemStatusBits;           /* '<S196>/uint16 (unsigned 16)3' */
  uint16_T ID;                         /* '<S196>/uint16 (unsigned 16)1' */
  uint16_T ProtocolVer;                /* '<S196>/uint16 (unsigned 16)2' */
  uint16_T UnitsStatusLF;              /* '<S196>/uint16 (unsigned 16)4' */
  uint16_T UnitsStatusRF;              /* '<S196>/uint16 (unsigned 16)5' */
  uint16_T UnitsStatusLR;              /* '<S196>/uint16 (unsigned 16)6' */
  uint16_T UnitsStatusRR;              /* '<S196>/uint16 (unsigned 16)7' */
  int16_T Modeofoperation;             /* '<S101>/Gain1' */
  int16_T StatusWord0Value;            /* '<S107>/Data Inport S-Fcn' */
  int16_T ControlWord_Final;           /* '<S98>/Data Type Conversion1' */
  uint8_T TrnRng_D_RqValue;            /* '<S1>/Data Inport S-Fcn' */
  uint8_T TrnPrkSys_D_ActlValue;       /* '<S1>/Data Inport S-Fcn' */
  uint8_T GearLvr_D_ActlDrvValue;      /* '<S1>/Data Inport S-Fcn' */
  uint8_T GearPos_No_CsValue;          /* '<S1>/Data Inport S-Fcn' */
  uint8_T GearPos_D_TrgValue;          /* '<S1>/Data Inport S-Fcn' */
  uint8_T GearPos_No_CntValue;         /* '<S1>/Data Inport S-Fcn' */
  uint8_T GearPos_D_ActlValue;         /* '<S1>/Data Inport S-Fcn' */
  uint8_T VehVActlBrk_No_CsValue;      /* '<S1>/Data Inport S-Fcn' */
  uint8_T VehVActlBrk_No_CntValue;     /* '<S1>/Data Inport S-Fcn' */
  uint8_T VehVActlBrk_D_QfValue;       /* '<S1>/Data Inport S-Fcn' */
  uint8_T BrkTotTqRqArb_No_CsValue;    /* '<S1>/Data Inport S-Fcn' */
  uint8_T BrkTotTqRqArb_No_CntValue;   /* '<S1>/Data Inport S-Fcn' */
  uint8_T HsaStat_D_ActlValue;         /* '<S1>/Data Inport S-Fcn' */
  uint8_T uint8unsigned88;             /* '<S273>/uint8 (unsigned 8) 8' */
  uint8_T uint8unsigned87;             /* '<S273>/uint8 (unsigned 8) 7' */
  uint8_T uint16unsigned162[2];        /* '<S273>/uint16 (unsigned 16) 2' */
  uint8_T DataVector[96];              /* '<S186>/Data Inport S-Fcn' */
  uint8_T SourceIPAddress[4];          /* '<S186>/Data Inport S-Fcn' */
  uint8_T uint8unsigned89;             /* '<S273>/uint8 (unsigned 8) 9' */
  uint8_T uint8unsigned85;             /* '<S273>/uint8 (unsigned 8) 5' */
  uint8_T Output;                      /* '<S282>/Output' */
  uint8_T uint8unsigned81;             /* '<S273>/uint8 (unsigned 8) 1' */
  uint8_T uint8unsigned82;             /* '<S273>/uint8 (unsigned 8) 2' */
  uint8_T uint8unsigned83;             /* '<S273>/uint8 (unsigned 8) 3' */
  uint8_T uint8unsigned84;             /* '<S273>/uint8 (unsigned 8) 4' */
  uint8_T TmpSignalConversionAtDataOutportSFcnInport1[26];/* '<S188>/Data Vector' */
  uint8_T TmpSignalConversionAtuint16unsigned163Inport1[2];/* '<S196>/Selector23' */
  uint8_T FaultActive;                 /* '<S197>/Gain' */
  uint8_T WarningsActive;              /* '<S197>/Gain1' */
  uint8_T SpTqRF_m;                    /* '<S197>/Gain10' */
  uint8_T SpTqLR;                      /* '<S197>/Gain11' */
  uint8_T SpTqRR;                      /* '<S197>/Gain12' */
  uint8_T Reserved_k;                  /* '<S197>/Gain13' */
  uint8_T Reserved_jr;                 /* '<S197>/Gain14' */
  uint8_T Reserved_e;                  /* '<S197>/Gain15' */
  uint8_T Reserved_f;                  /* '<S197>/Gain2' */
  uint8_T Reserved_h;                  /* '<S197>/Gain3' */
  uint8_T Reserved_d;                  /* '<S197>/Gain4' */
  uint8_T Reserved_il;                 /* '<S197>/Gain5' */
  uint8_T Reserved_p;                  /* '<S197>/Gain6' */
  uint8_T Reserved_ku;                 /* '<S197>/Gain7' */
  uint8_T SystemActive;                /* '<S197>/Gain8' */
  uint8_T SpTqLF;                      /* '<S197>/Gain9' */
  uint8_T TmpSignalConversionAtuint16unsigned161Inport1[2];/* '<S196>/Selector3' */
  uint8_T TmpSignalConversionAtuint16unsigned162Inport1[2];/* '<S196>/Selector1' */
  uint8_T TmpSignalConversionAtuint16unsigned164Inport1[2];/* '<S196>/Selector24' */
  uint8_T TmpSignalConversionAtuint16unsigned165Inport1[2];/* '<S196>/Selector25' */
  uint8_T TmpSignalConversionAtuint16unsigned166Inport1[2];/* '<S196>/Selector26' */
  uint8_T TmpSignalConversionAtuint16unsigned167Inport1[2];/* '<S196>/Selector27' */
  uint8_T TmpSignalConversionAtuint32unsigned321Inport1[4];/* '<S196>/Selector6' */
  uint8_T TmpSignalConversionAtuint32unsigned322Inport1[4];/* '<S196>/Selector10' */
  uint8_T TmpSignalConversionAtuint32unsigned323Inport1[4];/* '<S196>/Selector14' */
  uint8_T TmpSignalConversionAtuint32unsigned324Inport1[4];/* '<S196>/Selector18' */
  uint8_T WatchdogCnt;                 /* '<S196>/uint8 (unsigned 8)1' */
  uint8_T Reserved_ms;                 /* '<S196>/uint8 (unsigned 8)2' */
  uint8_T Reserved_f2;                 /* '<S281>/Bit7' */
  uint8_T Reserved_di;                 /* '<S281>/Bit6' */
  uint8_T Reserved_pj;                 /* '<S281>/Bit5' */
  uint8_T SpTqRR_k;                    /* '<S281>/Bit4' */
  uint8_T SpTqLR_n;                    /* '<S281>/Bit3' */
  uint8_T SpTqLF_k;                    /* '<S281>/Bit1' */
  uint8_T SystemActive_p;              /* '<S281>/Bit0' */
  uint8_T SystemCtrlBits;              /* '<S281>/Add' */
  uint8_T FixPtSum1;                   /* '<S287>/FixPt Sum1' */
  uint8_T FixPtSwitch;                 /* '<S288>/FixPt Switch' */
  uint8_T uint8unsigned86;             /* '<S273>/uint8 (unsigned 8) 6' */
  uint8_T y;                           /* '<S273>/MATLAB Function' */
  uint8_T ExtractDesiredBits;          /* '<S215>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly;           /* '<S215>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_i;        /* '<S229>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_o;         /* '<S229>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_m;        /* '<S214>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_i;         /* '<S214>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_b;        /* '<S222>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_m;         /* '<S222>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_l;        /* '<S223>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_p;         /* '<S223>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_k;        /* '<S224>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_c;         /* '<S224>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_mk;       /* '<S225>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_ix;        /* '<S225>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_p;        /* '<S226>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_pd;        /* '<S226>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_in;       /* '<S221>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_h;         /* '<S221>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_b2;       /* '<S220>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_oh;        /* '<S220>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_iy;       /* '<S219>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_c2;        /* '<S219>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_mv;       /* '<S218>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_c5;        /* '<S218>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_ms;       /* '<S217>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_c0;        /* '<S217>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_my;       /* '<S216>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_e;         /* '<S216>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_d;        /* '<S227>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_l;         /* '<S227>/Modify Scaling Only' */
  uint8_T ExtractDesiredBits_kq;       /* '<S228>/Extract Desired Bits' */
  uint8_T ModifyScalingOnly_cx;        /* '<S228>/Modify Scaling Only' */
  int8_T DataTypeConversion_g;         /* '<S98>/Data Type Conversion' */
  int8_T ModesOfOperationDisplayValue; /* '<S106>/Data Inport S-Fcn' */
  boolean_T Memory_du;                 /* '<S310>/Memory' */
  boolean_T PowertrainData_10_HS1State;/* '<S1>/Data Inport S-Fcn' */
  boolean_T WheelSpeed_HS1State;       /* '<S1>/Data Inport S-Fcn' */
  boolean_T BrakeSysFeatures_HS1State; /* '<S1>/Data Inport S-Fcn' */
  boolean_T EngVehicleSpThrottle_HS1State;/* '<S1>/Data Inport S-Fcn' */
  boolean_T BrakeSnData_4_HS1State;    /* '<S1>/Data Inport S-Fcn' */
  boolean_T BrakeSwitch;               /* '<S87>/Relational Operator' */
  boolean_T BRAKE_LAMP_BOOL_CAN;       /* '<S90>/Multiport Switch1' */
  boolean_T Status;                    /* '<S186>/Data Inport S-Fcn' */
  boolean_T DataTypeConversion3;       /* '<S100>/Data Type Conversion3' */
  boolean_T RelationalOperator2;       /* '<S108>/Relational Operator2' */
  boolean_T Compare;                   /* '<S112>/Compare' */
  boolean_T Compare_i;                 /* '<S170>/Compare' */
  boolean_T Compare_l;                 /* '<S177>/Compare' */
  boolean_T Compare_a;                 /* '<S178>/Compare' */
  boolean_T LogicalOperator;           /* '<S172>/Logical Operator' */
  boolean_T LowerRelop1;               /* '<S179>/LowerRelop1' */
  boolean_T UpperRelop;                /* '<S179>/UpperRelop' */
  boolean_T Compare_e;                 /* '<S173>/Compare' */
  boolean_T Compare_m;                 /* '<S182>/Compare' */
  boolean_T Compare_j;                 /* '<S183>/Compare' */
  boolean_T LogicalOperator_a;         /* '<S176>/Logical Operator' */
  boolean_T LowerRelop1_a;             /* '<S181>/LowerRelop1' */
  boolean_T UpperRelop_p;              /* '<S181>/UpperRelop' */
  boolean_T Status_n;                  /* '<S187>/Data Inport S-Fcn' */
  boolean_T Compare_d[4];              /* '<S265>/Compare' */
  boolean_T Compare_o[4];              /* '<S266>/Compare' */
  boolean_T LogicalOperator_av[4];     /* '<S193>/Logical Operator' */
  boolean_T Compare_k;                 /* '<S448>/Compare' */
  boolean_T Compare_oc;                /* '<S449>/Compare' */
  boolean_T LogicalOperator_p;         /* '<S447>/Logical Operator' */
  boolean_T Compare_lx;                /* '<S475>/Compare' */
  boolean_T Compare_b;                 /* '<S476>/Compare' */
  boolean_T LogicalOperator_j;         /* '<S474>/Logical Operator' */
  boolean_T Compare_ou;                /* '<S138>/Compare' */
  boolean_T Compare_h;                 /* '<S139>/Compare' */
  boolean_T Uk1;                       /* '<S140>/Delay Input1' */
  boolean_T Compare_hd;                /* '<S142>/Compare' */
  boolean_T FixPtRelationalOperator;   /* '<S140>/FixPt Relational Operator' */
  boolean_T Uk1_a;                     /* '<S141>/Delay Input1' */
  boolean_T Compare_ji;                /* '<S143>/Compare' */
  boolean_T FixPtRelationalOperator_g; /* '<S141>/FixPt Relational Operator' */
  boolean_T LogicalOperator_m;         /* '<S137>/Logical Operator' */
  boolean_T BrakeSwitch_g;             /* '<S137>/Relational Operator' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_ca_T CoreSubsys_pna[4];/* '<S484>/Wheel to Body Transform' */
  B_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_c;/* '<S590>/LockUp' */
  B_MagicTireConstInput_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_no_T
    sf_MagicTireConstInput_k;          /* '<S588>/Magic Tire Const Input' */
  B_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_h;/* '<S565>/LockUp' */
  B_MagicTireConstInput_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_no_T
    sf_MagicTireConstInput_f;          /* '<S563>/Magic Tire Const Input' */
  B_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_n;/* '<S540>/LockUp' */
  B_MagicTireConstInput_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_no_T
    sf_MagicTireConstInput_j;          /* '<S538>/Magic Tire Const Input' */
  B_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp;/* '<S515>/LockUp' */
  B_MagicTireConstInput_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_no_T
    sf_MagicTireConstInput;            /* '<S513>/Magic Tire Const Input' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_c_T CoreSubsys_pn[4];/* '<S482>/Wheel to Body Transform' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_b_T CoreSubsys_b[1];/* '<S415>/For Each Subsystem' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_n_T CoreSubsys_d[2];
  /* '<S309>/For each track and axle combination calculate suspension forces and moments' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_o_T CoreSubsys_n[1];
                                     /* '<S351>/For Each Axle With Anti-Sway' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_l0_T CoreSubsys_p[2];
  /* '<S304>/For each track and axle combination calculate suspension forces and moments' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_l_T CoreSubsys_m[1];
         /* '<S304>/For each axle calculate axle cg positions and velocities' */
  B_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T CoreSubsys[2];
  /* '<S304>/For each axle and track calculate suspension and wheel positions and velocities' */
  B_Defloater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Defloater_g;/* '<S279>/Defloater' */
  B_Defloater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Defloater_n;/* '<S278>/Defloater' */
  B_Defloater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Defloater_h;/* '<S277>/Defloater' */
  B_Defloater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Defloater;/* '<S276>/Defloater' */
  B_MATLABFunction_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
    sf_MATLABFunction_f;               /* '<S252>/MATLAB Function' */
  B_MATLABFunction_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
    sf_MATLABFunction_c;               /* '<S251>/MATLAB Function' */
  B_MATLABFunction_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
    sf_MATLABFunction_n;               /* '<S250>/MATLAB Function' */
  B_MATLABFunction_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
    sf_MATLABFunction;                 /* '<S249>/MATLAB Function' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_a;/* '<S213>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_ew;/* '<S212>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_e;/* '<S211>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_bz;/* '<S210>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_j;/* '<S209>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_d;/* '<S208>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_ns;/* '<S207>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_p;/* '<S206>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_m;/* '<S205>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_f;/* '<S204>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_b1;/* '<S203>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_bp;/* '<S202>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_b;/* '<S201>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_n;/* '<S200>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater_c;/* '<S199>/Floater' */
  B_Floater_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_Floater;/* '<S198>/Floater' */
} B_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator4_DSTATE;/* '<S190>/Discrete-Time Integrator4' */
  real_T DiscreteTimeIntegrator5_DSTATE;/* '<S190>/Discrete-Time Integrator5' */
  real_T DiscreteTimeIntegrator6_DSTATE;/* '<S190>/Discrete-Time Integrator6' */
  real_T DiscreteTimeIntegrator7_DSTATE;/* '<S190>/Discrete-Time Integrator7' */
  real_T DiscreteTimeIntegrator4_DSTATE_k;/* '<S247>/Discrete-Time Integrator4' */
  real_T DiscreteTimeIntegrator5_DSTATE_h;/* '<S247>/Discrete-Time Integrator5' */
  real_T DiscreteTimeIntegrator6_DSTATE_k;/* '<S247>/Discrete-Time Integrator6' */
  real_T DiscreteTimeIntegrator7_DSTATE_j;/* '<S247>/Discrete-Time Integrator7' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S249>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_c;/* '<S250>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_b;/* '<S251>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_j;/* '<S252>/Discrete-Time Integrator' */
  real_T Memory1_PreviousInput[2];     /* '<S310>/Memory1' */
  real_T Memory_PreviousInput;         /* '<S144>/Memory' */
  real_T Memory1_PreviousInput_j;      /* '<S144>/Memory1' */
  real_T Memory_PreviousInput_m;       /* '<S267>/Memory' */
  real_T Memory_PreviousInput_i;       /* '<S268>/Memory' */
  real_T Memory_PreviousInput_n;       /* '<S269>/Memory' */
  real_T Memory_PreviousInput_a;       /* '<S270>/Memory' */
  real_T PrevY[4];                     /* '<S195>/RateLimSpd' */
  real_T Memory2_PreviousInput;        /* '<S108>/Memory2' */
  real_T Memory_PreviousInput_j;       /* '<S117>/Memory' */
  real_T Memory_PreviousInput_b;       /* '<S118>/Memory' */
  real_T PrevY_c;                      /* '<S98>/Rate Limiter' */
  real_T Memory_PreviousInput_f;       /* '<S101>/Memory' */
  real_T Memory2_PreviousInput_o;      /* '<S101>/Memory2' */
  real_T Memory1_PreviousInput_b;      /* '<S101>/Memory1' */
  real_T Sum2_DWORK1;                  /* '<S101>/Sum2' */
  real_T PrevY_p;                      /* '<S99>/Rate Limiter' */
  real_T Memory_PreviousInput_fn;      /* '<S92>/Memory' */
  real_T Memory_PreviousInput_iw[4];   /* '<S189>/Memory' */
  real_T Memory3_PreviousInput;        /* '<S190>/Memory3' */
  real_T Memory1_PreviousInput_m;      /* '<S190>/Memory1' */
  real_T Memory2_PreviousInput_g;      /* '<S190>/Memory2' */
  real_T Memory4_PreviousInput;        /* '<S190>/Memory4' */
  real_T Memory3_PreviousInput_f;      /* '<S247>/Memory3' */
  real_T Memory1_PreviousInput_e;      /* '<S247>/Memory1' */
  real_T Memory2_PreviousInput_a;      /* '<S247>/Memory2' */
  real_T Memory4_PreviousInput_d;      /* '<S247>/Memory4' */
  real_T Memory_PreviousInput_e;       /* '<S249>/Memory' */
  real_T Memory_PreviousInput_h;       /* '<S250>/Memory' */
  real_T Memory_PreviousInput_ar;      /* '<S251>/Memory' */
  real_T Memory_PreviousInput_d;       /* '<S252>/Memory' */
  real_T PrevY_e;                      /* '<S299>/Backlash' */
  real_T Product2_DWORK1[9];           /* '<S400>/Product2' */
  real_T Product2_DWORK3[9];           /* '<S400>/Product2' */
  real_T Product2_DWORK4[9];           /* '<S400>/Product2' */
  real_T Product2_DWORK5[9];           /* '<S400>/Product2' */
  real_T Memory4_PreviousInput_b;      /* '<S136>/Memory4' */
  real_T Memory1_PreviousInput_bj;     /* '<S135>/Memory1' */
  real_T Memory2_PreviousInput_o3;     /* '<S135>/Memory2' */
  real_T Memory1_PreviousInput_h;      /* '<S137>/Memory1' */
  real_T Memory_PreviousInput_p;       /* '<S137>/Memory' */
  real_T PrevY_m;                      /* '<S89>/Rate Limiter' */
  void *DataInportSFcn_PWORK[44];      /* '<S1>/Data Inport S-Fcn' */
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

  void *DataInportSFcn_PWORK_f[5];     /* '<S186>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_p;        /* '<S111>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_e;        /* '<S107>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_j;        /* '<S105>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_b;        /* '<S106>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_d;        /* '<S185>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_k[2];     /* '<S187>/Data Inport S-Fcn' */
  int32_T Product2_DWORK2[3];          /* '<S400>/Product2' */
  uint32_T temporalCounter_i1;         /* '<S271>/Band-Aid' */
  uint32_T m_bpIndex;                /* '<S137>/Proportional Gain Scheduling' */
  uint32_T temporalCounter_i1_o;       /* '<S101>/MOOG State Machine' */
  int_T IntegratorLimited_IWORK;       /* '<S128>/Integrator Limited' */
  int_T IntegratorSecondOrder_MODE;    /* '<S514>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_d;  /* '<S539>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_h;  /* '<S564>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_l;  /* '<S589>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_a;  /* '<S503>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_o;  /* '<S504>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_dh; /* '<S505>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_hl; /* '<S506>/Integrator, Second-Order' */
  uint16_T temporalCounter_i2;         /* '<S101>/MOOG State Machine' */
  uint16_T temporalCounter_i3;         /* '<S101>/MOOG State Machine' */
  uint16_T temporalCounter_i4;         /* '<S101>/MOOG State Machine' */
  uint16_T temporalCounter_i5;         /* '<S101>/MOOG State Machine' */
  uint8_T Output_DSTATE;               /* '<S282>/Output' */
  boolean_T DelayInput1_DSTATE;        /* '<S140>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_e;      /* '<S141>/Delay Input1' */
  int8_T If_ActiveSubsystem;           /* '<S153>/If' */
  uint8_T DiscreteTimeIntegrator4_SYSTEM_ENABLE;/* '<S190>/Discrete-Time Integrator4' */
  uint8_T DiscreteTimeIntegrator5_SYSTEM_ENABLE;/* '<S190>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator6_SYSTEM_ENABLE;/* '<S190>/Discrete-Time Integrator6' */
  uint8_T DiscreteTimeIntegrator7_SYSTEM_ENABLE;/* '<S190>/Discrete-Time Integrator7' */
  uint8_T DiscreteTimeIntegrator4_SYSTEM_ENABLE_e;/* '<S247>/Discrete-Time Integrator4' */
  uint8_T DiscreteTimeIntegrator5_SYSTEM_ENABLE_i;/* '<S247>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator6_SYSTEM_ENABLE_f;/* '<S247>/Discrete-Time Integrator6' */
  uint8_T DiscreteTimeIntegrator7_SYSTEM_ENABLE_b;/* '<S247>/Discrete-Time Integrator7' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_ENABLE;/* '<S249>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_ENABLE_a;/* '<S250>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_ENABLE_o;/* '<S251>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_ENABLE_oa;/* '<S252>/Discrete-Time Integrator' */
  uint8_T is_active_c9_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom;/* '<S271>/Band-Aid' */
  uint8_T is_c9_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom;/* '<S271>/Band-Aid' */
  uint8_T is_Running;                  /* '<S271>/Band-Aid' */
  uint8_T is_SpeedMode;                /* '<S271>/Band-Aid' */
  uint8_T is_active_c2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom;/* '<S101>/MOOG State Machine' */
  uint8_T is_MainStateMachine;         /* '<S101>/MOOG State Machine' */
  uint8_T is_Learning;                 /* '<S101>/MOOG State Machine' */
  uint8_T is_LearnMinStop;             /* '<S101>/MOOG State Machine' */
  uint8_T is_LearnMaxStop;             /* '<S101>/MOOG State Machine' */
  uint8_T is_NormalOperation;          /* '<S101>/MOOG State Machine' */
  uint8_T is_MoogStateMachine;         /* '<S101>/MOOG State Machine' */
  uint8_T is_Initialisation;           /* '<S101>/MOOG State Machine' */
  uint8_T is_PositionControl;          /* '<S101>/MOOG State Machine' */
  boolean_T Memory_PreviousInput_b0;   /* '<S310>/Memory' */
  boolean_T IntegratorSecondOrder_DWORK1;/* '<S514>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_m;/* '<S539>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_mp;/* '<S564>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_h;/* '<S589>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_i;/* '<S503>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_d;/* '<S504>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_b;/* '<S505>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_c;/* '<S506>/Integrator, Second-Order' */
  DW_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_c;/* '<S590>/LockUp' */
  DW_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_h;/* '<S565>/LockUp' */
  DW_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_n;/* '<S540>/LockUp' */
  DW_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp;/* '<S515>/LockUp' */
} DW_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Continuous states (default storage) */
typedef struct {
  real_T ubvbwb_CSTATE[3];             /* '<S387>/ub,vb,wb' */
  real_T IntegratorSecondOrder_CSTATE[2];/* '<S514>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_l[2];/* '<S539>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_ln[2];/* '<S564>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_j[2];/* '<S589>/Integrator, Second-Order' */
  real_T xeyeze_CSTATE[3];             /* '<S387>/xe,ye,ze' */
  real_T phithetapsi_CSTATE[3];        /* '<S399>/phi theta psi' */
  real_T pqr_CSTATE[3];                /* '<S387>/p,q,r ' */
  real_T Integrator_CSTATE;            /* '<S516>/Integrator' */
  real_T Integrator_CSTATE_c;          /* '<S541>/Integrator' */
  real_T Integrator_CSTATE_b;          /* '<S566>/Integrator' */
  real_T Integrator_CSTATE_g;          /* '<S591>/Integrator' */
  real_T Integrator_CSTATE_i;          /* '<S517>/Integrator' */
  real_T Integrator_CSTATE_b0;         /* '<S542>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S567>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S592>/Integrator' */
  real_T Integrator1_CSTATE[4];        /* '<S479>/Integrator1' */
  real_T Integrator1_CSTATE_m[12];     /* '<S480>/Integrator1' */
  real_T Integrator_CSTATE_p;          /* '<S519>/Integrator' */
  real_T IntegratorLimited_CSTATE;     /* '<S128>/Integrator Limited' */
  real_T Integrator_CSTATE_c2;         /* '<S544>/Integrator' */
  real_T Integrator_CSTATE_d;          /* '<S569>/Integrator' */
  real_T Integrator_CSTATE_a;          /* '<S594>/Integrator' */
  real_T Integrator_CSTATE_i4;         /* '<S86>/Integrator' */
  real_T omegaWheel;                   /* '<S249>/omega wheel' */
  real_T omegaWheel_l;                 /* '<S250>/omega wheel' */
  real_T omegaWheel_n;                 /* '<S251>/omega wheel' */
  real_T omegaWheel_d;                 /* '<S252>/omega wheel' */
  real_T Integrator_CSTATE_as;         /* '<S113>/Integrator' */
  real_T Integrator_CSTATE_k;          /* '<S153>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_m[2];/* '<S503>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_i[2];/* '<S504>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_e[2];/* '<S505>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_jk[2];/* '<S506>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_j[3];       /* '<S395>/Integrator' */
  X_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_c;/* '<S515>/LockUp' */
  X_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_h;/* '<S515>/LockUp' */
  X_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_n;/* '<S515>/LockUp' */
  X_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp;/* '<S515>/LockUp' */
  X_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_o_T CoreSubsys_m[1];/* '<S312>/CoreSubsys' */
  real_T Limits5050_CSTATE;            /* '<S137>/Limits [-50,50]' */
  real_T LowpassFilter_CSTATE;         /* '<S137>/Lowpass Filter' */
} X_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Periodic continuous state vector (global) */
typedef int_T PeriodicIndX_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T[3];
typedef real_T PeriodicRngX_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T[6];

/* State derivatives (default storage) */
typedef struct {
  real_T ubvbwb_CSTATE[3];             /* '<S387>/ub,vb,wb' */
  real_T IntegratorSecondOrder_CSTATE[2];/* '<S514>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_l[2];/* '<S539>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_ln[2];/* '<S564>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_j[2];/* '<S589>/Integrator, Second-Order' */
  real_T xeyeze_CSTATE[3];             /* '<S387>/xe,ye,ze' */
  real_T phithetapsi_CSTATE[3];        /* '<S399>/phi theta psi' */
  real_T pqr_CSTATE[3];                /* '<S387>/p,q,r ' */
  real_T Integrator_CSTATE;            /* '<S516>/Integrator' */
  real_T Integrator_CSTATE_c;          /* '<S541>/Integrator' */
  real_T Integrator_CSTATE_b;          /* '<S566>/Integrator' */
  real_T Integrator_CSTATE_g;          /* '<S591>/Integrator' */
  real_T Integrator_CSTATE_i;          /* '<S517>/Integrator' */
  real_T Integrator_CSTATE_b0;         /* '<S542>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S567>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S592>/Integrator' */
  real_T Integrator1_CSTATE[4];        /* '<S479>/Integrator1' */
  real_T Integrator1_CSTATE_m[12];     /* '<S480>/Integrator1' */
  real_T Integrator_CSTATE_p;          /* '<S519>/Integrator' */
  real_T IntegratorLimited_CSTATE;     /* '<S128>/Integrator Limited' */
  real_T Integrator_CSTATE_c2;         /* '<S544>/Integrator' */
  real_T Integrator_CSTATE_d;          /* '<S569>/Integrator' */
  real_T Integrator_CSTATE_a;          /* '<S594>/Integrator' */
  real_T Integrator_CSTATE_i4;         /* '<S86>/Integrator' */
  real_T omegaWheel;                   /* '<S249>/omega wheel' */
  real_T omegaWheel_l;                 /* '<S250>/omega wheel' */
  real_T omegaWheel_n;                 /* '<S251>/omega wheel' */
  real_T omegaWheel_d;                 /* '<S252>/omega wheel' */
  real_T Integrator_CSTATE_as;         /* '<S113>/Integrator' */
  real_T Integrator_CSTATE_k;          /* '<S153>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_m[2];/* '<S503>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_i[2];/* '<S504>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_e[2];/* '<S505>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_jk[2];/* '<S506>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_j[3];       /* '<S395>/Integrator' */
  XDot_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_c;/* '<S515>/LockUp' */
  XDot_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_h;/* '<S515>/LockUp' */
  XDot_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_n;/* '<S515>/LockUp' */
  XDot_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp;/* '<S515>/LockUp' */
  XDot_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_k_T CoreSubsys_m[1];/* '<S312>/CoreSubsys' */
  real_T Limits5050_CSTATE;            /* '<S137>/Limits [-50,50]' */
  real_T LowpassFilter_CSTATE;         /* '<S137>/Lowpass Filter' */
} XDot_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* State disabled  */
typedef struct {
  boolean_T ubvbwb_CSTATE[3];          /* '<S387>/ub,vb,wb' */
  boolean_T IntegratorSecondOrder_CSTATE[2];/* '<S514>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_l[2];/* '<S539>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_ln[2];/* '<S564>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_j[2];/* '<S589>/Integrator, Second-Order' */
  boolean_T xeyeze_CSTATE[3];          /* '<S387>/xe,ye,ze' */
  boolean_T phithetapsi_CSTATE[3];     /* '<S399>/phi theta psi' */
  boolean_T pqr_CSTATE[3];             /* '<S387>/p,q,r ' */
  boolean_T Integrator_CSTATE;         /* '<S516>/Integrator' */
  boolean_T Integrator_CSTATE_c;       /* '<S541>/Integrator' */
  boolean_T Integrator_CSTATE_b;       /* '<S566>/Integrator' */
  boolean_T Integrator_CSTATE_g;       /* '<S591>/Integrator' */
  boolean_T Integrator_CSTATE_i;       /* '<S517>/Integrator' */
  boolean_T Integrator_CSTATE_b0;      /* '<S542>/Integrator' */
  boolean_T Integrator_CSTATE_n;       /* '<S567>/Integrator' */
  boolean_T Integrator_CSTATE_m;       /* '<S592>/Integrator' */
  boolean_T Integrator1_CSTATE[4];     /* '<S479>/Integrator1' */
  boolean_T Integrator1_CSTATE_m[12];  /* '<S480>/Integrator1' */
  boolean_T Integrator_CSTATE_p;       /* '<S519>/Integrator' */
  boolean_T IntegratorLimited_CSTATE;  /* '<S128>/Integrator Limited' */
  boolean_T Integrator_CSTATE_c2;      /* '<S544>/Integrator' */
  boolean_T Integrator_CSTATE_d;       /* '<S569>/Integrator' */
  boolean_T Integrator_CSTATE_a;       /* '<S594>/Integrator' */
  boolean_T Integrator_CSTATE_i4;      /* '<S86>/Integrator' */
  boolean_T omegaWheel;                /* '<S249>/omega wheel' */
  boolean_T omegaWheel_l;              /* '<S250>/omega wheel' */
  boolean_T omegaWheel_n;              /* '<S251>/omega wheel' */
  boolean_T omegaWheel_d;              /* '<S252>/omega wheel' */
  boolean_T Integrator_CSTATE_as;      /* '<S113>/Integrator' */
  boolean_T Integrator_CSTATE_k;       /* '<S153>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_m[2];/* '<S503>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_i[2];/* '<S504>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_e[2];/* '<S505>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_jk[2];/* '<S506>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_j[3];    /* '<S395>/Integrator' */
  XDis_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_c;/* '<S515>/LockUp' */
  XDis_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_h;/* '<S515>/LockUp' */
  XDis_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_n;/* '<S515>/LockUp' */
  XDis_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp;/* '<S515>/LockUp' */
  XDis_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_e_T CoreSubsys_m[1];/* '<S312>/CoreSubsys' */
  boolean_T Limits5050_CSTATE;         /* '<S137>/Limits [-50,50]' */
  boolean_T LowpassFilter_CSTATE;      /* '<S137>/Lowpass Filter' */
} XDis_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Integrator_Reset_ZCE;     /* '<S113>/Integrator' */
  ZCSigState Limits5050_Reset_ZCE;     /* '<S137>/Limits [-50,50]' */
} PrevZCX_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

#ifndef ODE1_INTG
#define ODE1_INTG

/* ODE1 Integration Data */
typedef struct {
  real_T *f[1];                        /* derivatives */
} ODE1_IntgData;

#endif

/* Parameters for system: '<S304>/For each axle and track calculate suspension and wheel positions and velocities' */
struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_ {
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S311>/Constant1'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S311>/Gain'
                                        */
  real_T Constant1_Value_b;            /* Expression: 1
                                        * Referenced by: '<S315>/Constant1'
                                        */
  real_T DCMStaringRow_Gain;           /* Expression: 3
                                        * Referenced by: '<S315>/DCM Staring Row'
                                        */
};

/* Parameters for system: '<S304>/For each axle calculate axle cg positions and velocities' */
struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_g_T_ {
  real_T SelectAxleMassByAxle_AxleNums;
                                /* Mask Parameter: SelectAxleMassByAxle_AxleNums
                                 * Referenced by: '<S324>/Axle Numbers'
                                 */
  real_T SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums;
            /* Mask Parameter: SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums
             * Referenced by: '<S325>/Axle Numbers'
             */
  real_T SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums;
            /* Mask Parameter: SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums
             * Referenced by: '<S322>/Axle Numbers'
             */
  real_T SelectAxleMassByAxle_AxleNums_h;
                              /* Mask Parameter: SelectAxleMassByAxle_AxleNums_h
                               * Referenced by: '<S321>/Axle Numbers'
                               */
  real_T SuspensionMomentDirectionOnSolidAxle_Gain;/* Expression: -1
                                                    * Referenced by: '<S318>/Suspension Moment Direction On Solid Axle'
                                                    */
  real_T SuspensionForceDirectionOnSolidAxle_Gain;/* Expression: -1
                                                   * Referenced by: '<S318>/Suspension Force Direction On Solid Axle'
                                                   */
  real_T Trackcoordinatesinaxlebodyframe1_Value[3];/* Expression: [1 0 0]
                                                    * Referenced by: '<S323>/Track coordinates in axle body frame1'
                                                    */
  real_T Trackcoordinatesinaxlebodyframe2_Value;/* Expression: 0
                                                 * Referenced by: '<S323>/Track coordinates in axle body frame2'
                                                 */
  real_T _IC;                          /* Expression: 0
                                        * Referenced by: '<S323>/ '
                                        */
  real_T _UpperSat;                    /* Expression: 0.99*pi/2
                                        * Referenced by: '<S323>/ '
                                        */
  real_T _LowerSat;                    /* Expression: -0.99*pi/2
                                        * Referenced by: '<S323>/ '
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S323>/Gain'
                                        */
  real_T cgcoordinates_IC;             /* Expression: 0
                                        * Referenced by: '<S319>/cg coordinates'
                                        */
  real_T Fy1_Value;                    /* Expression: 0
                                        * Referenced by: '<S317>/Fy1'
                                        */
  real_T Vz_IC;                        /* Expression: 0
                                        * Referenced by: '<S317>/Vz'
                                        */
  real_T Vy1_IC;                       /* Expression: 0
                                        * Referenced by: '<S317>/Vy1'
                                        */
  real_T Vy1_UpperSat;                 /* Expression: 0.99*pi/2
                                        * Referenced by: '<S317>/Vy1'
                                        */
  real_T Vy1_LowerSat;                 /* Expression: -0.99*pi/2
                                        * Referenced by: '<S317>/Vy1'
                                        */
  real_T Fy_Value;                     /* Expression: 0
                                        * Referenced by: '<S317>/Fy'
                                        */
  real_T Vy_IC;                        /* Expression: 0
                                        * Referenced by: '<S317>/Vy'
                                        */
  real_T gEarth_Value;                 /* Expression: 9.807
                                        * Referenced by: '<S317>/g (Earth)'
                                        */
};

/* Parameters for system: '<S341>/Min stop reached' */
struct P_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_ {
  real_T Gain5_Gain;                   /* Expression: 4
                                        * Referenced by: '<S347>/Gain5'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<S347>/Gain4'
                                        */
  real_T Constant_Value;               /* Expression: 3
                                        * Referenced by: '<S347>/Constant'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 4
                                        * Referenced by: '<S347>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S347>/Saturation'
                                        */
  real_T Gain_Gain;                    /* Expression: 3
                                        * Referenced by: '<S347>/Gain'
                                        */
  real_T LowerHardStopBlendMult_tableData[3];/* Expression: [1 1 0]
                                              * Referenced by: '<S347>/Lower Hard Stop Blend Mult'
                                              */
  real_T LowerHardStopBlendMult_bp01Data[3];/* Expression: [-0.02 -0.01 0]
                                             * Referenced by: '<S347>/Lower Hard Stop Blend Mult'
                                             */
};

/* Parameters for system: '<S341>/Max stop reached' */
struct P_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_ {
  real_T Gain5_Gain;                   /* Expression: 4
                                        * Referenced by: '<S346>/Gain5'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<S346>/Gain4'
                                        */
  real_T Constant_Value;               /* Expression: 3
                                        * Referenced by: '<S346>/Constant'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 4
                                        * Referenced by: '<S346>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S346>/Saturation'
                                        */
  real_T Gain_Gain;                    /* Expression: 3
                                        * Referenced by: '<S346>/Gain'
                                        */
  real_T UpperHardStopBlendMult_tableData[3];/* Expression: [0 1 1]
                                              * Referenced by: '<S346>/Upper Hard Stop Blend Mult'
                                              */
  real_T UpperHardStopBlendMult_bp01Data[3];/* Expression: [0 0.01 0.02]
                                             * Referenced by: '<S346>/Upper Hard Stop Blend Mult'
                                             */
};

/* Parameters for system: '<S304>/For each track and axle combination calculate suspension forces and moments' */
struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_d_T_ {
  real_T SelectCamberSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCamberSteeringCenter_AxleNums
                           * Referenced by: '<S333>/Axle Numbers'
                           */
  real_T SelectCamberHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCamberHeightSlope_AxleNums
                              * Referenced by: '<S332>/Axle Numbers'
                              */
  real_T Constrainedspringdampercombination_AxleNums;
                  /* Mask Parameter: Constrainedspringdampercombination_AxleNums
                   * Referenced by:
                   *   '<S342>/Axle Numbers'
                   *   '<S343>/Axle Numbers'
                   *   '<S344>/Axle Numbers'
                   *   '<S345>/Axle Numbers'
                   */
  real_T SelectCasterSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCasterSteeringCenter_AxleNums
                           * Referenced by: '<S335>/Axle Numbers'
                           */
  real_T SelectCasterHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCasterHeightSlope_AxleNums
                              * Referenced by: '<S334>/Axle Numbers'
                              */
  real_T SelectToeSteeringCenter_AxleNums;
                             /* Mask Parameter: SelectToeSteeringCenter_AxleNums
                              * Referenced by: '<S337>/Axle Numbers'
                              */
  real_T SelectRollSteerSlope_AxleNums;
                                /* Mask Parameter: SelectRollSteerSlope_AxleNums
                                 * Referenced by: '<S336>/Axle Numbers'
                                 */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S348>/Constant'
                                        */
  real_T Constant_Value_m;             /* Expression: 0
                                        * Referenced by: '<S327>/Constant'
                                        */
  real_T VehicleVehicleTrackOffset3_Value[2];/* Expression: [0 cumsum(NumTracksByAxl)]
                                              * Referenced by: '<S326>/Vehicle Vehicle Track Offset3'
                                              */
  real_T VehicleVehicleTrackOffset1_Value;/* Expression: StrgEnByAxl
                                           * Referenced by: '<S327>/Vehicle Vehicle Track Offset1'
                                           */
  real_T HeightSignConvention_Gain;    /* Expression: -1
                                        * Referenced by: '<S339>/Height Sign Convention'
                                        */
  real_T VehicleForceSign_Gain;        /* Expression: -1
                                        * Referenced by: '<S338>/Vehicle Force Sign'
                                        */
  real_T Signconvention_Gain;          /* Expression: -1
                                        * Referenced by: '<S338>/Sign convention'
                                        */
  real_T Constant_Value_a;             /* Expression: 0
                                        * Referenced by: '<S329>/Constant'
                                        */
  P_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Maxstopreached;/* '<S341>/Max stop reached' */
  P_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Minstopreached;/* '<S341>/Min stop reached' */
};

/* Parameters for system: '<S351>/For Each Axle With Anti-Sway' */
struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_e_T_ {
  real_T AntiSwayArmRadiusByAxle_AxleNums;
                             /* Mask Parameter: AntiSwayArmRadiusByAxle_AxleNums
                              * Referenced by: '<S355>/Axle Numbers'
                              */
  real_T AntiSwayArmNeutralAngleByAxle_AxleNums;
                       /* Mask Parameter: AntiSwayArmNeutralAngleByAxle_AxleNums
                        * Referenced by: '<S354>/Axle Numbers'
                        */
  real_T AntiSwayBarTorsionSpringConstantByAxle_AxleNums;
              /* Mask Parameter: AntiSwayBarTorsionSpringConstantByAxle_AxleNums
               * Referenced by: '<S356>/Axle Numbers'
               */
  real_T VehicleVehicleTrackOffset3_Value[2];/* Expression: [0 cumsum(NumTracksByAxl)]
                                              * Referenced by: '<S352>/Vehicle Vehicle Track Offset3'
                                              */
  real_T Constant_Value[2];            /* Expression: 1:2
                                        * Referenced by: '<S352>/Constant'
                                        */
  real_T AngleTangentLimit_tableData[2];/* Expression: [-1 1]
                                         * Referenced by: '<S353>/Angle Tangent Limit'
                                         */
  real_T AngleTangentLimit_bp01Data[2];/* Expression: [-1 1]
                                        * Referenced by: '<S353>/Angle Tangent Limit'
                                        */
};

/* Parameters for system: '<S309>/For each track and axle combination calculate suspension forces and moments' */
struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_a_T_ {
  real_T SelectCamberSteeringSlope_AxleNums;
                           /* Mask Parameter: SelectCamberSteeringSlope_AxleNums
                            * Referenced by: '<S365>/Axle Numbers'
                            */
  real_T SelectCamberSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCamberSteeringCenter_AxleNums
                           * Referenced by: '<S364>/Axle Numbers'
                           */
  real_T SelectCamberHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCamberHeightSlope_AxleNums
                              * Referenced by: '<S363>/Axle Numbers'
                              */
  real_T SteeringHeightSlopeBySteeredAxle_AxleNums;
                    /* Mask Parameter: SteeringHeightSlopeBySteeredAxle_AxleNums
                     * Referenced by: '<S383>/Axle Numbers'
                     */
  real_T Constrainedspringdampercombination_AxleNums;
                  /* Mask Parameter: Constrainedspringdampercombination_AxleNums
                   * Referenced by:
                   *   '<S376>/Axle Numbers'
                   *   '<S377>/Axle Numbers'
                   *   '<S378>/Axle Numbers'
                   *   '<S379>/Axle Numbers'
                   */
  real_T SelectCasterSteeringSlope_AxleNums;
                           /* Mask Parameter: SelectCasterSteeringSlope_AxleNums
                            * Referenced by: '<S368>/Axle Numbers'
                            */
  real_T SelectCasterSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCasterSteeringCenter_AxleNums
                           * Referenced by: '<S367>/Axle Numbers'
                           */
  real_T SelectCasterHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCasterHeightSlope_AxleNums
                              * Referenced by: '<S366>/Axle Numbers'
                              */
  real_T SelectToeSteeringSlope_AxleNums;
                              /* Mask Parameter: SelectToeSteeringSlope_AxleNums
                               * Referenced by: '<S371>/Axle Numbers'
                               */
  real_T SelectToeSteeringCenter_AxleNums;
                             /* Mask Parameter: SelectToeSteeringCenter_AxleNums
                              * Referenced by: '<S370>/Axle Numbers'
                              */
  real_T SelectRollSteerSlope_AxleNums;
                                /* Mask Parameter: SelectRollSteerSlope_AxleNums
                                 * Referenced by: '<S369>/Axle Numbers'
                                 */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S358>/Constant'
                                        */
  real_T VehicleVehicleTrackOffset3_Value[2];/* Expression: [0 cumsum(NumTracksByAxl)]
                                              * Referenced by: '<S357>/Vehicle Vehicle Track Offset3'
                                              */
  real_T VehicleVehicleTrackOffset1_Value;/* Expression: StrgEnByAxl
                                           * Referenced by: '<S358>/Vehicle Vehicle Track Offset1'
                                           */
  real_T HeightSignConvention_Gain;    /* Expression: -1
                                        * Referenced by: '<S373>/Height Sign Convention'
                                        */
  real_T VehicleForceSign_Gain;        /* Expression: -1
                                        * Referenced by: '<S372>/Vehicle Force Sign'
                                        */
  real_T Signconvention_Gain;          /* Expression: -1
                                        * Referenced by: '<S372>/Sign convention'
                                        */
  real_T Constant_Value_j;             /* Expression: 0
                                        * Referenced by: '<S360>/Constant'
                                        */
  P_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Maxstopreached;/* '<S375>/Max stop reached' */
  P_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T Minstopreached;/* '<S375>/Min stop reached' */
};

/* Parameters for system: '<S515>/LockUp' */
struct P_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S527>/Constant'
                                        */
  real_T locked_Value;                 /* Expression: 0
                                        * Referenced by: '<S525>/locked'
                                        */
  real_T locked1_Value;                /* Expression: 0
                                        * Referenced by: '<S525>/locked1'
                                        */
  real_T locked2_Value;                /* Expression: 0
                                        * Referenced by: '<S525>/locked2'
                                        */
  real_T u_Gain;                       /* Expression: -4
                                        * Referenced by: '<S526>/-4'
                                        */
  boolean_T yn_Y0;                     /* Computed Parameter: yn_Y0
                                        * Referenced by: '<S528>/yn'
                                        */
  boolean_T yn_Y0_m;                   /* Computed Parameter: yn_Y0_m
                                        * Referenced by: '<S527>/yn'
                                        */
  boolean_T UnitDelay_InitialCondition;
                               /* Computed Parameter: UnitDelay_InitialCondition
                                * Referenced by: '<S532>/Unit Delay'
                                */
  boolean_T CombinatorialLogic_table[8];
                                 /* Computed Parameter: CombinatorialLogic_table
                                  * Referenced by: '<S532>/Combinatorial  Logic'
                                  */
};

/* Parameters (default storage) */
struct P_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_ {
  struct_KVvNvmPYkDwFa3qoVeWWJC MCT;   /* Variable: MCT
                                        * Referenced by: '<S89>/MCT'
                                        */
  struct_akRwpZPsceW5LlhYlX5DVF pMCT;  /* Variable: pMCT
                                        * Referenced by: '<S89>/pMCT'
                                        */
  struct_LqXVUGdvzWUJgx4oCacWrF CoastDown;/* Variable: CoastDown
                                           * Referenced by: '<S89>/CoastDown'
                                           */
  struct_UDems7iN2Kzz7DJ6EtAQJH limitedMCT;/* Variable: limitedMCT
                                            * Referenced by: '<S89>/limitedMCT'
                                            */
  struct_H6OzwuYMZP7H9bFCBCIB9C US06;  /* Variable: US06
                                        * Referenced by: '<S89>/US06'
                                        */
  struct_m2VjwNiXoluKspK4Fr7zNG InitStates[34];/* Variable: InitStates
                                                * Referenced by: '<S124>/Constant1'
                                                */
  real_T BattChargeLimitMaxPwr;        /* Variable: BattChargeLimitMaxPwr
                                        * Referenced by: '<S166>/MaxChrg'
                                        */
  real_T BattChargeLimitSocBpts[12];   /* Variable: BattChargeLimitSocBpts
                                        * Referenced by:
                                        *   '<S166>/ChrgLmt'
                                        *   '<S168>/ChrgLmt'
                                        */
  real_T BattChargeLimitTbl[12];       /* Variable: BattChargeLimitTbl
                                        * Referenced by:
                                        *   '<S166>/ChrgLmt'
                                        *   '<S168>/ChrgLmt'
                                        */
  real_T BattChrgCapcty;               /* Variable: BattChrgCapcty
                                        * Referenced by:
                                        *   '<S124>/Constant1'
                                        *   '<S128>/Constant1'
                                        *   '<S128>/Integrator Limited'
                                        *   '<S128>/Switch'
                                        *   '<S129>/Constant1'
                                        */
  real_T BattDischargeLimitSocBpts[11];/* Variable: BattDischargeLimitSocBpts
                                        * Referenced by: '<S166>/DischrgLmt'
                                        */
  real_T BattDischargeLimitTbl[11];    /* Variable: BattDischargeLimitTbl
                                        * Referenced by: '<S166>/DischrgLmt'
                                        */
  real_T BattDischargeMaxPwr;          /* Variable: BattDischargeMaxPwr
                                        * Referenced by: '<S166>/MaxDischrg'
                                        */
  real_T BattNumCellsParallel;         /* Variable: BattNumCellsParallel
                                        * Referenced by:
                                        *   '<S128>/Gain1'
                                        *   '<S130>/Gain2'
                                        *   '<S130>/Gain4'
                                        */
  real_T BattNumCellsSeries;           /* Variable: BattNumCellsSeries
                                        * Referenced by:
                                        *   '<S130>/Gain1'
                                        *   '<S130>/Gain3'
                                        */
  real_T BattOpenVoltCapBpts[11];      /* Variable: BattOpenVoltCapBpts
                                        * Referenced by: '<S130>/Em'
                                        */
  real_T BattOpenVoltTbl[11];          /* Variable: BattOpenVoltTbl
                                        * Referenced by: '<S130>/Em'
                                        */
  real_T BattResistSocBpts[6];         /* Variable: BattResistSocBpts
                                        * Referenced by: '<S130>/R'
                                        */
  real_T BattResistTbl[42];            /* Variable: BattResistTbl
                                        * Referenced by: '<S130>/R'
                                        */
  real_T BattResistTempBpts[7];        /* Variable: BattResistTempBpts
                                        * Referenced by: '<S130>/R'
                                        */
  real_T BrakeMaxPrs;                  /* Variable: BrakeMaxPrs
                                        * Referenced by:
                                        *   '<S290>/Gain'
                                        *   '<S290>/Saturation'
                                        *   '<S290>/Saturation1'
                                        *   '<S290>/Saturation2'
                                        *   '<S290>/Saturation3'
                                        */
  real_T BrakeMaxTrq;                  /* Variable: BrakeMaxTrq
                                        * Referenced by:
                                        *   '<S168>/Gain1'
                                        *   '<S168>/Gain2'
                                        */
  real_T ChassisDistCg2FrontAxle;      /* Variable: ChassisDistCg2FrontAxle
                                        * Referenced by:
                                        *   '<S386>/vehdyncginert'
                                        *   '<S419>/Constant3'
                                        */
  real_T ChassisDistCg2RearAxle;       /* Variable: ChassisDistCg2RearAxle
                                        * Referenced by:
                                        *   '<S386>/vehdyncginert'
                                        *   '<S419>/Constant3'
                                        */
  real_T ChassisFrontalArea;           /* Variable: ChassisFrontalArea
                                        * Referenced by: '<S419>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T DiffRatio;                    /* Variable: DiffRatio
                                        * Referenced by:
                                        *   '<S144>/Gain2'
                                        *   '<S144>/Gain3'
                                        *   '<S168>/MotTrqReflectedToWheels'
                                        *   '<S168>/WhlTrqReflectedToMot'
                                        */
  real_T EnvGravCnst;                  /* Variable: EnvGravCnst
                                        * Referenced by: '<S390>/g'
                                        */
  real_T EnvPrs;                       /* Variable: EnvPrs
                                        * Referenced by:
                                        *   '<S83>/Constant'
                                        *   '<S419>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T EnvTemp;                      /* Variable: EnvTemp
                                        * Referenced by:
                                        *   '<S83>/Constant1'
                                        *   '<S121>/Constant'
                                        *   '<S386>/AirTempConstant'
                                        */
  real_T MtrEffSpdBpts[12];            /* Variable: MtrEffSpdBpts
                                        * Referenced by: '<S174>/Eff Map'
                                        */
  real_T MtrEffTbl[180];               /* Variable: MtrEffTbl
                                        * Referenced by: '<S174>/Eff Map'
                                        */
  real_T MtrEffTrqBpts[15];            /* Variable: MtrEffTrqBpts
                                        * Referenced by: '<S174>/Eff Map'
                                        */
  real_T MtrPwrMax;                    /* Variable: MtrPwrMax
                                        * Referenced by:
                                        *   '<S153>/Constant'
                                        *   '<S169>/Constant'
                                        *   '<S184>/Constant'
                                        *   '<S157>/Constant'
                                        *   '<S180>/Constant'
                                        */
  real_T MtrTrqMax;                    /* Variable: MtrTrqMax
                                        * Referenced by:
                                        *   '<S153>/Constant1'
                                        *   '<S169>/Saturation'
                                        *   '<S184>/Saturation'
                                        *   '<S156>/Saturation'
                                        *   '<S180>/Saturation'
                                        */
  real_T MtrTrqTimeCnst;               /* Variable: MtrTrqTimeCnst
                                        * Referenced by: '<S153>/Gain1'
                                        */
  real_T SupvsryCtrlRegenBrkCutOffTbl[2];/* Variable: SupvsryCtrlRegenBrkCutOffTbl
                                          * Referenced by: '<S168>/RegenBrakingCutoff'
                                          */
  real_T SupvsryCtrlRegenSpdBpts[2];   /* Variable: SupvsryCtrlRegenSpdBpts
                                        * Referenced by: '<S168>/RegenBrakingCutoff'
                                        */
  real_T TestScnrioCycleNum;           /* Variable: TestScnrioCycleNum
                                        * Referenced by: '<S124>/Constant1'
                                        */
  real_T CombinedSlipWheel2DOF1_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_ALPMAX
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_ALPMAX
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_ALPMAX
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_ALPMAX
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_ALPMIN
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_ALPMIN
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_ALPMIN
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_ALPMIN
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T independentSuspensionsMacPherson_AntiSwayNtrlAng;
             /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayNtrlAng
              * Referenced by: '<S353>/Constant2'
              */
  real_T independentSuspensionsMacPherson_AntiSwayR;
                   /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayR
                    * Referenced by: '<S353>/Constant1'
                    */
  real_T independentSuspensionsMacPherson_AntiSwayTrsK;
                /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayTrsK
                 * Referenced by: '<S353>/Constant3'
                 */
  real_T SolidAxleSuspensionCoilSpring_AxlIxx;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxlIxx
                          * Referenced by:
                          *   '<S317>/Axle I'
                          *   '<S320>/Axle I1'
                          */
  real_T SolidAxleSuspensionCoilSpring_AxlM;
                           /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxlM
                            * Referenced by:
                            *   '<S317>/Axle M'
                            *   '<S320>/Axle M'
                            */
  real_T SolidAxleSuspensionCoilSpring_AxleNumVec[2];
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxleNumVec
                      * Referenced by:
                      *   '<S304>/Axle Number'
                      *   '<S304>/Axle Number2'
                      *   '<S318>/Constant2'
                      *   '<S320>/Constant2'
                      */
  real_T independentSuspensionsMacPherson_AxleNumVec[2];
                  /* Mask Parameter: independentSuspensionsMacPherson_AxleNumVec
                   * Referenced by: '<S309>/Axle Number'
                   */
  real_T CombinedSlipWheel2DOF1_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_BREFF
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_BREFF
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_BREFF
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_BREFF
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_CAMMAX
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_CAMMAX
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_CAMMAX
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_CAMMAX
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_CAMMIN
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_CAMMIN
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_CAMMIN
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_CAMMIN
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T SolidAxleSuspensionCoilSpring_Camber;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_Camber
                          * Referenced by: '<S331>/Constant4'
                          */
  real_T independentSuspensionsMacPherson_Camber;
                      /* Mask Parameter: independentSuspensionsMacPherson_Camber
                       * Referenced by: '<S362>/Constant4'
                       */
  real_T SolidAxleSuspensionCoilSpring_CamberHslp;
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_CamberHslp
                      * Referenced by: '<S331>/Constant5'
                      */
  real_T independentSuspensionsMacPherson_CamberHslp;
                  /* Mask Parameter: independentSuspensionsMacPherson_CamberHslp
                   * Referenced by: '<S362>/Constant5'
                   */
  real_T independentSuspensionsMacPherson_CamberStrgSlp;
               /* Mask Parameter: independentSuspensionsMacPherson_CamberStrgSlp
                * Referenced by: '<S362>/Constant3'
                */
  real_T SolidAxleSuspensionCoilSpring_Caster;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_Caster
                          * Referenced by: '<S331>/Constant7'
                          */
  real_T independentSuspensionsMacPherson_Caster;
                      /* Mask Parameter: independentSuspensionsMacPherson_Caster
                       * Referenced by: '<S362>/Constant7'
                       */
  real_T SolidAxleSuspensionCoilSpring_CasterHslp;
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_CasterHslp
                      * Referenced by: '<S331>/Constant8'
                      */
  real_T independentSuspensionsMacPherson_CasterHslp;
                  /* Mask Parameter: independentSuspensionsMacPherson_CasterHslp
                   * Referenced by: '<S362>/Constant8'
                   */
  real_T independentSuspensionsMacPherson_CasterStrgSlp;
               /* Mask Parameter: independentSuspensionsMacPherson_CasterStrgSlp
                * Referenced by: '<S362>/Constant6'
                */
  real_T VehicleBody6DOF1_Cd;          /* Mask Parameter: VehicleBody6DOF1_Cd
                                        * Referenced by: '<S419>/Constant'
                                        */
  real_T VehicleBody6DOF1_Cl;          /* Mask Parameter: VehicleBody6DOF1_Cl
                                        * Referenced by: '<S419>/Constant1'
                                        */
  real_T VehicleBody6DOF1_Cpm;         /* Mask Parameter: VehicleBody6DOF1_Cpm
                                        * Referenced by: '<S419>/Constant2'
                                        */
  real_T VehicleBody6DOF1_Cs[31];      /* Mask Parameter: VehicleBody6DOF1_Cs
                                        * Referenced by: '<S419>/Cs'
                                        */
  real_T VehicleBody6DOF1_Cym[31];     /* Mask Parameter: VehicleBody6DOF1_Cym
                                        * Referenced by: '<S419>/Cym'
                                        */
  real_T SolidAxleSuspensionCoilSpring_Cz;
                             /* Mask Parameter: SolidAxleSuspensionCoilSpring_Cz
                              * Referenced by: '<S339>/Constant2'
                              */
  real_T independentSuspensionsMacPherson_Cz;
                          /* Mask Parameter: independentSuspensionsMacPherson_Cz
                           * Referenced by: '<S373>/Constant2'
                           */
  real_T SolidAxleSuspensionCoilSpring_CzWhlAxl;
                       /* Mask Parameter: SolidAxleSuspensionCoilSpring_CzWhlAxl
                        * Referenced by: '<S314>/Carrier to Axle Damping'
                        */
  real_T CombinedSlipWheel2DOF1_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_DREFF
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_DREFF
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_DREFF
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_DREFF
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T KinematicSteering_Db;         /* Mask Parameter: KinematicSteering_Db
                                        * Referenced by: '<S299>/Backlash'
                                        */
  real_T SolidAxleSuspensionCoilSpring_F0z;
                            /* Mask Parameter: SolidAxleSuspensionCoilSpring_F0z
                             * Referenced by: '<S339>/Constant1'
                             */
  real_T independentSuspensionsMacPherson_F0z;
                         /* Mask Parameter: independentSuspensionsMacPherson_F0z
                          * Referenced by: '<S373>/Constant1'
                          */
  real_T SolidAxleSuspensionCoilSpring_F0zWhlAxl;
                      /* Mask Parameter: SolidAxleSuspensionCoilSpring_F0zWhlAxl
                       * Referenced by: '<S314>/Preload'
                       */
  real_T CombinedSlipWheel2DOF1_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_FNOMIN
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_FNOMIN
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_FNOMIN
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_FNOMIN
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FREFF
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FREFF
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FREFF
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FREFF
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FZMAX
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FZMAX
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FZMAX
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FZMAX
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FZMIN
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FZMIN
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FZMIN
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FZMIN
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_GRAVITY
                                * Referenced by: '<S514>/Fg'
                                */
  real_T CombinedSlipWheel2DOF2_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_GRAVITY
                                * Referenced by: '<S539>/Fg'
                                */
  real_T CombinedSlipWheel2DOF3_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_GRAVITY
                                * Referenced by: '<S564>/Fg'
                                */
  real_T CombinedSlipWheel2DOF4_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_GRAVITY
                                * Referenced by: '<S589>/Fg'
                                */
  real_T SolidAxleSuspensionCoilSpring_Hmax;
                           /* Mask Parameter: SolidAxleSuspensionCoilSpring_Hmax
                            * Referenced by:
                            *   '<S339>/Constant3'
                            *   '<S341>/Max stop reached'
                            *   '<S341>/Min stop reached'
                            */
  real_T independentSuspensionsMacPherson_Hmax;
                        /* Mask Parameter: independentSuspensionsMacPherson_Hmax
                         * Referenced by:
                         *   '<S373>/Constant3'
                         *   '<S375>/Max stop reached'
                         *   '<S375>/Min stop reached'
                         */
  real_T CombinedSlipWheel2DOF1_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF1_IYY
                                    * Referenced by: '<S515>/LockUp'
                                    */
  real_T CombinedSlipWheel2DOF2_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF2_IYY
                                    * Referenced by: '<S540>/LockUp'
                                    */
  real_T CombinedSlipWheel2DOF3_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF3_IYY
                                    * Referenced by: '<S565>/LockUp'
                                    */
  real_T CombinedSlipWheel2DOF4_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF4_IYY
                                    * Referenced by: '<S590>/LockUp'
                                    */
  real_T VehicleBody6DOF1_Iveh[9];     /* Mask Parameter: VehicleBody6DOF1_Iveh
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T CombinedSlipWheel2DOF1_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_KPUMAX
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_KPUMAX
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_KPUMAX
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_KPUMAX
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_KPUMIN
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_KPUMIN
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_KPUMIN
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_KPUMIN
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T SolidAxleSuspensionCoilSpring_Kz;
                             /* Mask Parameter: SolidAxleSuspensionCoilSpring_Kz
                              * Referenced by: '<S339>/Constant'
                              */
  real_T independentSuspensionsMacPherson_Kz;
                          /* Mask Parameter: independentSuspensionsMacPherson_Kz
                           * Referenced by: '<S373>/Constant'
                           */
  real_T SolidAxleSuspensionCoilSpring_KzWhlAxl;
                       /* Mask Parameter: SolidAxleSuspensionCoilSpring_KzWhlAxl
                        * Referenced by: '<S314>/Carrier to Axle Compliance'
                        */
  real_T CombinedSlipWheel2DOF1_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF1_LATERAL_STIFFNESS
                      * Referenced by: '<S513>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF2_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF2_LATERAL_STIFFNESS
                      * Referenced by: '<S538>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF3_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF3_LATERAL_STIFFNESS
                      * Referenced by: '<S563>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF4_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF4_LATERAL_STIFFNESS
                      * Referenced by: '<S588>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF1_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF1_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S513>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF2_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF2_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S538>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF3_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF3_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S563>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF4_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF4_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S588>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF1_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_LONGVL
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_LONGVL
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_LONGVL
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_LONGVL
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_MASS
                                   * Referenced by:
                                   *   '<S514>/Fg'
                                   *   '<S514>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF2_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_MASS
                                   * Referenced by:
                                   *   '<S539>/Fg'
                                   *   '<S539>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF3_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_MASS
                                   * Referenced by:
                                   *   '<S564>/Fg'
                                   *   '<S564>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF4_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_MASS
                                   * Referenced by:
                                   *   '<S589>/Fg'
                                   *   '<S589>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF1_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_NOMPRES
                                * Referenced by: '<S513>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_NOMPRES
                                * Referenced by: '<S538>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_NOMPRES
                                * Referenced by: '<S563>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_NOMPRES
                                * Referenced by: '<S588>/Magic Tire Const Input'
                                */
  real_T LockUp_OmegaTol;              /* Mask Parameter: LockUp_OmegaTol
                                        * Referenced by: '<S515>/LockUp'
                                        */
  real_T LockUp_OmegaTol_p;            /* Mask Parameter: LockUp_OmegaTol_p
                                        * Referenced by: '<S540>/LockUp'
                                        */
  real_T LockUp_OmegaTol_l;            /* Mask Parameter: LockUp_OmegaTol_l
                                        * Referenced by: '<S565>/LockUp'
                                        */
  real_T LockUp_OmegaTol_d;            /* Mask Parameter: LockUp_OmegaTol_d
                                        * Referenced by: '<S590>/LockUp'
                                        */
  real_T CombinedSlipWheel2DOF1_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX3
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX3
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX3
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX3
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY3
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY3
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY3
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY3
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PCX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PCX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PCX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PCX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PCY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PCY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PCY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PCY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP3
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP3
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP3
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP3
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP3
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP3
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP3
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP3
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP4
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP4
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP4
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP4
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PECP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PECP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PECP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PECP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PECP2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PECP2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PECP2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PECP2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PFZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PFZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PFZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PFZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP3
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP3
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP3
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP3
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP4
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP4
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP4
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP4
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY6
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY6
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY6
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY6
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY7
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY7
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY7
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY7
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PKYP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PKYP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PKYP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PKYP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PPMX1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PPMX1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PPMX1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PPMX1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPZ2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPZ2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPZ2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPZ2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_PRESMAX
                                * Referenced by: '<S513>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_PRESMAX
                                * Referenced by: '<S538>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_PRESMAX
                                * Referenced by: '<S563>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_PRESMAX
                                * Referenced by: '<S588>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF1_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_PRESMIN
                                * Referenced by: '<S513>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_PRESMIN
                                * Referenced by: '<S538>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_PRESMIN
                                * Referenced by: '<S563>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_PRESMIN
                                * Referenced by: '<S588>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF1_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QBRP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QBRP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QBRP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QBRP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ10
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ10
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ10
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ10
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ6
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ6
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ6
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ6
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ9
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ9
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ9
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ9
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QCRP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QCRP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QCRP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QCRP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QCRP2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QCRP2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QCRP2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QCRP2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QCZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QCZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QCZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QCZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDRP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDRP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDRP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDRP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDRP2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDRP2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDRP2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDRP2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDTP1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDTP1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDTP1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDTP1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ10
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ10
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ10
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ10
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ11
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ11
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ11
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ11
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ6
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ6
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ6
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ6
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ7
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ7
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ7
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ7
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ8
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ8
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ8
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ8
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ9
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ9
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ9
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ9
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX10
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX10
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX10
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX10
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX11
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX11
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX11
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX11
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX12
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX12
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX12
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX12
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX13
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX13
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX13
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX13
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX14
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX14
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX14
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX14
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX6
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX6
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX6
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX6
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX7
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX7
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX7
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX7
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX8
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX8
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX8
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX8
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX9
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX9
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX9
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX9
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY6
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY6
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY6
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY6
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY7
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY7
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY7
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY7
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY8
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY8
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY8
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY8
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCX
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCX
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCX
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCX
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCY
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCY
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCY
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCY
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCY2
                                 * Referenced by: '<S513>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCY2
                                 * Referenced by: '<S538>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCY2
                                 * Referenced by: '<S563>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCY2
                                 * Referenced by: '<S588>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ3
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ3
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ3
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ3
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RA1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RA1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RA1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RA1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RA2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RA2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RA2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RA2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RB1
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RB1
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RB1
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RB1
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RB2
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RB2
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RB2
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RB2
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RE0
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RE0
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RE0
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RE0
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_Q_V1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_Q_V1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_Q_V1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_Q_V1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_Q_V2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_Q_V2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_Q_V2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_Q_V2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T DragForce_R;                  /* Mask Parameter: DragForce_R
                                        * Referenced by: '<S419>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T CombinedSlipWheel2DOF1_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RCX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RCX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RCX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RCX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RCY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RCY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RCY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RCY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REX2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REX2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REX2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REX2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHX1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHX1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHX1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHX1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY5
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY5
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY5
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY5
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY6
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY6
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY6
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY6
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Rm; /* Mask Parameter: CombinedSlipWheel2DOF1_Rm
                                     * Referenced by: '<S524>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF2_Rm; /* Mask Parameter: CombinedSlipWheel2DOF2_Rm
                                     * Referenced by: '<S549>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF3_Rm; /* Mask Parameter: CombinedSlipWheel2DOF3_Rm
                                     * Referenced by: '<S574>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF4_Rm; /* Mask Parameter: CombinedSlipWheel2DOF4_Rm
                                     * Referenced by: '<S599>/Torque Conversion'
                                     */
  real_T SolidAxleSuspensionCoilSpring_RollStrgSlp;
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_RollStrgSlp
                     * Referenced by: '<S331>/Constant2'
                     */
  real_T independentSuspensionsMacPherson_RollStrgSlp;
                 /* Mask Parameter: independentSuspensionsMacPherson_RollStrgSlp
                  * Referenced by: '<S362>/Constant2'
                  */
  real_T CombinedSlipWheel2DOF1_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ1
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ1
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ1
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ1
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ2
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ2
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ2
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ2
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ3
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ3
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ3
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ3
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ4
                                   * Referenced by: '<S513>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ4
                                   * Referenced by: '<S538>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ4
                                   * Referenced by: '<S563>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ4
                                   * Referenced by: '<S588>/Magic Tire Const Input'
                                   */
  real_T independentSuspensionsMacPherson_StrgHgtSlp;
                  /* Mask Parameter: independentSuspensionsMacPherson_StrgHgtSlp
                   * Referenced by: '<S382>/Constant'
                   */
  real_T KinematicSteering_StrgRatio;
                                  /* Mask Parameter: KinematicSteering_StrgRatio
                                   * Referenced by:
                                   *   '<S300>/Constant'
                                   *   '<S300>/Gain'
                                   *   '<S300>/Gain1'
                                   *   '<S300>/Gain2'
                                   */
  real_T KinematicSteering_StrgRng; /* Mask Parameter: KinematicSteering_StrgRng
                                     * Referenced by: '<S299>/Saturation'
                                     */
  real_T SolidAxleSuspensionCoilSpring_SuspCoords[6];
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_SuspCoords
                      * Referenced by:
                      *   '<S311>/Suspension axle connection coordinates in axle body frame'
                      *   '<S318>/Suspension connection point coordinates in axle body frame'
                      */
  real_T CANdiagnosticCounter_TimeOutDuration;
                         /* Mask Parameter: CANdiagnosticCounter_TimeOutDuration
                          * Referenced by: '<S112>/Constant'
                          */
  real_T SolidAxleSuspensionCoilSpring_Toe;
                            /* Mask Parameter: SolidAxleSuspensionCoilSpring_Toe
                             * Referenced by: '<S331>/Constant1'
                             */
  real_T independentSuspensionsMacPherson_Toe;
                         /* Mask Parameter: independentSuspensionsMacPherson_Toe
                          * Referenced by: '<S362>/Constant1'
                          */
  real_T independentSuspensionsMacPherson_ToeStrgSlp;
                  /* Mask Parameter: independentSuspensionsMacPherson_ToeStrgSlp
                   * Referenced by: '<S362>/Constant'
                   */
  real_T SolidAxleSuspensionCoilSpring_TrackCoords[6];
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_TrackCoords
                     * Referenced by:
                     *   '<S311>/Track coordinates in axle body frame'
                     *   '<S320>/Track coordinates in axle body frame'
                     */
  real_T SolidAxleSuspensionCoilSpring_TrackNumVec[2];
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_TrackNumVec
                     * Referenced by: '<S304>/Track Number'
                     */
  real_T independentSuspensionsMacPherson_TrackNumVec[2];
                 /* Mask Parameter: independentSuspensionsMacPherson_TrackNumVec
                  * Referenced by: '<S309>/Track Number'
                  */
  real_T CombinedSlipWheel2DOF1_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF1_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S513>/Magic Tire Const Input'
                        *   '<S515>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF2_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF2_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S538>/Magic Tire Const Input'
                        *   '<S540>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF3_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF3_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S563>/Magic Tire Const Input'
                        *   '<S565>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF4_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF4_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S588>/Magic Tire Const Input'
                        *   '<S590>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF1_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF1_VERTICAL_DAMPING
                       * Referenced by: '<S514>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF2_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF2_VERTICAL_DAMPING
                       * Referenced by: '<S539>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF3_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF3_VERTICAL_DAMPING
                       * Referenced by: '<S564>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF4_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF4_VERTICAL_DAMPING
                       * Referenced by: '<S589>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF1_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF1_VERTICAL_STIFFNESS
                     * Referenced by: '<S513>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF2_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF2_VERTICAL_STIFFNESS
                     * Referenced by: '<S538>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF3_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF3_VERTICAL_STIFFNESS
                     * Referenced by: '<S563>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF4_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF4_VERTICAL_STIFFNESS
                     * Referenced by: '<S588>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF1_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_VXLOW
                                  * Referenced by:
                                  *   '<S513>/Magic Tire Const Input'
                                  *   '<S516>/Dead Zone'
                                  *   '<S517>/Dead Zone'
                                  *   '<S519>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF2_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_VXLOW
                                  * Referenced by:
                                  *   '<S538>/Magic Tire Const Input'
                                  *   '<S541>/Dead Zone'
                                  *   '<S542>/Dead Zone'
                                  *   '<S544>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF3_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_VXLOW
                                  * Referenced by:
                                  *   '<S563>/Magic Tire Const Input'
                                  *   '<S566>/Dead Zone'
                                  *   '<S567>/Dead Zone'
                                  *   '<S569>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF4_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_VXLOW
                                  * Referenced by:
                                  *   '<S588>/Magic Tire Const Input'
                                  *   '<S591>/Dead Zone'
                                  *   '<S592>/Dead Zone'
                                  *   '<S594>/Dead Zone'
                                  */
  real_T CombinedSlipWheel2DOF1_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_WIDTH
                                  * Referenced by: '<S513>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_WIDTH
                                  * Referenced by: '<S538>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_WIDTH
                                  * Referenced by: '<S563>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_WIDTH
                                  * Referenced by: '<S588>/Magic Tire Const Input'
                                  */
  real_T VehicleBody6DOF1_Xe_o[3];     /* Mask Parameter: VehicleBody6DOF1_Xe_o
                                        * Referenced by: '<S387>/xe,ye,ze'
                                        */
  real_T VehicleBody6DOF1_beta_w[31]; /* Mask Parameter: VehicleBody6DOF1_beta_w
                                       * Referenced by:
                                       *   '<S419>/Cs'
                                       *   '<S419>/Cym'
                                       */
  real_T CombinedSlipWheel2DOF1_br; /* Mask Parameter: CombinedSlipWheel2DOF1_br
                                     * Referenced by: '<S515>/LockUp'
                                     */
  real_T CombinedSlipWheel2DOF2_br; /* Mask Parameter: CombinedSlipWheel2DOF2_br
                                     * Referenced by: '<S540>/LockUp'
                                     */
  real_T CombinedSlipWheel2DOF3_br; /* Mask Parameter: CombinedSlipWheel2DOF3_br
                                     * Referenced by: '<S565>/LockUp'
                                     */
  real_T CombinedSlipWheel2DOF4_br; /* Mask Parameter: CombinedSlipWheel2DOF4_br
                                     * Referenced by: '<S590>/LockUp'
                                     */
  real_T VerticalWheelandUnsprungMassResponse_bz;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse_bz
                       * Referenced by: '<S503>/Gain2'
                       */
  real_T VerticalWheelandUnsprungMassResponse1_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_bz
                      * Referenced by: '<S504>/Gain2'
                      */
  real_T VerticalWheelandUnsprungMassResponse2_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_bz
                      * Referenced by: '<S505>/Gain2'
                      */
  real_T VerticalWheelandUnsprungMassResponse3_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_bz
                      * Referenced by: '<S506>/Gain2'
                      */
  real_T CompareToConstant1_const;   /* Mask Parameter: CompareToConstant1_const
                                      * Referenced by: '<S138>/Constant'
                                      */
  real_T CompareToConstant3_const;   /* Mask Parameter: CompareToConstant3_const
                                      * Referenced by: '<S139>/Constant'
                                      */
  real_T Pressure_const;               /* Mask Parameter: Pressure_const
                                        * Referenced by: '<S481>/Pressure'
                                        */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S170>/Constant'
                                       */
  real_T CompareToConstant_const_l; /* Mask Parameter: CompareToConstant_const_l
                                     * Referenced by: '<S173>/Constant'
                                     */
  real_T CompareToConstant_const_i; /* Mask Parameter: CompareToConstant_const_i
                                     * Referenced by: '<S265>/Constant'
                                     */
  real_T CompareToConstant1_const_o;
                                   /* Mask Parameter: CompareToConstant1_const_o
                                    * Referenced by: '<S266>/Constant'
                                    */
  real_T VehicleBody6DOF1_d;           /* Mask Parameter: VehicleBody6DOF1_d
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T CombinedSlipWheel2DOF1_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF1_disk_abore
                             * Referenced by: '<S524>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF2_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF2_disk_abore
                             * Referenced by: '<S549>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF3_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF3_disk_abore
                             * Referenced by: '<S574>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF4_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF4_disk_abore
                             * Referenced by: '<S599>/Disk brake actuator bore'
                             */
  real_T VehicleBody6DOF1_eul_o[3];    /* Mask Parameter: VehicleBody6DOF1_eul_o
                                        * Referenced by: '<S399>/phi theta psi'
                                        */
  real_T VerticalWheelandUnsprungMassResponse_g;
                       /* Mask Parameter: VerticalWheelandUnsprungMassResponse_g
                        * Referenced by: '<S503>/Fg'
                        */
  real_T VerticalWheelandUnsprungMassResponse1_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_g
                       * Referenced by: '<S504>/Fg'
                       */
  real_T VerticalWheelandUnsprungMassResponse2_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_g
                       * Referenced by: '<S505>/Fg'
                       */
  real_T VerticalWheelandUnsprungMassResponse3_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_g
                       * Referenced by: '<S506>/Fg'
                       */
  real_T VehicleBody6DOF1_h;           /* Mask Parameter: VehicleBody6DOF1_h
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_latOff;     /* Mask Parameter: VehicleBody6DOF1_latOff
                                       * Referenced by: '<S422>/latOff'
                                       */
  real_T VehicleBody6DOF1_longOff;   /* Mask Parameter: VehicleBody6DOF1_longOff
                                      * Referenced by: '<S422>/longOff'
                                      */
  real_T VehicleBody6DOF1_m;           /* Mask Parameter: VehicleBody6DOF1_m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VerticalWheelandUnsprungMassResponse_m;
                       /* Mask Parameter: VerticalWheelandUnsprungMassResponse_m
                        * Referenced by:
                        *   '<S503>/Fg'
                        *   '<S503>/Gain1'
                        */
  real_T VerticalWheelandUnsprungMassResponse1_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_m
                       * Referenced by:
                       *   '<S504>/Fg'
                       *   '<S504>/Gain1'
                       */
  real_T VerticalWheelandUnsprungMassResponse2_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_m
                       * Referenced by:
                       *   '<S505>/Fg'
                       *   '<S505>/Gain1'
                       */
  real_T VerticalWheelandUnsprungMassResponse3_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_m
                       * Referenced by:
                       *   '<S506>/Fg'
                       *   '<S506>/Gain1'
                       */
  real_T CombinedSlipWheel2DOF1_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF1_mu_kinetic
                             * Referenced by:
                             *   '<S521>/Ratio of static to kinetic'
                             *   '<S524>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF2_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF2_mu_kinetic
                             * Referenced by:
                             *   '<S546>/Ratio of static to kinetic'
                             *   '<S549>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF3_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF3_mu_kinetic
                             * Referenced by:
                             *   '<S571>/Ratio of static to kinetic'
                             *   '<S574>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF4_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF4_mu_kinetic
                             * Referenced by:
                             *   '<S596>/Ratio of static to kinetic'
                             *   '<S599>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF1_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF1_mu_static
                              * Referenced by: '<S521>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF2_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF2_mu_static
                              * Referenced by: '<S546>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF3_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF3_mu_static
                              * Referenced by: '<S571>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF4_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF4_mu_static
                              * Referenced by: '<S596>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF1_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF1_num_pads
                               * Referenced by: '<S524>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF2_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF2_num_pads
                               * Referenced by: '<S549>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF3_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF3_num_pads
                               * Referenced by: '<S574>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF4_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF4_num_pads
                               * Referenced by: '<S599>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF1_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_omegao
                                 * Referenced by: '<S515>/LockUp'
                                 */
  real_T CombinedSlipWheel2DOF2_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_omegao
                                 * Referenced by: '<S540>/LockUp'
                                 */
  real_T CombinedSlipWheel2DOF3_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_omegao
                                 * Referenced by: '<S565>/LockUp'
                                 */
  real_T CombinedSlipWheel2DOF4_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_omegao
                                 * Referenced by: '<S590>/LockUp'
                                 */
  real_T VehicleBody6DOF1_p_o[3];      /* Mask Parameter: VehicleBody6DOF1_p_o
                                        * Referenced by: '<S387>/p,q,r '
                                        */
  real_T div0protectpoly_thresh;       /* Mask Parameter: div0protectpoly_thresh
                                        * Referenced by:
                                        *   '<S182>/Constant'
                                        *   '<S183>/Constant'
                                        */
  real_T VehicleBody6DOF1_vertOff;   /* Mask Parameter: VehicleBody6DOF1_vertOff
                                      * Referenced by: '<S422>/vertOff '
                                      */
  real_T VehicleBody6DOF1_w[2];        /* Mask Parameter: VehicleBody6DOF1_w
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T ContLPF_wc;                   /* Mask Parameter: ContLPF_wc
                                        * Referenced by: '<S479>/Constant'
                                        */
  real_T ContLPF1_wc;                  /* Mask Parameter: ContLPF1_wc
                                        * Referenced by: '<S480>/Constant'
                                        */
  real_T VehicleBody6DOF1_xbdot_o[3];/* Mask Parameter: VehicleBody6DOF1_xbdot_o
                                      * Referenced by: '<S387>/ub,vb,wb'
                                      */
  real_T VehicleBody6DOF1_xdot_tol; /* Mask Parameter: VehicleBody6DOF1_xdot_tol
                                     * Referenced by:
                                     *   '<S475>/Constant'
                                     *   '<S476>/Constant'
                                     *   '<S448>/Constant'
                                     *   '<S449>/Constant'
                                     */
  real_T VehicleBody6DOF1_z1I[9];      /* Mask Parameter: VehicleBody6DOF1_z1I
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z1R[3];      /* Mask Parameter: VehicleBody6DOF1_z1R
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z1m;         /* Mask Parameter: VehicleBody6DOF1_z1m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z2I[9];      /* Mask Parameter: VehicleBody6DOF1_z2I
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z2R[3];      /* Mask Parameter: VehicleBody6DOF1_z2R
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z2m;         /* Mask Parameter: VehicleBody6DOF1_z2m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z3I[9];      /* Mask Parameter: VehicleBody6DOF1_z3I
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z3R[3];      /* Mask Parameter: VehicleBody6DOF1_z3R
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z3m;         /* Mask Parameter: VehicleBody6DOF1_z3m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z4I[9];      /* Mask Parameter: VehicleBody6DOF1_z4I
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z4R[3];      /* Mask Parameter: VehicleBody6DOF1_z4R
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z4m;         /* Mask Parameter: VehicleBody6DOF1_z4m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z5I[9];      /* Mask Parameter: VehicleBody6DOF1_z5I
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z5R[3];      /* Mask Parameter: VehicleBody6DOF1_z5R
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z5m;         /* Mask Parameter: VehicleBody6DOF1_z5m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z6I[9];      /* Mask Parameter: VehicleBody6DOF1_z6I
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z6R[3];      /* Mask Parameter: VehicleBody6DOF1_z6R
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z6m;         /* Mask Parameter: VehicleBody6DOF1_z6m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z7I[9];      /* Mask Parameter: VehicleBody6DOF1_z7I
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z7R[3];      /* Mask Parameter: VehicleBody6DOF1_z7R
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T VehicleBody6DOF1_z7m;         /* Mask Parameter: VehicleBody6DOF1_z7m
                                        * Referenced by: '<S386>/vehdyncginert'
                                        */
  real_T CombinedSlipWheel2DOF1_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_zdoto
                                  * Referenced by: '<S514>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF2_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_zdoto
                                  * Referenced by: '<S539>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF3_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_zdoto
                                  * Referenced by: '<S564>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF4_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_zdoto
                                  * Referenced by: '<S589>/Integrator, Second-Order'
                                  */
  real_T VerticalWheelandUnsprungMassResponse_zdoto;
                   /* Mask Parameter: VerticalWheelandUnsprungMassResponse_zdoto
                    * Referenced by: '<S503>/Integrator, Second-Order'
                    */
  real_T VerticalWheelandUnsprungMassResponse1_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_zdoto
                   * Referenced by: '<S504>/Integrator, Second-Order'
                   */
  real_T VerticalWheelandUnsprungMassResponse2_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_zdoto
                   * Referenced by: '<S505>/Integrator, Second-Order'
                   */
  real_T VerticalWheelandUnsprungMassResponse3_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_zdoto
                   * Referenced by: '<S506>/Integrator, Second-Order'
                   */
  real_T CombinedSlipWheel2DOF1_zo; /* Mask Parameter: CombinedSlipWheel2DOF1_zo
                                     * Referenced by: '<S514>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF2_zo; /* Mask Parameter: CombinedSlipWheel2DOF2_zo
                                     * Referenced by: '<S539>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF3_zo; /* Mask Parameter: CombinedSlipWheel2DOF3_zo
                                     * Referenced by: '<S564>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF4_zo; /* Mask Parameter: CombinedSlipWheel2DOF4_zo
                                     * Referenced by: '<S589>/Integrator, Second-Order'
                                     */
  real_T VerticalWheelandUnsprungMassResponse_zo;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse_zo
                       * Referenced by: '<S503>/Integrator, Second-Order'
                       */
  real_T VerticalWheelandUnsprungMassResponse1_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_zo
                      * Referenced by: '<S504>/Integrator, Second-Order'
                      */
  real_T VerticalWheelandUnsprungMassResponse2_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_zo
                      * Referenced by: '<S505>/Integrator, Second-Order'
                      */
  real_T VerticalWheelandUnsprungMassResponse3_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_zo
                      * Referenced by: '<S506>/Integrator, Second-Order'
                      */
  boolean_T DetectFallNegative_vinit;/* Mask Parameter: DetectFallNegative_vinit
                                      * Referenced by: '<S140>/Delay Input1'
                                      */
  boolean_T DetectRisePositive_vinit;/* Mask Parameter: DetectRisePositive_vinit
                                      * Referenced by: '<S141>/Delay Input1'
                                      */
  uint8_T WrapToZero_Threshold;        /* Mask Parameter: WrapToZero_Threshold
                                        * Referenced by: '<S288>/FixPt Switch'
                                        */
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S115>/Out1'
                                        */
  real_T MaxStopOffset_Value;          /* Expression: -5500
                                        * Referenced by: '<S115>/MaxStopOffset'
                                        */
  real_T Out1_Y0_m;                    /* Expression: [0]
                                        * Referenced by: '<S116>/Out1'
                                        */
  real_T MinStopOffset_Value;          /* Expression: 3500
                                        * Referenced by: '<S116>/MinStopOffset'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S142>/Constant'
                                        */
  real_T Constant_Value_k;             /* Expression: 0
                                        * Referenced by: '<S143>/Constant'
                                        */
  real_T driverlookaheadseconds_Value; /* Expression: 0.5
                                        * Referenced by: '<S89>/driver look ahead seconds'
                                        */
  real_T Memory4_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S136>/Memory4'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S136>/Switch'
                                        */
  real_T DriveCycleSelect_Value;       /* Expression: 1
                                        * Referenced by: '<S89>/DriveCycleSelect'
                                        */
  real_T Constant_Value_h;             /* Expression: 1
                                        * Referenced by: '<S135>/Constant'
                                        */
  real_T Memory1_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S135>/Memory1'
                                        */
  real_T Memory2_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S135>/Memory2'
                                        */
  real_T SpdSat_UpperSat;              /* Expression: 190
                                        * Referenced by: '<S135>/SpdSat'
                                        */
  real_T SpdSat_LowerSat;              /* Expression: -5
                                        * Referenced by: '<S135>/SpdSat'
                                        */
  real_T Memory1_InitialCondition_c;   /* Expression: 0
                                        * Referenced by: '<S137>/Memory1'
                                        */
  real_T Memory_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S137>/Memory'
                                        */
  real_T Constant_Value_n;             /* Expression: 0
                                        * Referenced by: '<S137>/Constant'
                                        */
  real_T Feedforward_tableData[17];
       /* Expression: [0	3.5	12.5	16	20	21	22	24.5	26	29.5	32	33	34	36	37	38	39]
        * Referenced by: '<S137>/Feedforward'
        */
  real_T Feedforward_bp01Data[17];
  /* Expression: [0 10    20    30    40    50    60    70    80    90   100   110   120   130   140   150   160]
   * Referenced by: '<S137>/Feedforward'
   */
  real_T FeedforwardGain_Gain;         /* Expression: 1
                                        * Referenced by: '<S137>/FeedforwardGain'
                                        */
  real_T Limits5050_IC;                /* Expression: 0
                                        * Referenced by: '<S137>/Limits [-50,50]'
                                        */
  real_T Limits5050_UpperSat;          /* Expression: 50
                                        * Referenced by: '<S137>/Limits [-50,50]'
                                        */
  real_T Limits5050_LowerSat;          /* Expression: -50
                                        * Referenced by: '<S137>/Limits [-50,50]'
                                        */
  real_T LowpassFilter_A;              /* Computed Parameter: LowpassFilter_A
                                        * Referenced by: '<S137>/Lowpass Filter'
                                        */
  real_T LowpassFilter_C;              /* Computed Parameter: LowpassFilter_C
                                        * Referenced by: '<S137>/Lowpass Filter'
                                        */
  real_T Kp_Gain;                      /* Expression: 7
                                        * Referenced by: '<S137>/Kp'
                                        */
  real_T ProportionalGainScheduling_tableData[3];/* Expression: [1 1 1]
                                                  * Referenced by: '<S137>/Proportional Gain Scheduling'
                                                  */
  real_T ProportionalGainScheduling_bp01Data[3];/* Expression: [ 0 30 70]
                                                 * Referenced by: '<S137>/Proportional Gain Scheduling'
                                                 */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S137>/Gain'
                                        */
  real_T Ki_Gain;                      /* Expression: 1
                                        * Referenced by: '<S137>/Ki'
                                        */
  real_T MinthresholdforBrakeswitch_Value;/* Expression: 0
                                           * Referenced by: '<S137>/Min threshold for Brake switch'
                                           */
  real_T Switch2_Threshold;            /* Expression: 0
                                        * Referenced by: '<S137>/Switch2'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<S137>/Switch1'
                                        */
  real_T standstillbrake0100_Value;    /* Expression: 50
                                        * Referenced by: '<S137>/standstill brake [0,100]'
                                        */
  real_T kmtomiles_Gain;               /* Expression: 1/1.609344
                                        * Referenced by: '<S135>/km to miles'
                                        */
  real_T Gain_Gain_i;                  /* Expression: 1/1.609
                                        * Referenced by: '<S89>/Gain'
                                        */
  real_T ManSpdSel_Value;              /* Expression: 4
                                        * Referenced by: '<S89>/ManSpdSel'
                                        */
  real_T ManSpd_Value;                 /* Expression: 0
                                        * Referenced by: '<S89>/ManSpd'
                                        */
  real_T RateLimiter_RisingLim;     /* Computed Parameter: RateLimiter_RisingLim
                                     * Referenced by: '<S89>/Rate Limiter'
                                     */
  real_T RateLimiter_FallingLim;   /* Computed Parameter: RateLimiter_FallingLim
                                    * Referenced by: '<S89>/Rate Limiter'
                                    */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<S89>/Rate Limiter'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1/1.609
                                        * Referenced by: '<S89>/Gain1'
                                        */
  real_T Interpolatedzerocrossing_tableData[2];/* Expression: [-1 1]
                                                * Referenced by: '<S157>/Interpolated zero-crossing'
                                                */
  real_T Interpolatedzerocrossing_bp01Data[2];/* Expression: [-1 1]
                                               * Referenced by: '<S157>/Interpolated zero-crossing'
                                               */
  real_T Constant_Value_o;             /* Expression: 0
                                        * Referenced by: '<S177>/Constant'
                                        */
  real_T Constant_Value_d;             /* Expression: 0
                                        * Referenced by: '<S178>/Constant'
                                        */
  real_T Gain1_Gain_p;                 /* Expression: 3.6
                                        * Referenced by: '<S81>/Gain1'
                                        */
  real_T DriveCycle_trigger_Value;     /* Expression: 0
                                        * Referenced by: '<S81>/DriveCycle_trigger'
                                        */
  real_T Constant1_Value;              /* Expression: zeros(1,NumAxl)
                                        * Referenced by: '<S310>/Constant1'
                                        */
  real_T Memory1_InitialCondition_p;   /* Expression: 0
                                        * Referenced by: '<S310>/Memory1'
                                        */
  real_T MeanWheelPosition_Gain;       /* Expression: 1/2
                                        * Referenced by: '<S310>/Mean Wheel Position'
                                        */
  real_T TrackNumber2_Value[2];        /* Expression: 1:length(TrackNumVec)
                                        * Referenced by: '<S304>/Track Number2'
                                        */
  real_T phithetapsi_WrappedStateUpperValue;/* Expression: pi
                                             * Referenced by: '<S399>/phi theta psi'
                                             */
  real_T phithetapsi_WrappedStateLowerValue;/* Expression: -pi
                                             * Referenced by: '<S399>/phi theta psi'
                                             */
  real_T InertialFrameCGtoAxleOffset_Value[12];
                                     /* Expression: [zeros(2,4);0.134*ones(1,4)]
                                      * Referenced by: '<S307>/Inertial Frame CG to Axle Offset'
                                      */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S516>/Integrator'
                                        */
  real_T Integrator_IC_k;              /* Expression: 0
                                        * Referenced by: '<S541>/Integrator'
                                        */
  real_T Integrator_IC_m;              /* Expression: 0
                                        * Referenced by: '<S566>/Integrator'
                                        */
  real_T Integrator_IC_e;              /* Expression: 0
                                        * Referenced by: '<S591>/Integrator'
                                        */
  real_T DeadZone2_Start;              /* Expression: -5
                                        * Referenced by: '<S482>/Dead Zone2'
                                        */
  real_T DeadZone2_End;                /* Expression: 5
                                        * Referenced by: '<S482>/Dead Zone2'
                                        */
  real_T Integrator_IC_o;              /* Expression: 0
                                        * Referenced by: '<S517>/Integrator'
                                        */
  real_T Integrator_IC_d;              /* Expression: 0
                                        * Referenced by: '<S542>/Integrator'
                                        */
  real_T Integrator_IC_b;              /* Expression: 0
                                        * Referenced by: '<S567>/Integrator'
                                        */
  real_T Integrator_IC_l;              /* Expression: 0
                                        * Referenced by: '<S592>/Integrator'
                                        */
  real_T DeadZone3_Start;              /* Expression: -10
                                        * Referenced by: '<S482>/Dead Zone3'
                                        */
  real_T DeadZone3_End;                /* Expression: 10
                                        * Referenced by: '<S482>/Dead Zone3'
                                        */
  real_T Integrator1_IC;               /* Expression: 0
                                        * Referenced by: '<S479>/Integrator1'
                                        */
  real_T Saturation_UpperSat;          /* Expression: inf
                                        * Referenced by: '<S297>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -10*9.81*2000
                                        * Referenced by: '<S297>/Saturation'
                                        */
  real_T Constant3_Value[4];           /* Expression: zeros(4,1)
                                        * Referenced by: '<S297>/Constant3'
                                        */
  real_T Integrator1_IC_k;             /* Expression: 0
                                        * Referenced by: '<S480>/Integrator1'
                                        */
  real_T Constant1_Value_o;            /* Expression: pi
                                        * Referenced by: '<S487>/Constant1'
                                        */
  real_T Constant3_Value_e[4];         /* Expression: ones(1,4).*0
                                        * Referenced by: '<S487>/Constant3'
                                        */
  real_T Constant2_Value[4];       /* Expression: [pi;0;pi;0].*0+[0;pi;0;pi].*.0
                                    * Referenced by: '<S487>/Constant2'
                                    */
  real_T Memory_InitialCondition_i;    /* Expression: 0
                                        * Referenced by: '<S144>/Memory'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S144>/Gain4'
                                        */
  real_T Constant_Value_e;             /* Expression: 0
                                        * Referenced by: '<S144>/Constant'
                                        */
  real_T Gain4_Gain_e[4];              /* Expression: [1;-1;1;-1]*0+1
                                        * Referenced by: '<S484>/Gain4'
                                        */
  real_T Integrator_IC_f;              /* Expression: 0
                                        * Referenced by: '<S519>/Integrator'
                                        */
  real_T GroundZLevel_Value[4];        /* Expression: zeros(4,1)
                                        * Referenced by: '<S79>/Ground Z Level'
                                        */
  real_T Saturation_UpperSat_l;        /* Expression: inf
                                        * Referenced by: '<S514>/Saturation'
                                        */
  real_T Saturation_LowerSat_h;        /* Expression: 0
                                        * Referenced by: '<S514>/Saturation'
                                        */
  real_T Brake_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/Brake'
                                        */
  real_T uformanual1forautopilot_Value;/* Expression: 0
                                        * Referenced by: '<Root>/0 for manual, 1 for autopilot'
                                        */
  real_T Constant6_Value;              /* Expression: 0
                                        * Referenced by: '<S2>/Constant6'
                                        */
  real_T Switch1_Threshold_c;          /* Expression: 0
                                        * Referenced by: '<S2>/Switch1'
                                        */
  real_T Saturation_UpperSat_a;        /* Expression: 1
                                        * Referenced by: '<S168>/Saturation'
                                        */
  real_T Saturation_LowerSat_hu;       /* Expression: 0
                                        * Referenced by: '<S168>/Saturation'
                                        */
  real_T Memory1_InitialCondition_ph;  /* Expression: 0
                                        * Referenced by: '<S144>/Memory1'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: inf
                                        * Referenced by: '<S184>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: 1
                                        * Referenced by: '<S184>/Saturation1'
                                        */
  real_T Saturation_LowerSat_hi;       /* Expression: 0
                                        * Referenced by: '<S184>/Saturation'
                                        */
  real_T IntegratorLimited_LowerSat;   /* Expression: 0
                                        * Referenced by: '<S128>/Integrator Limited'
                                        */
  real_T Saturation1_UpperSat_a;       /* Expression: 1
                                        * Referenced by: '<S168>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_l;       /* Expression: 0
                                        * Referenced by: '<S168>/Saturation1'
                                        */
  real_T Gain2_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S290>/Gain2'
                                        */
  real_T Gain3_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S290>/Gain3'
                                        */
  real_T Saturation_LowerSat_j;        /* Expression: 0
                                        * Referenced by: '<S290>/Saturation'
                                        */
  real_T Saturation1_LowerSat_i;       /* Expression: 0
                                        * Referenced by: '<S290>/Saturation1'
                                        */
  real_T Gain1_Gain_e;                 /* Expression: 0.5
                                        * Referenced by: '<S290>/Gain1'
                                        */
  real_T Gain4_Gain_h;                 /* Expression: 0.5
                                        * Referenced by: '<S290>/Gain4'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S290>/Saturation2'
                                        */
  real_T Saturation3_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S290>/Saturation3'
                                        */
  real_T TorqueConversion1_Gain;       /* Expression: pi/4
                                        * Referenced by: '<S524>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat;/* Expression: inf
                                               * Referenced by: '<S524>/Disallow Negative Brake Torque'
                                               */
  real_T DisallowNegativeBrakeTorque_LowerSat;/* Expression: eps
                                               * Referenced by: '<S524>/Disallow Negative Brake Torque'
                                               */
  real_T AxleNumber3_Value[2];       /* Expression: zeros(1,length(TrackNumVec))
                                      * Referenced by: '<S304>/Axle Number3'
                                      */
  real_T Constant1_Value_c;            /* Expression: pi
                                        * Referenced by: '<S612>/Constant1'
                                        */
  real_T Constant3_Value_o[4];         /* Expression: ones(1,4).*0
                                        * Referenced by: '<S612>/Constant3'
                                        */
  real_T Constant2_Value_f[4];      /* Expression: [pi;0;pi;0].*0+[0;pi;0;pi].*0
                                     * Referenced by: '<S612>/Constant2'
                                     */
  real_T SteerRates_Value[4];          /* Expression: zeros(1,4)
                                        * Referenced by: '<S305>/SteerRates'
                                        */
  real_T Constant_Value_l[4];          /* Expression: zeros(1,4)
                                        * Referenced by: '<S305>/Constant'
                                        */
  real_T ones2_Value[4];               /* Expression: ones(1,numWheels)
                                        * Referenced by: '<S485>/ones2'
                                        */
  real_T Friction_Value[4];            /* Expression: ones(4,1).*1
                                        * Referenced by: '<S79>/Friction'
                                        */
  real_T u_Value[4];                   /* Expression: [zeros(1,numWheels)]
                                        * Referenced by: '<S485>/0'
                                        */
  real_T ones_Value[92];               /* Expression: [ones(23,numWheels)]
                                        * Referenced by: '<S485>/ones'
                                        */
  real_T Integrator_IC_fs;             /* Expression: 0
                                        * Referenced by: '<S544>/Integrator'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: inf
                                        * Referenced by: '<S539>/Saturation'
                                        */
  real_T Saturation_LowerSat_p;        /* Expression: 0
                                        * Referenced by: '<S539>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_p;     /* Expression: pi/4
                                        * Referenced by: '<S549>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_k;/* Expression: inf
                                                 * Referenced by: '<S549>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_f;/* Expression: eps
                                                 * Referenced by: '<S549>/Disallow Negative Brake Torque'
                                                 */
  real_T Integrator_IC_eu;             /* Expression: 0
                                        * Referenced by: '<S569>/Integrator'
                                        */
  real_T Saturation_UpperSat_b;        /* Expression: inf
                                        * Referenced by: '<S564>/Saturation'
                                        */
  real_T Saturation_LowerSat_e;        /* Expression: 0
                                        * Referenced by: '<S564>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_b;     /* Expression: pi/4
                                        * Referenced by: '<S574>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_i;/* Expression: inf
                                                 * Referenced by: '<S574>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_i;/* Expression: eps
                                                 * Referenced by: '<S574>/Disallow Negative Brake Torque'
                                                 */
  real_T Integrator_IC_dr;             /* Expression: 0
                                        * Referenced by: '<S594>/Integrator'
                                        */
  real_T Saturation_UpperSat_d;        /* Expression: inf
                                        * Referenced by: '<S589>/Saturation'
                                        */
  real_T Saturation_LowerSat_o;        /* Expression: 0
                                        * Referenced by: '<S589>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_n;     /* Expression: pi/4
                                        * Referenced by: '<S599>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_c;/* Expression: inf
                                                 * Referenced by: '<S599>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_k;/* Expression: eps
                                                 * Referenced by: '<S599>/Disallow Negative Brake Torque'
                                                 */
  real_T Throttle_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/Throttle'
                                        */
  real_T Steer_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/Steer'
                                        */
  real_T Integrator_IC_i;              /* Expression: 0
                                        * Referenced by: '<S86>/Integrator'
                                        */
  real_T Constant_Value_b;             /* Expression: 0
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T Constant1_Value_k;            /* Expression: 0
                                        * Referenced by: '<S2>/Constant1'
                                        */
  real_T Constant2_Value_p;            /* Expression: 1
                                        * Referenced by: '<S2>/Constant2'
                                        */
  real_T Constant3_Value_k;            /* Expression: 0
                                        * Referenced by: '<S2>/Constant3'
                                        */
  real_T Constant4_Value;              /* Expression: 1
                                        * Referenced by: '<S2>/Constant4'
                                        */
  real_T Switch2_Threshold_g;          /* Expression: 0
                                        * Referenced by: '<S2>/Switch2'
                                        */
  real_T EVBoltSteer_tableData[183];
  /* Expression: [-9.10000000000000	-9	-8.90000000000000	-8.80000000000000	-8.70000000000000	-8.60000000000000	-8.50000000000000	-8.40000000000000	-8.30000000000000	-8.20000000000000	-8.10000000000000	-8	-7.90000000000000	-7.80000000000000	-7.70000000000000	-7.60000000000000	-7.50000000000000	-7.40000000000000	-7.30000000000000	-7.20000000000000	-7.10000000000000	-7	-6.90000000000000	-6.80000000000000	-6.70000000000000	-6.60000000000000	-6.50000000000000	-6.40000000000000	-6.30000000000000	-6.20000000000000	-6.10000000000000	-6	-5.90000000000000	-5.80000000000000	-5.70000000000000	-5.60000000000000	-5.50000000000000	-5.40000000000000	-5.30000000000000	-5.20000000000000	-5.10000000000000	-5	-4.90000000000000	-4.80000000000000	-4.70000000000000	-4.60000000000000	-4.50000000000000	-4.40000000000000	-4.30000000000000	-4.20000000000000	-4.10000000000000	-4	-3.90000000000000	-3.80000000000000	-3.70000000000000	-3.60000000000000	-3.50000000000000	-3.40000000000000	-3.30000000000000	-3.20000000000000	-3.10000000000000	-3	-2.90000000000000	-2.80000000000000	-2.70000000000000	-2.60000000000000	-2.50000000000000	-2.40000000000000	-2.30000000000000	-2.20000000000000	-2.10000000000000	-2	-1.90000000000000	-1.80000000000000	-1.70000000000000	-1.60000000000000	-1.50000000000000	-1.40000000000000	-1.30000000000000	-1.20000000000000	-1.10000000000000	-1	-0.900000000000000	-0.800000000000000	-0.700000000000000	-0.600000000000000	-0.500000000000000	-0.400000000000000	-0.300000000000000	-0.200000000000000	-0.100000000000000	0	0.100000000000000	0.200000000000000	0.300000000000000	0.400000000000000	0.500000000000000	0.600000000000000	0.700000000000000	0.800000000000000	0.900000000000000	1	1.10000000000000	1.20000000000000	1.30000000000000	1.40000000000000	1.50000000000000	1.60000000000000	1.70000000000000	1.80000000000000	1.90000000000000	2	2.10000000000000	2.20000000000000	2.30000000000000	2.40000000000000	2.50000000000000	2.60000000000000	2.70000000000000	2.80000000000000	2.90000000000000	3	3.10000000000000	3.20000000000000	3.30000000000000	3.40000000000000	3.50000000000000	3.60000000000000	3.70000000000000	3.80000000000000	3.90000000000000	4	4.10000000000000	4.20000000000000	4.30000000000000	4.40000000000000	4.50000000000000	4.60000000000000	4.70000000000000	4.80000000000000	4.90000000000000	5	5.10000000000000	5.20000000000000	5.30000000000000	5.40000000000000	5.50000000000000	5.60000000000000	5.70000000000000	5.80000000000000	5.90000000000000	6	6.10000000000000	6.20000000000000	6.30000000000000	6.40000000000000	6.50000000000000	6.60000000000000	6.70000000000000	6.80000000000000	6.90000000000000	7	7.10000000000000	7.20000000000000	7.30000000000000	7.40000000000000	7.50000000000000	7.60000000000000	7.70000000000000	7.80000000000000	7.90000000000000	8	8.10000000000000	8.20000000000000	8.30000000000000	8.40000000000000	8.50000000000000	8.60000000000000	8.70000000000000	8.80000000000000	8.90000000000000	9	9.10000000000000]
   * Referenced by: '<S2>/EV Bolt Steer'
   */
  real_T EVBoltSteer_bp01Data[183];
  /* Expression: [-0.541571931581492	-0.535619550629112	-0.529667169676731	-0.523714788724350	-0.517762407771969	-0.511810026819588	-0.505857645867207	-0.499905264914826	-0.493952883962445	-0.488000503010064	-0.482048122057683	-0.476095741105302	-0.470143360152921	-0.464190979200541	-0.458238598248160	-0.452286217295779	-0.446333836343398	-0.440381455391017	-0.434429074438636	-0.428476693486255	-0.422524312533874	-0.416571931581493	-0.410619550629112	-0.404667169676731	-0.398714788724350	-0.392762407771969	-0.386810026819588	-0.380857645867207	-0.374905264914826	-0.368952883962445	-0.363000503010064	-0.357048122057683	-0.351095741105302	-0.345143360152921	-0.339190979200541	-0.333238598248160	-0.327286217295779	-0.321333836343398	-0.315381455391017	-0.309429074438636	-0.303476693486255	-0.297524312533874	-0.291571931581493	-0.285619550629112	-0.279667169676731	-0.273714788724350	-0.267762407771969	-0.261810026819588	-0.255857645867207	-0.249905264914826	-0.243952883962445	-0.238000503010065	-0.232048122057684	-0.226095741105303	-0.220143360152922	-0.214190979200541	-0.208238598248160	-0.202286217295779	-0.196333836343398	-0.190381455391017	-0.184429074438636	-0.178476693486255	-0.172524312533874	-0.166571931581493	-0.160619550629112	-0.154667169676731	-0.148714788724350	-0.142762407771969	-0.136810026819588	-0.130857645867207	-0.124905264914826	-0.118952883962445	-0.113000503010064	-0.107048122057683	-0.101095741105303	-0.0951433601529216	-0.0891909792005406	-0.0832385982481596	-0.0772862172957787	-0.0713338363433977	-0.0653814553910168	-0.0594290744386358	-0.0534766934862548	-0.0475243125338739	-0.0415719315814929	-0.0356195506291120	-0.0296671696767310	-0.0237147887243500	-0.0177624077719691	-0.0118100268195881	-0.00585764586720720	0	0.00585764586720720	0.0118100268195881	0.0177624077719691	0.0237147887243500	0.0296671696767310	0.0356195506291120	0.0415719315814929	0.0475243125338739	0.0534766934862548	0.0594290744386358	0.0653814553910168	0.0713338363433977	0.0772862172957787	0.0832385982481596	0.0891909792005406	0.0951433601529216	0.101095741105303	0.107048122057683	0.113000503010064	0.118952883962445	0.124905264914826	0.130857645867207	0.136810026819588	0.142762407771969	0.148714788724350	0.154667169676731	0.160619550629112	0.166571931581493	0.172524312533874	0.178476693486255	0.184429074438636	0.190381455391017	0.196333836343398	0.202286217295779	0.208238598248160	0.214190979200541	0.220143360152922	0.226095741105303	0.232048122057684	0.238000503010065	0.243952883962445	0.249905264914826	0.255857645867207	0.261810026819588	0.267762407771969	0.273714788724350	0.279667169676731	0.285619550629112	0.291571931581493	0.297524312533874	0.303476693486255	0.309429074438636	0.315381455391017	0.321333836343398	0.327286217295779	0.333238598248160	0.339190979200541	0.345143360152921	0.351095741105302	0.357048122057683	0.363000503010064	0.368952883962445	0.374905264914826	0.380857645867207	0.386810026819588	0.392762407771969	0.398714788724350	0.404667169676731	0.410619550629112	0.416571931581493	0.422524312533874	0.428476693486255	0.434429074438636	0.440381455391017	0.446333836343398	0.452286217295779	0.458238598248160	0.464190979200541	0.470143360152921	0.476095741105302	0.482048122057683	0.488000503010064	0.493952883962445	0.499905264914826	0.505857645867207	0.511810026819588	0.517762407771969	0.523714788724350	0.529667169676731	0.535619550629112	0.541571931581492]
   * Referenced by: '<S2>/EV Bolt Steer'
   */
  real_T Constant2_Value_m;            /* Expression: 0
                                        * Referenced by: '<S83>/Constant2'
                                        */
  real_T Constant3_Value_p;            /* Expression: 0
                                        * Referenced by: '<S83>/Constant3'
                                        */
  real_T Constant4_Value_e;            /* Expression: 0
                                        * Referenced by: '<S83>/Constant4'
                                        */
  real_T Step_Time;                    /* Expression: 5
                                        * Referenced by: '<S2>/Step'
                                        */
  real_T Step_Y0;                      /* Expression: 0
                                        * Referenced by: '<S2>/Step'
                                        */
  real_T Step_YFinal;                  /* Expression: 0.35
                                        * Referenced by: '<S2>/Step'
                                        */
  real_T Switch_Threshold_l;           /* Expression: 0
                                        * Referenced by: '<S2>/Switch'
                                        */
  real_T LaunchSpdThr_rpm_Value;       /* Expression: 2
                                        * Referenced by: '<S271>/LaunchSpdThr_rpm'
                                        */
  real_T LaunchTrqThr_Nm_Value;        /* Expression: 20
                                        * Referenced by: '<S271>/LaunchTrqThr_Nm'
                                        */
  real_T SpdLatchThr_rpm_Value;        /* Expression: 1e-5
                                        * Referenced by: '<S271>/SpdLatchThr_rpm'
                                        */
  real_T Brakeswitchourceselection_Value;/* Expression: 2
                                          * Referenced by: '<S90>/Brake switch ource selection'
                                          */
  real_T PedalRobotControlSelection_Value;/* Expression: 1
                                           * Referenced by: '<S87>/Pedal Robot Control Selection'
                                           */
  real_T BR_command_0to100_Value;      /* Expression: 0
                                        * Referenced by: '<S87>/BR_command_0to100'
                                        */
  real_T Constant1_Value_p;            /* Expression: 0
                                        * Referenced by: '<S87>/Constant1'
                                        */
  real_T BR_limit_UpperSat;            /* Expression: 100
                                        * Referenced by: '<S87>/BR_limit'
                                        */
  real_T BR_limit_LowerSat;            /* Expression: 0
                                        * Referenced by: '<S87>/BR_limit'
                                        */
  real_T MinthresholdforBrakeswitch_Value_d;/* Expression: 0
                                             * Referenced by: '<S87>/Min threshold for Brake switch'
                                             */
  real_T omegawheel_IC;                /* Expression: 0
                                        * Referenced by: '<S249>/omega wheel'
                                        */
  real_T omegawheel_IC_b;              /* Expression: 0
                                        * Referenced by: '<S250>/omega wheel'
                                        */
  real_T omegawheel_IC_o;              /* Expression: 0
                                        * Referenced by: '<S251>/omega wheel'
                                        */
  real_T omegawheel_IC_l;              /* Expression: 0
                                        * Referenced by: '<S252>/omega wheel'
                                        */
  real_T Saturation_UpperSat_ey;       /* Expression: 100
                                        * Referenced by: '<S192>/Saturation'
                                        */
  real_T Saturation_LowerSat_hun;      /* Expression: -100
                                        * Referenced by: '<S192>/Saturation'
                                        */
  real_T rads2rpm_Gain;                /* Expression: 9.5492965964254
                                        * Referenced by: '<S195>/rads2rpm'
                                        */
  real_T dt_Value;                     /* Expression: 0.001
                                        * Referenced by: '<S267>/dt'
                                        */
  real_T Tau_Value;                    /* Expression: 0.01
                                        * Referenced by: '<S193>/Tau'
                                        */
  real_T one_Value;                    /* Expression: 1
                                        * Referenced by: '<S267>/one'
                                        */
  real_T Memory_InitialCondition_k;    /* Expression: 0
                                        * Referenced by: '<S267>/Memory'
                                        */
  real_T dt_Value_c;                   /* Expression: 0.001
                                        * Referenced by: '<S268>/dt'
                                        */
  real_T one_Value_j;                  /* Expression: 1
                                        * Referenced by: '<S268>/one'
                                        */
  real_T Memory_InitialCondition_g;    /* Expression: 0
                                        * Referenced by: '<S268>/Memory'
                                        */
  real_T dt_Value_p;                   /* Expression: 0.001
                                        * Referenced by: '<S269>/dt'
                                        */
  real_T one_Value_a;                  /* Expression: 1
                                        * Referenced by: '<S269>/one'
                                        */
  real_T Memory_InitialCondition_f;    /* Expression: 0
                                        * Referenced by: '<S269>/Memory'
                                        */
  real_T dt_Value_e;                   /* Expression: 0.001
                                        * Referenced by: '<S270>/dt'
                                        */
  real_T one_Value_l;                  /* Expression: 1
                                        * Referenced by: '<S270>/one'
                                        */
  real_T Memory_InitialCondition_a;    /* Expression: 0
                                        * Referenced by: '<S270>/Memory'
                                        */
  real_T SettleTime_s_Value;           /* Expression: 5
                                        * Referenced by: '<S271>/SettleTime_s'
                                        */
  real_T dStop_Value;                  /* Expression: 0
                                        * Referenced by: '<S271>/dStop'
                                        */
  real_T VehSpdThr_kph_Value;          /* Expression: 0.2
                                        * Referenced by: '<S271>/VehSpdThr_kph'
                                        */
  real_T MasterSw_Value;               /* Expression: 1
                                        * Referenced by: '<S96>/MasterSw'
                                        */
  real_T RefLF_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S274>/RefLF_NmORrpm'
                                        */
  real_T RefRF_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S274>/RefRF_NmORrpm'
                                        */
  real_T RefLR_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S274>/RefLR_NmORrpm'
                                        */
  real_T RefRR_NmORrpm_Value;          /* Expression: 0
                                        * Referenced by: '<S274>/RefRR_NmORrpm'
                                        */
  real_T SpeedOut_Threshold;           /* Expression: 0
                                        * Referenced by: '<S195>/SpeedOut'
                                        */
  real_T RateLimSpd_RisingLim;       /* Computed Parameter: RateLimSpd_RisingLim
                                      * Referenced by: '<S195>/RateLimSpd'
                                      */
  real_T RateLimSpd_FallingLim;     /* Computed Parameter: RateLimSpd_FallingLim
                                     * Referenced by: '<S195>/RateLimSpd'
                                     */
  real_T RateLimSpd_IC;                /* Expression: 0
                                        * Referenced by: '<S195>/RateLimSpd'
                                        */
  real_T SatSpd_UpperSat;              /* Expression: 700
                                        * Referenced by: '<S195>/SatSpd'
                                        */
  real_T SatSpd_LowerSat;              /* Expression: 0
                                        * Referenced by: '<S195>/SatSpd'
                                        */
  real_T ModeOut_Threshold;            /* Expression: 0
                                        * Referenced by: '<S195>/ModeOut'
                                        */
  real_T constant_Value;               /* Expression: 1
                                        * Referenced by: '<S98>/constant'
                                        */
  real_T ControlWordbypassOverridevalue_Value;/* Expression: 0
                                               * Referenced by: '<S98>/ControlWord bypass Override value'
                                               */
  real_T ControlWordbypassEnable_Value;/* Expression: 0
                                        * Referenced by: '<S98>/ControlWord bypass Enable'
                                        */
  real_T RPDO3Enable_Value;            /* Expression: 1
                                        * Referenced by: '<S100>/RPDO3 Enable '
                                        */
  real_T RPDO3period_Value;            /* Expression: 0.001
                                        * Referenced by: '<S100>/RPDO3 period'
                                        */
  real_T Memory2_InitialCondition_g;   /* Expression: 0
                                        * Referenced by: '<S108>/Memory2'
                                        */
  real_T Integrator_IC_ix;             /* Expression: 0
                                        * Referenced by: '<S113>/Integrator'
                                        */
  real_T Integrator_UpperSat;          /* Expression: inf
                                        * Referenced by: '<S113>/Integrator'
                                        */
  real_T Integrator_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S113>/Integrator'
                                        */
  real_T MoogResetCmd_Value;           /* Expression: 0
                                        * Referenced by: '<S101>/MoogResetCmd'
                                        */
  real_T Memory_InitialCondition_l;    /* Expression: 0
                                        * Referenced by: '<S117>/Memory'
                                        */
  real_T PositionLearningTrigger_Value;/* Expression: 0
                                        * Referenced by: '<S101>/PositionLearningTrigger'
                                        */
  real_T Memory_InitialCondition_lt;   /* Expression: 0
                                        * Referenced by: '<S118>/Memory'
                                        */
  real_T SineWave_Amp;                 /* Expression: 50
                                        * Referenced by: '<S98>/Sine Wave'
                                        */
  real_T SineWave_Bias;                /* Expression: 50
                                        * Referenced by: '<S98>/Sine Wave'
                                        */
  real_T SineWave_Freq;                /* Expression: 1
                                        * Referenced by: '<S98>/Sine Wave'
                                        */
  real_T SineWave_Phase;               /* Expression: 0
                                        * Referenced by: '<S98>/Sine Wave'
                                        */
  real_T traceselection_Value;         /* Expression: 0
                                        * Referenced by: '<S98>/trace selection'
                                        */
  real_T BrakePedalTargetPositionManualBypass_Value;/* Expression: 0
                                                     * Referenced by: '<S98>/BrakePedal Target Position Manual Bypass'
                                                     */
  real_T RateLimiter_RisingLim_l; /* Computed Parameter: RateLimiter_RisingLim_l
                                   * Referenced by: '<S98>/Rate Limiter'
                                   */
  real_T RateLimiter_FallingLim_d;
                                 /* Computed Parameter: RateLimiter_FallingLim_d
                                  * Referenced by: '<S98>/Rate Limiter'
                                  */
  real_T RateLimiter_IC_a;             /* Expression: 0
                                        * Referenced by: '<S98>/Rate Limiter'
                                        */
  real_T Switch_Threshold_d;           /* Expression: 0
                                        * Referenced by: '<S98>/Switch'
                                        */
  real_T BrakePedalTargetPositionBypassEnable_Value;/* Expression: 0
                                                     * Referenced by: '<S98>/BrakePedal TargetPosition Bypass Enable'
                                                     */
  real_T bped2br_N_tableData[4];       /* Expression: [0 0 25 100]
                                        * Referenced by: '<S87>/bped2br_N'
                                        */
  real_T bped2br_N_bp01Data[4];        /* Expression: [0 0.1 0.2 100]
                                        * Referenced by: '<S87>/bped2br_N'
                                        */
  real_T Switch2_Threshold_c;          /* Expression: 0
                                        * Referenced by: '<S98>/Switch2'
                                        */
  real_T Gain_Gain_c;                  /* Expression: 1
                                        * Referenced by: '<S98>/Gain'
                                        */
  real_T Constant2_Value_c;            /* Expression: 100
                                        * Referenced by: '<S101>/Constant2'
                                        */
  real_T Memory_InitialCondition_h;    /* Expression: 0
                                        * Referenced by: '<S101>/Memory'
                                        */
  real_T Memory2_InitialCondition_e;   /* Expression: 0
                                        * Referenced by: '<S101>/Memory2'
                                        */
  real_T Memory1_InitialCondition_g;   /* Expression: 0
                                        * Referenced by: '<S101>/Memory1'
                                        */
  real_T ActuatorMinTravelForReady_Value;/* Expression: 35000
                                          * Referenced by: '<S101>/ActuatorMinTravelForReady'
                                          */
  real_T Switch1_Threshold_p;          /* Expression: 0
                                        * Referenced by: '<S98>/Switch1'
                                        */
  real_T Gain2_Gain_n;                 /* Expression: 1
                                        * Referenced by: '<S101>/Gain2'
                                        */
  real_T Constant_Value_l3;            /* Expression: 1
                                        * Referenced by: '<S113>/Constant'
                                        */
  real_T Constant1_Value_cy;           /* Expression: 100
                                        * Referenced by: '<S101>/Constant1'
                                        */
  real_T AccelPositionManualBypass_Value;/* Expression: 0
                                          * Referenced by: '<S99>/AccelPosition Manual Bypass [%]'
                                          */
  real_T RateLimiter_RisingLim_a; /* Computed Parameter: RateLimiter_RisingLim_a
                                   * Referenced by: '<S99>/Rate Limiter'
                                   */
  real_T RateLimiter_FallingLim_h;
                                 /* Computed Parameter: RateLimiter_FallingLim_h
                                  * Referenced by: '<S99>/Rate Limiter'
                                  */
  real_T RateLimiter_IC_l;             /* Expression: 0
                                        * Referenced by: '<S99>/Rate Limiter'
                                        */
  real_T AccelPositionBypassEnable_Value;/* Expression: 0
                                          * Referenced by: '<S99>/AccelPosition Bypass Enable'
                                          */
  real_T APP_Value;                    /* Expression: 0
                                        * Referenced by: '<S87>/APP'
                                        */
  real_T Constant_Value_j;             /* Expression: 0
                                        * Referenced by: '<S87>/Constant'
                                        */
  real_T AR_limit_UpperSat;            /* Expression: 100
                                        * Referenced by: '<S87>/AR_limit'
                                        */
  real_T AR_limit_LowerSat;            /* Expression: 0
                                        * Referenced by: '<S87>/AR_limit'
                                        */
  real_T Switch2_Threshold_j;          /* Expression: 0
                                        * Referenced by: '<S99>/Switch2'
                                        */
  real_T APP1Tbl_tableData[2];         /* Expression: [16 78.8]
                                        * Referenced by: '<S99>/APP1Tbl'
                                        */
  real_T APP1Tbl_bp01Data[2];          /* Expression: [0 100]
                                        * Referenced by: '<S99>/APP1Tbl'
                                        */
  real_T Gain3_Gain_b;                 /* Expression: 1/100
                                        * Referenced by: '<S99>/Gain3'
                                        */
  real_T VRef_Value;                   /* Expression: 5.01
                                        * Referenced by: '<S99>/VRef'
                                        */
  real_T Gain1_Gain_ez;                /* Expression: 1
                                        * Referenced by: '<S99>/Gain1'
                                        */
  real_T APP2Tbl_tableData[2];         /* Expression: [8 39.4]
                                        * Referenced by: '<S99>/APP2Tbl'
                                        */
  real_T APP2Tbl_bp01Data[2];          /* Expression: [0 100]
                                        * Referenced by: '<S99>/APP2Tbl'
                                        */
  real_T Gain4_Gain_a;                 /* Expression: 1/100
                                        * Referenced by: '<S99>/Gain4'
                                        */
  real_T Gain_Gain_p;                  /* Expression: 1
                                        * Referenced by: '<S99>/Gain'
                                        */
  real_T Gain2_Gain_h;                 /* Expression: 1
                                        * Referenced by: '<S99>/Gain2'
                                        */
  real_T Memory_InitialCondition_p;    /* Expression: 0
                                        * Referenced by: '<S92>/Memory'
                                        */
  real_T Gain_Gain_j;                  /* Expression: -1
                                        * Referenced by: '<S125>/Gain'
                                        */
  real_T Gain1_Gain_n;                 /* Expression: -1
                                        * Referenced by: '<S125>/Gain1'
                                        */
  real_T ChargingMode_Value;           /* Expression: 0
                                        * Referenced by: '<S96>/ChargingMode'
                                        */
  real_T CmdSpeedmph_Value;            /* Expression: 0
                                        * Referenced by: '<S96>/CmdSpeed (mph)'
                                        */
  real_T mph2ms_Gain;                  /* Expression: 0.44704
                                        * Referenced by: '<S96>/mph2m//s'
                                        */
  real_T utireRadius_Gain;             /* Expression: 1/0.3232
                                        * Referenced by: '<S96>/1//tireRadius'
                                        */
  real_T VehicleSimulationType_Value;  /* Expression: 1
                                        * Referenced by: '<S81>/VehicleSimulationType'
                                        */
  real_T Gain1_Gain_p0;                /* Expression: 0.5
                                        * Referenced by: '<S144>/Gain1'
                                        */
  real_T Integrator_IC_mm;             /* Expression: 0
                                        * Referenced by: '<S153>/Integrator'
                                        */
  real_T Merge_InitialOutput;          /* Expression: 0
                                        * Referenced by: '<S153>/Merge'
                                        */
  real_T Constant_Value_dv;            /* Expression: 0
                                        * Referenced by: '<S145>/Constant'
                                        */
  real_T Constant1_Value_i;            /* Expression: 0
                                        * Referenced by: '<S145>/Constant1'
                                        */
  real_T Constant2_Value_o;            /* Expression: 0
                                        * Referenced by: '<S145>/Constant2'
                                        */
  real_T Constant_Value_lf;            /* Expression: 0
                                        * Referenced by: '<S151>/Constant'
                                        */
  real_T uDLookupTable_tableData[667]; /* Expression: x_losses_mat
                                        * Referenced by: '<S155>/2-D Lookup Table'
                                        */
  real_T uDLookupTable_bp01Data[23];   /* Expression: x_w_eff_vec
                                        * Referenced by: '<S155>/2-D Lookup Table'
                                        */
  real_T uDLookupTable_bp02Data[29];   /* Expression: x_T_eff_vec
                                        * Referenced by: '<S155>/2-D Lookup Table'
                                        */
  real_T Saturation_UpperSat_h;        /* Expression: Inf
                                        * Referenced by: '<S152>/Saturation'
                                        */
  real_T Saturation_LowerSat_oa;       /* Expression: 0.0001
                                        * Referenced by: '<S152>/Saturation'
                                        */
  real_T Gain_Gain_o;                  /* Expression: -1
                                        * Referenced by: '<S154>/Gain'
                                        */
  real_T Gain1_Gain_g;                 /* Expression: 1
                                        * Referenced by: '<S154>/Gain1'
                                        */
  real_T Saturation1_UpperSat_ak;      /* Expression: inf
                                        * Referenced by: '<S169>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_c;       /* Expression: 1
                                        * Referenced by: '<S169>/Saturation1'
                                        */
  real_T Saturation_LowerSat_p0;       /* Expression: 0
                                        * Referenced by: '<S169>/Saturation'
                                        */
  real_T Gain_Gain_e;                  /* Expression: -1
                                        * Referenced by: '<S168>/Gain'
                                        */
  real_T rads_to_rpm_Gain;             /* Expression: 30/pi
                                        * Referenced by: '<S174>/rads_to_rpm'
                                        */
  real_T Gain1_Gain_o;                 /* Expression: 1/100
                                        * Referenced by: '<S174>/Gain1'
                                        */
  real_T Constant1_Value_g;            /* Expression: -1
                                        * Referenced by: '<S174>/Constant1'
                                        */
  real_T Constant2_Value_k;            /* Expression: 1
                                        * Referenced by: '<S174>/Constant2'
                                        */
  real_T Switch2_Threshold_o;          /* Expression: 0
                                        * Referenced by: '<S174>/Switch2'
                                        */
  real_T Constant_Value_jx;            /* Expression: 1
                                        * Referenced by: '<S176>/Constant'
                                        */
  real_T Switch1_Threshold_k;          /* Expression: 0
                                        * Referenced by: '<S176>/Switch1'
                                        */
  real_T Saturation1_UpperSat_l;       /* Expression: inf
                                        * Referenced by: '<S180>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_j;       /* Expression: 1
                                        * Referenced by: '<S180>/Saturation1'
                                        */
  real_T Saturation_LowerSat_ej;       /* Expression: 0
                                        * Referenced by: '<S180>/Saturation'
                                        */
  real_T Gain_Gain_k;                  /* Expression: -1
                                        * Referenced by: '<S175>/Gain'
                                        */
  real_T Constant_Value_nr;            /* Expression: 3000
                                        * Referenced by: '<S96>/Constant'
                                        */
  real_T DriverSwitch_Value;           /* Expression: 0
                                        * Referenced by: '<S96>/DriverSwitch'
                                        */
  real_T APP_pct_Value;                /* Expression: 0
                                        * Referenced by: '<S189>/APP_pct'
                                        */
  real_T APPToTorque_tableData[2];     /* Expression: [0 184]
                                        * Referenced by: '<S189>/APPToTorque'
                                        */
  real_T APPToTorque_bp01Data[2];      /* Expression: [0 100]
                                        * Referenced by: '<S189>/APPToTorque'
                                        */
  real_T BPP_pct_Value;                /* Expression: 0
                                        * Referenced by: '<S189>/BPP_pct'
                                        */
  real_T APPToTorque1_tableData[2];    /* Expression: [0 184]
                                        * Referenced by: '<S189>/APPToTorque1'
                                        */
  real_T APPToTorque1_bp01Data[2];     /* Expression: [0 100]
                                        * Referenced by: '<S189>/APPToTorque1'
                                        */
  real_T Flip_Gain;                    /* Expression: -1
                                        * Referenced by: '<S189>/Flip'
                                        */
  real_T WheelRadius_Gain;             /* Expression: 0.3235
                                        * Referenced by: '<S189>/WheelRadius'
                                        */
  real_T mps2kph_Gain;                 /* Expression: 3.6
                                        * Referenced by: '<S189>/mps2kph'
                                        */
  real_T kph2mph_Gain;                 /* Expression: 0.621
                                        * Referenced by: '<S189>/kph2mph'
                                        */
  real_T NissanTrans_tableData[8];
                        /* Expression: [5.25 3.03 1.95 1.46 1.22 1.00 0.81 0.67]
                         * Referenced by: '<S189>/NissanTrans'
                         */
  real_T NissanTrans_bp01Data[8];      /* Expression: [0 5 10 15 25 34 45 55]
                                        * Referenced by: '<S189>/NissanTrans'
                                        */
  real_T FDR_Gain;                     /* Expression: 3.18
                                        * Referenced by: '<S189>/FDR'
                                        */
  real_T Memory_InitialCondition_e;    /* Expression: 0
                                        * Referenced by: '<S189>/Memory'
                                        */
  real_T DiscreteTimeIntegrator4_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator4_gainval
                           * Referenced by: '<S190>/Discrete-Time Integrator4'
                           */
  real_T DiscreteTimeIntegrator4_IC;   /* Expression: 0
                                        * Referenced by: '<S190>/Discrete-Time Integrator4'
                                        */
  real_T DiscreteTimeIntegrator5_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator5_gainval
                           * Referenced by: '<S190>/Discrete-Time Integrator5'
                           */
  real_T DiscreteTimeIntegrator5_IC;   /* Expression: 0
                                        * Referenced by: '<S190>/Discrete-Time Integrator5'
                                        */
  real_T DiscreteTimeIntegrator6_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator6_gainval
                           * Referenced by: '<S190>/Discrete-Time Integrator6'
                           */
  real_T DiscreteTimeIntegrator6_IC;   /* Expression: 0
                                        * Referenced by: '<S190>/Discrete-Time Integrator6'
                                        */
  real_T DiscreteTimeIntegrator7_gainval;
                          /* Computed Parameter: DiscreteTimeIntegrator7_gainval
                           * Referenced by: '<S190>/Discrete-Time Integrator7'
                           */
  real_T DiscreteTimeIntegrator7_IC;   /* Expression: 0
                                        * Referenced by: '<S190>/Discrete-Time Integrator7'
                                        */
  real_T Constant1_Value_ir;           /* Expression: 0
                                        * Referenced by: '<S194>/Constant1'
                                        */
  real_T Memory3_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S190>/Memory3'
                                        */
  real_T Saturation3_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S190>/Saturation3'
                                        */
  real_T Saturation3_LowerSat_h;       /* Expression: -1
                                        * Referenced by: '<S190>/Saturation3'
                                        */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<S190>/Gain5'
                                        */
  real_T Memory1_InitialCondition_m;   /* Expression: 0
                                        * Referenced by: '<S190>/Memory1'
                                        */
  real_T Saturation1_UpperSat_e;       /* Expression: 1
                                        * Referenced by: '<S190>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_e;       /* Expression: -1
                                        * Referenced by: '<S190>/Saturation1'
                                        */
  real_T Gain6_Gain;                   /* Expression: -1
                                        * Referenced by: '<S190>/Gain6'
                                        */
  real_T Constant_Value_om;            /* Expression: 0
                                        * Referenced by: '<S194>/Constant'
                                        */
  real_T Memory2_InitialCondition_l;   /* Expression: 0
                                        * Referenced by: '<S190>/Memory2'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S190>/Saturation2'
                                        */
  real_T Saturation2_LowerSat_j;       /* Expression: -1
                                        * Referenced by: '<S190>/Saturation2'
                                        */
  real_T Gain7_Gain;                   /* Expression: -1
                                        * Referenced by: '<S190>/Gain7'
                                        */
  real_T Memory4_InitialCondition_e;   /* Expression: 0
                                        * Referenced by: '<S190>/Memory4'
                                        */
  real_T Saturation4_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S190>/Saturation4'
                                        */
  real_T Saturation4_LowerSat;         /* Expression: -1
                                        * Referenced by: '<S190>/Saturation4'
                                        */
  real_T Gain8_Gain;                   /* Expression: -1
                                        * Referenced by: '<S190>/Gain8'
                                        */
  real_T Ignition_Value;               /* Expression: 1
                                        * Referenced by: '<S190>/Ignition'
                                        */
  real_T OperationError_Value;         /* Expression: 4
                                        * Referenced by: '<S190>/Operation Error'
                                        */
  real_T OperationStateDriving_Value;  /* Expression: 4
                                        * Referenced by: '<S190>/Operation State Driving'
                                        */
  real_T Zero1_Value;                  /* Expression: 0
                                        * Referenced by: '<S190>/Zero1'
                                        */
  real_T Zero2_Value;                  /* Expression: 0
                                        * Referenced by: '<S190>/Zero2'
                                        */
  real_T Zero3_Value;                  /* Expression: 0
                                        * Referenced by: '<S190>/Zero3'
                                        */
  real_T Zero4_Value;                  /* Expression: 0
                                        * Referenced by: '<S190>/Zero4'
                                        */
  real_T Zero5_Value;                  /* Expression: 0
                                        * Referenced by: '<S190>/Zero5'
                                        */
  real_T DiscreteTimeIntegrator4_gainval_i;
                        /* Computed Parameter: DiscreteTimeIntegrator4_gainval_i
                         * Referenced by: '<S247>/Discrete-Time Integrator4'
                         */
  real_T DiscreteTimeIntegrator4_IC_d; /* Expression: 0
                                        * Referenced by: '<S247>/Discrete-Time Integrator4'
                                        */
  real_T DiscreteTimeIntegrator5_gainval_b;
                        /* Computed Parameter: DiscreteTimeIntegrator5_gainval_b
                         * Referenced by: '<S247>/Discrete-Time Integrator5'
                         */
  real_T DiscreteTimeIntegrator5_IC_a; /* Expression: 0
                                        * Referenced by: '<S247>/Discrete-Time Integrator5'
                                        */
  real_T DiscreteTimeIntegrator6_gainval_h;
                        /* Computed Parameter: DiscreteTimeIntegrator6_gainval_h
                         * Referenced by: '<S247>/Discrete-Time Integrator6'
                         */
  real_T DiscreteTimeIntegrator6_IC_d; /* Expression: 0
                                        * Referenced by: '<S247>/Discrete-Time Integrator6'
                                        */
  real_T DiscreteTimeIntegrator7_gainval_e;
                        /* Computed Parameter: DiscreteTimeIntegrator7_gainval_e
                         * Referenced by: '<S247>/Discrete-Time Integrator7'
                         */
  real_T DiscreteTimeIntegrator7_IC_b; /* Expression: 0
                                        * Referenced by: '<S247>/Discrete-Time Integrator7'
                                        */
  real_T Memory3_InitialCondition_j;   /* Expression: 0
                                        * Referenced by: '<S247>/Memory3'
                                        */
  real_T Saturation3_UpperSat_l;       /* Expression: 1
                                        * Referenced by: '<S247>/Saturation3'
                                        */
  real_T Saturation3_LowerSat_o;       /* Expression: -1
                                        * Referenced by: '<S247>/Saturation3'
                                        */
  real_T Gain5_Gain_f;                 /* Expression: -1
                                        * Referenced by: '<S247>/Gain5'
                                        */
  real_T Memory1_InitialCondition_l;   /* Expression: 0
                                        * Referenced by: '<S247>/Memory1'
                                        */
  real_T Saturation1_UpperSat_ad;      /* Expression: 1
                                        * Referenced by: '<S247>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_c1;      /* Expression: -1
                                        * Referenced by: '<S247>/Saturation1'
                                        */
  real_T Gain6_Gain_d;                 /* Expression: -1
                                        * Referenced by: '<S247>/Gain6'
                                        */
  real_T Memory2_InitialCondition_h;   /* Expression: 0
                                        * Referenced by: '<S247>/Memory2'
                                        */
  real_T Saturation2_UpperSat_g;       /* Expression: 1
                                        * Referenced by: '<S247>/Saturation2'
                                        */
  real_T Saturation2_LowerSat_e;       /* Expression: -1
                                        * Referenced by: '<S247>/Saturation2'
                                        */
  real_T Gain7_Gain_d;                 /* Expression: -1
                                        * Referenced by: '<S247>/Gain7'
                                        */
  real_T Memory4_InitialCondition_j;   /* Expression: 0
                                        * Referenced by: '<S247>/Memory4'
                                        */
  real_T Saturation4_UpperSat_k;       /* Expression: 1
                                        * Referenced by: '<S247>/Saturation4'
                                        */
  real_T Saturation4_LowerSat_o;       /* Expression: -1
                                        * Referenced by: '<S247>/Saturation4'
                                        */
  real_T Gain8_Gain_b;                 /* Expression: -1
                                        * Referenced by: '<S247>/Gain8'
                                        */
  real_T Ignition_Value_k;             /* Expression: 1
                                        * Referenced by: '<S247>/Ignition'
                                        */
  real_T OperationError_Value_l;       /* Expression: 4
                                        * Referenced by: '<S247>/Operation Error'
                                        */
  real_T OperationStateDriving_Value_k;/* Expression: 4
                                        * Referenced by: '<S247>/Operation State Driving'
                                        */
  real_T ManualTrq_Nm_Value;           /* Expression: 0
                                        * Referenced by: '<S193>/ManualTrq_Nm'
                                        */
  real_T TorqueIn_Threshold;           /* Expression: 0
                                        * Referenced by: '<S193>/TorqueIn'
                                        */
  real_T SatTrq_UpperSat;              /* Expression: 1000
                                        * Referenced by: '<S193>/SatTrq'
                                        */
  real_T SatTrq_LowerSat;              /* Expression: -1000
                                        * Referenced by: '<S193>/SatTrq'
                                        */
  real_T Saturation_UpperSat_n;        /* Expression: inf
                                        * Referenced by: '<S503>/Saturation'
                                        */
  real_T Saturation_LowerSat_i;        /* Expression: 0
                                        * Referenced by: '<S503>/Saturation'
                                        */
  real_T Constant9_Value;              /* Expression: 0.3135
                                        * Referenced by: '<S249>/Constant9'
                                        */
  real_T Saturation_UpperSat_j;        /* Expression: inf
                                        * Referenced by: '<S249>/Saturation'
                                        */
  real_T Saturation_LowerSat_iy;       /* Expression: 0
                                        * Referenced by: '<S249>/Saturation'
                                        */
  real_T Constant_Value_nq;            /* Expression: 0
                                        * Referenced by: '<S249>/Constant'
                                        */
  real_T Memory_InitialCondition_in;   /* Expression: 0
                                        * Referenced by: '<S249>/Memory'
                                        */
  real_T OutputDamping_Gain;           /* Expression: 0
                                        * Referenced by: '<S249>/Output Damping'
                                        */
  real_T Zero1_Value_c;                /* Expression: 0.82
                                        * Referenced by: '<S255>/Zero1'
                                        */
  real_T Zero_Value;                   /* Expression: 1
                                        * Referenced by: '<S255>/Zero'
                                        */
  real_T Switch_Threshold_b;           /* Expression: 0
                                        * Referenced by: '<S249>/Switch'
                                        */
  real_T Zero1_Value_d;                /* Expression: 0.74063
                                        * Referenced by: '<S254>/Zero1'
                                        */
  real_T Zero_Value_e;                 /* Expression: 1
                                        * Referenced by: '<S254>/Zero'
                                        */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S249>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0/3.6/0.3235
                                        * Referenced by: '<S249>/Discrete-Time Integrator'
                                        */
  real_T Saturation_UpperSat_i;        /* Expression: inf
                                        * Referenced by: '<S504>/Saturation'
                                        */
  real_T Saturation_LowerSat_d;        /* Expression: 0
                                        * Referenced by: '<S504>/Saturation'
                                        */
  real_T Constant9_Value_l;            /* Expression: 0.3135
                                        * Referenced by: '<S250>/Constant9'
                                        */
  real_T Saturation_UpperSat_lm;       /* Expression: inf
                                        * Referenced by: '<S250>/Saturation'
                                        */
  real_T Saturation_LowerSat_ea;       /* Expression: 0
                                        * Referenced by: '<S250>/Saturation'
                                        */
  real_T Constant_Value_dr;            /* Expression: 0
                                        * Referenced by: '<S250>/Constant'
                                        */
  real_T Memory_InitialCondition_n;    /* Expression: 0
                                        * Referenced by: '<S250>/Memory'
                                        */
  real_T Zero1_Value_k;                /* Expression: 0.82
                                        * Referenced by: '<S258>/Zero1'
                                        */
  real_T Zero_Value_n;                 /* Expression: 1
                                        * Referenced by: '<S258>/Zero'
                                        */
  real_T OutputDamping_Gain_g;         /* Expression: -1
                                        * Referenced by: '<S250>/Output Damping'
                                        */
  real_T Switch_Threshold_h;           /* Expression: 0
                                        * Referenced by: '<S250>/Switch'
                                        */
  real_T Zero1_Value_j;                /* Expression: 0.74063
                                        * Referenced by: '<S257>/Zero1'
                                        */
  real_T Zero_Value_m;                 /* Expression: 1
                                        * Referenced by: '<S257>/Zero'
                                        */
  real_T DiscreteTimeIntegrator_gainval_c;
                         /* Computed Parameter: DiscreteTimeIntegrator_gainval_c
                          * Referenced by: '<S250>/Discrete-Time Integrator'
                          */
  real_T DiscreteTimeIntegrator_IC_b;  /* Expression: 0/3.6/0.3235
                                        * Referenced by: '<S250>/Discrete-Time Integrator'
                                        */
  real_T Saturation_UpperSat_ew;       /* Expression: inf
                                        * Referenced by: '<S505>/Saturation'
                                        */
  real_T Saturation_LowerSat_ejw;      /* Expression: 0
                                        * Referenced by: '<S505>/Saturation'
                                        */
  real_T Constant9_Value_b;            /* Expression: 0.3135
                                        * Referenced by: '<S251>/Constant9'
                                        */
  real_T Saturation_UpperSat_g;        /* Expression: inf
                                        * Referenced by: '<S251>/Saturation'
                                        */
  real_T Saturation_LowerSat_hn;       /* Expression: 0
                                        * Referenced by: '<S251>/Saturation'
                                        */
  real_T Constant_Value_od;            /* Expression: 0
                                        * Referenced by: '<S251>/Constant'
                                        */
  real_T OutputDamping_Gain_d;         /* Expression: 0
                                        * Referenced by: '<S251>/Output Damping'
                                        */
  real_T Zero1_Value_cl;               /* Expression: 0.82
                                        * Referenced by: '<S261>/Zero1'
                                        */
  real_T Zero_Value_mj;                /* Expression: 1
                                        * Referenced by: '<S261>/Zero'
                                        */
  real_T Switch_Threshold_g;           /* Expression: 0
                                        * Referenced by: '<S251>/Switch'
                                        */
  real_T Zero1_Value_i;                /* Expression: 0.74063
                                        * Referenced by: '<S260>/Zero1'
                                        */
  real_T Zero_Value_g;                 /* Expression: 1
                                        * Referenced by: '<S260>/Zero'
                                        */
  real_T DiscreteTimeIntegrator_gainval_b;
                         /* Computed Parameter: DiscreteTimeIntegrator_gainval_b
                          * Referenced by: '<S251>/Discrete-Time Integrator'
                          */
  real_T DiscreteTimeIntegrator_IC_p;  /* Expression: 0/3.6/0.3235
                                        * Referenced by: '<S251>/Discrete-Time Integrator'
                                        */
  real_T Memory_InitialCondition_ib;   /* Expression: 0
                                        * Referenced by: '<S251>/Memory'
                                        */
  real_T Saturation_UpperSat_ik;       /* Expression: inf
                                        * Referenced by: '<S506>/Saturation'
                                        */
  real_T Saturation_LowerSat_a;        /* Expression: 0
                                        * Referenced by: '<S506>/Saturation'
                                        */
  real_T Constant9_Value_i;            /* Expression: 0.3135
                                        * Referenced by: '<S252>/Constant9'
                                        */
  real_T Saturation_UpperSat_go;       /* Expression: inf
                                        * Referenced by: '<S252>/Saturation'
                                        */
  real_T Saturation_LowerSat_oz;       /* Expression: 0
                                        * Referenced by: '<S252>/Saturation'
                                        */
  real_T Constant_Value_lk;            /* Expression: 0
                                        * Referenced by: '<S252>/Constant'
                                        */
  real_T Zero1_Value_h;                /* Expression: 0.82
                                        * Referenced by: '<S264>/Zero1'
                                        */
  real_T Zero_Value_mc;                /* Expression: 1
                                        * Referenced by: '<S264>/Zero'
                                        */
  real_T Gain_Gain_d;                  /* Expression: -1
                                        * Referenced by: '<S252>/Gain'
                                        */
  real_T Switch_Threshold_dw;          /* Expression: 0
                                        * Referenced by: '<S252>/Switch'
                                        */
  real_T Zero1_Value_ic;               /* Expression: 0.74063
                                        * Referenced by: '<S263>/Zero1'
                                        */
  real_T Zero_Value_i;                 /* Expression: 1
                                        * Referenced by: '<S263>/Zero'
                                        */
  real_T DiscreteTimeIntegrator_gainval_i;
                         /* Computed Parameter: DiscreteTimeIntegrator_gainval_i
                          * Referenced by: '<S252>/Discrete-Time Integrator'
                          */
  real_T DiscreteTimeIntegrator_IC_o;  /* Expression: 0/3.6/0.3235
                                        * Referenced by: '<S252>/Discrete-Time Integrator'
                                        */
  real_T Memory_InitialCondition_pl;   /* Expression: 0
                                        * Referenced by: '<S252>/Memory'
                                        */
  real_T Zero1_Value_a;                /* Expression: 0
                                        * Referenced by: '<S247>/Zero1'
                                        */
  real_T Zero2_Value_j;                /* Expression: 0
                                        * Referenced by: '<S247>/Zero2'
                                        */
  real_T Zero3_Value_k;                /* Expression: 0
                                        * Referenced by: '<S247>/Zero3'
                                        */
  real_T Zero4_Value_m;                /* Expression: 0
                                        * Referenced by: '<S247>/Zero4'
                                        */
  real_T Zero5_Value_l;                /* Expression: 0
                                        * Referenced by: '<S247>/Zero5'
                                        */
  real_T Constant_Value_f;             /* Expression: 500
                                        * Referenced by: '<S247>/Constant'
                                        */
  real_T Constant_Value_lb;            /* Expression: 4.1
                                        * Referenced by: '<S248>/Constant'
                                        */
  real_T Gain10_Gain;                  /* Expression: 0.5
                                        * Referenced by: '<S248>/Gain10'
                                        */
  real_T Saturation1_UpperSat_h;       /* Expression: 300
                                        * Referenced by: '<S192>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_eu;      /* Expression: -300
                                        * Referenced by: '<S192>/Saturation1'
                                        */
  real_T Zero1_Value_g;                /* Expression: 0
                                        * Referenced by: '<S194>/Zero1'
                                        */
  real_T Zero2_Value_f;                /* Expression: 0
                                        * Referenced by: '<S194>/Zero2'
                                        */
  real_T Zero5_Value_g;                /* Expression: 0
                                        * Referenced by: '<S194>/Zero5'
                                        */
  real_T BrkLatchThr_Nm_Value;         /* Expression: -50
                                        * Referenced by: '<S271>/BrkLatchThr_Nm'
                                        */
  real_T Bit4_Gain;                    /* Expression: 16
                                        * Referenced by: '<S281>/Bit4'
                                        */
  real_T Bit3_Gain;                    /* Expression: 8
                                        * Referenced by: '<S281>/Bit3'
                                        */
  real_T Bit2_Gain;                    /* Expression: 4
                                        * Referenced by: '<S281>/Bit2'
                                        */
  real_T Bit1_Gain;                    /* Expression: 2
                                        * Referenced by: '<S281>/Bit1'
                                        */
  real_T Constant_Value_jxh;           /* Expression: 0
                                        * Referenced by: '<S291>/Constant'
                                        */
  real_T Backlash_InitialOutput;       /* Expression: 0
                                        * Referenced by: '<S299>/Backlash'
                                        */
  real_T Constant_Value_a;             /* Expression: 0
                                        * Referenced by: '<S299>/Constant'
                                        */
  real_T index_Value;                  /* Expression: 1
                                        * Referenced by: '<S298>/index'
                                        */
  real_T Switch_Threshold_dn;          /* Expression: 0
                                        * Referenced by: '<S298>/Switch'
                                        */
  real_T Switch1_Threshold_f;          /* Expression: 0
                                        * Referenced by: '<S298>/Switch1'
                                        */
  real_T AxlesUsingAntiSway_Value;     /* Expression: find(AntiSwayEnByAxl==1)
                                        * Referenced by: '<S351>/Axles Using Anti-Sway'
                                        */
  real_T Gain_Gain_l;                  /* Expression: -1
                                        * Referenced by: '<S304>/Gain'
                                        */
  real_T AxleNumber1_Value;            /* Expression: 1:NumAxl
                                        * Referenced by: '<S304>/Axle Number1'
                                        */
  real_T Constant_Value_kk[3];         /* Expression: [0;0;0]
                                        * Referenced by: '<S385>/Constant'
                                        */
  real_T Constant1_Value_cs[3];        /* Expression: [0;0;0]
                                        * Referenced by: '<S385>/Constant1'
                                        */
  real_T Constant1_Value_j[9];         /* Expression: zeros(3,3)
                                        * Referenced by: '<S386>/Constant1'
                                        */
  real_T Crm_tableData[2];             /* Expression: [0 0]
                                        * Referenced by: '<S419>/Crm'
                                        */
  real_T Crm_bp01Data[2];              /* Expression: [-1 1]
                                        * Referenced by: '<S419>/Crm'
                                        */
  real_T u_Gain[3];                    /* Expression: 4.*ones(3,1)
                                        * Referenced by: '<S419>/4'
                                        */
  real_T u_Value_g;                    /* Expression: 0
                                        * Referenced by: '<S390>/0'
                                        */
  real_T Constant_Value_kb[12];        /* Expression: zeros(12,1)
                                        * Referenced by: '<S386>/Constant'
                                        */
  real_T Integrator_IC_fb[3];          /* Expression: [0 0 0]
                                        * Referenced by: '<S395>/Integrator'
                                        */
  real_T DeadZone_Start;               /* Expression: 0
                                        * Referenced by: '<S516>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_g;       /* Expression: 10
                                        * Referenced by: '<S516>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_d;       /* Expression: .01
                                        * Referenced by: '<S516>/Saturation1'
                                        */
  real_T Saturation_UpperSat_h4;       /* Expression: 400*pi*2
                                        * Referenced by: '<S516>/Saturation'
                                        */
  real_T Saturation_LowerSat_oj;       /* Expression: 2*pi
                                        * Referenced by: '<S516>/Saturation'
                                        */
  real_T DeadZone_Start_m;             /* Expression: 0
                                        * Referenced by: '<S517>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_j;       /* Expression: 10
                                        * Referenced by: '<S517>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_d1;      /* Expression: .01
                                        * Referenced by: '<S517>/Saturation1'
                                        */
  real_T Saturation_UpperSat_o;        /* Expression: 400*pi
                                        * Referenced by: '<S517>/Saturation'
                                        */
  real_T Saturation_LowerSat_dq;       /* Expression: 2*pi
                                        * Referenced by: '<S517>/Saturation'
                                        */
  real_T DeadZone_Start_l;             /* Expression: 0
                                        * Referenced by: '<S519>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_p;       /* Expression: 10
                                        * Referenced by: '<S519>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_m;       /* Expression: .01
                                        * Referenced by: '<S519>/Saturation1'
                                        */
  real_T Saturation_UpperSat_f;        /* Expression: 400*pi*2
                                        * Referenced by: '<S519>/Saturation'
                                        */
  real_T Saturation_LowerSat_d4;       /* Expression: 2*pi
                                        * Referenced by: '<S519>/Saturation'
                                        */
  real_T Constant_Value_jx1;           /* Expression: 0
                                        * Referenced by: '<S514>/Constant'
                                        */
  real_T Switch_Threshold_bx;          /* Expression: 0
                                        * Referenced by: '<S514>/Switch'
                                        */
  real_T DeadZone_Start_k;             /* Expression: 0
                                        * Referenced by: '<S541>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_k;       /* Expression: 10
                                        * Referenced by: '<S541>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dd;      /* Expression: .01
                                        * Referenced by: '<S541>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ig;       /* Expression: 400*pi*2
                                        * Referenced by: '<S541>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: 2*pi
                                        * Referenced by: '<S541>/Saturation'
                                        */
  real_T DeadZone_Start_my;            /* Expression: 0
                                        * Referenced by: '<S542>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_d;       /* Expression: 10
                                        * Referenced by: '<S542>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_n;       /* Expression: .01
                                        * Referenced by: '<S542>/Saturation1'
                                        */
  real_T Saturation_UpperSat_aq;       /* Expression: 400*pi
                                        * Referenced by: '<S542>/Saturation'
                                        */
  real_T Saturation_LowerSat_g;        /* Expression: 2*pi
                                        * Referenced by: '<S542>/Saturation'
                                        */
  real_T DeadZone_Start_g;             /* Expression: 0
                                        * Referenced by: '<S544>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_c;       /* Expression: 10
                                        * Referenced by: '<S544>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dh;      /* Expression: .01
                                        * Referenced by: '<S544>/Saturation1'
                                        */
  real_T Saturation_UpperSat_hc;       /* Expression: 400*pi*2
                                        * Referenced by: '<S544>/Saturation'
                                        */
  real_T Saturation_LowerSat_l;        /* Expression: 2*pi
                                        * Referenced by: '<S544>/Saturation'
                                        */
  real_T Constant_Value_a2;            /* Expression: 0
                                        * Referenced by: '<S539>/Constant'
                                        */
  real_T Switch_Threshold_bq;          /* Expression: 0
                                        * Referenced by: '<S539>/Switch'
                                        */
  real_T DeadZone_Start_d;             /* Expression: 0
                                        * Referenced by: '<S566>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_pk;      /* Expression: 10
                                        * Referenced by: '<S566>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_h;       /* Expression: .01
                                        * Referenced by: '<S566>/Saturation1'
                                        */
  real_T Saturation_UpperSat_c;        /* Expression: 400*pi*2
                                        * Referenced by: '<S566>/Saturation'
                                        */
  real_T Saturation_LowerSat_d1;       /* Expression: 2*pi
                                        * Referenced by: '<S566>/Saturation'
                                        */
  real_T DeadZone_Start_h;             /* Expression: 0
                                        * Referenced by: '<S567>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_b;       /* Expression: 10
                                        * Referenced by: '<S567>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dq;      /* Expression: .01
                                        * Referenced by: '<S567>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ak;       /* Expression: 400*pi
                                        * Referenced by: '<S567>/Saturation'
                                        */
  real_T Saturation_LowerSat_jc;       /* Expression: 2*pi
                                        * Referenced by: '<S567>/Saturation'
                                        */
  real_T DeadZone_Start_c;             /* Expression: 0
                                        * Referenced by: '<S569>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_n;       /* Expression: 10
                                        * Referenced by: '<S569>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_b;       /* Expression: .01
                                        * Referenced by: '<S569>/Saturation1'
                                        */
  real_T Saturation_UpperSat_p;        /* Expression: 400*pi*2
                                        * Referenced by: '<S569>/Saturation'
                                        */
  real_T Saturation_LowerSat_jx;       /* Expression: 2*pi
                                        * Referenced by: '<S569>/Saturation'
                                        */
  real_T Constant_Value_ex;            /* Expression: 0
                                        * Referenced by: '<S564>/Constant'
                                        */
  real_T Switch_Threshold_a;           /* Expression: 0
                                        * Referenced by: '<S564>/Switch'
                                        */
  real_T DeadZone_Start_e;             /* Expression: 0
                                        * Referenced by: '<S591>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_kn;      /* Expression: 10
                                        * Referenced by: '<S591>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_na;      /* Expression: .01
                                        * Referenced by: '<S591>/Saturation1'
                                        */
  real_T Saturation_UpperSat_nj;       /* Expression: 400*pi*2
                                        * Referenced by: '<S591>/Saturation'
                                        */
  real_T Saturation_LowerSat_pi;       /* Expression: 2*pi
                                        * Referenced by: '<S591>/Saturation'
                                        */
  real_T DeadZone_Start_a;             /* Expression: 0
                                        * Referenced by: '<S592>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_a1;      /* Expression: 10
                                        * Referenced by: '<S592>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_p;       /* Expression: .01
                                        * Referenced by: '<S592>/Saturation1'
                                        */
  real_T Saturation_UpperSat_dh;       /* Expression: 400*pi
                                        * Referenced by: '<S592>/Saturation'
                                        */
  real_T Saturation_LowerSat_eje;      /* Expression: 2*pi
                                        * Referenced by: '<S592>/Saturation'
                                        */
  real_T DeadZone_Start_b;             /* Expression: 0
                                        * Referenced by: '<S594>/Dead Zone'
                                        */
  real_T Saturation1_UpperSat_f;       /* Expression: 10
                                        * Referenced by: '<S594>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dn;      /* Expression: .01
                                        * Referenced by: '<S594>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ep;       /* Expression: 400*pi*2
                                        * Referenced by: '<S594>/Saturation'
                                        */
  real_T Saturation_LowerSat_gb;       /* Expression: 2*pi
                                        * Referenced by: '<S594>/Saturation'
                                        */
  real_T Constant_Value_i;             /* Expression: 0
                                        * Referenced by: '<S589>/Constant'
                                        */
  real_T Switch_Threshold_lb;          /* Expression: 0
                                        * Referenced by: '<S589>/Switch'
                                        */
  real_T Constant_Value_dro;           /* Expression: 0
                                        * Referenced by: '<S503>/Constant'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0
                                        * Referenced by: '<S503>/Switch'
                                        */
  real_T Constant_Value_ic;            /* Expression: 0
                                        * Referenced by: '<S504>/Constant'
                                        */
  real_T Switch_Threshold_e;           /* Expression: 0
                                        * Referenced by: '<S504>/Switch'
                                        */
  real_T Constant_Value_lt;            /* Expression: 0
                                        * Referenced by: '<S505>/Constant'
                                        */
  real_T Switch_Threshold_c;           /* Expression: 0
                                        * Referenced by: '<S505>/Switch'
                                        */
  real_T Constant_Value_jw;            /* Expression: 0
                                        * Referenced by: '<S506>/Constant'
                                        */
  real_T Switch_Threshold_j;           /* Expression: 0
                                        * Referenced by: '<S506>/Switch'
                                        */
  real_T Ackermansteer_tableData[189];
  /* Expression: [-9.40000000000000	-9.30000000000000	-9.20000000000000	-9.10000000000000	-9	-8.90000000000000	-8.80000000000000	-8.70000000000000	-8.60000000000000	-8.50000000000000	-8.40000000000000	-8.30000000000000	-8.20000000000000	-8.10000000000000	-8	-7.90000000000000	-7.80000000000000	-7.70000000000000	-7.60000000000000	-7.50000000000000	-7.40000000000000	-7.30000000000000	-7.20000000000000	-7.10000000000000	-7	-6.90000000000000	-6.80000000000000	-6.70000000000000	-6.60000000000000	-6.50000000000000	-6.40000000000000	-6.30000000000000	-6.20000000000000	-6.10000000000000	-6	-5.90000000000000	-5.80000000000000	-5.70000000000000	-5.60000000000000	-5.50000000000000	-5.40000000000000	-5.30000000000000	-5.20000000000000	-5.10000000000000	-5	-4.90000000000000	-4.80000000000000	-4.70000000000000	-4.60000000000000	-4.50000000000000	-4.40000000000000	-4.30000000000000	-4.20000000000000	-4.10000000000000	-4	-3.90000000000000	-3.80000000000000	-3.70000000000000	-3.60000000000000	-3.50000000000000	-3.40000000000000	-3.30000000000000	-3.20000000000000	-3.10000000000000	-3	-2.90000000000000	-2.80000000000000	-2.70000000000000	-2.60000000000000	-2.50000000000000	-2.40000000000000	-2.30000000000000	-2.20000000000000	-2.10000000000000	-2	-1.90000000000000	-1.80000000000000	-1.70000000000000	-1.60000000000000	-1.50000000000000	-1.40000000000000	-1.30000000000000	-1.20000000000000	-1.10000000000000	-1	-0.900000000000000	-0.800000000000000	-0.700000000000000	-0.600000000000000	-0.500000000000000	-0.400000000000000	-0.300000000000000	-0.200000000000000	-0.100000000000000	0	0.100000000000000	0.200000000000000	0.300000000000000	0.400000000000000	0.500000000000000	0.600000000000000	0.700000000000000	0.800000000000000	0.900000000000000	1	1.10000000000000	1.20000000000000	1.30000000000000	1.40000000000000	1.50000000000000	1.60000000000000	1.70000000000000	1.80000000000000	1.90000000000000	2	2.10000000000000	2.20000000000000	2.30000000000000	2.40000000000000	2.50000000000000	2.60000000000000	2.70000000000000	2.80000000000000	2.90000000000000	3	3.10000000000000	3.20000000000000	3.30000000000000	3.40000000000000	3.50000000000000	3.60000000000000	3.70000000000000	3.80000000000000	3.90000000000000	4	4.10000000000000	4.20000000000000	4.30000000000000	4.40000000000000	4.50000000000000	4.60000000000000	4.70000000000000	4.80000000000000	4.90000000000000	5	5.10000000000000	5.20000000000000	5.30000000000000	5.40000000000000	5.50000000000000	5.60000000000000	5.70000000000000	5.80000000000000	5.90000000000000	6	6.10000000000000	6.20000000000000	6.30000000000000	6.40000000000000	6.50000000000000	6.60000000000000	6.70000000000000	6.80000000000000	6.90000000000000	7	7.10000000000000	7.20000000000000	7.30000000000000	7.40000000000000	7.50000000000000	7.60000000000000	7.70000000000000	7.80000000000000	7.90000000000000	8	8.10000000000000	8.20000000000000	8.30000000000000	8.40000000000000	8.50000000000000	8.60000000000000	8.70000000000000	8.80000000000000	8.90000000000000	9	9.10000000000000	9.20000000000000	9.30000000000000	9.40000000000000]
   * Referenced by: '<S2>/Ackerman steer'
   */
  real_T Ackermansteer_bp01Data[189];
  /* Expression: [-0.446316297916244	-0.442150786830951	-0.437977161945005	-0.433795286969319	-0.429605025209166	-0.425406239561593	-0.421198792513030	-0.416982546137136	-0.412757362092861	-0.408523101622746	-0.404279625551461	-0.400026794284603	-0.395764467807732	-0.391492505685692	-0.387210767062193	-0.382919110659678	-0.378617394779480	-0.374305477302279	-0.369983215688866	-0.365650466981227	-0.361307087803951	-0.356952934365975	-0.352587862462675	-0.348211727478321	-0.343824384388886	-0.339425687765245	-0.335015491776758	-0.330593650195247	-0.326160016399396	-0.321714443379563	-0.317256783743034	-0.312786889719717	-0.308304613168298	-0.303809805582871	-0.299302318100048	-0.294782001506567	-0.290248706247410	-0.285702282434450	-0.281142579855627	-0.276569447984685	-0.271982735991464	-0.267382292752788	-0.262767966863931	-0.258139606650706	-0.253497060182174	-0.248840175283994	-0.244168799552432	-0.239482780369037	-0.234781964916014	-0.230066200192295	-0.225335333030331	-0.220589210113630	-0.215827677995039	-0.211050583115807	-0.206257771825433	-0.201449090402323	-0.196624385075265	-0.191783502045761	-0.186926287511198	-0.182052587688919	-0.177162248841170	-0.172255117300976	-0.167331039498945	-0.162389861991014	-0.157431431487183	-0.152455594881217	-0.147462199281363	-0.142451092042095	-0.137422120796894	-0.132375133492088	-0.127309978421782	-0.122226504263869	-0.117124560117169	-0.112003995539695	-0.106864660588066	-0.101706405858090	-0.0965290825265349	-0.0913325423940914	-0.0861166379295599	-0.0808812223152671	-0.0756261494937309	-0.0703512742155897	-0.0650564520888085	-0.0597415396291770	-0.0544063943121130	-0.0490508746257825	-0.0436748401255510	-0.0382781514897739	-0.0328606705769398	-0.0274222604841732	-0.0219627856071077	-0.0164821117011354	-0.0109801059440403	-0.00545663700002112	0	0.00547767577609933	0.0110656090794470	0.0166755107772618	0.0223075038233658	0.0279617092740365	0.0336382462142580	0.0393372316825870	0.0450587805946413	0.0508030056652199	0.0565700173290652	0.0623599236602815	0.0681728302904245	0.0740088403252790	0.0798680542603446	0.0857505698950509	0.0916564822457268	0.0975858834573503	0.103538862714108	0.109515506148794	0.115515896751090	0.121540114274751	0.127588235143750	0.133660332357415	0.139756475394610	0.145876730117007	0.152021158671496	0.158189819391799	0.164382766699339	0.170600051003425	0.176841718600822	0.183107811574771	0.189398367693541	0.195713420308563	0.202052998252265	0.208417125735647	0.214805822245717	0.221219102442855	0.227656976058216	0.234119447791246	0.240606517207442	0.247118178636427	0.253654421070478	0.260215228063589	0.266800577631217	0.273410442150791	0.280044788263140	0.286703576774936	0.293386762562311	0.300094294475738	0.306826115246357	0.313582161393837	0.320362363135953	0.327166644299990	0.333994922236136	0.340847107733006	0.347723104935443	0.354622811264752	0.361546117341510	0.368492906911125	0.375463056772267	0.382456436708368	0.389472909422305	0.396512330474460	0.403574548224288	0.410659403775562	0.417766730925451	0.424896356117582	0.432048098399246	0.439221769382891	0.446417173212069	0.453634106531970	0.460872358464695	0.468131710589418	0.475411936927561	0.482712803933137	0.490034070488372	0.497375487904758	0.504736799929627	0.512117742758400	0.519518045052588	0.526937427963674	0.534375605162959	0.541832282877473	0.549307159932028	0.556799927797496	0.564310270645373	0.571837865408689	0.579382381849329	0.586943482631776	0.594520823403350	0.602114052880920	0.609722812944144	0.617346738735206	0.624985458765058]
   * Referenced by: '<S2>/Ackerman steer'
   */
  int32_T Gain_Gain_o0;                /* Computed Parameter: Gain_Gain_o0
                                        * Referenced by: '<S101>/Gain'
                                        */
  int32_T Gain1_Gain_m;                /* Computed Parameter: Gain1_Gain_m
                                        * Referenced by: '<S98>/Gain1'
                                        */
  uint32_T R_maxIndex[2];              /* Computed Parameter: R_maxIndex
                                        * Referenced by: '<S130>/R'
                                        */
  uint32_T uDLookupTable_maxIndex[2];
                                   /* Computed Parameter: uDLookupTable_maxIndex
                                    * Referenced by: '<S155>/2-D Lookup Table'
                                    */
  uint32_T EffMap_maxIndex[2];         /* Computed Parameter: EffMap_maxIndex
                                        * Referenced by: '<S174>/Eff Map'
                                        */
  uint16_T ProtocolVer_Value;          /* Computed Parameter: ProtocolVer_Value
                                        * Referenced by: '<S273>/ProtocolVer'
                                        */
  boolean_T Memory_InitialCondition_j; /* Expression: false
                                        * Referenced by: '<S310>/Memory'
                                        */
  boolean_T Constant_Value_ig;         /* Expression: true
                                        * Referenced by: '<S310>/Constant'
                                        */
  int8_T Gain1_Gain_o0;                /* Computed Parameter: Gain1_Gain_o0
                                        * Referenced by: '<S101>/Gain1'
                                        */
  uint8_T Gain7_Gain_n;                /* Computed Parameter: Gain7_Gain_n
                                        * Referenced by: '<S197>/Gain7'
                                        */
  uint8_T Gain6_Gain_g;                /* Computed Parameter: Gain6_Gain_g
                                        * Referenced by: '<S197>/Gain6'
                                        */
  uint8_T Gain10_Gain_o;               /* Computed Parameter: Gain10_Gain_o
                                        * Referenced by: '<S197>/Gain10'
                                        */
  uint8_T Gain11_Gain;                 /* Computed Parameter: Gain11_Gain
                                        * Referenced by: '<S197>/Gain11'
                                        */
  uint8_T Gain12_Gain;                 /* Computed Parameter: Gain12_Gain
                                        * Referenced by: '<S197>/Gain12'
                                        */
  uint8_T Gain13_Gain;                 /* Computed Parameter: Gain13_Gain
                                        * Referenced by: '<S197>/Gain13'
                                        */
  uint8_T Gain14_Gain;                 /* Computed Parameter: Gain14_Gain
                                        * Referenced by: '<S197>/Gain14'
                                        */
  uint8_T Gain15_Gain;                 /* Computed Parameter: Gain15_Gain
                                        * Referenced by: '<S197>/Gain15'
                                        */
  uint8_T Gain5_Gain_o;                /* Computed Parameter: Gain5_Gain_o
                                        * Referenced by: '<S197>/Gain5'
                                        */
  uint8_T Gain4_Gain_m;                /* Computed Parameter: Gain4_Gain_m
                                        * Referenced by: '<S197>/Gain4'
                                        */
  uint8_T Gain3_Gain_j;                /* Computed Parameter: Gain3_Gain_j
                                        * Referenced by: '<S197>/Gain3'
                                        */
  uint8_T Gain2_Gain_a;                /* Computed Parameter: Gain2_Gain_a
                                        * Referenced by: '<S197>/Gain2'
                                        */
  uint8_T Gain1_Gain_j;                /* Computed Parameter: Gain1_Gain_j
                                        * Referenced by: '<S197>/Gain1'
                                        */
  uint8_T Gain_Gain_m;                 /* Computed Parameter: Gain_Gain_m
                                        * Referenced by: '<S197>/Gain'
                                        */
  uint8_T Gain8_Gain_c;                /* Computed Parameter: Gain8_Gain_c
                                        * Referenced by: '<S197>/Gain8'
                                        */
  uint8_T Gain9_Gain;                  /* Computed Parameter: Gain9_Gain
                                        * Referenced by: '<S197>/Gain9'
                                        */
  uint8_T Bit6_Gain;                   /* Computed Parameter: Bit6_Gain
                                        * Referenced by: '<S281>/Bit6'
                                        */
  uint8_T Bit5_Gain;                   /* Computed Parameter: Bit5_Gain
                                        * Referenced by: '<S281>/Bit5'
                                        */
  uint8_T Bit0_Gain;                   /* Computed Parameter: Bit0_Gain
                                        * Referenced by: '<S281>/Bit0'
                                        */
  uint8_T ManualSwitch6_CurrentSetting;
                             /* Computed Parameter: ManualSwitch6_CurrentSetting
                              * Referenced by: '<S297>/Manual Switch6'
                              */
  uint8_T ID1_Value;                   /* Computed Parameter: ID1_Value
                                        * Referenced by: '<S273>/ID1'
                                        */
  uint8_T ID2_Value;                   /* Computed Parameter: ID2_Value
                                        * Referenced by: '<S273>/ID2'
                                        */
  uint8_T ZeroTorque_Value;            /* Computed Parameter: ZeroTorque_Value
                                        * Referenced by: '<S195>/ZeroTorque'
                                        */
  uint8_T SpTqRR_Value;                /* Computed Parameter: SpTqRR_Value
                                        * Referenced by: '<S272>/SpTqRR'
                                        */
  uint8_T SpTqLR_Value;                /* Computed Parameter: SpTqLR_Value
                                        * Referenced by: '<S272>/SpTqLR'
                                        */
  uint8_T SpTqRF_Value;                /* Computed Parameter: SpTqRF_Value
                                        * Referenced by: '<S272>/SpTqRF'
                                        */
  uint8_T SpTqLF_Value;                /* Computed Parameter: SpTqLF_Value
                                        * Referenced by: '<S272>/SpTqLF'
                                        */
  uint8_T SystemActive_Value;          /* Computed Parameter: SystemActive_Value
                                        * Referenced by: '<S195>/SystemActive'
                                        */
  uint8_T SystemCtrlBits2_Value;    /* Computed Parameter: SystemCtrlBits2_Value
                                     * Referenced by: '<S273>/SystemCtrlBits2'
                                     */
  uint8_T Output_InitialCondition;/* Computed Parameter: Output_InitialCondition
                                   * Referenced by: '<S282>/Output'
                                   */
  uint8_T Constant3_Value_el;          /* Computed Parameter: Constant3_Value_el
                                        * Referenced by: '<S273>/Constant3'
                                        */
  uint8_T Constant2_Value_n;           /* Computed Parameter: Constant2_Value_n
                                        * Referenced by: '<S273>/Constant2'
                                        */
  uint8_T Constant1_Value_d;           /* Computed Parameter: Constant1_Value_d
                                        * Referenced by: '<S273>/Constant1'
                                        */
  uint8_T Bit7_Gain;                   /* Computed Parameter: Bit7_Gain
                                        * Referenced by: '<S281>/Bit7'
                                        */
  uint8_T FixPtConstant_Value;        /* Computed Parameter: FixPtConstant_Value
                                       * Referenced by: '<S287>/FixPt Constant'
                                       */
  uint8_T Constant_Value_nw;           /* Computed Parameter: Constant_Value_nw
                                        * Referenced by: '<S288>/Constant'
                                        */
  uint8_T Constant_Value_h4;           /* Computed Parameter: Constant_Value_h4
                                        * Referenced by: '<S273>/Constant'
                                        */
  P_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_c;/* '<S590>/LockUp' */
  P_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_h;/* '<S565>/LockUp' */
  P_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp_n;/* '<S540>/LockUp' */
  P_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T sf_LockUp;/* '<S515>/LockUp' */
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_a_T CoreSubsys_d;
  /* '<S309>/For each track and axle combination calculate suspension forces and moments' */
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_e_T CoreSubsys_n;
                                     /* '<S351>/For Each Axle With Anti-Sway' */
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_d_T CoreSubsys_p;
  /* '<S304>/For each track and axle combination calculate suspension forces and moments' */
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_g_T CoreSubsys_m;
         /* '<S304>/For each axle calculate axle cg positions and velocities' */
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T CoreSubsys;
  /* '<S304>/For each axle and track calculate suspension and wheel positions and velocities' */
};

/* Real-time Model Data Structure */
struct tag_RTM_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][80];
  ODE1_IntgData intgData;

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
    struct {
      uint8_T TID[3];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* Block parameters (default storage) */
extern P_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P;

/* Block signals (default storage) */
extern B_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B;

/* Continuous states (default storage) */
extern X_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X;

/* Block states (default storage) */
extern DW_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW;

/* Model entry point functions */
extern void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_initialize(void);
extern void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_output(void);
extern void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_update(void);
extern void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_terminate(void);

/* Real-time Model object */
extern RT_MODEL_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T *const
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_M;

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
 * '<Root>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom'
 * '<S1>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In'
 * '<S2>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test'
 * '<S3>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad'
 * '<S4>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad'
 * '<S5>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad'
 * '<S6>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad'
 * '<S7>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1'
 * '<S8>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2'
 * '<S9>'   : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad'
 * '<S10>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S11>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S12>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1'
 * '<S13>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2'
 * '<S14>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3'
 * '<S15>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S16>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad1'
 * '<S17>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad2'
 * '<S18>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad3'
 * '<S19>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad4'
 * '<S20>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad5'
 * '<S21>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad6'
 * '<S22>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad7'
 * '<S23>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S24>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad'
 * '<S25>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad'
 * '<S26>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad'
 * '<S27>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad4/Ad'
 * '<S28>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad5/Ad'
 * '<S29>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad6/Ad'
 * '<S30>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad7/Ad'
 * '<S31>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad'
 * '<S32>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad1'
 * '<S33>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad2'
 * '<S34>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad3'
 * '<S35>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad4'
 * '<S36>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad/Ad'
 * '<S37>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad1/Ad'
 * '<S38>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad2/Ad'
 * '<S39>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad3/Ad'
 * '<S40>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad4/Ad'
 * '<S41>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad'
 * '<S42>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad1'
 * '<S43>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad2'
 * '<S44>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad3'
 * '<S45>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad4'
 * '<S46>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad/Ad'
 * '<S47>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad1/Ad'
 * '<S48>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad2/Ad'
 * '<S49>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad3/Ad'
 * '<S50>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad4/Ad'
 * '<S51>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad'
 * '<S52>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad1'
 * '<S53>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad2'
 * '<S54>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad/Ad'
 * '<S55>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad1/Ad'
 * '<S56>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad2/Ad'
 * '<S57>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad'
 * '<S58>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad'
 * '<S59>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad'
 * '<S60>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad'
 * '<S61>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad1'
 * '<S62>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad2'
 * '<S63>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad3'
 * '<S64>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad4'
 * '<S65>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad5'
 * '<S66>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad/Ad'
 * '<S67>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad1/Ad'
 * '<S68>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad2/Ad'
 * '<S69>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad3/Ad'
 * '<S70>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad4/Ad'
 * '<S71>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad5/Ad'
 * '<S72>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad'
 * '<S73>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad'
 * '<S74>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad'
 * '<S75>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad'
 * '<S76>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad1'
 * '<S77>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad/Ad'
 * '<S78>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad1/Ad'
 * '<S79>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Environment'
 * '<S80>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Monitor'
 * '<S81>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle'
 * '<S82>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Environment/Ground Feedback'
 * '<S83>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Environment/Subsystem'
 * '<S84>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Environment/Subsystem1'
 * '<S85>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Environment/Ground Feedback/Constant'
 * '<S86>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Monitor/Calculate Powertrain Energy'
 * '<S87>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs'
 * '<S88>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery'
 * '<S89>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle'
 * '<S90>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/From CAN'
 * '<S91>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Front Wheel Drive'
 * '<S92>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor'
 * '<S93>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller '
 * '<S94>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest UDP RX ExternalMode  '
 * '<S95>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest UDP TX ExternalMode  '
 * '<S96>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface'
 * '<S97>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics'
 * '<S98>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator'
 * '<S99>'  : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output'
 * '<S100>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings'
 * '<S101>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface'
 * '<S102>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\ControlWord0\ISignal Value\ControlWord0 Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S103>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\ModesOfOperation\ISignal Value\ModesOfOperation Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S104>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\TargetPosition\ISignal Value\TargetPosition Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S105>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\ActualPosition\ISignal Value\ActualPosition Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S106>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\ModesOfOperationDisplay\ISignal Value\ModesOfOperationDisplay Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S107>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\StatusWord0\ISignal Value\StatusWord0 Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S108>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/ CAN diagnostic Counter'
 * '<S109>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/RPDO3\PDU Features\PDU Cyclic Timing Control\RPDO3 Timing Control Period [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S110>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/RPDO3\PDU Features\PDU Enable\RPDO3 Enable [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S111>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/TPDO3\PDU Features\PDU RX Status\TPDO3 Counter [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S112>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/ CAN diagnostic Counter/Compare To Constant3'
 * '<S113>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/ CAN diagnostic Counter/Timer'
 * '<S114>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/MOOG State Machine'
 * '<S115>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Max Stop Adjustment'
 * '<S116>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Min Stop Adjustment'
 * '<S117>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Rising edge'
 * '<S118>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Rising edge1'
 * '<S119>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output/Voltage Out (1)'
 * '<S120>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output/Voltage Out (2)'
 * '<S121>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery'
 * '<S122>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Subsystem1'
 * '<S123>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1'
 * '<S124>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal'
 * '<S125>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus'
 * '<S126>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Output Passthrough'
 * '<S127>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery'
 * '<S128>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/Charge Model'
 * '<S129>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/State of Charge Capacity'
 * '<S130>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/Voltage and Power Calculation'
 * '<S131>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator'
 * '<S132>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S133>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrStored Input'
 * '<S134>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S135>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control'
 * '<S136>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Get Cycle Clock Time'
 * '<S137>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Vehicle Speed Controller'
 * '<S138>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Vehicle Speed Controller/Compare To Constant1'
 * '<S139>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Vehicle Speed Controller/Compare To Constant3'
 * '<S140>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Vehicle Speed Controller/Detect Fall Negative'
 * '<S141>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Vehicle Speed Controller/Detect Rise Positive'
 * '<S142>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Vehicle Speed Controller/Detect Fall Negative/Negative'
 * '<S143>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Vehicle Speed Controller/Detect Rise Positive/Positive'
 * '<S144>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff'
 * '<S145>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus'
 * '<S146>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator'
 * '<S147>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S148>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator/PwrStored Input'
 * '<S149>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Front Wheel Drive/Ideal Diff/Power Bus/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S150>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor'
 * '<S151>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2'
 * '<S152>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Electrical Current'
 * '<S153>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power '
 * '<S154>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units'
 * '<S155>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Tabular Power Loss Data'
 * '<S156>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem'
 * '<S157>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem1'
 * '<S158>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator'
 * '<S159>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S160>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrStored Input'
 * '<S161>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S162>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command'
 * '<S163>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Subsystem1'
 * '<S164>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Subsystem4'
 * '<S165>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Accel Pedal to Traction Wheel Torque Request1'
 * '<S166>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Battery Management System'
 * '<S167>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management'
 * '<S168>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Series Regen Braking'
 * '<S169>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Accel Pedal to Traction Wheel Torque Request1/Max Motor Torque'
 * '<S170>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Compare To Constant'
 * '<S171>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management'
 * '<S172>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits'
 * '<S173>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Compare To Constant'
 * '<S174>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Mech to Elec Power Estimate'
 * '<S175>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit'
 * '<S176>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly'
 * '<S177>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Compare To Zero'
 * '<S178>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Compare To Zero1'
 * '<S179>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Saturation Dynamic1'
 * '<S180>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit/Max Motor Torque'
 * '<S181>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit/Saturation Dynamic'
 * '<S182>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly/Compare To Constant'
 * '<S183>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly/Compare To Constant1'
 * '<S184>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Series Regen Braking/Max Motor Torque'
 * '<S185>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest UDP RX ExternalMode  /RotoTest ExternalControlEthernet'
 * '<S186>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest UDP RX ExternalMode  /UDP Receive (1)'
 * '<S187>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest UDP RX ExternalMode  /UDP Transmit (4)_In'
 * '<S188>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest UDP TX ExternalMode  /UDP Transmit (4)_Out'
 * '<S189>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/DummyPT'
 * '<S190>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Format CM Powertrain Signals'
 * '<S191>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest'
 * '<S192>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics'
 * '<S193>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess'
 * '<S194>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Subsystem2'
 * '<S195>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid'
 * '<S196>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX'
 * '<S197>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker'
 * '<S198>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater'
 * '<S199>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater1'
 * '<S200>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater10'
 * '<S201>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater11'
 * '<S202>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater12'
 * '<S203>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater13'
 * '<S204>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater14'
 * '<S205>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater15'
 * '<S206>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater2'
 * '<S207>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater3'
 * '<S208>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater4'
 * '<S209>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater5'
 * '<S210>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater6'
 * '<S211>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater7'
 * '<S212>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater8'
 * '<S213>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater9'
 * '<S214>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits1'
 * '<S215>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits10'
 * '<S216>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits11'
 * '<S217>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits12'
 * '<S218>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits13'
 * '<S219>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits14'
 * '<S220>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits15'
 * '<S221>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits16'
 * '<S222>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits2'
 * '<S223>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits3'
 * '<S224>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits4'
 * '<S225>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits5'
 * '<S226>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits6'
 * '<S227>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits7'
 * '<S228>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits8'
 * '<S229>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Depacker/Extract Bits9'
 * '<S230>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater/Floater'
 * '<S231>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater1/Floater'
 * '<S232>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater10/Floater'
 * '<S233>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater11/Floater'
 * '<S234>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater12/Floater'
 * '<S235>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater13/Floater'
 * '<S236>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater14/Floater'
 * '<S237>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater15/Floater'
 * '<S238>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater2/Floater'
 * '<S239>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater3/Floater'
 * '<S240>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater4/Floater'
 * '<S241>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater5/Floater'
 * '<S242>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater6/Floater'
 * '<S243>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater7/Floater'
 * '<S244>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater8/Floater'
 * '<S245>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/From Rototest/RotoTestEthRX/Floater9/Floater'
 * '<S246>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/WheelSpdCmd'
 * '<S247>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal'
 * '<S248>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Diff Trq'
 * '<S249>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem'
 * '<S250>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2'
 * '<S251>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3'
 * '<S252>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4'
 * '<S253>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem/MATLAB Function'
 * '<S254>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem/WheelInertia'
 * '<S255>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem/WheelInertia1'
 * '<S256>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2/MATLAB Function'
 * '<S257>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2/WheelInertia'
 * '<S258>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem2/WheelInertia1'
 * '<S259>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3/MATLAB Function'
 * '<S260>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3/WheelInertia'
 * '<S261>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem3/WheelInertia1'
 * '<S262>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4/MATLAB Function'
 * '<S263>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4/WheelInertia'
 * '<S264>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem4/WheelInertia1'
 * '<S265>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/Compare To Constant'
 * '<S266>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/Compare To Constant1'
 * '<S267>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau'
 * '<S268>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau1'
 * '<S269>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau2'
 * '<S270>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/RotoTrqPreprocess/FirstOrderFilterTau3'
 * '<S271>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid'
 * '<S272>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Mode Override'
 * '<S273>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx'
 * '<S274>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/TrqSpd Override'
 * '<S275>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid/Band-Aid'
 * '<S276>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater'
 * '<S277>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater1'
 * '<S278>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater2'
 * '<S279>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater3'
 * '<S280>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/MATLAB Function'
 * '<S281>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Packer'
 * '<S282>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/WatchdogCnt'
 * '<S283>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater/Defloater'
 * '<S284>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater1/Defloater'
 * '<S285>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater2/Defloater'
 * '<S286>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/Defloater3/Defloater'
 * '<S287>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/WatchdogCnt/Increment Real World'
 * '<S288>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/WatchdogCnt/Wrap To Zero'
 * '<S289>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF'
 * '<S290>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Brake Pressure'
 * '<S291>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering'
 * '<S292>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Subsystem4'
 * '<S293>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension'
 * '<S294>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle'
 * '<S295>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle Routing'
 * '<S296>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheel Routing'
 * '<S297>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires'
 * '<S298>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering'
 * '<S299>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput'
 * '<S300>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput/ParalConstRatio'
 * '<S301>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput/ParalConstRatio/Parallel'
 * '<S302>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Subsystem4/Subsystem'
 * '<S303>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension'
 * '<S304>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring'
 * '<S305>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Steer Rate Adapter'
 * '<S306>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Steering Adapter'
 * '<S307>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Vehicle Adapter'
 * '<S308>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Wheel  Adapter'
 * '<S309>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson'
 * '<S310>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/Add Axle Offsets'
 * '<S311>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle and track calculate suspension and wheel positions and velocities'
 * '<S312>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities'
 * '<S313>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments'
 * '<S314>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/Wheel Carrier to Axle Interface Compliance'
 * '<S315>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle and track calculate suspension and wheel positions and velocities/Select DCM'
 * '<S316>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics'
 * '<S317>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics'
 * '<S318>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Suspension Forces and Moments'
 * '<S319>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Track location transforms'
 * '<S320>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments'
 * '<S321>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics/Select Axle Mass By Axle'
 * '<S322>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics/Select X Axis Axle Mass Moment of Inertia By Axle'
 * '<S323>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Track location transforms/DCM Transpose'
 * '<S324>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments/Select Axle Mass By Axle'
 * '<S325>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments/Select Y Axis Axle Mass Moment of Inertia By Axle'
 * '<S326>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension'
 * '<S327>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Steering Delta Select'
 * '<S328>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations'
 * '<S329>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Vehicle Moments From X and Y Forces'
 * '<S330>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic'
 * '<S331>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled'
 * '<S332>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Camber Height Slope'
 * '<S333>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Camber Steering Center'
 * '<S334>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Caster Height Slope'
 * '<S335>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Caster Steering Center'
 * '<S336>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Roll Steer Slope'
 * '<S337>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Toe Steering Center'
 * '<S338>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension'
 * '<S339>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination'
 * '<S340>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope'
 * '<S341>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force'
 * '<S342>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select C By Axle'
 * '<S343>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select F0 By Axle'
 * '<S344>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select K By Axle'
 * '<S345>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select Max Travel By Axle'
 * '<S346>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Max stop reached'
 * '<S347>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Min stop reached'
 * '<S348>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Disabled'
 * '<S349>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force'
 * '<S350>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments'
 * '<S351>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force'
 * '<S352>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway'
 * '<S353>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta'
 * '<S354>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Arm Neutral Angle By Axle'
 * '<S355>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Arm Radius By Axle'
 * '<S356>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Bar Torsion Spring Constant By Axle'
 * '<S357>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension'
 * '<S358>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Steering Delta Select'
 * '<S359>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations'
 * '<S360>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Vehicle Moments From X and Y Forces'
 * '<S361>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic'
 * '<S362>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled'
 * '<S363>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Height Slope'
 * '<S364>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Steering Center'
 * '<S365>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Steering Slope'
 * '<S366>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Height Slope'
 * '<S367>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Steering Center'
 * '<S368>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Steering Slope'
 * '<S369>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Roll Steer Slope'
 * '<S370>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Toe Steering Center'
 * '<S371>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Toe Steering Slope'
 * '<S372>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension'
 * '<S373>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination'
 * '<S374>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope'
 * '<S375>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force'
 * '<S376>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select C By Axle'
 * '<S377>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select F0 By Axle'
 * '<S378>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select K By Axle'
 * '<S379>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select Max Travel By Axle'
 * '<S380>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Max stop reached'
 * '<S381>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Min stop reached'
 * '<S382>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Enabled'
 * '<S383>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Enabled/Steering Height Slope By Steered Axle'
 * '<S384>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem'
 * '<S385>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF'
 * '<S386>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1'
 * '<S387>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1'
 * '<S388>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Aero Drag'
 * '<S389>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Forces'
 * '<S390>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Gravity'
 * '<S391>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Moment Calc'
 * '<S392>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Moments'
 * '<S393>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Power'
 * '<S394>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing'
 * '<S395>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection'
 * '<S396>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Susp2Chassis'
 * '<S397>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Wheel to CG'
 * '<S398>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/vehdyncginert'
 * '<S399>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles'
 * '<S400>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot'
 * '<S401>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Determine Force,  Mass & Inertia'
 * '<S402>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Vbxw'
 * '<S403>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Velocity Conversion'
 * '<S404>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Velocity Conversion1'
 * '<S405>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Velocity Conversion2'
 * '<S406>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/transform to Inertial axes '
 * '<S407>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix'
 * '<S408>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles/phidot thetadot psidot'
 * '<S409>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S410>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/3x3 Cross Product'
 * '<S411>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/I x w'
 * '<S412>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/I x w1'
 * '<S413>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/3x3 Cross Product/Subsystem'
 * '<S414>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Calculate omega_dot/3x3 Cross Product/Subsystem1'
 * '<S415>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Determine Force,  Mass & Inertia/Mass input//output  momentum'
 * '<S416>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Determine Force,  Mass & Inertia/Mass input//output  momentum/For Each Subsystem'
 * '<S417>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Vbxw/Subsystem'
 * '<S418>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/6DOF (Euler Angles)1/Vbxw/Subsystem1'
 * '<S419>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Aero Drag/Drag Force'
 * '<S420>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left'
 * '<S421>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right'
 * '<S422>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric'
 * '<S423>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left'
 * '<S424>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right'
 * '<S425>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement'
 * '<S426>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S427>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S428>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S429>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S430>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S431>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S432>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S433>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement'
 * '<S434>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S435>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S436>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S437>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S438>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S439>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S440>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S441>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta'
 * '<S442>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip'
 * '<S443>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Rotation Angles to Direction Cosine Matrix'
 * '<S444>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/transform to Inertial axes'
 * '<S445>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/transform to Inertial axes1'
 * '<S446>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR'
 * '<S447>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly'
 * '<S448>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly/Compare To Constant'
 * '<S449>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly/Compare To Constant1'
 * '<S450>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S451>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR/Subsystem'
 * '<S452>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR/Subsystem1'
 * '<S453>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement'
 * '<S454>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S455>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S456>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S457>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S458>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S459>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S460>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S461>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement'
 * '<S462>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S463>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S464>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S465>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S466>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S467>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S468>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/Signal Routing/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S469>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Angle Wrap'
 * '<S470>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip'
 * '<S471>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/phidot thetadot psidot'
 * '<S472>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/wxR'
 * '<S473>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Angle Wrap/None'
 * '<S474>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip/div0protect - abs poly'
 * '<S475>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip/div0protect - abs poly/Compare To Constant'
 * '<S476>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/Body Slip/div0protect - abs poly/Compare To Constant1'
 * '<S477>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/wxR/Subsystem'
 * '<S478>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/6DOF/Vehicle Body 6DOF1/SignalCollection/wxR/Subsystem1'
 * '<S479>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/Cont LPF'
 * '<S480>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/Cont LPF1'
 * '<S481>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS'
 * '<S482>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh'
 * '<S483>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires'
 * '<S484>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire'
 * '<S485>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/scale factors with friction'
 * '<S486>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Routiong'
 * '<S487>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel Angles'
 * '<S488>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform'
 * '<S489>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix'
 * '<S490>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S491>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires'
 * '<S492>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1'
 * '<S493>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2'
 * '<S494>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3'
 * '<S495>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4'
 * '<S496>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fx'
 * '<S497>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fy'
 * '<S498>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fz'
 * '<S499>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Mx'
 * '<S500>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/My'
 * '<S501>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Mz'
 * '<S502>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Re'
 * '<S503>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response'
 * '<S504>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response1'
 * '<S505>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response2'
 * '<S506>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response3'
 * '<S507>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/alpha'
 * '<S508>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/kappa'
 * '<S509>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/omega'
 * '<S510>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/z'
 * '<S511>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/zdot'
 * '<S512>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Bus Routing'
 * '<S513>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input'
 * '<S514>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Vertical Wheel and Unsprung Mass Response'
 * '<S515>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module'
 * '<S516>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Fx Relaxation'
 * '<S517>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Fy Relaxation'
 * '<S518>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Magic Tire Const Input'
 * '<S519>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/My Relaxation'
 * '<S520>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes'
 * '<S521>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Friction Model'
 * '<S522>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp'
 * '<S523>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes/Disk Brake'
 * '<S524>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S525>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/Locked'
 * '<S526>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/Slipping'
 * '<S527>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup'
 * '<S528>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectSlip'
 * '<S529>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S530>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S531>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S532>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S533>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S534>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S535>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S536>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S537>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Bus Routing'
 * '<S538>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input'
 * '<S539>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Vertical Wheel and Unsprung Mass Response'
 * '<S540>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module'
 * '<S541>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Fx Relaxation'
 * '<S542>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Fy Relaxation'
 * '<S543>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Magic Tire Const Input'
 * '<S544>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/My Relaxation'
 * '<S545>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes'
 * '<S546>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Friction Model'
 * '<S547>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp'
 * '<S548>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes/Disk Brake'
 * '<S549>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S550>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/Locked'
 * '<S551>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/Slipping'
 * '<S552>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup'
 * '<S553>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectSlip'
 * '<S554>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S555>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S556>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S557>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S558>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S559>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S560>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S561>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S562>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Bus Routing'
 * '<S563>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input'
 * '<S564>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Vertical Wheel and Unsprung Mass Response'
 * '<S565>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module'
 * '<S566>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Fx Relaxation'
 * '<S567>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Fy Relaxation'
 * '<S568>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Magic Tire Const Input'
 * '<S569>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/My Relaxation'
 * '<S570>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes'
 * '<S571>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Friction Model'
 * '<S572>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp'
 * '<S573>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes/Disk Brake'
 * '<S574>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S575>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/Locked'
 * '<S576>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/Slipping'
 * '<S577>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup'
 * '<S578>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectSlip'
 * '<S579>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S580>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S581>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S582>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S583>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S584>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S585>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S586>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S587>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Bus Routing'
 * '<S588>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input'
 * '<S589>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Vertical Wheel and Unsprung Mass Response'
 * '<S590>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module'
 * '<S591>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Fx Relaxation'
 * '<S592>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Fy Relaxation'
 * '<S593>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Magic Tire Const Input'
 * '<S594>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/My Relaxation'
 * '<S595>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes'
 * '<S596>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Friction Model'
 * '<S597>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp'
 * '<S598>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes/Disk Brake'
 * '<S599>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S600>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/Locked'
 * '<S601>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/Slipping'
 * '<S602>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup'
 * '<S603>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectSlip'
 * '<S604>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic'
 * '<S605>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S606>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S607>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S608>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S609>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S610>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S611>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/LockUp/detectSlip/Break Apart Detection'
 * '<S612>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel Angles'
 * '<S613>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform'
 * '<S614>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix'
 * '<S615>' : 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/Vehicle Under Test/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 */
#endif            /* RTW_HEADER_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_h_ */
