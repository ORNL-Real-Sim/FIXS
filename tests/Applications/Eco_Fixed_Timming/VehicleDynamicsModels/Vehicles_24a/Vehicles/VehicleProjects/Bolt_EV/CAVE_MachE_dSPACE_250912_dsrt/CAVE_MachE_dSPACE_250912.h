/*
 * CAVE_MachE_dSPACE_250912.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "CAVE_MachE_dSPACE_250912".
 *
 * Model version              : 10.86
 * Simulink Coder version : 24.1 (R2024a) 19-Nov-2023
 * C source code generated on : Sun Sep 21 17:10:19 2025
 *
 * Target selection: dsrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef CAVE_MachE_dSPACE_250912_h_
#define CAVE_MachE_dSPACE_250912_h_
#ifndef CAVE_MachE_dSPACE_250912_COMMON_INCLUDES_
#define CAVE_MachE_dSPACE_250912_COMMON_INCLUDES_
#include <stdio.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "CAVE_MachE_dSPACE_250912_types.h"
#include "CAVE_MachE_dSPACE_250912_dsmpbap.h"
#endif                           /* CAVE_MachE_dSPACE_250912_COMMON_INCLUDES_ */

#include <string.h>
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals for system '<S175>/Pass Through' */
typedef struct {
  real_T u;                            /* '<S176>/u' */
} B_PassThrough_CAVE_MachE_dSPACE_250912_T;

/* Block states (default storage) for system '<S175>/Pass Through' */
typedef struct {
  boolean_T PassThrough_MODE;          /* '<S175>/Pass Through' */
} DW_PassThrough_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S216>/If Action Subsystem1' */
typedef struct {
  real_T Divide;                       /* '<S220>/Divide' */
  real_T Divide1;                      /* '<S220>/Divide1' */
  real_T Interpolatedzerocrossing;     /* '<S220>/Interpolated zero-crossing' */
  real_T Divide2;                      /* '<S220>/Divide2' */
} B_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S274>/For each axle and track calculate suspension and wheel positions and velocities' */
typedef struct {
  real_T Selector6;                    /* '<S281>/Selector6' */
  real_T TrigonometricFunction;        /* '<S281>/Trigonometric Function' */
  real_T Gain;                         /* '<S281>/Gain' */
  real_T Sum;                          /* '<S285>/Sum' */
  real_T DCMStaringRow;                /* '<S285>/DCM Staring Row' */
  real_T Sum1;                         /* '<S285>/Sum1' */
  real_T SelectDCM[9];                 /* '<S285>/Select DCM' */
  real_T MathFunction[9];              /* '<S281>/Math Function' */
  real_T Selector1[3];                 /* '<S281>/Selector1' */
  real_T MatrixMultiply1[3];           /* '<S281>/Matrix Multiply1' */
  real_T Translationeffectonpositions[3];/* '<S281>/Selector2' */
  real_T Sum1_d[3];                    /* '<S281>/Sum1' */
  real_T Selector5[3];                 /* '<S281>/Selector5' */
  real_T TrigonometricFunction1;       /* '<S281>/Trigonometric Function1' */
  real_T MatrixConcatenate4[3];        /* '<S281>/Matrix Concatenate4' */
  real_T Product2[3];                  /* '<S281>/Product2' */
  real_T Sum3[3];                      /* '<S281>/Sum3' */
  real_T Selector[3];                  /* '<S281>/Selector' */
  real_T Rotationeffectonpositions[3]; /* '<S281>/Matrix Multiply3' */
  real_T Sum4[3];                      /* '<S281>/Sum4' */
  real_T Product1[3];                  /* '<S281>/Product1' */
  real_T Sum2[3];                      /* '<S281>/Sum2' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S274>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T Reshape[3];                   /* '<S293>/Reshape' */
  real_T u;                            /* '<S293>/ ' */
  real_T TrigonometricFunction;        /* '<S293>/Trigonometric Function' */
  real_T TrigonometricFunction1;       /* '<S293>/Trigonometric Function1' */
  real_T MatrixConcatenate4[3];        /* '<S293>/Matrix Concatenate4' */
  real_T MatrixConcatenate5[3];        /* '<S293>/Matrix Concatenate5' */
  real_T MatrixConcatenate3[9];        /* '<S293>/Matrix Concatenate3' */
  real_T cgcoordinates[3];             /* '<S289>/cg coordinates' */
  real_T MathFunction1[9];             /* '<S289>/Math Function1' */
  real_T zdot;                         /* '<S287>/Vz' */
  real_T TmpSignalConversionAtMatrixMultiply2Inport2[3];/* '<S289>/cgV in' */
  real_T MatrixMultiply2[3];           /* '<S289>/Matrix Multiply2' */
  real_T p;                            /* '<S287>/Vy1' */
  real_T yaxistrackcoordinates[2];     /* '<S290>/y axis track coordinates' */
  real_T MathFunction[2];              /* '<S290>/Math Function' */
  real_T yaxistrackcoordinates_d[2];   /* '<S288>/y axis track coordinates' */
  real_T Product1;                     /* '<S287>/Product1' */
  real_T Sum;                          /* '<S287>/Sum' */
  real_T ydot;                         /* '<S287>/Vy' */
  real_T ydotp;                        /* '<S287>/Vyp' */
  real_T DataTypeConversion[2];        /* '<S290>/Data Type Conversion' */
  real_T Product[2];                   /* '<S290>/Product' */
  real_T DotProduct1;                  /* '<S290>/Dot Product1' */
  real_T DataTypeConversion_n;         /* '<S294>/Data Type Conversion' */
  real_T Product_h;                    /* '<S294>/Product' */
  real_T SumofElements;                /* '<S294>/Sum of Elements' */
  real_T Product2[2];                  /* '<S290>/Product2' */
  real_T DataTypeConversion_f;         /* '<S295>/Data Type Conversion' */
  real_T Product_c;                    /* '<S295>/Product' */
  real_T SumofElements_o;              /* '<S295>/Sum of Elements' */
  real_T Sum_a[2];                     /* '<S290>/Sum' */
  real_T Product3[2];                  /* '<S290>/Product3' */
  real_T Product4[2];                  /* '<S290>/Product4' */
  real_T xaxiswheelmoments[2];         /* '<S290>/x axis wheel moments' */
  real_T DotProduct2;                  /* '<S290>/Dot Product2' */
  real_T Sum1;                         /* '<S290>/Sum1' */
  real_T DataTypeConversion_k[2];      /* '<S288>/Data Type Conversion' */
  real_T Product_l[2];                 /* '<S288>/Product' */
  real_T DotProduct1_e;                /* '<S288>/Dot Product1' */
  real_T SuspensionMomentDirectionOnSolidAxle;
                        /* '<S288>/Suspension Moment Direction On Solid Axle' */
  real_T Sum1_b;                       /* '<S286>/Sum1' */
  real_T DataTypeConversion_h;         /* '<S292>/Data Type Conversion' */
  real_T Product_o;                    /* '<S292>/Product' */
  real_T SumofElements_d;              /* '<S292>/Sum of Elements' */
  real_T pdot;                         /* '<S287>/Divide' */
  real_T DotProduct;                   /* '<S290>/Dot Product' */
  real_T DotProduct_i;                 /* '<S288>/Dot Product' */
  real_T SuspensionForceDirectionOnSolidAxle;
                         /* '<S288>/Suspension Force Direction On Solid Axle' */
  real_T Sum_p;                        /* '<S286>/Sum' */
  real_T DataTypeConversion_b;         /* '<S291>/Data Type Conversion' */
  real_T Product_le;                   /* '<S291>/Product' */
  real_T SumofElements_j;              /* '<S291>/Sum of Elements' */
  real_T Divide1;                      /* '<S287>/Divide1' */
  real_T Sum2;                         /* '<S287>/Sum2' */
  real_T zddot;                        /* '<S287>/Sum1' */
  boolean_T RelationalOperator[2];     /* '<S290>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S294>/Relational Operator' */
  boolean_T RelationalOperator_c;      /* '<S295>/Relational Operator' */
  boolean_T RelationalOperator_n[2];   /* '<S288>/Relational Operator' */
  boolean_T RelationalOperator_m;      /* '<S292>/Relational Operator' */
  boolean_T RelationalOperator_e;      /* '<S291>/Relational Operator' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T;

/* Continuous states for system '<S274>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T _CSTATE;                      /* '<S293>/ ' */
  real_T cgcoordinates_CSTATE[3];      /* '<S289>/cg coordinates' */
  real_T Vz_CSTATE;                    /* '<S287>/Vz' */
  real_T Vy1_CSTATE;                   /* '<S287>/Vy1' */
  real_T Vy_CSTATE;                    /* '<S287>/Vy' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_k_T;

/* State derivatives for system '<S274>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  real_T _CSTATE;                      /* '<S293>/ ' */
  real_T cgcoordinates_CSTATE[3];      /* '<S289>/cg coordinates' */
  real_T Vz_CSTATE;                    /* '<S287>/Vz' */
  real_T Vy1_CSTATE;                   /* '<S287>/Vy1' */
  real_T Vy_CSTATE;                    /* '<S287>/Vy' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_p_T;

/* State Disabled for system '<S274>/For each axle calculate axle cg positions and velocities' */
typedef struct {
  boolean_T _CSTATE;                   /* '<S293>/ ' */
  boolean_T cgcoordinates_CSTATE[3];   /* '<S289>/cg coordinates' */
  boolean_T Vz_CSTATE;                 /* '<S287>/Vz' */
  boolean_T Vy1_CSTATE;                /* '<S287>/Vy1' */
  boolean_T Vy_CSTATE;                 /* '<S287>/Vy' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_n_T;

/* Block signals for system '<S314>/Min stop reached' */
typedef struct {
  real_T Sum1;                         /* '<S320>/Sum1' */
  real_T Gain5;                        /* '<S320>/Gain5' */
  real_T Gain4;                        /* '<S320>/Gain4' */
  real_T Product3;                     /* '<S320>/Product3' */
  real_T Abs1;                         /* '<S320>/Abs1' */
  real_T Saturation;                   /* '<S320>/Saturation' */
  real_T TrigonometricFunction;        /* '<S320>/Trigonometric Function' */
  real_T Gain;                         /* '<S320>/Gain' */
  real_T Sum2;                         /* '<S320>/Sum2' */
  real_T LowerHardStopBlendMult;       /* '<S320>/Lower Hard Stop Blend Mult' */
  real_T MathFunction;                 /* '<S320>/Math Function' */
  real_T Product2;                     /* '<S320>/Product2' */
  real_T Product;                      /* '<S320>/Product' */
  real_T Product1;                     /* '<S320>/Product1' */
  real_T Sum;                          /* '<S320>/Sum' */
  real_T Product4;                     /* '<S320>/Product4' */
} B_Minstopreached_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S314>/Max stop reached' */
typedef struct {
  real_T Sum1;                         /* '<S319>/Sum1' */
  real_T Gain5;                        /* '<S319>/Gain5' */
  real_T Gain4;                        /* '<S319>/Gain4' */
  real_T Product3;                     /* '<S319>/Product3' */
  real_T Abs1;                         /* '<S319>/Abs1' */
  real_T Saturation;                   /* '<S319>/Saturation' */
  real_T TrigonometricFunction;        /* '<S319>/Trigonometric Function' */
  real_T Gain;                         /* '<S319>/Gain' */
  real_T MathFunction;                 /* '<S319>/Math Function' */
  real_T Product2;                     /* '<S319>/Product2' */
  real_T Product;                      /* '<S319>/Product' */
  real_T Product1;                     /* '<S319>/Product1' */
  real_T Sum;                          /* '<S319>/Sum' */
  real_T Sum2;                         /* '<S319>/Sum2' */
  real_T UpperHardStopBlendMult;       /* '<S319>/Upper Hard Stop Blend Mult' */
  real_T Product4;                     /* '<S319>/Product4' */
} B_Maxstopreached_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S274>/For each track and axle combination calculate suspension forces and moments' */
typedef struct {
  real_T DataTypeConversion;           /* '<S306>/Data Type Conversion' */
  real_T Product;                      /* '<S306>/Product' */
  real_T SumofElements;                /* '<S306>/Sum of Elements' */
  real_T DataTypeConversion_d;         /* '<S305>/Data Type Conversion' */
  real_T Product_m;                    /* '<S305>/Product' */
  real_T SumofElements_i;              /* '<S305>/Sum of Elements' */
  real_T TmpSignalConversionAtSelector3Inport1[2];/* '<S297>/Mux' */
  real_T Selector1;                    /* '<S296>/Selector1' */
  real_T Sum2;                         /* '<S296>/Sum2' */
  real_T Selector5;                    /* '<S297>/Selector5' */
  real_T Product_c;                    /* '<S297>/Product' */
  real_T Selector3;                    /* '<S297>/Selector3' */
  real_T Abs;                          /* '<S311>/Abs' */
  real_T Product_o;                    /* '<S311>/Product' */
  real_T Selector2[2];                 /* '<S296>/Selector2' */
  real_T Selector[2];                  /* '<S296>/Selector' */
  real_T Add;                          /* '<S311>/Add' */
  real_T DataTypeConversion_b;         /* '<S317>/Data Type Conversion' */
  real_T Product_cj;                   /* '<S317>/Product' */
  real_T SumofElements_p;              /* '<S317>/Sum of Elements' */
  real_T DataTypeConversion_f;         /* '<S316>/Data Type Conversion' */
  real_T Product_k;                    /* '<S316>/Product' */
  real_T SumofElements_f;              /* '<S316>/Sum of Elements' */
  real_T Product4;                     /* '<S312>/Product4' */
  real_T Add4;                         /* '<S312>/Add4' */
  real_T HeightSignConvention;         /* '<S312>/Height Sign Convention' */
  real_T Product3;                     /* '<S301>/Product3' */
  real_T Sum2_j;                       /* '<S301>/Sum2' */
  real_T DataTypeConversion_a;         /* '<S308>/Data Type Conversion' */
  real_T Product_mg;                   /* '<S308>/Product' */
  real_T SumofElements_p3;             /* '<S308>/Sum of Elements' */
  real_T DataTypeConversion_i;         /* '<S307>/Data Type Conversion' */
  real_T Product_b;                    /* '<S307>/Product' */
  real_T SumofElements_n;              /* '<S307>/Sum of Elements' */
  real_T Product5;                     /* '<S301>/Product5' */
  real_T Sum1;                         /* '<S301>/Sum1' */
  real_T DataTypeConversion_l;         /* '<S310>/Data Type Conversion' */
  real_T Product_o5;                   /* '<S310>/Product' */
  real_T SumofElements_m;              /* '<S310>/Sum of Elements' */
  real_T DataTypeConversion_fr;        /* '<S309>/Data Type Conversion' */
  real_T Product_mgi;                  /* '<S309>/Product' */
  real_T SumofElements_ic;             /* '<S309>/Sum of Elements' */
  real_T Product1;                     /* '<S301>/Product1' */
  real_T Sum;                          /* '<S301>/Sum' */
  real_T Selector1_e;                  /* '<S304>/Selector1' */
  real_T Sum_j;                        /* '<S304>/Sum' */
  real_T Gain;                         /* '<S304>/Gain' */
  real_T Sum1_d;                       /* '<S304>/Sum1' */
  real_T Sign;                         /* '<S304>/Sign' */
  real_T Product_i;                    /* '<S304>/Product' */
  real_T Sum3;                         /* '<S301>/Sum3' */
  real_T Selector1_a;                  /* '<S303>/Selector1' */
  real_T Sum_n;                        /* '<S303>/Sum' */
  real_T Gain_f;                       /* '<S303>/Gain' */
  real_T Sum1_g;                       /* '<S303>/Sum1' */
  real_T Sign_g;                       /* '<S303>/Sign' */
  real_T Product_e;                    /* '<S303>/Product' */
  real_T Product3_c;                   /* '<S312>/Product3' */
  real_T DataTypeConversion_e;         /* '<S315>/Data Type Conversion' */
  real_T Product_n;                    /* '<S315>/Product' */
  real_T SumofElements_fb;             /* '<S315>/Sum of Elements' */
  real_T Add2;                         /* '<S311>/Add2' */
  real_T Product5_f;                   /* '<S312>/Product5' */
  real_T Add1;                         /* '<S312>/Add1' */
  real_T Product1_c;                   /* '<S312>/Product1' */
  real_T Sign1;                        /* '<S312>/Sign1' */
  real_T Product2;                     /* '<S312>/Product2' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[8];/* '<S283>/Suspension' */
  real_T DataTypeConversion_m;         /* '<S318>/Data Type Conversion' */
  real_T Product_ny;                   /* '<S318>/Product' */
  real_T SumofElements_mu;             /* '<S318>/Sum of Elements' */
  real_T Sum_p;                        /* '<S314>/Sum' */
  real_T Sum_c;                        /* '<S312>/Sum' */
  real_T VehicleForceSign;             /* '<S311>/Vehicle Force Sign' */
  real_T Selector1_b;                  /* '<S299>/Selector1' */
  real_T Selector_a;                   /* '<S299>/Selector' */
  real_T VehicleHeight;                /* '<S311>/Sign convention' */
  real_T Sum_ns;                       /* '<S299>/Sum' */
  real_T Product_or;                   /* '<S299>/Product' */
  real_T UnaryMinus;                   /* '<S299>/Unary Minus' */
  real_T Selector2_b;                  /* '<S299>/Selector2' */
  real_T Product1_o;                   /* '<S299>/Product1' */
  real_T Reshape[3];                   /* '<S299>/Reshape' */
  real_T Selector1_ac;                 /* '<S302>/Selector1' */
  real_T Sum_i;                        /* '<S302>/Sum' */
  real_T Gain_fc;                      /* '<S302>/Gain' */
  real_T Sum1_i;                       /* '<S302>/Sum1' */
  real_T Sign_d;                       /* '<S302>/Sign' */
  real_T Product_f;                    /* '<S302>/Product' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[3];/* '<S283>/Suspension' */
  real_T Reshape19[2];                 /* '<S283>/Reshape19' */
  real_T Selector3_h;                  /* '<S296>/Selector3' */
  boolean_T RelationalOperator;        /* '<S306>/Relational Operator' */
  boolean_T RelationalOperator_f;      /* '<S305>/Relational Operator' */
  boolean_T RelationalOperator_i;      /* '<S317>/Relational Operator' */
  boolean_T RelationalOperator_k;      /* '<S316>/Relational Operator' */
  boolean_T RelationalOperator_o;      /* '<S308>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S307>/Relational Operator' */
  boolean_T RelationalOperator_n;      /* '<S310>/Relational Operator' */
  boolean_T RelationalOperator_ka;     /* '<S309>/Relational Operator' */
  boolean_T RelationalOperator_j;      /* '<S315>/Relational Operator' */
  boolean_T RelationalOperator_d;      /* '<S318>/Relational Operator' */
  B_Maxstopreached_CAVE_MachE_dSPACE_250912_T Maxstopreached;/* '<S314>/Max stop reached' */
  B_Minstopreached_CAVE_MachE_dSPACE_250912_T Minstopreached;/* '<S314>/Min stop reached' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T;

/* Block signals for system '<S324>/For Each Axle With Anti-Sway' */
typedef struct {
  real_T DataTypeConversion;           /* '<S328>/Data Type Conversion' */
  real_T Product;                      /* '<S328>/Product' */
  real_T SumofElements;                /* '<S328>/Sum of Elements' */
  real_T DataTypeConversion_f;         /* '<S327>/Data Type Conversion' */
  real_T Product_c;                    /* '<S327>/Product' */
  real_T SumofElements_p;              /* '<S327>/Sum of Elements' */
  real_T TrigonometricFunction;        /* '<S326>/Trigonometric Function' */
  real_T Z0;                           /* '<S326>/Product' */
  real_T Selector5[2];                 /* '<S325>/Selector5' */
  real_T Selector1;                    /* '<S325>/Selector1' */
  real_T Sum2[2];                      /* '<S325>/Sum2' */
  real_T Selector3[2];                 /* '<S325>/Selector3' */
  real_T Selector6[2];                 /* '<S325>/Selector6' */
  real_T Selector4[2];                 /* '<S325>/Selector4' */
  real_T Sum3;                         /* '<S326>/Sum3' */
  real_T Sum6;                         /* '<S326>/Sum6' */
  real_T Sum[2];                       /* '<S326>/Sum' */
  real_T Product1[2];                  /* '<S326>/Product1' */
  real_T AngleTangentLimit[2];         /* '<S326>/Angle Tangent Limit' */
  real_T DataTypeConversion_k;         /* '<S329>/Data Type Conversion' */
  real_T Product_f;                    /* '<S329>/Product' */
  real_T SumofElements_d;              /* '<S329>/Sum of Elements' */
  real_T TrigonometricFunction1[2];    /* '<S326>/Trigonometric Function1' */
  real_T Sum1[2];                      /* '<S326>/Sum1' */
  real_T deltaTheta;                   /* '<S326>/Sum2' */
  real_T antiswaybartorque;            /* '<S326>/Product4' */
  real_T Gain;                         /* '<S326>/Gain' */
  real_T Product2[2];                  /* '<S326>/Product2' */
  real_T TrigonometricFunction2[2];    /* '<S326>/Trigonometric Function2' */
  real_T Product3[2];                  /* '<S326>/Product3' */
  real_T Selector[2];                  /* '<S325>/Selector' */
  real_T Sum4[2];                      /* '<S326>/Sum4' */
  real_T Selector2[2];                 /* '<S325>/Selector2' */
  real_T Sum5[2];                      /* '<S326>/Sum5' */
  boolean_T RelationalOperator;        /* '<S328>/Relational Operator' */
  boolean_T RelationalOperator_l;      /* '<S327>/Relational Operator' */
  boolean_T RelationalOperator_h;      /* '<S329>/Relational Operator' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_hf_T;

/* Block signals for system '<S279>/For each track and axle combination calculate suspension forces and moments' */
typedef struct {
  real_T TmpSignalConversionAtSelector3Inport1[3];/* '<S331>/Mux' */
  real_T Selector1;                    /* '<S330>/Selector1' */
  real_T Sum2;                         /* '<S330>/Sum2' */
  real_T Selector5;                    /* '<S331>/Selector5' */
  real_T Product;                      /* '<S331>/Product' */
  real_T Selector3;                    /* '<S331>/Selector3' */
  real_T Abs2;                         /* '<S335>/Abs2' */
  real_T DataTypeConversion;           /* '<S341>/Data Type Conversion' */
  real_T Product_a;                    /* '<S341>/Product' */
  real_T SumofElements;                /* '<S341>/Sum of Elements' */
  real_T Product2;                     /* '<S335>/Product2' */
  real_T DataTypeConversion_n;         /* '<S340>/Data Type Conversion' */
  real_T Product_h;                    /* '<S340>/Product' */
  real_T SumofElements_n;              /* '<S340>/Sum of Elements' */
  real_T DataTypeConversion_j;         /* '<S339>/Data Type Conversion' */
  real_T Product_i;                    /* '<S339>/Product' */
  real_T SumofElements_nb;             /* '<S339>/Sum of Elements' */
  real_T DataTypeConversion_e;         /* '<S359>/Data Type Conversion' */
  real_T Product_f;                    /* '<S359>/Product' */
  real_T SumofElements_g;              /* '<S359>/Sum of Elements' */
  real_T Abs;                          /* '<S348>/Abs' */
  real_T Product_c;                    /* '<S348>/Product' */
  real_T Selector2[2];                 /* '<S330>/Selector2' */
  real_T Selector[2];                  /* '<S330>/Selector' */
  real_T Add;                          /* '<S348>/Add' */
  real_T DataTypeConversion_g;         /* '<S354>/Data Type Conversion' */
  real_T Product_b;                    /* '<S354>/Product' */
  real_T SumofElements_e;              /* '<S354>/Sum of Elements' */
  real_T DataTypeConversion_k;         /* '<S353>/Data Type Conversion' */
  real_T Product_cs;                   /* '<S353>/Product' */
  real_T SumofElements_o;              /* '<S353>/Sum of Elements' */
  real_T Product4;                     /* '<S349>/Product4' */
  real_T Add4;                         /* '<S349>/Add4' */
  real_T HeightSignConvention;         /* '<S349>/Height Sign Convention' */
  real_T Product3;                     /* '<S335>/Product3' */
  real_T Sum2_p;                       /* '<S335>/Sum2' */
  real_T Abs1;                         /* '<S335>/Abs1' */
  real_T DataTypeConversion_c;         /* '<S344>/Data Type Conversion' */
  real_T Product_g;                    /* '<S344>/Product' */
  real_T SumofElements_h;              /* '<S344>/Sum of Elements' */
  real_T Product4_e;                   /* '<S335>/Product4' */
  real_T DataTypeConversion_b;         /* '<S343>/Data Type Conversion' */
  real_T Product_n;                    /* '<S343>/Product' */
  real_T SumofElements_hk;             /* '<S343>/Sum of Elements' */
  real_T DataTypeConversion_l;         /* '<S342>/Data Type Conversion' */
  real_T Product_p;                    /* '<S342>/Product' */
  real_T SumofElements_eu;             /* '<S342>/Sum of Elements' */
  real_T Product5;                     /* '<S335>/Product5' */
  real_T Sum1;                         /* '<S335>/Sum1' */
  real_T Abs_h;                        /* '<S335>/Abs' */
  real_T DataTypeConversion_i;         /* '<S347>/Data Type Conversion' */
  real_T Product_k;                    /* '<S347>/Product' */
  real_T SumofElements_gg;             /* '<S347>/Sum of Elements' */
  real_T Product_l;                    /* '<S335>/Product' */
  real_T DataTypeConversion_kr;        /* '<S346>/Data Type Conversion' */
  real_T Product_m;                    /* '<S346>/Product' */
  real_T SumofElements_p;              /* '<S346>/Sum of Elements' */
  real_T DataTypeConversion_o;         /* '<S345>/Data Type Conversion' */
  real_T Product_n5;                   /* '<S345>/Product' */
  real_T SumofElements_f;              /* '<S345>/Sum of Elements' */
  real_T Product1;                     /* '<S335>/Product1' */
  real_T Sum;                          /* '<S335>/Sum' */
  real_T Selector1_c;                  /* '<S338>/Selector1' */
  real_T Sum_e;                        /* '<S338>/Sum' */
  real_T Gain;                         /* '<S338>/Gain' */
  real_T Sum1_f;                       /* '<S338>/Sum1' */
  real_T Sign;                         /* '<S338>/Sign' */
  real_T Product_na;                   /* '<S338>/Product' */
  real_T Sum3;                         /* '<S335>/Sum3' */
  real_T Selector1_l;                  /* '<S337>/Selector1' */
  real_T Sum_o;                        /* '<S337>/Sum' */
  real_T Gain_p;                       /* '<S337>/Gain' */
  real_T Sum1_e;                       /* '<S337>/Sum1' */
  real_T Sign_f;                       /* '<S337>/Sign' */
  real_T Product_kc;                   /* '<S337>/Product' */
  real_T Product3_k;                   /* '<S349>/Product3' */
  real_T DataTypeConversion_p;         /* '<S352>/Data Type Conversion' */
  real_T Product_ca;                   /* '<S352>/Product' */
  real_T SumofElements_j;              /* '<S352>/Sum of Elements' */
  real_T Add2;                         /* '<S348>/Add2' */
  real_T Product5_o;                   /* '<S349>/Product5' */
  real_T Add1;                         /* '<S349>/Add1' */
  real_T Product1_g;                   /* '<S349>/Product1' */
  real_T Sign1;                        /* '<S349>/Sign1' */
  real_T Product2_n;                   /* '<S349>/Product2' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[8];/* '<S323>/Suspension' */
  real_T DataTypeConversion_ix;        /* '<S355>/Data Type Conversion' */
  real_T Product_ii;                   /* '<S355>/Product' */
  real_T SumofElements_hd;             /* '<S355>/Sum of Elements' */
  real_T Sum_a;                        /* '<S351>/Sum' */
  real_T Sum_aa;                       /* '<S349>/Sum' */
  real_T VehicleForceSign;             /* '<S348>/Vehicle Force Sign' */
  real_T Selector1_i;                  /* '<S333>/Selector1' */
  real_T Selector_j;                   /* '<S333>/Selector' */
  real_T VehicleHeight;                /* '<S348>/Sign convention' */
  real_T Sum_ew;                       /* '<S333>/Sum' */
  real_T Product_o;                    /* '<S333>/Product' */
  real_T UnaryMinus;                   /* '<S333>/Unary Minus' */
  real_T Selector2_j;                  /* '<S333>/Selector2' */
  real_T Product1_p;                   /* '<S333>/Product1' */
  real_T Reshape[3];                   /* '<S333>/Reshape' */
  real_T Selector1_a;                  /* '<S336>/Selector1' */
  real_T Sum_n;                        /* '<S336>/Sum' */
  real_T Gain_py;                      /* '<S336>/Gain' */
  real_T Sum1_o;                       /* '<S336>/Sum1' */
  real_T Sign_p;                       /* '<S336>/Sign' */
  real_T Product_fv;                   /* '<S336>/Product' */
  real_T TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[3];/* '<S323>/Suspension' */
  real_T Selector3_i;                  /* '<S330>/Selector3' */
  boolean_T RelationalOperator;        /* '<S341>/Relational Operator' */
  boolean_T RelationalOperator_k;      /* '<S340>/Relational Operator' */
  boolean_T RelationalOperator_h;      /* '<S339>/Relational Operator' */
  boolean_T RelationalOperator_m;      /* '<S359>/Relational Operator' */
  boolean_T RelationalOperator_m3;     /* '<S354>/Relational Operator' */
  boolean_T RelationalOperator_i;      /* '<S353>/Relational Operator' */
  boolean_T RelationalOperator_d;      /* '<S344>/Relational Operator' */
  boolean_T RelationalOperator_e;      /* '<S343>/Relational Operator' */
  boolean_T RelationalOperator_p;      /* '<S342>/Relational Operator' */
  boolean_T RelationalOperator_e0;     /* '<S347>/Relational Operator' */
  boolean_T RelationalOperator_hr;     /* '<S346>/Relational Operator' */
  boolean_T RelationalOperator_o;      /* '<S345>/Relational Operator' */
  boolean_T RelationalOperator_j;      /* '<S352>/Relational Operator' */
  boolean_T RelationalOperator_mz;     /* '<S355>/Relational Operator' */
  B_Maxstopreached_CAVE_MachE_dSPACE_250912_T Maxstopreached;/* '<S351>/Max stop reached' */
  B_Minstopreached_CAVE_MachE_dSPACE_250912_T Minstopreached;/* '<S351>/Min stop reached' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T;

/* Block signals for system '<S472>/Wheel to Body Transform' */
typedef struct {
  real_T TmpSignalConversionAtSinCosInport1[3];/* '<S479>/In1' */
  real_T SinCos_o1[3];                 /* '<S479>/SinCos' */
  real_T SinCos_o2[3];                 /* '<S479>/SinCos' */
  real_T VectorConcatenate[9];         /* '<S480>/Vector Concatenate' */
  real_T Reshape[9];                   /* '<S480>/Reshape' */
  real_T Divide1[3];                   /* '<S478>/Divide1' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T;

/* Block signals for system '<S512>/Magic Tire Const Input' */
typedef struct {
  real_T Fx;                           /* '<S512>/Magic Tire Const Input' */
  real_T Fy;                           /* '<S512>/Magic Tire Const Input' */
  real_T FzTire;                       /* '<S512>/Magic Tire Const Input' */
  real_T Mx;                           /* '<S512>/Magic Tire Const Input' */
  real_T My;                           /* '<S512>/Magic Tire Const Input' */
  real_T Mz;                           /* '<S512>/Magic Tire Const Input' */
  real_T Re;                           /* '<S512>/Magic Tire Const Input' */
  real_T sig_x;                        /* '<S512>/Magic Tire Const Input' */
  real_T sig_y;                        /* '<S512>/Magic Tire Const Input' */
  real_T a;                            /* '<S512>/Magic Tire Const Input' */
  real_T b;                            /* '<S512>/Magic Tire Const Input' */
} B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S526>/detectSlip' */
typedef struct {
  real_T Abs;                          /* '<S538>/Abs' */
  boolean_T RelationalOperator;        /* '<S538>/Relational Operator' */
} B_detectSlip_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S526>/detectLockup' */
typedef struct {
  real_T OutputDamping;                /* '<S535>/Output Damping' */
  real_T Sum2;                         /* '<S535>/Sum2' */
  real_T Sum1;                         /* '<S535>/Sum1' */
  real_T Abs;                          /* '<S532>/Abs' */
  real_T UnaryMinus;                   /* '<S536>/Unary Minus' */
  real_T Abs_h;                        /* '<S537>/Abs' */
  boolean_T RelationalOperator;        /* '<S532>/Relational Operator' */
  boolean_T RelationalOperator_d;      /* '<S537>/Relational Operator' */
  boolean_T UnitDelay;                 /* '<S534>/Unit Delay' */
  boolean_T CombinatorialLogic;        /* '<S534>/Combinatorial  Logic' */
} B_detectLockup_CAVE_MachE_dSPACE_250912_T;

/* Block states (default storage) for system '<S526>/detectLockup' */
typedef struct {
  boolean_T UnitDelay_DSTATE;          /* '<S534>/Unit Delay' */
} DW_detectLockup_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S526>/Slipping' */
typedef struct {
  real_T omegawheel;                   /* '<S528>/omega wheel' */
  real_T u;                            /* '<S528>/-4' */
  real_T TrigonometricFunction;        /* '<S528>/Trigonometric Function' */
  real_T MaxDynamicFrictionTorque1;  /* '<S528>/Max Dynamic Friction Torque1' */
  real_T OutputDamping;                /* '<S528>/Output Damping' */
  real_T OutputSum;                    /* '<S528>/Output Sum' */
  real_T OutputInertia;                /* '<S528>/Output Inertia' */
} B_Slipping_CAVE_MachE_dSPACE_250912_T;

/* Continuous states for system '<S526>/Slipping' */
typedef struct {
  real_T omegaWheel;                   /* '<S528>/omega wheel' */
} X_Slipping_CAVE_MachE_dSPACE_250912_T;

/* State derivatives for system '<S526>/Slipping' */
typedef struct {
  real_T omegaWheel;                   /* '<S528>/omega wheel' */
} XDot_Slipping_CAVE_MachE_dSPACE_250912_T;

/* State Disabled for system '<S526>/Slipping' */
typedef struct {
  boolean_T omegaWheel;                /* '<S528>/omega wheel' */
} XDis_Slipping_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S525>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S525>/Clutch' */
  real_T Tfmaxs;                       /* '<S525>/Clutch' */
  real_T Tout_m;                       /* '<S525>/Clutch' */
  real_T Tfmaxs_m;                     /* '<S525>/Clutch' */
  real_T Omega;                        /* '<S525>/Clutch' */
  real_T Omegadot;                     /* '<S525>/Clutch' */
  real_T ReactionTorque;               /* '<S525>/Clutch' */
  real_T Myb;                          /* '<S525>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S526>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S526>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_T;

/* Block states (default storage) for system '<S525>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S525>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S525>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S525>/Clutch' */
  boolean_T Slipping_entered;          /* '<S525>/Clutch' */
  boolean_T Locked_entered;            /* '<S525>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S526>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_T;

/* Continuous states for system '<S525>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_T;

/* State derivatives for system '<S525>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_T;

/* State Disabled for system '<S525>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S521>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_T sf_Clutch;/* '<S525>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_b_T;

/* Block states (default storage) for system '<S521>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_T sf_Clutch;/* '<S525>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T;

/* Continuous states for system '<S521>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_T sf_Clutch;/* '<S525>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T;

/* State derivatives for system '<S521>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_T sf_Clutch;/* '<S525>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T;

/* State Disabled for system '<S521>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_T sf_Clutch;/* '<S525>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_i_T;

/* Block signals for system '<S553>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S553>/Clutch' */
  real_T Tfmaxs;                       /* '<S553>/Clutch' */
  real_T Tout_k;                       /* '<S553>/Clutch' */
  real_T Tfmaxs_g;                     /* '<S553>/Clutch' */
  real_T Omega;                        /* '<S553>/Clutch' */
  real_T Omegadot;                     /* '<S553>/Clutch' */
  real_T ReactionTorque;               /* '<S553>/Clutch' */
  real_T Myb;                          /* '<S553>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S554>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S554>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S554>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_f_T;

/* Block states (default storage) for system '<S553>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S553>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S553>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S553>/Clutch' */
  boolean_T Slipping_entered;          /* '<S553>/Clutch' */
  boolean_T Locked_entered;            /* '<S553>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S554>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_o_T;

/* Continuous states for system '<S553>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_f_T;

/* State derivatives for system '<S553>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_g_T;

/* State Disabled for system '<S553>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_f_T;

/* Block signals for system '<S549>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_f_T sf_Clutch;/* '<S553>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T;

/* Block states (default storage) for system '<S549>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_o_T sf_Clutch;/* '<S553>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_o_T;

/* Continuous states for system '<S549>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_f_T sf_Clutch;/* '<S553>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_m_T;

/* State derivatives for system '<S549>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_g_T sf_Clutch;/* '<S553>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T;

/* State Disabled for system '<S549>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_f_T sf_Clutch;/* '<S553>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T;

/* Block signals for system '<S581>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S581>/Clutch' */
  real_T Tfmaxs;                       /* '<S581>/Clutch' */
  real_T Tout_m;                       /* '<S581>/Clutch' */
  real_T Tfmaxs_m;                     /* '<S581>/Clutch' */
  real_T Omega;                        /* '<S581>/Clutch' */
  real_T Omegadot;                     /* '<S581>/Clutch' */
  real_T ReactionTorque;               /* '<S581>/Clutch' */
  real_T Myb;                          /* '<S581>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S582>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S582>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S582>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_k_T;

/* Block states (default storage) for system '<S581>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S581>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S581>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S581>/Clutch' */
  boolean_T Slipping_entered;          /* '<S581>/Clutch' */
  boolean_T Locked_entered;            /* '<S581>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S582>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_j_T;

/* Continuous states for system '<S581>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_o_T;

/* State derivatives for system '<S581>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_a_T;

/* State Disabled for system '<S581>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_e_T;

/* Block signals for system '<S577>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_k_T sf_Clutch;/* '<S581>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_k_T;

/* Block states (default storage) for system '<S577>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_j_T sf_Clutch;/* '<S581>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T;

/* Continuous states for system '<S577>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_o_T sf_Clutch;/* '<S581>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_g_T;

/* State derivatives for system '<S577>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_a_T sf_Clutch;/* '<S581>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T;

/* State Disabled for system '<S577>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_e_T sf_Clutch;/* '<S581>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_g_T;

/* Block signals for system '<S609>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S609>/Clutch' */
  real_T Tfmaxs;                       /* '<S609>/Clutch' */
  real_T Tout_j;                       /* '<S609>/Clutch' */
  real_T Tfmaxs_n;                     /* '<S609>/Clutch' */
  real_T Omega;                        /* '<S609>/Clutch' */
  real_T Omegadot;                     /* '<S609>/Clutch' */
  real_T ReactionTorque;               /* '<S609>/Clutch' */
  real_T Myb;                          /* '<S609>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S610>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S610>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S610>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_i_T;

/* Block states (default storage) for system '<S609>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S609>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S609>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S609>/Clutch' */
  boolean_T Slipping_entered;          /* '<S609>/Clutch' */
  boolean_T Locked_entered;            /* '<S609>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S610>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_c_T;

/* Continuous states for system '<S609>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_i_T;

/* State derivatives for system '<S609>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_p_T;

/* State Disabled for system '<S609>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_m_T;

/* Block signals for system '<S605>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_i_T sf_Clutch;/* '<S609>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_f_T;

/* Block states (default storage) for system '<S605>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_c_T sf_Clutch;/* '<S609>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_m_T;

/* Continuous states for system '<S605>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_i_T sf_Clutch;/* '<S609>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_nc_T;

/* State derivatives for system '<S605>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_p_T sf_Clutch;/* '<S609>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_o_T;

/* State Disabled for system '<S605>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_m_T sf_Clutch;/* '<S609>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_js_T;

/* Block signals for system '<S624>/Magic Tire Const Input' */
typedef struct {
  real_T Fx;                           /* '<S624>/Magic Tire Const Input' */
  real_T Fy;                           /* '<S624>/Magic Tire Const Input' */
  real_T FzTire;                       /* '<S624>/Magic Tire Const Input' */
  real_T Mx;                           /* '<S624>/Magic Tire Const Input' */
  real_T My;                           /* '<S624>/Magic Tire Const Input' */
  real_T Mz;                           /* '<S624>/Magic Tire Const Input' */
  real_T Re;                           /* '<S624>/Magic Tire Const Input' */
  real_T Kappa;                        /* '<S624>/Magic Tire Const Input' */
  real_T Alpha;                        /* '<S624>/Magic Tire Const Input' */
  real_T sig_x;                        /* '<S624>/Magic Tire Const Input' */
  real_T sig_y;                        /* '<S624>/Magic Tire Const Input' */
  real_T a;                            /* '<S624>/Magic Tire Const Input' */
  real_T b;                            /* '<S624>/Magic Tire Const Input' */
} B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_c_T;

/* Block signals for system '<S637>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S637>/Clutch' */
  real_T Tfmaxs;                       /* '<S637>/Clutch' */
  real_T Tout_k;                       /* '<S637>/Clutch' */
  real_T Tfmaxs_g;                     /* '<S637>/Clutch' */
  real_T Omega;                        /* '<S637>/Clutch' */
  real_T Omegadot;                     /* '<S637>/Clutch' */
  real_T ReactionTorque;               /* '<S637>/Clutch' */
  real_T Myb;                          /* '<S637>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S638>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S638>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S638>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_o_T;

/* Block states (default storage) for system '<S637>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S637>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S637>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S637>/Clutch' */
  boolean_T Slipping_entered;          /* '<S637>/Clutch' */
  boolean_T Locked_entered;            /* '<S637>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S638>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_h_T;

/* Continuous states for system '<S637>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_on_T;

/* State derivatives for system '<S637>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_gb_T;

/* State Disabled for system '<S637>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_c_T;

/* Block signals for system '<S633>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_o_T sf_Clutch;/* '<S637>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_ey_T;

/* Block states (default storage) for system '<S633>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_h_T sf_Clutch;/* '<S637>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_m1_T;

/* Continuous states for system '<S633>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_on_T sf_Clutch;/* '<S637>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T;

/* State derivatives for system '<S633>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_gb_T sf_Clutch;/* '<S637>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_m_T;

/* State Disabled for system '<S633>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_c_T sf_Clutch;/* '<S637>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T;

/* Block signals for system '<S665>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S665>/Clutch' */
  real_T Tfmaxs;                       /* '<S665>/Clutch' */
  real_T Tout_b;                       /* '<S665>/Clutch' */
  real_T Tfmaxs_c;                     /* '<S665>/Clutch' */
  real_T Omega;                        /* '<S665>/Clutch' */
  real_T Omegadot;                     /* '<S665>/Clutch' */
  real_T ReactionTorque;               /* '<S665>/Clutch' */
  real_T Myb;                          /* '<S665>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S666>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S666>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S666>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_g_T;

/* Block states (default storage) for system '<S665>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S665>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S665>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S665>/Clutch' */
  boolean_T Slipping_entered;          /* '<S665>/Clutch' */
  boolean_T Locked_entered;            /* '<S665>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S666>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_l_T;

/* Continuous states for system '<S665>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_e_T;

/* State derivatives for system '<S665>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_c_T;

/* State Disabled for system '<S665>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_g_T;

/* Block signals for system '<S661>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_g_T sf_Clutch;/* '<S665>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_ha_T;

/* Block states (default storage) for system '<S661>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_l_T sf_Clutch;/* '<S665>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T;

/* Continuous states for system '<S661>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_e_T sf_Clutch;/* '<S665>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_o_T;

/* State derivatives for system '<S661>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_c_T sf_Clutch;/* '<S665>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T;

/* State Disabled for system '<S661>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_g_T sf_Clutch;/* '<S665>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_p_T;

/* Block signals for system '<S693>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S693>/Clutch' */
  real_T Tfmaxs;                       /* '<S693>/Clutch' */
  real_T Tout_c;                       /* '<S693>/Clutch' */
  real_T Tfmaxs_f;                     /* '<S693>/Clutch' */
  real_T Omega;                        /* '<S693>/Clutch' */
  real_T Omegadot;                     /* '<S693>/Clutch' */
  real_T ReactionTorque;               /* '<S693>/Clutch' */
  real_T Myb;                          /* '<S693>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S694>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S694>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S694>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_o3_T;

/* Block states (default storage) for system '<S693>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S693>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S693>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S693>/Clutch' */
  boolean_T Slipping_entered;          /* '<S693>/Clutch' */
  boolean_T Locked_entered;            /* '<S693>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S694>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_k_T;

/* Continuous states for system '<S693>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_m_T;

/* State derivatives for system '<S693>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_ga_T;

/* State Disabled for system '<S693>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_k_T;

/* Block signals for system '<S689>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_o3_T sf_Clutch;/* '<S693>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_g_T;

/* Block states (default storage) for system '<S689>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_k_T sf_Clutch;/* '<S693>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T;

/* Continuous states for system '<S689>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_m_T sf_Clutch;/* '<S693>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_i_T;

/* State derivatives for system '<S689>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_ga_T sf_Clutch;/* '<S693>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_p1_T;

/* State Disabled for system '<S689>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_k_T sf_Clutch;/* '<S693>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_nd_T;

/* Block signals for system '<S721>/Clutch' */
typedef struct {
  real_T Tout;                         /* '<S721>/Clutch' */
  real_T Tfmaxs;                       /* '<S721>/Clutch' */
  real_T Tout_i;                       /* '<S721>/Clutch' */
  real_T Tfmaxs_j;                     /* '<S721>/Clutch' */
  real_T Omega;                        /* '<S721>/Clutch' */
  real_T Omegadot;                     /* '<S721>/Clutch' */
  real_T ReactionTorque;               /* '<S721>/Clutch' */
  real_T Myb;                          /* '<S721>/Clutch' */
  B_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S722>/Slipping' */
  B_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S722>/detectLockup' */
  B_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S722>/detectSlip' */
} B_Clutch_CAVE_MachE_dSPACE_250912_oa_T;

/* Block states (default storage) for system '<S721>/Clutch' */
typedef struct {
  real_T lastMajorTime;                /* '<S721>/Clutch' */
  int8_T TmpIfAtSlippingInport3_ActiveSubsystem;/* synthesized block */
  int8_T TmpIfAtLockedInport2_ActiveSubsystem;/* synthesized block */
  uint8_T is_active_c8_autolibshared;  /* '<S721>/Clutch' */
  uint8_T is_c8_autolibshared;         /* '<S721>/Clutch' */
  boolean_T Slipping_entered;          /* '<S721>/Clutch' */
  boolean_T Locked_entered;            /* '<S721>/Clutch' */
  DW_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S722>/detectLockup' */
} DW_Clutch_CAVE_MachE_dSPACE_250912_or_T;

/* Continuous states for system '<S721>/Clutch' */
typedef struct {
  X_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} X_Clutch_CAVE_MachE_dSPACE_250912_l_T;

/* State derivatives for system '<S721>/Clutch' */
typedef struct {
  XDot_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDot_Clutch_CAVE_MachE_dSPACE_250912_i_T;

/* State Disabled for system '<S721>/Clutch' */
typedef struct {
  XDis_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
} XDis_Clutch_CAVE_MachE_dSPACE_250912_e0_T;

/* Block signals for system '<S717>/Clutch Scalar Parameters' */
typedef struct {
  B_Clutch_CAVE_MachE_dSPACE_250912_oa_T sf_Clutch;/* '<S721>/Clutch' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_b1_T;

/* Block states (default storage) for system '<S717>/Clutch Scalar Parameters' */
typedef struct {
  DW_Clutch_CAVE_MachE_dSPACE_250912_or_T sf_Clutch;/* '<S721>/Clutch' */
} DW_CoreSubsys_CAVE_MachE_dSPACE_250912_nf_T;

/* Continuous states for system '<S717>/Clutch Scalar Parameters' */
typedef struct {
  X_Clutch_CAVE_MachE_dSPACE_250912_l_T sf_Clutch;/* '<S721>/Clutch' */
} X_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T;

/* State derivatives for system '<S717>/Clutch Scalar Parameters' */
typedef struct {
  XDot_Clutch_CAVE_MachE_dSPACE_250912_i_T sf_Clutch;/* '<S721>/Clutch' */
} XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T;

/* State Disabled for system '<S717>/Clutch Scalar Parameters' */
typedef struct {
  XDis_Clutch_CAVE_MachE_dSPACE_250912_e0_T sf_Clutch;/* '<S721>/Clutch' */
} XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_pq_T;

/* Block signals for system '<S481>/MATLAB Function' */
typedef struct {
  real_T switchFlag;                   /* '<S481>/MATLAB Function' */
} B_MATLABFunction_CAVE_MachE_dSPACE_250912_T;

/* Block signals for system '<S474>/Wheel to Body Transform' */
typedef struct {
  real_T TmpSignalConversionAtSinCosInport1[3];/* '<S737>/In1' */
  real_T SinCos_o1[3];                 /* '<S737>/SinCos' */
  real_T SinCos_o2[3];                 /* '<S737>/SinCos' */
  real_T VectorConcatenate[9];         /* '<S738>/Vector Concatenate' */
  real_T Reshape[9];                   /* '<S738>/Reshape' */
  real_T Divide1[3];                   /* '<S736>/Divide1' */
} B_CoreSubsys_CAVE_MachE_dSPACE_250912_ca_T;

/* Block signals (default storage) */
typedef struct {
  VehDataBus BusConversion_InsertedFor_RealSimPack_at_inport_2_BusCreator1;
  VehDataBus VehDataBusIn;             /* '<S4>/RealSimDepack' */
  int64_T Gain;                        /* '<S117>/Gain' */
  int64_T BrakePedal_CmdFinal_pct;     /* '<S114>/Gain1' */
  real_T PowertrainData_10_HS1Time;    /* '<S2>/Data Inport S-Fcn' */
  real_T PowertrainData_10_HS1DeltaTime;/* '<S2>/Data Inport S-Fcn' */
  real_T WheelSpeed_HS1Time;           /* '<S2>/Data Inport S-Fcn' */
  real_T WheelSpeed_HS1DeltaTime;      /* '<S2>/Data Inport S-Fcn' */
  real_T WhlRr_W_MeasValue;            /* '<S2>/Data Inport S-Fcn' */
  real_T WhlRl_W_MeasValue;            /* '<S2>/Data Inport S-Fcn' */
  real_T WhlFr_W_MeasValue;            /* '<S2>/Data Inport S-Fcn' */
  real_T WhlFl_W_MeasValue;            /* '<S2>/Data Inport S-Fcn' */
  real_T BrakeSysFeatures_HS1Time;     /* '<S2>/Data Inport S-Fcn' */
  real_T BrakeSysFeatures_HS1DeltaTime;/* '<S2>/Data Inport S-Fcn' */
  real_T Veh_V_ActlBrkValue;           /* '<S2>/Data Inport S-Fcn' */
  real_T EngVehicleSpThrottle_HS1Time; /* '<S2>/Data Inport S-Fcn' */
  real_T EngVehicleSpThrottle_HS1DeltaTime;/* '<S2>/Data Inport S-Fcn' */
  real_T EngAout3_N_ActlValue;         /* '<S2>/Data Inport S-Fcn' */
  real_T ApedPos_Pc_ActlArbValue;      /* '<S2>/Data Inport S-Fcn' */
  real_T BrakeSnData_4_HS1Time;        /* '<S2>/Data Inport S-Fcn' */
  real_T BrakeSnData_4_HS1DeltaTime;   /* '<S2>/Data Inport S-Fcn' */
  real_T BattTrac_U2_ActlValue;        /* '<S2>/Data Inport S-Fcn' */
  real_T BattTrac_I2_ActlValue;        /* '<S2>/Data Inport S-Fcn' */
  real_T Brake;                        /* '<Root>/Brake' */
  real_T VectorConcatenate[4];         /* '<S362>/Vector Concatenate' */
  real_T Integrator[4];                /* '<S467>/Integrator' */
  real_T Memory1;                      /* '<S206>/Memory1' */
  real_T Abs;                          /* '<S216>/Abs' */
  real_T Divide;                       /* '<S216>/Divide' */
  real_T Integrator_j;                 /* '<S216>/Integrator' */
  real_T Gain1;                        /* '<S216>/Gain1' */
  real_T Merge;                        /* '<S216>/Merge' */
  real_T Switch;                       /* '<S144>/Switch' */
  real_T IntegratorLimited;            /* '<S144>/Integrator Limited' */
  real_T Divide_h;                     /* '<S145>/Divide' */
  real_T Curr;                         /* '<S111>/Memory' */
  real_T Em;                           /* '<S146>/Em' */
  real_T R;                            /* '<S146>/R' */
  real_T Gain2;                        /* '<S146>/Gain2' */
  real_T Product;                      /* '<S146>/Product' */
  real_T Subtract;                     /* '<S146>/Subtract' */
  real_T Gain1_p;                      /* '<S146>/Gain1' */
  real_T BattPwr;                      /* '<S3>/Product' */
  real_T Integrator_jl;                /* '<S100>/Integrator' */
  real_T mtomile;                      /* '<S100>/m to mile' */
  real_T Integrator1;                  /* '<S100>/Integrator1' */
  real_T m3toUSGal;                    /* '<S100>/m^3 to US Gal' */
  real_T Saturation1;                  /* '<S100>/Saturation1' */
  real_T USMPGCalc;                    /* '<S100>/US MPG Calc' */
  real_T VectorConcatenate_a[7];       /* '<S3>/Vector Concatenate' */
  real_T Gain_i;                       /* '<S101>/Gain' */
  real_T ConnectionState;              /* '<S9>/Data Type Conversion4' */
  real_T speedDesired;                 /* '<S4>/Bus Selector' */
  real_T Clock;                        /* '<S4>/Clock' */
  real_T Memory;                       /* '<S4>/Memory' */
  real_T Memory_e;                     /* '<Root>/Memory' */
  real_T speedActual;                  /* '<S4>/Multiport Switch' */
  real_T ManualSwitch2;                /* '<S4>/Manual Switch2' */
  real_T Add;                          /* '<S12>/Add' */
  real_T Switch1;                      /* '<S12>/Switch1' */
  real_T Clock_n;                      /* '<S1>/Clock' */
  real_T ManualSwitch;                 /* '<S1>/Manual Switch' */
  real_T ManualSwitch1;                /* '<S1>/Manual Switch1' */
  real_T Clock1;                       /* '<Root>/Clock1' */
  real_T US1;                          /* '<Root>/US1' */
  real_T Gain2_n;                      /* '<Root>/Gain2' */
  real_T VehSpdRef;                    /* '<Root>/Manual Switch' */
  real_T Gain_d;                       /* '<S107>/Gain' */
  real_T MultiportSwitch1;             /* '<S107>/Multiport Switch1' */
  real_T AccelPedalCmd_pct;            /* '<S107>/AR_limit' */
  real_T GasDriver;                    /* '<Root>/Gain' */
  real_T Gain1_n;                      /* '<S107>/Gain1' */
  real_T MultiportSwitch2;             /* '<S107>/Multiport Switch2' */
  real_T BrakePedalCmd_pct;            /* '<S107>/BR_limit' */
  real_T BrakeDriver;                  /* '<Root>/Gain1' */
  real_T VectorConcatenate4[2];        /* '<S462>/Vector Concatenate4' */
  real_T VectorConcatenate4_g[2];      /* '<S465>/Vector Concatenate4' */
  real_T VectorConcatenate2[2];        /* '<S460>/Vector Concatenate2' */
  real_T VectorConcatenate3[2];        /* '<S460>/Vector Concatenate3' */
  real_T VectorConcatenate4_f[4];      /* '<S460>/Vector Concatenate4' */
  real_T Memory_i;                     /* '<S206>/Memory' */
  real_T Gain3;                        /* '<S206>/Gain3' */
  real_T T_RR;                         /* '<S206>/Gain4' */
  real_T VectorConcatenate_i[4];       /* '<S206>/Vector Concatenate' */
  real_T Reshape4[4];                  /* '<S474>/Reshape4' */
  real_T Gain4[4];                     /* '<S474>/Gain4' */
  real_T Integrator_b;                 /* '<S518>/Integrator' */
  real_T Integrator_o;                 /* '<S515>/Integrator' */
  real_T VectorConcatenate_h[4];       /* '<S259>/Vector Concatenate' */
  real_T IntegratorSecondOrder_o1;     /* '<S519>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2;     /* '<S519>/Integrator, Second-Order' */
  real_T Sum6;                         /* '<S519>/Sum6' */
  real_T Saturation;                   /* '<S519>/Saturation' */
  real_T Add2;                         /* '<S514>/Add2' */
  real_T Saturation_a;                 /* '<S514>/Saturation' */
  real_T Product3;                     /* '<S514>/Product3' */
  real_T Add1;                         /* '<S514>/Add1' */
  real_T Reshape;                      /* '<S514>/Reshape' */
  real_T DecelCmd;                     /* '<S3>/Switch1' */
  real_T Saturation_i;                 /* '<S242>/Saturation' */
  real_T BrkTrqReqTotal;               /* '<S242>/Gain1' */
  real_T Saturation1_o;                /* '<S258>/Saturation1' */
  real_T Divide_p;                     /* '<S258>/Divide' */
  real_T Saturation_j;                 /* '<S258>/Saturation' */
  real_T MotTrqMaxWhls;                /* '<S242>/MotTrqReflectedToWheels' */
  real_T min;                          /* '<S242>/MinMax' */
  real_T RegenBrakingCutoff;           /* '<S242>/RegenBrakingCutoff' */
  real_T ChrgLmt;                      /* '<S242>/ChrgLmt' */
  real_T RegenFactor;                  /* '<S242>/Product1' */
  real_T MotTrqRegenWhl;               /* '<S242>/Product3' */
  real_T Subtract_k;                   /* '<S242>/Subtract' */
  real_T BrkTrqReqTotal_h;             /* '<S242>/Gain2' */
  real_T Saturation1_d;                /* '<S242>/Saturation1' */
  real_T Gain_n;                       /* '<S260>/Gain' */
  real_T Gain2_i;                      /* '<S260>/Gain2' */
  real_T Gain3_j;                      /* '<S260>/Gain3' */
  real_T Gain1_b;                      /* '<S260>/Gain1' */
  real_T Gain4_a;                      /* '<S260>/Gain4' */
  real_T VectorConcatenate2_n[4];      /* '<S260>/Vector Concatenate2' */
  real_T Reshape3[4];                  /* '<S474>/Reshape3' */
  real_T TorqueConversion1;            /* '<S524>/Torque Conversion1' */
  real_T product;                      /* '<S524>/product' */
  real_T DisallowNegativeBrakeTorque;
                                   /* '<S524>/Disallow Negative Brake Torque' */
  real_T TorqueConversion;             /* '<S524>/Torque Conversion' */
  real_T Ratioofstatictokinetic;       /* '<S522>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1;     /* '<S522>/Ratio of static to kinetic1' */
  real_T Switch_o;                     /* '<S481>/Switch' */
  real_T Integrator_i;                 /* '<S546>/Integrator' */
  real_T Integrator_p;                 /* '<S543>/Integrator' */
  real_T IntegratorSecondOrder_o1_j;   /* '<S547>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_i;   /* '<S547>/Integrator, Second-Order' */
  real_T Sum6_f;                       /* '<S547>/Sum6' */
  real_T Saturation_p;                 /* '<S547>/Saturation' */
  real_T Add2_d;                       /* '<S542>/Add2' */
  real_T Saturation_c;                 /* '<S542>/Saturation' */
  real_T Product3_b;                   /* '<S542>/Product3' */
  real_T Add1_m;                       /* '<S542>/Add1' */
  real_T Reshape_c;                    /* '<S542>/Reshape' */
  real_T TorqueConversion1_a;          /* '<S552>/Torque Conversion1' */
  real_T product_f;                    /* '<S552>/product' */
  real_T DisallowNegativeBrakeTorque_e;
                                   /* '<S552>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_a;           /* '<S552>/Torque Conversion' */
  real_T Ratioofstatictokinetic_k;     /* '<S550>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1_k;   /* '<S550>/Ratio of static to kinetic1' */
  real_T Switch1_h;                    /* '<S481>/Switch1' */
  real_T Integrator_g;                 /* '<S574>/Integrator' */
  real_T Integrator_ob;                /* '<S571>/Integrator' */
  real_T IntegratorSecondOrder_o1_a;   /* '<S575>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_j;   /* '<S575>/Integrator, Second-Order' */
  real_T Sum6_m;                       /* '<S575>/Sum6' */
  real_T Saturation_f;                 /* '<S575>/Saturation' */
  real_T Add2_p;                       /* '<S570>/Add2' */
  real_T Saturation_b;                 /* '<S570>/Saturation' */
  real_T Product3_n;                   /* '<S570>/Product3' */
  real_T Add1_d;                       /* '<S570>/Add1' */
  real_T Reshape_k;                    /* '<S570>/Reshape' */
  real_T TorqueConversion1_i;          /* '<S580>/Torque Conversion1' */
  real_T product_h;                    /* '<S580>/product' */
  real_T DisallowNegativeBrakeTorque_k;
                                   /* '<S580>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_e;           /* '<S580>/Torque Conversion' */
  real_T Ratioofstatictokinetic_o;     /* '<S578>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1_j;   /* '<S578>/Ratio of static to kinetic1' */
  real_T Switch2;                      /* '<S481>/Switch2' */
  real_T Integrator_oo;                /* '<S602>/Integrator' */
  real_T Integrator_oq;                /* '<S599>/Integrator' */
  real_T IntegratorSecondOrder_o1_jo;  /* '<S603>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_g;   /* '<S603>/Integrator, Second-Order' */
  real_T Sum6_n;                       /* '<S603>/Sum6' */
  real_T Saturation_je;                /* '<S603>/Saturation' */
  real_T Add2_m;                       /* '<S598>/Add2' */
  real_T Saturation_n;                 /* '<S598>/Saturation' */
  real_T Product3_bv;                  /* '<S598>/Product3' */
  real_T Add1_h;                       /* '<S598>/Add1' */
  real_T Reshape_o;                    /* '<S598>/Reshape' */
  real_T TorqueConversion1_ig;         /* '<S608>/Torque Conversion1' */
  real_T product_a;                    /* '<S608>/product' */
  real_T DisallowNegativeBrakeTorque_o;
                                   /* '<S608>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_p;           /* '<S608>/Torque Conversion' */
  real_T Ratioofstatictokinetic_d;     /* '<S606>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1_h;   /* '<S606>/Ratio of static to kinetic1' */
  real_T Switch3;                      /* '<S481>/Switch3' */
  real_T Fx[4];                        /* '<S481>/Vector Concatenate' */
  real_T Integrator_if;                /* '<S516>/Integrator' */
  real_T Integrator_h;                 /* '<S544>/Integrator' */
  real_T Integrator_f;                 /* '<S572>/Integrator' */
  real_T Integrator_f1;                /* '<S600>/Integrator' */
  real_T VectorConcatenate_hs[4];      /* '<S491>/Vector Concatenate' */
  real_T Integrator1_a[4];             /* '<S469>/Integrator1' */
  real_T Saturation_nw[4];             /* '<S267>/Saturation' */
  real_T VectorConcatenate8[12];       /* '<S472>/Vector Concatenate8' */
  real_T MathFunction[12];             /* '<S472>/Math Function' */
  real_T Integrator1_i[12];            /* '<S470>/Integrator1' */
  real_T CamberAngles[4];              /* '<S267>/Selector3' */
  real_T CamberAngles_p[4];            /* '<S267>/Manual Switch6' */
  real_T Add2_mc[4];                   /* '<S477>/Add2' */
  real_T Reshape1[4];                  /* '<S477>/Reshape1' */
  real_T Reshape2[4];                  /* '<S477>/Reshape2' */
  real_T WheelAngles[4];               /* '<S267>/Selector2' */
  real_T Add1_o[4];                    /* '<S477>/Add1' */
  real_T Reshape_a[4];                 /* '<S477>/Reshape' */
  real_T VectorConcatenate3_m[12];     /* '<S477>/Vector Concatenate3' */
  real_T Reshape6[4];                  /* '<S278>/Reshape6' */
  real_T Reshape3_e[2];                /* '<S279>/Reshape3' */
  real_T Reshape7[4];                  /* '<S278>/Reshape7' */
  real_T Reshape4_f[2];                /* '<S279>/Reshape4' */
  real_T MatrixConcatenate4[4];        /* '<S279>/Matrix Concatenate4' */
  real_T VectorConcatenate_e[4];       /* '<S509>/Vector Concatenate' */
  real_T z[4];                         /* '<S476>/Unary Minus1' */
  real_T z_e[4];                       /* '<S278>/Reshape' */
  real_T Reshape9[2];                  /* '<S279>/Reshape9' */
  real_T VectorConcatenate_f[4];       /* '<S510>/Vector Concatenate' */
  real_T zdot[4];                      /* '<S476>/Unary Minus2' */
  real_T zdot_k[4];                    /* '<S278>/Reshape2' */
  real_T Reshape8[2];                  /* '<S279>/Reshape8' */
  real_T MatrixConcatenate2[4];        /* '<S279>/Matrix Concatenate2' */
  real_T VectorConcatenate3_i[2];      /* '<S362>/Vector Concatenate3' */
  real_T Integrator_a[2];              /* '<S384>/Integrator' */
  real_T Sign2;                        /* '<S454>/Sign2' */
  real_T Gain_i4;                      /* '<S454>/Gain' */
  real_T Abs2;                         /* '<S454>/Abs2' */
  real_T MathFunction1;                /* '<S454>/Math Function1' */
  real_T Product1;                     /* '<S454>/Product1' */
  real_T Add1_e;                       /* '<S454>/Add1' */
  real_T Divide1;                      /* '<S454>/Divide1' */
  real_T N;                            /* '<S454>/Floor1' */
  real_T MathFunction3;                /* '<S454>/Math Function3' */
  real_T Switch_c;                     /* '<S454>/Switch' */
  real_T TmpSignalConversionAtsincosInport1[3];/* '<S392>/In1' */
  real_T sincos_o1[3];                 /* '<S392>/sincos' */
  real_T sincos_o2[3];                 /* '<S392>/sincos' */
  real_T VectorConcatenate_c[9];       /* '<S396>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor[9];
                                /* '<S396>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1[9];                /* '<S391>/Transpose1' */
  real_T Gain_ig;                      /* '<S385>/Gain' */
  real_T VectorConcatenate_j[3];       /* '<S385>/Vector Concatenate' */
  real_T Reshape1_c[3];                /* '<S394>/Reshape1' */
  real_T Product_i[3];                 /* '<S394>/Product' */
  real_T Reshape2_n[3];                /* '<S394>/Reshape2' */
  real_T Add_f[3];                     /* '<S391>/Add' */
  real_T sincos_o1_l[3];               /* '<S400>/sincos' */
  real_T sincos_o2_c[3];               /* '<S400>/sincos' */
  real_T VectorConcatenate_et[9];      /* '<S404>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_o[9];
                                /* '<S404>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_b[9];              /* '<S399>/Transpose1' */
  real_T Gain_c;                       /* '<S386>/Gain' */
  real_T VectorConcatenate_l[3];       /* '<S386>/Vector Concatenate' */
  real_T Reshape1_f[3];                /* '<S402>/Reshape1' */
  real_T Product_a[3];                 /* '<S402>/Product' */
  real_T Reshape2_g[3];                /* '<S402>/Reshape2' */
  real_T Add_h[3];                     /* '<S399>/Add' */
  real_T sincos_o1_c[3];               /* '<S420>/sincos' */
  real_T sincos_o2_p[3];               /* '<S420>/sincos' */
  real_T VectorConcatenate_p[9];       /* '<S424>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_b[9];
                                /* '<S424>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_e[9];              /* '<S419>/Transpose1' */
  real_T Gain_n5;                      /* '<S388>/Gain' */
  real_T VectorConcatenate_ic[3];      /* '<S388>/Vector Concatenate' */
  real_T Reshape1_e[3];                /* '<S422>/Reshape1' */
  real_T Product_g[3];                 /* '<S422>/Product' */
  real_T Reshape2_a[3];                /* '<S422>/Reshape2' */
  real_T Add_e[3];                     /* '<S419>/Add' */
  real_T sincos_o1_f[3];               /* '<S428>/sincos' */
  real_T sincos_o2_k[3];               /* '<S428>/sincos' */
  real_T VectorConcatenate_pt[9];      /* '<S432>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_f[9];
                                /* '<S432>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_g[9];              /* '<S427>/Transpose1' */
  real_T Gain_j;                       /* '<S389>/Gain' */
  real_T VectorConcatenate_b[3];       /* '<S389>/Vector Concatenate' */
  real_T Reshape1_m[3];                /* '<S430>/Reshape1' */
  real_T Product_k[3];                 /* '<S430>/Product' */
  real_T Reshape2_c[3];                /* '<S430>/Reshape2' */
  real_T Add_p[3];                     /* '<S427>/Add' */
  real_T Reshape_p[12];                /* '<S277>/Reshape' */
  real_T P[12];                        /* '<S277>/Sum' */
  real_T Selector10[6];                /* '<S273>/Selector10' */
  real_T Selector[2];                  /* '<S279>/Selector' */
  real_T Reshape7_c[2];                /* '<S279>/Reshape7' */
  real_T jxk;                          /* '<S397>/j x k' */
  real_T kxi;                          /* '<S397>/k x i' */
  real_T ixj;                          /* '<S397>/i x j' */
  real_T kxj;                          /* '<S398>/k x j' */
  real_T ixk;                          /* '<S398>/i x k' */
  real_T jxi;                          /* '<S398>/j x i' */
  real_T Sum[3];                       /* '<S395>/Sum' */
  real_T Add1_g[3];                    /* '<S391>/Add1' */
  real_T jxk_c;                        /* '<S405>/j x k' */
  real_T kxi_d;                        /* '<S405>/k x i' */
  real_T ixj_f;                        /* '<S405>/i x j' */
  real_T kxj_b;                        /* '<S406>/k x j' */
  real_T ixk_c;                        /* '<S406>/i x k' */
  real_T jxi_n;                        /* '<S406>/j x i' */
  real_T Sum_e[3];                     /* '<S403>/Sum' */
  real_T Add1_l[3];                    /* '<S399>/Add1' */
  real_T jxk_p;                        /* '<S425>/j x k' */
  real_T kxi_dk;                       /* '<S425>/k x i' */
  real_T ixj_k;                        /* '<S425>/i x j' */
  real_T kxj_h;                        /* '<S426>/k x j' */
  real_T ixk_l;                        /* '<S426>/i x k' */
  real_T jxi_a;                        /* '<S426>/j x i' */
  real_T Sum_h[3];                     /* '<S423>/Sum' */
  real_T Add1_b[3];                    /* '<S419>/Add1' */
  real_T jxk_d;                        /* '<S433>/j x k' */
  real_T kxi_m;                        /* '<S433>/k x i' */
  real_T ixj_h;                        /* '<S433>/i x j' */
  real_T kxj_i;                        /* '<S434>/k x j' */
  real_T ixk_m;                        /* '<S434>/i x k' */
  real_T jxi_g;                        /* '<S434>/j x i' */
  real_T Sum_j[3];                     /* '<S431>/Sum' */
  real_T Add1_mg[3];                   /* '<S427>/Add1' */
  real_T V[12];                        /* '<S277>/Reshape2' */
  real_T Selector11[6];                /* '<S273>/Selector11' */
  real_T Selector1[2];                 /* '<S279>/Selector1' */
  real_T Reshape6_e[2];                /* '<S279>/Reshape6' */
  real_T MatrixConcatenate3[4];        /* '<S279>/Matrix Concatenate3' */
  real_T Switch2_p;                    /* '<S3>/Switch2' */
  real_T SteeringCmd;                  /* '<S3>/EV Bolt Steer' */
  real_T Backlash;                     /* '<S269>/Backlash' */
  real_T Saturation_jp;                /* '<S269>/Saturation' */
  real_T Gain_h;                       /* '<S270>/Gain' */
  real_T UnaryMinus1;                  /* '<S268>/Unary Minus1' */
  real_T UnaryMinus;                   /* '<S268>/Unary Minus' */
  real_T VectorConcatenate2_b[4];      /* '<S261>/Vector Concatenate2' */
  real_T Reshape1_c4[2];               /* '<S276>/Reshape1' */
  real_T Selector4[2];                 /* '<S279>/Selector4' */
  real_T Reshape15[2];                 /* '<S279>/Reshape15' */
  real_T Selector5[2];                 /* '<S279>/Selector5' */
  real_T Reshape13[2];                 /* '<S279>/Reshape13' */
  real_T MatrixConcatenate6[6];        /* '<S279>/Matrix Concatenate6' */
  real_T Selector18[6];                /* '<S273>/Selector18' */
  real_T Selector7[2];                 /* '<S274>/Selector7' */
  real_T Selector8[2];                 /* '<S274>/Selector8' */
  real_T Reshape15_c[2];               /* '<S274>/Reshape15' */
  real_T MatrixConcatenate5[6];        /* '<S274>/Matrix Concatenate5' */
  real_T Reshape1_o;                   /* '<S280>/Reshape1' */
  real_T Memory1_l[2];                 /* '<S280>/Memory1' */
  real_T Reshape3_j[2];                /* '<S274>/Reshape3' */
  real_T Switch_i[2];                  /* '<S280>/Switch' */
  real_T Reshape21[2];                 /* '<S280>/Reshape21' */
  real_T SumofElements;                /* '<S280>/Sum of Elements' */
  real_T MatrixConcatenate[3];         /* '<S280>/Matrix Concatenate' */
  real_T Reshape8_m[3];                /* '<S274>/Reshape8' */
  real_T Sum_d[3];                     /* '<S280>/Sum' */
  real_T Reshape9_d[3];                /* '<S274>/Reshape9' */
  real_T Reshape5[6];                  /* '<S274>/Reshape5' */
  real_T Sum2[6];                      /* '<S274>/Sum2' */
  real_T xdot[4];                      /* '<S273>/Matrix Concatenate1' */
  real_T ydot[4];                      /* '<S273>/Matrix Concatenate' */
  real_T Reshape1_h[4];                /* '<S476>/Reshape1' */
  real_T MatrixConcatenate_c[12];      /* '<S267>/Matrix Concatenate' */
  real_T Add2_l[4];                    /* '<S735>/Add2' */
  real_T Reshape1_oh[4];               /* '<S735>/Reshape1' */
  real_T Reshape2_p[4];                /* '<S735>/Reshape2' */
  real_T Add1_i[4];                    /* '<S735>/Add1' */
  real_T Reshape_g[4];                 /* '<S735>/Reshape' */
  real_T VectorConcatenate3_d[12];     /* '<S735>/Vector Concatenate3' */
  real_T Reshape2_cw[4];               /* '<S275>/Reshape2' */
  real_T MatrixConcatenate1[12];       /* '<S275>/Matrix Concatenate1' */
  real_T Reshape3_g[4];                /* '<S275>/Reshape3' */
  real_T Reshape4_b[4];                /* '<S275>/Reshape4' */
  real_T MatrixConcatenate_k[12];      /* '<S275>/Matrix Concatenate' */
  real_T AngVel[12];                   /* '<S275>/Add' */
  real_T Selector1_d[4];               /* '<S267>/Selector1' */
  real_T Reshape1_g[4];                /* '<S267>/Reshape1' */
  real_T UnaryMinus_p[4];              /* '<S474>/Unary Minus' */
  real_T VectorConcatenate1[4];        /* '<S259>/Vector Concatenate1' */
  real_T Reshape_g1[4];                /* '<S475>/Reshape' */
  real_T Reshape1_l[4];                /* '<S475>/Reshape1' */
  real_T VectorConcatenate_m[108];     /* '<S475>/Vector Concatenate' */
  real_T Selector9[27];                /* '<S481>/Selector9' */
  real_T Product_n;                    /* '<S519>/Product' */
  real_T Selector19[27];               /* '<S481>/Selector19' */
  real_T Product_n2;                   /* '<S547>/Product' */
  real_T Selector29[27];               /* '<S481>/Selector29' */
  real_T Product_ie;                   /* '<S575>/Product' */
  real_T Selector39[27];               /* '<S481>/Selector39' */
  real_T Product_f;                    /* '<S603>/Product' */
  real_T VectorConcatenate_hm[4];      /* '<S500>/Vector Concatenate' */
  real_T Re[4];                        /* '<S278>/Reshape1' */
  real_T Reshape17[2];                 /* '<S279>/Reshape17' */
  real_T Reshape19[2];                 /* '<S279>/Reshape19' */
  real_T AssignVehFz[2];               /* '<S324>/Assign VehFz' */
  real_T Reshape1_p[2];                /* '<S279>/Reshape1' */
  real_T MatrixConcatenate_a[6];       /* '<S279>/Matrix Concatenate' */
  real_T Reshape10[2];                 /* '<S274>/Reshape10' */
  real_T Reshape11[2];                 /* '<S274>/Reshape11' */
  real_T MatrixConcatenate2_c[4];      /* '<S274>/Matrix Concatenate2' */
  real_T Reshape7_d[6];                /* '<S274>/Reshape7' */
  real_T Selector_d[2];                /* '<S274>/Selector' */
  real_T Reshape6_m[6];                /* '<S274>/Reshape6' */
  real_T Selector1_g[2];               /* '<S274>/Selector1' */
  real_T MatrixConcatenate1_p[4];      /* '<S274>/Matrix Concatenate1' */
  real_T Selector1_dh[6];              /* '<S273>/Selector1' */
  real_T Selector3[2];                 /* '<S274>/Selector3' */
  real_T Reshape12[2];                 /* '<S274>/Reshape12' */
  real_T Selector4_p[2];               /* '<S274>/Selector4' */
  real_T Reshape13_e[2];               /* '<S274>/Reshape13' */
  real_T MatrixConcatenate3_f[4];      /* '<S274>/Matrix Concatenate3' */
  real_T Reshape4_a[2];                /* '<S274>/Reshape4' */
  real_T Reshape1_lq[2];               /* '<S274>/Reshape1' */
  real_T MatrixConcatenate_p[6];       /* '<S274>/Matrix Concatenate' */
  real_T F[12];                        /* '<S273>/Matrix Concatenate6' */
  real_T Selector_g[4];                /* '<S361>/Selector' */
  real_T PermuteDimensions[4];         /* '<S361>/Permute Dimensions' */
  real_T UnitConversion3[4];           /* '<S461>/Unit Conversion3' */
  real_T Selector1_l[4];               /* '<S361>/Selector1' */
  real_T PermuteDimensions1[4];        /* '<S361>/Permute Dimensions1' */
  real_T UnitConversion3_a[4];         /* '<S464>/Unit Conversion3' */
  real_T VectorConcatenate1_g[3];      /* '<S364>/Vector Concatenate1' */
  real_T TrigonometricFunction_o1;     /* '<S378>/Trigonometric Function' */
  real_T TrigonometricFunction_o2;     /* '<S378>/Trigonometric Function' */
  real_T VectorConcatenate_g[3];       /* '<S103>/Vector Concatenate' */
  real_T UnitConversion5[3];           /* '<S468>/Unit Conversion5' */
  real_T Product3_a;                   /* '<S378>/Product3' */
  real_T Product1_h;                   /* '<S378>/Product1' */
  real_T Product_ic;                   /* '<S378>/Product' */
  real_T Product2;                     /* '<S378>/Product2' */
  real_T VectorConcatenate2_g[3];      /* '<S378>/Vector Concatenate2' */
  real_T Add1_gq[3];                   /* '<S377>/Add1' */
  real_T u[3];                         /* '<S377>/4' */
  real_T Tanh[3];                      /* '<S377>/Tanh' */
  real_T Add2_k[3];                    /* '<S377>/Add2' */
  real_T Product_nm[3];                /* '<S377>/Product' */
  real_T SumofElements_l;              /* '<S377>/Sum of Elements' */
  real_T Sqrt;                         /* '<S377>/Sqrt' */
  real_T Product2_k;                   /* '<S377>/Product2' */
  real_T TrigonometricFunction;        /* '<S377>/Trigonometric Function' */
  real_T VectorConcatenate_iu[6];      /* '<S377>/Vector Concatenate' */
  real_T Product1_b[6];                /* '<S377>/Product1' */
  real_T uAPabsRT[6];                  /* '<S377>/.5.*A.*Pabs.//R.//T' */
  real_T Product3_c[3];                /* '<S377>/Product3' */
  real_T UnaryMinus_j[3];              /* '<S364>/Unary Minus' */
  real_T Add1_n[3];                    /* '<S362>/Add1' */
  real_T Product4[3];                  /* '<S377>/Product4' */
  real_T UnaryMinus1_c[3];             /* '<S364>/Unary Minus1' */
  real_T Add_n[3];                     /* '<S362>/Add' */
  real_T VectorConcatenate2_o[2];      /* '<S376>/Vector Concatenate2' */
  real_T VectorConcatenate1_b[2];      /* '<S376>/Vector Concatenate1' */
  real_T HIL_SpeedCmd_kph;             /* '<S1>/Gain3' */
  real_T heading;                      /* '<S4>/Bus Selector2' */
  real_T signalLightHeadId;            /* '<S4>/Bus Selector2' */
  real_T signalLightDistance;          /* '<S4>/Bus Selector2' */
  real_T signalLightColor;             /* '<S4>/Bus Selector2' */
  real_T speedLimit;                   /* '<S4>/Bus Selector2' */
  real_T speedLimitNext;               /* '<S4>/Bus Selector2' */
  real_T speedLimitChangeDistance;     /* '<S4>/Bus Selector2' */
  real_T Memory1_k;                    /* '<Root>/Memory1' */
  real_T accelerationActual;           /* '<S4>/Multiport Switch1' */
  real_T timeSimulation;               /* '<S4>/timeSimulation' */
  real_T timeInOut;                    /* '<S4>/timeInOut' */
  real_T Divide_k[4];                  /* '<S13>/Divide' */
  real_T ManualSwitch_f;               /* '<S4>/Manual Switch' */
  real_T sendTrigger;                  /* '<S4>/manualTriggerSW' */
  real_T ManualSwitch3;                /* '<S4>/Manual Switch3' */
  real_T ManualSwitch1_p;              /* '<S4>/Manual Switch1' */
  real_T LinkSpeed;                    /* '<S14>/Data Inport S-Fcn' */
  real_T SimTime;                      /* '<S1>/SimTime' */
  real_T EnrgyTrans;                   /* '<S106>/Integrator' */
  real_T UnitConversion;               /* '<S106>/Unit Conversion' */
  real_T AccelCmd;                     /* '<S3>/Step' */
  real_T AccelCmd_a;                   /* '<S3>/Switch' */
  real_T Clock_f;                      /* '<S3>/Clock' */
  real_T Constant;                     /* '<S3>/Constant' */
  real_T CltchCmd;                     /* '<S3>/Constant3' */
  real_T lgSw;                         /* '<S3>/Constant4' */
  real_T Constant_n;                   /* '<S103>/Constant' */
  real_T Constant1;                    /* '<S103>/Constant1' */
  real_T Divide_l;                     /* '<S100>/Divide' */
  real_T Divide2;                      /* '<S100>/Divide2' */
  real_T Divide1_h;                    /* '<S100>/Divide1' */
  real_T mto100Km;                     /* '<S100>/m to 100Km' */
  real_T Saturation_bg;                /* '<S100>/Saturation' */
  real_T m3toL;                        /* '<S100>/m^3 to L' */
  real_T L100Km;                       /* '<S100>/L//100 Km Calc' */
  real_T Product_fn;                   /* '<S100>/Product' */
  real_T Sqrt_a;                       /* '<S100>/Sqrt' */
  real_T m3pergal;                     /* '<S100>/m^3 per gal' */
  real_T TransferFcn;                  /* '<S507>/Transfer Fcn' */
  real_T Abs_k;                        /* '<S507>/Abs' */
  real_T Switch1_b;                    /* '<S507>/Switch1' */
  real_T TransferFcn1;                 /* '<S507>/Transfer Fcn1' */
  real_T Abs1;                         /* '<S507>/Abs1' */
  real_T TransferFcn2;                 /* '<S507>/Transfer Fcn2' */
  real_T Abs2_k;                       /* '<S507>/Abs2' */
  real_T TransferFcn3;                 /* '<S507>/Transfer Fcn3' */
  real_T Abs3;                         /* '<S507>/Abs3' */
  real_T VectorConcatenate_iuc[4];     /* '<S507>/Vector Concatenate' */
  real_T MultiportSwitch[4];           /* '<S101>/Multiport Switch' */
  real_T Gain1_k;                      /* '<S144>/Gain1' */
  real_T Product1_n;                   /* '<S146>/Product1' */
  real_T Gain3_d;                      /* '<S146>/Gain3' */
  real_T Gain4_m;                      /* '<S146>/Gain4' */
  real_T Product_p;                    /* '<S141>/Product' */
  real_T Gain_b;                       /* '<S141>/Gain' */
  real_T Gain1_d;                      /* '<S141>/Gain1' */
  real_T Add_o;                        /* '<S141>/Add' */
  real_T AxlTrqLump;                   /* '<S206>/Add' */
  real_T Add1_j;                       /* '<S206>/Add1' */
  real_T Gain1_c;                      /* '<S206>/Gain1' */
  real_T Spd;                          /* '<S206>/Gain2' */
  real_T TransferredPower;             /* '<S207>/Transferred Power' */
  real_T PwrMechLoss;                  /* '<S207>/Constant' */
  real_T PwrDampLoss;                  /* '<S207>/Constant1' */
  real_T PwrStoredShft;                /* '<S207>/Constant2' */
  real_T VectorConcatenate_d[3];       /* '<S207>/Vector Concatenate' */
  real_T Product_ak;                   /* '<S218>/Product' */
  real_T uDLookupTable;                /* '<S218>/2-D Lookup Table' */
  real_T Add_i;                        /* '<S215>/Add' */
  real_T Saturation_h;                 /* '<S215>/Saturation' */
  real_T Divide_d;                     /* '<S215>/Divide' */
  real_T Saturation1_i;                /* '<S254>/Saturation1' */
  real_T Divide_j;                     /* '<S254>/Divide' */
  real_T Saturation_m;                 /* '<S254>/Saturation' */
  real_T Saturation1_h;                /* '<S243>/Saturation1' */
  real_T Divide_e;                     /* '<S243>/Divide' */
  real_T Saturation_fh;                /* '<S243>/Saturation' */
  real_T Product1_nl;                  /* '<S239>/Product1' */
  real_T WhlTrqReflectedToMot;         /* '<S242>/WhlTrqReflectedToMot' */
  real_T MotTrqCmdRegen;               /* '<S242>/Gain' */
  real_T AccelDecelSwitch;             /* '<S241>/Accel Decel Switch' */
  real_T Abs_j;                        /* '<S245>/Abs' */
  real_T DischrgLmt;                   /* '<S240>/DischrgLmt' */
  real_T Product_it;                   /* '<S240>/Product' */
  real_T Product3_l;                   /* '<S248>/Product3' */
  real_T rads_to_rpm;                  /* '<S248>/rads_to_rpm' */
  real_T Abs_c;                        /* '<S248>/Abs' */
  real_T Abs1_e;                       /* '<S248>/Abs1' */
  real_T EffMap;                       /* '<S248>/Eff Map' */
  real_T Gain1_bs;                     /* '<S248>/Gain1' */
  real_T Product_kv;                   /* '<S248>/Product' */
  real_T Switch2_i;                    /* '<S248>/Switch2' */
  real_T MathFunction_o;               /* '<S248>/Math Function' */
  real_T Product4_b;                   /* '<S248>/Product4' */
  real_T Subtract_n;                   /* '<S246>/Subtract' */
  real_T ChrgLmt_e;                    /* '<S240>/ChrgLmt' */
  real_T Product1_g;                   /* '<S240>/Product1' */
  real_T Subtract1;                    /* '<S246>/Subtract1' */
  real_T Switch_l;                     /* '<S253>/Switch' */
  real_T Switch2_l;                    /* '<S253>/Switch2' */
  real_T ElecToMechPwr;                /* '<S245>/ElecToMechPwr' */
  real_T UnaryMinus_i;                 /* '<S250>/Unary Minus' */
  real_T Switch1_e;                    /* '<S250>/Switch1' */
  real_T Fcn;                          /* '<S250>/Fcn' */
  real_T Product_e;                    /* '<S250>/Product' */
  real_T Switch_m;                     /* '<S250>/Switch' */
  real_T MechPwrToTrq;                 /* '<S245>/MechPwrToTrq' */
  real_T Switch_f;                     /* '<S245>/Switch' */
  real_T Switch1_hf;                   /* '<S245>/Switch1' */
  real_T Gain_f;                       /* '<S249>/Gain' */
  real_T Switch_iq;                    /* '<S255>/Switch' */
  real_T Switch2_f;                    /* '<S255>/Switch2' */
  real_T Sum_b;                        /* '<S216>/Sum' */
  real_T Add_b;                        /* '<S217>/Add' */
  real_T Gain_nu;                      /* '<S217>/Gain' */
  real_T Gain1_l;                      /* '<S217>/Gain1' */
  real_T Subtract_i;                   /* '<S217>/Subtract' */
  real_T Abs_l;                        /* '<S227>/Abs' */
  real_T Divide_o;                     /* '<S227>/Divide' */
  real_T Integrator_a0;                /* '<S227>/Integrator' */
  real_T Gain1_h;                      /* '<S227>/Gain1' */
  real_T Merge_i;                      /* '<S227>/Merge' */
  real_T Product_o;                    /* '<S229>/Product' */
  real_T uDLookupTable_l;              /* '<S229>/2-D Lookup Table' */
  real_T Add_h3;                       /* '<S226>/Add' */
  real_T Saturation_a0;                /* '<S226>/Saturation' */
  real_T Divide_oq;                    /* '<S226>/Divide' */
  real_T Sum_c;                        /* '<S227>/Sum' */
  real_T Add_a;                        /* '<S228>/Add' */
  real_T Gain_k;                       /* '<S228>/Gain' */
  real_T Gain1_g;                      /* '<S228>/Gain1' */
  real_T Subtract_kq;                  /* '<S228>/Subtract' */
  real_T Switch4;                      /* '<S481>/Switch4' */
  real_T Switch5;                      /* '<S481>/Switch5' */
  real_T Switch6;                      /* '<S481>/Switch6' */
  real_T Switch7;                      /* '<S481>/Switch7' */
  real_T My[4];                        /* '<S481>/Vector Concatenate1' */
  real_T CarTrq_T2WFL;                 /* '<S259>/Transfer Fcn3' */
  real_T CarTrq_T2WFR;                 /* '<S259>/Transfer Fcn1' */
  real_T CarTrq_T2WRL;                 /* '<S259>/Transfer Fcn2' */
  real_T CarTrq_T2WRR;                 /* '<S259>/Transfer Fcn4' */
  real_T TransferFcn_a;                /* '<S259>/Transfer Fcn' */
  real_T TransferFcn9;                 /* '<S259>/Transfer Fcn9' */
  real_T TransferFcn10;                /* '<S259>/Transfer Fcn10' */
  real_T TransferFcn11;                /* '<S259>/Transfer Fcn11' */
  real_T PwrLoss;                      /* '<S269>/Constant' */
  real_T InstStrgRatio;                /* '<S270>/Constant' */
  real_T Gain1_j;                      /* '<S270>/Gain1' */
  real_T Add_fu;                       /* '<S271>/Add' */
  real_T TrqIn;                        /* '<S270>/Gain2' */
  real_T TrqIn_b;                      /* '<S269>/Unary Minus' */
  real_T PerAckInConstant;             /* '<S268>/PerAckInConstant' */
  real_T AssignWhlFz[2];               /* '<S324>/Assign WhlFz' */
  real_T Reshape2_f[2];                /* '<S279>/Reshape2' */
  real_T MatrixConcatenate1_l[6];      /* '<S279>/Matrix Concatenate1' */
  real_T Sum_l[2];                     /* '<S284>/Sum' */
  real_T CarriertoAxleCompliance[2];   /* '<S284>/Carrier to Axle Compliance' */
  real_T Reshape2_nx[2];               /* '<S274>/Reshape2' */
  real_T Sum2_a[2];                    /* '<S284>/Sum2' */
  real_T CarriertoAxleDamping[2];      /* '<S284>/Carrier to Axle Damping' */
  real_T Sum1[2];                      /* '<S284>/Sum1' */
  real_T Gain_g[2];                    /* '<S274>/Gain' */
  real_T MatrixConcatenate6_b[6];      /* '<S274>/Matrix Concatenate6' */
  real_T Reshape14[6];                 /* '<S279>/Reshape14' */
  real_T Reshape18[6];                 /* '<S274>/Reshape18' */
  real_T Ang[12];                      /* '<S273>/Matrix Concatenate2' */
  real_T F_b[12];                      /* '<S273>/Matrix Concatenate4' */
  real_T M[12];                        /* '<S273>/Matrix Concatenate5' */
  real_T VectorConcatenate_hh[4];      /* '<S497>/Vector Concatenate' */
  real_T Reshape3_h[4];                /* '<S278>/Reshape3' */
  real_T Reshape4_ac[4];               /* '<S278>/Reshape4' */
  real_T VectorConcatenate_bn[4];      /* '<S499>/Vector Concatenate' */
  real_T Reshape5_f[4];                /* '<S278>/Reshape5' */
  real_T M_i[12];                      /* '<S278>/Matrix Concatenate' */
  real_T Selector17[6];                /* '<S273>/Selector17' */
  real_T Selector9_d[6];               /* '<S273>/Selector9' */
  real_T Reshape_oc[16];               /* '<S274>/Reshape' */
  real_T Camberselect[2];              /* '<S274>/Camber select' */
  real_T Casterselect[2];              /* '<S274>/Caster select' */
  real_T Energyselect[2];              /* '<S274>/Energy select' */
  real_T Heightselect[2];              /* '<S274>/Height select' */
  real_T Powerselect[2];               /* '<S274>/Power select' */
  real_T Reshape16[6];                 /* '<S274>/Reshape16' */
  real_T Reshape17_a[6];               /* '<S274>/Reshape17' */
  real_T Reshape19_g[6];               /* '<S274>/Reshape19' */
  real_T Reshape20[6];                 /* '<S274>/Reshape20' */
  real_T Toeselect[2];                 /* '<S274>/Toe select' */
  real_T dWhlXselect[2];               /* '<S274>/dWhlX select' */
  real_T dWhlYselect[2];               /* '<S274>/dWhlY select' */
  real_T Reshape_h[16];                /* '<S279>/Reshape' */
  real_T Camberselect_b[2];            /* '<S279>/Camber select' */
  real_T Casterselect_g[2];            /* '<S279>/Caster select' */
  real_T Energyselect_l[2];            /* '<S279>/Energy select' */
  real_T Heightselect_g[2];            /* '<S279>/Height select' */
  real_T Powerselect_g[2];             /* '<S279>/Power select' */
  real_T Reshape10_o[6];               /* '<S279>/Reshape10' */
  real_T Reshape11_a[6];               /* '<S279>/Reshape11' */
  real_T Reshape12_o[6];               /* '<S279>/Reshape12' */
  real_T Reshape16_o[6];               /* '<S279>/Reshape16' */
  real_T Reshape18_j[6];               /* '<S279>/Reshape18' */
  real_T Toeselect_h[2];               /* '<S279>/Toe select' */
  real_T dWhlXselect_a[2];             /* '<S279>/dWhlX select' */
  real_T dWhlYselect_l[2];             /* '<S279>/dWhlY select' */
  real_T ActTqLF_Nm_;                  /* '<S259>/Transfer Fcn5' */
  real_T ActTqRF_Nm_;                  /* '<S259>/Transfer Fcn6' */
  real_T ActTqLR_Nm_;                  /* '<S259>/Transfer Fcn7' */
  real_T ActTqRR_Nm_;                  /* '<S259>/Transfer Fcn8' */
  real_T Reshape1_mf[3];               /* '<S393>/Reshape1' */
  real_T Product_j[3];                 /* '<S393>/Product' */
  real_T Reshape2_m[3];                /* '<S393>/Reshape2' */
  real_T V_wb[3];                      /* '<S391>/Add4' */
  real_T Reshape1_hz[3];               /* '<S401>/Reshape1' */
  real_T Product_c[3];                 /* '<S401>/Product' */
  real_T Reshape2_b[3];                /* '<S401>/Reshape2' */
  real_T V_wb_o[3];                    /* '<S399>/Add4' */
  real_T sincos_o1_lw[3];              /* '<S409>/sincos' */
  real_T sincos_o2_m[3];               /* '<S409>/sincos' */
  real_T VectorConcatenate_gy[9];      /* '<S416>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_g[9];
                                /* '<S416>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_a[9];              /* '<S407>/Transpose1' */
  real_T VectorConcatenate_bu[3];      /* '<S387>/Vector Concatenate' */
  real_T Reshape1_a[3];                /* '<S411>/Reshape1' */
  real_T Product_m[3];                 /* '<S411>/Product' */
  real_T Reshape2_j[3];                /* '<S411>/Reshape2' */
  real_T Add_hj[3];                    /* '<S407>/Add' */
  real_T jxk_h;                        /* '<S417>/j x k' */
  real_T kxi_n;                        /* '<S417>/k x i' */
  real_T ixj_p;                        /* '<S417>/i x j' */
  real_T kxj_e;                        /* '<S418>/k x j' */
  real_T ixk_a;                        /* '<S418>/i x k' */
  real_T jxi_ar;                       /* '<S418>/j x i' */
  real_T Sum_m[3];                     /* '<S412>/Sum' */
  real_T Add1_go[3];                   /* '<S407>/Add1' */
  real_T Reshape1_g4[3];               /* '<S410>/Reshape1' */
  real_T Product_er[3];                /* '<S410>/Product' */
  real_T Reshape2_gv[3];               /* '<S410>/Reshape2' */
  real_T V_wb_c[3];                    /* '<S407>/Add4' */
  real_T Fcn_b;                        /* '<S413>/Fcn' */
  real_T Abs_c4;                       /* '<S413>/Abs' */
  real_T Switch_k;                     /* '<S413>/Switch' */
  real_T Divide_om;                    /* '<S408>/Divide' */
  real_T Beta;                         /* '<S408>/Trigonometric Function' */
  real_T Reshape1_i[3];                /* '<S421>/Reshape1' */
  real_T Product_ph[3];                /* '<S421>/Product' */
  real_T Reshape2_l[3];                /* '<S421>/Reshape2' */
  real_T V_wb_i[3];                    /* '<S419>/Add4' */
  real_T Reshape1_pn[3];               /* '<S429>/Reshape1' */
  real_T Product_m3[3];                /* '<S429>/Product' */
  real_T Reshape2_h[3];                /* '<S429>/Reshape2' */
  real_T V_wb_h[3];                    /* '<S427>/Add4' */
  real_T sincos_o1_j[3];               /* '<S436>/sincos' */
  real_T sincos_o2_a[3];               /* '<S436>/sincos' */
  real_T VectorConcatenate_b1[9];      /* '<S440>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_p[9];
                                /* '<S440>/Reshape (9) to [3x3] column-major' */
  real_T Transpose1_go[9];             /* '<S435>/Transpose1' */
  real_T VectorConcatenate_o[3];       /* '<S390>/Vector Concatenate' */
  real_T Reshape1_k[3];                /* '<S438>/Reshape1' */
  real_T Product_nz[3];                /* '<S438>/Product' */
  real_T Reshape2_d[3];                /* '<S438>/Reshape2' */
  real_T Add_g[3];                     /* '<S435>/Add' */
  real_T jxk_f;                        /* '<S441>/j x k' */
  real_T kxi_b;                        /* '<S441>/k x i' */
  real_T ixj_a;                        /* '<S441>/i x j' */
  real_T kxj_n;                        /* '<S442>/k x j' */
  real_T ixk_d;                        /* '<S442>/i x k' */
  real_T jxi_j;                        /* '<S442>/j x i' */
  real_T Sum_g[3];                     /* '<S439>/Sum' */
  real_T Add1_oh[3];                   /* '<S435>/Add1' */
  real_T Reshape1_o4[3];               /* '<S437>/Reshape1' */
  real_T Product_jo[3];                /* '<S437>/Product' */
  real_T Reshape2_nm[3];               /* '<S437>/Reshape2' */
  real_T V_wb_c3[3];                   /* '<S435>/Add4' */
  real_T Product12[3];                 /* '<S383>/Product12' */
  real_T Product1_f[3];                /* '<S383>/Product1' */
  real_T Add_pt[3];                    /* '<S383>/Add' */
  real_T Product13[3];                 /* '<S383>/Product13' */
  real_T Product2_d[3];                /* '<S383>/Product2' */
  real_T Add1_f[3];                    /* '<S383>/Add1' */
  real_T Product9[3];                  /* '<S383>/Product9' */
  real_T Product8[3];                  /* '<S383>/Product8' */
  real_T Product14[3];                 /* '<S383>/Product14' */
  real_T Product3_p[3];                /* '<S383>/Product3' */
  real_T Product_gb;                   /* '<S383>/Product' */
  real_T Divide1_i;                    /* '<S453>/Divide1' */
  real_T ax;                           /* '<S453>/Sum of Elements1' */
  real_T UnitConversion1;              /* '<S459>/Unit Conversion1' */
  real_T UnitConversion_c;             /* '<S383>/Unit Conversion' */
  real_T Product5;                     /* '<S383>/Product5' */
  real_T Divide_og;                    /* '<S453>/Divide' */
  real_T ay;                           /* '<S453>/Sum of Elements' */
  real_T UnitConversion_h;             /* '<S459>/Unit Conversion' */
  real_T UnitConversion1_b;            /* '<S383>/Unit Conversion1' */
  real_T Product6;                     /* '<S383>/Product6' */
  real_T Product7;                     /* '<S383>/Product7' */
  real_T VectorConcatenate1_d[6];      /* '<S383>/Vector Concatenate1' */
  real_T SumofElements1;               /* '<S383>/Sum of Elements1' */
  real_T VectorConcatenate2_gv[6];     /* '<S383>/Vector Concatenate2' */
  real_T SumofElements2;               /* '<S383>/Sum of Elements2' */
  real_T VectorConcatenate3_j[6];      /* '<S383>/Vector Concatenate3' */
  real_T SumofElements3;               /* '<S383>/Sum of Elements3' */
  real_T UnaryMinus5;                  /* '<S383>/Unary Minus5' */
  real_T Sum1_n[2];                    /* '<S448>/Sum1' */
  real_T Sum_a;                        /* '<S448>/Sum' */
  real_T Fcn_m;                        /* '<S455>/Fcn' */
  real_T Abs_e;                        /* '<S455>/Abs' */
  real_T Switch_g;                     /* '<S455>/Switch' */
  real_T Divide_on;                    /* '<S450>/Divide' */
  real_T Beta_m;                       /* '<S450>/Trigonometric Function' */
  real_T az;                           /* '<S384>/Constant1' */
  real_T Constant10;                   /* '<S384>/Constant10' */
  real_T zddot;                        /* '<S384>/Constant3' */
  real_T Constant9;                    /* '<S384>/Constant9' */
  real_T sincos_o1_jm[3];              /* '<S452>/sincos' */
  real_T sincos_o2_j[3];               /* '<S452>/sincos' */
  real_T VectorConcatenate_lq[9];      /* '<S458>/Vector Concatenate' */
  real_T Reshape9to3x3columnmajor_l[9];
                                /* '<S458>/Reshape (9) to [3x3] column-major' */
  real_T VectorConcatenate4_l[4];      /* '<S266>/Vector Concatenate4' */
  real_T VectorConcatenate3_a[4];      /* '<S266>/Vector Concatenate3' */
  real_T Subtract_f[4];                /* '<S266>/Subtract' */
  real_T VectorConcatenate1_b1[4];     /* '<S266>/Vector Concatenate1' */
  real_T VectorConcatenate2_c[4];      /* '<S266>/Vector Concatenate2' */
  real_T Selector_b[4];                /* '<S267>/Selector' */
  real_T Reshape2_hu[4];               /* '<S267>/Reshape2' */
  real_T Sum_d2[4];                    /* '<S469>/Sum' */
  real_T Divide_hd[4];                 /* '<S469>/Divide' */
  real_T Sum_p[12];                    /* '<S470>/Sum' */
  real_T Divide_b[12];                 /* '<S470>/Divide' */
  real_T Reshape6_k[4];                /* '<S472>/Reshape6' */
  real_T Reshape_l[4];                 /* '<S476>/Reshape' */
  real_T VectorConcatenate5[4];        /* '<S472>/Vector Concatenate5' */
  real_T VectorConcatenate6[4];        /* '<S472>/Vector Concatenate6' */
  real_T Integrator_l;                 /* '<S630>/Integrator' */
  real_T Integrator_m;                 /* '<S627>/Integrator' */
  real_T IntegratorSecondOrder_o1_b;   /* '<S631>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_m;   /* '<S631>/Integrator, Second-Order' */
  real_T Sum6_g;                       /* '<S631>/Sum6' */
  real_T Saturation_iv;                /* '<S631>/Saturation' */
  real_T Add2_c;                       /* '<S626>/Add2' */
  real_T Saturation_mh;                /* '<S626>/Saturation' */
  real_T Product3_k;                   /* '<S626>/Product3' */
  real_T Add1_ny;                      /* '<S626>/Add1' */
  real_T Reshape_cu;                   /* '<S626>/Reshape' */
  real_T TorqueConversion1_g;          /* '<S636>/Torque Conversion1' */
  real_T product_ax;                   /* '<S636>/product' */
  real_T DisallowNegativeBrakeTorque_m;
                                   /* '<S636>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_k;           /* '<S636>/Torque Conversion' */
  real_T Ratioofstatictokinetic_b;     /* '<S634>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1_f;   /* '<S634>/Ratio of static to kinetic1' */
  real_T SignalCopy;                   /* '<S511>/Signal Copy' */
  real_T Product2_o;                   /* '<S515>/Product2' */
  real_T Add1_ea;                      /* '<S515>/Add1' */
  real_T Abs_l5;                       /* '<S515>/Abs' */
  real_T Add_h5;                       /* '<S515>/Add' */
  real_T Saturation1_o2;               /* '<S515>/Saturation1' */
  real_T Product3_j;                   /* '<S515>/Product3' */
  real_T Saturation_a4;                /* '<S515>/Saturation' */
  real_T Product1_d;                   /* '<S515>/Product1' */
  real_T Product2_j;                   /* '<S516>/Product2' */
  real_T Add1_ja;                      /* '<S516>/Add1' */
  real_T Abs_cb;                       /* '<S516>/Abs' */
  real_T Add_k;                        /* '<S516>/Add' */
  real_T Saturation1_hz;               /* '<S516>/Saturation1' */
  real_T Product3_h;                   /* '<S516>/Product3' */
  real_T Saturation_o;                 /* '<S516>/Saturation' */
  real_T Product1_p;                   /* '<S516>/Product1' */
  real_T Product2_l;                   /* '<S518>/Product2' */
  real_T Add1_il;                      /* '<S518>/Add1' */
  real_T Abs_i;                        /* '<S518>/Abs' */
  real_T Add_d;                        /* '<S518>/Add' */
  real_T Saturation1_a;                /* '<S518>/Saturation1' */
  real_T Product3_o;                   /* '<S518>/Product3' */
  real_T Saturation_mt;                /* '<S518>/Saturation' */
  real_T Product1_d2;                  /* '<S518>/Product1' */
  real_T Selector_n[27];               /* '<S512>/Selector' */
  real_T Switch_j;                     /* '<S519>/Switch' */
  real_T Gain2_ic;                     /* '<S519>/Gain2' */
  real_T Sum2_m;                       /* '<S519>/Sum2' */
  real_T Gain1_i;                      /* '<S519>/Gain1' */
  real_T SignalCopy_h;                 /* '<S539>/Signal Copy' */
  real_T Product2_g;                   /* '<S543>/Product2' */
  real_T Add1_be;                      /* '<S543>/Add1' */
  real_T Abs_ll;                       /* '<S543>/Abs' */
  real_T Add_n5;                       /* '<S543>/Add' */
  real_T Saturation1_k;                /* '<S543>/Saturation1' */
  real_T Product3_ju;                  /* '<S543>/Product3' */
  real_T Saturation_k;                 /* '<S543>/Saturation' */
  real_T Product1_a;                   /* '<S543>/Product1' */
  real_T Product2_i;                   /* '<S544>/Product2' */
  real_T Add1_fp;                      /* '<S544>/Add1' */
  real_T Abs_jo;                       /* '<S544>/Abs' */
  real_T Add_dj;                       /* '<S544>/Add' */
  real_T Saturation1_an;               /* '<S544>/Saturation1' */
  real_T Product3_jn;                  /* '<S544>/Product3' */
  real_T Saturation_jw;                /* '<S544>/Saturation' */
  real_T Product1_j;                   /* '<S544>/Product1' */
  real_T Product2_n;                   /* '<S546>/Product2' */
  real_T Add1_h4;                      /* '<S546>/Add1' */
  real_T Abs_jk;                       /* '<S546>/Abs' */
  real_T Add_oj;                       /* '<S546>/Add' */
  real_T Saturation1_n;                /* '<S546>/Saturation1' */
  real_T Product3_hu;                  /* '<S546>/Product3' */
  real_T Saturation_l;                 /* '<S546>/Saturation' */
  real_T Product1_dw;                  /* '<S546>/Product1' */
  real_T Selector_dp[27];              /* '<S540>/Selector' */
  real_T Switch_h;                     /* '<S547>/Switch' */
  real_T Gain2_d;                      /* '<S547>/Gain2' */
  real_T Sum2_f;                       /* '<S547>/Sum2' */
  real_T Gain1_jh;                     /* '<S547>/Gain1' */
  real_T SignalCopy_j;                 /* '<S567>/Signal Copy' */
  real_T Product2_id;                  /* '<S571>/Product2' */
  real_T Add1_dt;                      /* '<S571>/Add1' */
  real_T Abs_o;                        /* '<S571>/Abs' */
  real_T Add_i0;                       /* '<S571>/Add' */
  real_T Saturation1_l;                /* '<S571>/Saturation1' */
  real_T Product3_f;                   /* '<S571>/Product3' */
  real_T Saturation_hj;                /* '<S571>/Saturation' */
  real_T Product1_pr;                  /* '<S571>/Product1' */
  real_T Product2_e;                   /* '<S572>/Product2' */
  real_T Add1_ob;                      /* '<S572>/Add1' */
  real_T Abs_h;                        /* '<S572>/Abs' */
  real_T Add_er;                       /* '<S572>/Add' */
  real_T Saturation1_h0;               /* '<S572>/Saturation1' */
  real_T Product3_au;                  /* '<S572>/Product3' */
  real_T Saturation_ju;                /* '<S572>/Saturation' */
  real_T Product1_gs;                  /* '<S572>/Product1' */
  real_T Product2_f;                   /* '<S574>/Product2' */
  real_T Add1_ix;                      /* '<S574>/Add1' */
  real_T Abs_kv;                       /* '<S574>/Abs' */
  real_T Add_l;                        /* '<S574>/Add' */
  real_T Saturation1_oz;               /* '<S574>/Saturation1' */
  real_T Product3_ji;                  /* '<S574>/Product3' */
  real_T Saturation_n3;                /* '<S574>/Saturation' */
  real_T Product1_e;                   /* '<S574>/Product1' */
  real_T Selector_j[27];               /* '<S568>/Selector' */
  real_T Switch_li;                    /* '<S575>/Switch' */
  real_T Gain2_o;                      /* '<S575>/Gain2' */
  real_T Sum2_mu;                      /* '<S575>/Sum2' */
  real_T Gain1_a;                      /* '<S575>/Gain1' */
  real_T SignalCopy_g;                 /* '<S595>/Signal Copy' */
  real_T Product2_ex;                  /* '<S599>/Product2' */
  real_T Add1_hb;                      /* '<S599>/Add1' */
  real_T Abs_hl;                       /* '<S599>/Abs' */
  real_T Add_ds;                       /* '<S599>/Add' */
  real_T Saturation1_lw;               /* '<S599>/Saturation1' */
  real_T Product3_g;                   /* '<S599>/Product3' */
  real_T Saturation_km;                /* '<S599>/Saturation' */
  real_T Product1_jp;                  /* '<S599>/Product1' */
  real_T Product2_h;                   /* '<S600>/Product2' */
  real_T Add1_k;                       /* '<S600>/Add1' */
  real_T Abs_d;                        /* '<S600>/Abs' */
  real_T Add_ez;                       /* '<S600>/Add' */
  real_T Saturation1_lw1;              /* '<S600>/Saturation1' */
  real_T Product3_h5;                  /* '<S600>/Product3' */
  real_T Saturation_e;                 /* '<S600>/Saturation' */
  real_T Product1_px;                  /* '<S600>/Product1' */
  real_T Product2_nh;                  /* '<S602>/Product2' */
  real_T Add1_bg;                      /* '<S602>/Add1' */
  real_T Abs_dm;                       /* '<S602>/Abs' */
  real_T Add_gy;                       /* '<S602>/Add' */
  real_T Saturation1_io;               /* '<S602>/Saturation1' */
  real_T Product3_fk;                  /* '<S602>/Product3' */
  real_T Saturation_fr;                /* '<S602>/Saturation' */
  real_T Product1_l;                   /* '<S602>/Product1' */
  real_T Selector_gh[27];              /* '<S596>/Selector' */
  real_T Switch_p;                     /* '<S603>/Switch' */
  real_T Gain2_e;                      /* '<S603>/Gain2' */
  real_T Sum2_al;                      /* '<S603>/Sum2' */
  real_T Gain1_o;                      /* '<S603>/Gain1' */
  real_T Integrator_d;                 /* '<S628>/Integrator' */
  real_T Product_nx;                   /* '<S631>/Product' */
  real_T SignalCopy_e;                 /* '<S623>/Signal Copy' */
  real_T Product2_b;                   /* '<S627>/Product2' */
  real_T Add1_b4;                      /* '<S627>/Add1' */
  real_T Abs_g;                        /* '<S627>/Abs' */
  real_T Add_kv;                       /* '<S627>/Add' */
  real_T Saturation1_e;                /* '<S627>/Saturation1' */
  real_T Product3_jz;                  /* '<S627>/Product3' */
  real_T Saturation_g;                 /* '<S627>/Saturation' */
  real_T Product1_ln;                  /* '<S627>/Product1' */
  real_T Product2_gb;                  /* '<S628>/Product2' */
  real_T Add1_mf;                      /* '<S628>/Add1' */
  real_T Abs_es;                       /* '<S628>/Abs' */
  real_T Add_ec;                       /* '<S628>/Add' */
  real_T Saturation1_c;                /* '<S628>/Saturation1' */
  real_T Product3_lv;                  /* '<S628>/Product3' */
  real_T Saturation_fr1;               /* '<S628>/Saturation' */
  real_T Product1_by;                  /* '<S628>/Product1' */
  real_T Product2_gq;                  /* '<S630>/Product2' */
  real_T Add1_h3;                      /* '<S630>/Add1' */
  real_T Abs_ot;                       /* '<S630>/Abs' */
  real_T Add_gq;                       /* '<S630>/Add' */
  real_T Saturation1_p;                /* '<S630>/Saturation1' */
  real_T Product3_e;                   /* '<S630>/Product3' */
  real_T Saturation_cm;                /* '<S630>/Saturation' */
  real_T Product1_a3;                  /* '<S630>/Product1' */
  real_T Selector_m[27];               /* '<S624>/Selector' */
  real_T Switch_of;                    /* '<S631>/Switch' */
  real_T Gain2_l;                      /* '<S631>/Gain2' */
  real_T Sum2_e;                       /* '<S631>/Sum2' */
  real_T Gain1_hh;                     /* '<S631>/Gain1' */
  real_T Integrator_c;                 /* '<S658>/Integrator' */
  real_T Integrator_oi;                /* '<S655>/Integrator' */
  real_T IntegratorSecondOrder_o1_l;   /* '<S659>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_o;   /* '<S659>/Integrator, Second-Order' */
  real_T Sum6_k;                       /* '<S659>/Sum6' */
  real_T Saturation_b4;                /* '<S659>/Saturation' */
  real_T Add2_kc;                      /* '<S654>/Add2' */
  real_T Saturation_pv;                /* '<S654>/Saturation' */
  real_T Product3_ll;                  /* '<S654>/Product3' */
  real_T Add1_f2;                      /* '<S654>/Add1' */
  real_T Reshape_f;                    /* '<S654>/Reshape' */
  real_T TorqueConversion1_j;          /* '<S664>/Torque Conversion1' */
  real_T product_j;                    /* '<S664>/product' */
  real_T DisallowNegativeBrakeTorque_l;
                                   /* '<S664>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_n;           /* '<S664>/Torque Conversion' */
  real_T Ratioofstatictokinetic_p;     /* '<S662>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1_b;   /* '<S662>/Ratio of static to kinetic1' */
  real_T Integrator_bi;                /* '<S656>/Integrator' */
  real_T Product_h;                    /* '<S659>/Product' */
  real_T SignalCopy_e1;                /* '<S651>/Signal Copy' */
  real_T Product2_nd;                  /* '<S655>/Product2' */
  real_T Add1_a;                       /* '<S655>/Add1' */
  real_T Abs_n;                        /* '<S655>/Abs' */
  real_T Add_erk;                      /* '<S655>/Add' */
  real_T Saturation1_g;                /* '<S655>/Saturation1' */
  real_T Product3_d;                   /* '<S655>/Product3' */
  real_T Saturation_hb;                /* '<S655>/Saturation' */
  real_T Product1_o;                   /* '<S655>/Product1' */
  real_T Product2_hr;                  /* '<S656>/Product2' */
  real_T Add1_ni;                      /* '<S656>/Add1' */
  real_T Abs_m;                        /* '<S656>/Abs' */
  real_T Add_an;                       /* '<S656>/Add' */
  real_T Saturation1_gz;               /* '<S656>/Saturation1' */
  real_T Product3_m;                   /* '<S656>/Product3' */
  real_T Saturation_ob;                /* '<S656>/Saturation' */
  real_T Product1_le;                  /* '<S656>/Product1' */
  real_T Product2_nm;                  /* '<S658>/Product2' */
  real_T Add1_or;                      /* '<S658>/Add1' */
  real_T Abs_c2;                       /* '<S658>/Abs' */
  real_T Add_kk;                       /* '<S658>/Add' */
  real_T Saturation1_ij;               /* '<S658>/Saturation1' */
  real_T Product3_mv;                  /* '<S658>/Product3' */
  real_T Saturation_bj;                /* '<S658>/Saturation' */
  real_T Product1_hd;                  /* '<S658>/Product1' */
  real_T Selector_p[27];               /* '<S652>/Selector' */
  real_T Switch_ot;                    /* '<S659>/Switch' */
  real_T Gain2_l2;                     /* '<S659>/Gain2' */
  real_T Sum2_c;                       /* '<S659>/Sum2' */
  real_T Gain1_e;                      /* '<S659>/Gain1' */
  real_T Integrator_e;                 /* '<S686>/Integrator' */
  real_T Integrator_c5;                /* '<S683>/Integrator' */
  real_T IntegratorSecondOrder_o1_g;   /* '<S687>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_ij;  /* '<S687>/Integrator, Second-Order' */
  real_T Sum6_i;                       /* '<S687>/Sum6' */
  real_T Saturation_fa;                /* '<S687>/Saturation' */
  real_T Add2_ls;                      /* '<S682>/Add2' */
  real_T Saturation_gs;                /* '<S682>/Saturation' */
  real_T Product3_b5;                  /* '<S682>/Product3' */
  real_T Add1_oa;                      /* '<S682>/Add1' */
  real_T Reshape_j;                    /* '<S682>/Reshape' */
  real_T TorqueConversion1_ie;         /* '<S692>/Torque Conversion1' */
  real_T product_i;                    /* '<S692>/product' */
  real_T DisallowNegativeBrakeTorque_ll;
                                   /* '<S692>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_pz;          /* '<S692>/Torque Conversion' */
  real_T Ratioofstatictokinetic_g;     /* '<S690>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1_o;   /* '<S690>/Ratio of static to kinetic1' */
  real_T Integrator_d4;                /* '<S684>/Integrator' */
  real_T Product_ki;                   /* '<S687>/Product' */
  real_T SignalCopy_m;                 /* '<S679>/Signal Copy' */
  real_T Product2_p;                   /* '<S683>/Product2' */
  real_T Add1_c;                       /* '<S683>/Add1' */
  real_T Abs_ke;                       /* '<S683>/Abs' */
  real_T Add_bv;                       /* '<S683>/Add' */
  real_T Saturation1_cv;               /* '<S683>/Saturation1' */
  real_T Product3_h4;                  /* '<S683>/Product3' */
  real_T Saturation_i1;                /* '<S683>/Saturation' */
  real_T Product1_eu;                  /* '<S683>/Product1' */
  real_T Product2_ng;                  /* '<S684>/Product2' */
  real_T Add1_cq;                      /* '<S684>/Add1' */
  real_T Abs_kj;                       /* '<S684>/Abs' */
  real_T Add_l4;                       /* '<S684>/Add' */
  real_T Saturation1_h2;               /* '<S684>/Saturation1' */
  real_T Product3_l3;                  /* '<S684>/Product3' */
  real_T Saturation_fx;                /* '<S684>/Saturation' */
  real_T Product1_fq;                  /* '<S684>/Product1' */
  real_T Product2_gr;                  /* '<S686>/Product2' */
  real_T Add1_e1;                      /* '<S686>/Add1' */
  real_T Abs_p;                        /* '<S686>/Abs' */
  real_T Add_pi;                       /* '<S686>/Add' */
  real_T Saturation1_gn;               /* '<S686>/Saturation1' */
  real_T Product3_jk;                  /* '<S686>/Product3' */
  real_T Saturation_nwn;               /* '<S686>/Saturation' */
  real_T Product1_h1;                  /* '<S686>/Product1' */
  real_T Selector_h[27];               /* '<S680>/Selector' */
  real_T Switch_gg;                    /* '<S687>/Switch' */
  real_T Gain2_k;                      /* '<S687>/Gain2' */
  real_T Sum2_er;                      /* '<S687>/Sum2' */
  real_T Gain1_hn;                     /* '<S687>/Gain1' */
  real_T Integrator_a1;                /* '<S714>/Integrator' */
  real_T Integrator_gf;                /* '<S711>/Integrator' */
  real_T IntegratorSecondOrder_o1_bk;  /* '<S715>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_l;   /* '<S715>/Integrator, Second-Order' */
  real_T Sum6_ix;                      /* '<S715>/Sum6' */
  real_T Saturation_fi;                /* '<S715>/Saturation' */
  real_T Add2_i;                       /* '<S710>/Add2' */
  real_T Saturation_i0;                /* '<S710>/Saturation' */
  real_T Product3_dc;                  /* '<S710>/Product3' */
  real_T Add1_mx;                      /* '<S710>/Add1' */
  real_T Reshape_hb;                   /* '<S710>/Reshape' */
  real_T TorqueConversion1_gn;         /* '<S720>/Torque Conversion1' */
  real_T product_m;                    /* '<S720>/product' */
  real_T DisallowNegativeBrakeTorque_n;
                                   /* '<S720>/Disallow Negative Brake Torque' */
  real_T TorqueConversion_h;           /* '<S720>/Torque Conversion' */
  real_T Ratioofstatictokinetic_e;     /* '<S718>/Ratio of static to kinetic' */
  real_T Ratioofstatictokinetic1_n;   /* '<S718>/Ratio of static to kinetic1' */
  real_T Integrator_gs;                /* '<S712>/Integrator' */
  real_T Product_l;                    /* '<S715>/Product' */
  real_T SignalCopy_c;                 /* '<S707>/Signal Copy' */
  real_T Product2_dg;                  /* '<S711>/Product2' */
  real_T Add1_el;                      /* '<S711>/Add1' */
  real_T Abs_ej;                       /* '<S711>/Abs' */
  real_T Add_m;                        /* '<S711>/Add' */
  real_T Saturation1_ks;               /* '<S711>/Saturation1' */
  real_T Product3_f1;                  /* '<S711>/Product3' */
  real_T Saturation_n0;                /* '<S711>/Saturation' */
  real_T Product1_jd;                  /* '<S711>/Product1' */
  real_T Product2_nn;                  /* '<S712>/Product2' */
  real_T Add1_hy;                      /* '<S712>/Add1' */
  real_T Abs_e0;                       /* '<S712>/Abs' */
  real_T Add_c;                        /* '<S712>/Add' */
  real_T Saturation1_cz;               /* '<S712>/Saturation1' */
  real_T Product3_ki;                  /* '<S712>/Product3' */
  real_T Saturation_pj;                /* '<S712>/Saturation' */
  real_T Product1_o5;                  /* '<S712>/Product1' */
  real_T Product2_gk;                  /* '<S714>/Product2' */
  real_T Add1_ll;                      /* '<S714>/Add1' */
  real_T Abs_nh;                       /* '<S714>/Abs' */
  real_T Add_g5;                       /* '<S714>/Add' */
  real_T Saturation1_il;               /* '<S714>/Saturation1' */
  real_T Product3_kw;                  /* '<S714>/Product3' */
  real_T Saturation_gg;                /* '<S714>/Saturation' */
  real_T Product1_i;                   /* '<S714>/Product1' */
  real_T Selector_bt[27];              /* '<S708>/Selector' */
  real_T Switch_jv;                    /* '<S715>/Switch' */
  real_T Gain2_a;                      /* '<S715>/Gain2' */
  real_T Sum2_e0;                      /* '<S715>/Sum2' */
  real_T Gain1_he;                     /* '<S715>/Gain1' */
  real_T VectorConcatenate_n[4];       /* '<S490>/Vector Concatenate' */
  real_T VectorConcatenate_mq[4];      /* '<S492>/Vector Concatenate' */
  real_T VectorConcatenate_b2[4];      /* '<S498>/Vector Concatenate' */
  real_T IntegratorSecondOrder_o1_n;   /* '<S501>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_f;   /* '<S501>/Integrator, Second-Order' */
  real_T Sum6_g4;                      /* '<S501>/Sum6' */
  real_T Switch_ll;                    /* '<S501>/Switch' */
  real_T Gain2_oq;                     /* '<S501>/Gain2' */
  real_T Sum2_cy;                      /* '<S501>/Sum2' */
  real_T Gain1_k4;                     /* '<S501>/Gain1' */
  real_T Product_aw;                   /* '<S501>/Product' */
  real_T Saturation_d;                 /* '<S501>/Saturation' */
  real_T IntegratorSecondOrder_o1_nz;  /* '<S502>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_p;   /* '<S502>/Integrator, Second-Order' */
  real_T Sum6_fk;                      /* '<S502>/Sum6' */
  real_T Switch_pn;                    /* '<S502>/Switch' */
  real_T Gain2_o0;                     /* '<S502>/Gain2' */
  real_T Sum2_a1;                      /* '<S502>/Sum2' */
  real_T Gain1_id;                     /* '<S502>/Gain1' */
  real_T Product_lg;                   /* '<S502>/Product' */
  real_T Saturation_le;                /* '<S502>/Saturation' */
  real_T IntegratorSecondOrder_o1_aa;  /* '<S503>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_ik;  /* '<S503>/Integrator, Second-Order' */
  real_T Sum6_h;                       /* '<S503>/Sum6' */
  real_T Switch_mt;                    /* '<S503>/Switch' */
  real_T Gain2_im;                     /* '<S503>/Gain2' */
  real_T Sum2_p;                       /* '<S503>/Sum2' */
  real_T Gain1_o1;                     /* '<S503>/Gain1' */
  real_T Product_m0;                   /* '<S503>/Product' */
  real_T Saturation_hp;                /* '<S503>/Saturation' */
  real_T IntegratorSecondOrder_o1_c;   /* '<S504>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_o2_c;   /* '<S504>/Integrator, Second-Order' */
  real_T Sum6_mh;                      /* '<S504>/Sum6' */
  real_T Switch_ol;                    /* '<S504>/Switch' */
  real_T Gain2_b;                      /* '<S504>/Gain2' */
  real_T Sum2_o;                       /* '<S504>/Sum2' */
  real_T Gain1_it;                     /* '<S504>/Gain1' */
  real_T Product_gy;                   /* '<S504>/Product' */
  real_T Saturation_i4;                /* '<S504>/Saturation' */
  real_T VectorConcatenate_cd[4];      /* '<S505>/Vector Concatenate' */
  real_T VectorConcatenate_nm[4];      /* '<S506>/Vector Concatenate' */
  real_T TransferFcn_i;                /* '<S508>/Transfer Fcn' */
  real_T Abs_nm;                       /* '<S508>/Abs' */
  real_T TransferFcn1_d;               /* '<S508>/Transfer Fcn1' */
  real_T Abs1_b;                       /* '<S508>/Abs1' */
  real_T TransferFcn2_l;               /* '<S508>/Transfer Fcn2' */
  real_T Abs2_o;                       /* '<S508>/Abs2' */
  real_T TransferFcn3_k;               /* '<S508>/Transfer Fcn3' */
  real_T Abs3_c;                       /* '<S508>/Abs3' */
  real_T RateLimiter;                  /* '<S508>/Rate Limiter' */
  real_T RateLimiter1;                 /* '<S508>/Rate Limiter1' */
  real_T RateLimiter2;                 /* '<S508>/Rate Limiter2' */
  real_T RateLimiter3;                 /* '<S508>/Rate Limiter3' */
  real_T VectorConcatenate_cm[4];      /* '<S508>/Vector Concatenate' */
  real_T DataTypeConversion1;          /* '<S116>/Data Type Conversion1' */
  real_T Memory2;                      /* '<S124>/Memory2' */
  real_T Integrator_k;                 /* '<S129>/Integrator' */
  real_T Memory_k;                     /* '<S133>/Memory' */
  real_T Sum3;                         /* '<S133>/Sum3' */
  real_T Memory_g;                     /* '<S134>/Memory' */
  real_T Sum3_k;                       /* '<S134>/Sum3' */
  real_T StatusWord;                   /* '<S117>/Data Type Conversion' */
  real_T ActualPosition_cnt;           /* '<S117>/Data Type Conversion1' */
  real_T SineWave;                     /* '<S114>/Sine Wave' */
  real_T RateLimiter_b;                /* '<S114>/Rate Limiter' */
  real_T Switch_py;                    /* '<S114>/Switch' */
  real_T bped2br_N;                    /* '<S107>/bped2br_N' */
  real_T Switch2_d;                    /* '<S114>/Switch2' */
  real_T BrakePedal_TargetlPosition_pct;/* '<S114>/Gain' */
  real_T Memory_o;                     /* '<S117>/Memory' */
  real_T Memory2_g;                    /* '<S117>/Memory2' */
  real_T Memory1_n;                    /* '<S117>/Memory1' */
  real_T Sum_bz;                       /* '<S117>/Sum' */
  real_T Product_jx;                   /* '<S117>/Product' */
  real_T TargetPosition_cnt;           /* '<S117>/Sum1' */
  real_T Switch1_eb;                   /* '<S114>/Switch1' */
  real_T TargetPositionFinal_cnt;      /* '<S117>/Gain2' */
  real_T Sum2_g;                       /* '<S117>/Sum2' */
  real_T ActualPosition_pct;           /* '<S117>/Product1' */
  real_T RateLimiter_n;                /* '<S115>/Rate Limiter' */
  real_T APP_CmdFinal_pct;             /* '<S115>/Switch2' */
  real_T APP1Tbl;                      /* '<S115>/APP1Tbl' */
  real_T Gain3_i;                      /* '<S115>/Gain3' */
  real_T Product2_lc;                  /* '<S115>/Product2' */
  real_T APP1_CmdFinal_V;              /* '<S115>/Gain1' */
  real_T APP2Tbl;                      /* '<S115>/APP2Tbl' */
  real_T Gain4_f;                      /* '<S115>/Gain4' */
  real_T Product3_km;                  /* '<S115>/Product3' */
  real_T APP2_CmdFinal_V;              /* '<S115>/Gain' */
  real_T AccelPedal_CmdFinal_pct;      /* '<S115>/Gain2' */
  real_T SteeringCmd_l;                /* '<S3>/Ackerman steer' */
  real_T Steer;                        /* '<Root>/Steer' */
  real_T Throttle;                     /* '<Root>/Throttle' */
  real_T ImpAsg_InsertedFor_zdotWheel_at_inport_0[4];/* '<S736>/Demux1' */
  real_T ImpAsg_InsertedFor_ydotWheel_at_inport_0[4];/* '<S736>/Demux1' */
  real_T ImpAsg_InsertedFor_xdotWheel_at_inport_0[4];/* '<S736>/Demux1' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0;/* '<S721>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0;/* '<S721>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0;/* '<S721>/Clutch' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0_d;/* '<S693>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0_a;/* '<S693>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0_h;/* '<S693>/Clutch' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0_a;/* '<S665>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0_p;/* '<S665>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0_o;/* '<S665>/Clutch' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0_m;/* '<S637>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0_o;/* '<S637>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0_b;/* '<S637>/Clutch' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0_mv;/* '<S609>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0_c;/* '<S609>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0_g;/* '<S609>/Clutch' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0_f;/* '<S581>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0_d;/* '<S581>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0_e;/* '<S581>/Clutch' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0_n;/* '<S553>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0_b;/* '<S553>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0_n;/* '<S553>/Clutch' */
  real_T ImpAsg_InsertedFor_Omegadot_at_inport_0_b;/* '<S525>/Clutch' */
  real_T ImpAsg_InsertedFor_Omega_at_inport_0_f;/* '<S525>/Clutch' */
  real_T ImpAsg_InsertedFor_Myb_at_inport_0_k;/* '<S525>/Clutch' */
  real_T ImpAsg_InsertedFor_Fz_at_inport_0[4];/* '<S478>/Demux1' */
  real_T ImpAsg_InsertedFor_Fy_at_inport_0[4];/* '<S478>/Demux1' */
  real_T ImpAsg_InsertedFor_Fx_at_inport_0[4];/* '<S478>/Demux1' */
  real_T TmpSignalConversionAtSFunctionInport8[3];/* '<S362>/vehicle model' */
  real_T TmpSignalConversionAtSFunctionInport9[3];/* '<S362>/vehicle model' */
  real_T yOut[3];                      /* '<S362>/vehicle model' */
  real_T FBody[3];                     /* '<S362>/vehicle model' */
  real_T MBody[3];                     /* '<S362>/vehicle model' */
  real_T FOut[12];                     /* '<S362>/vehicle model' */
  real_T FTire[12];                    /* '<S362>/vehicle model' */
  real_T Fg[3];                        /* '<S362>/vehicle model' */
  real_T wheelInfo[8];                 /* '<S362>/vehicle model' */
  real_T stateDer[4];                  /* '<S362>/vehicle model' */
  real_T status;                       /* '<S362>/vehicle model' */
  real_T y[2];                         /* '<S384>/COMB2I' */
  real_T ImpAsg_InsertedFor_WhlFz_at_inport_0[2];/* '<S323>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlAng_at_inport_0[6];/* '<S323>/Suspension' */
  real_T ImpAsg_InsertedFor_VehM_at_inport_0[6];/* '<S323>/Suspension' */
  real_T ImpAsg_InsertedFor_VehFz_at_inport_0[2];/* '<S323>/Suspension' */
  real_T ImpAsg_InsertedFor_Info_at_inport_0[16];/* '<S323>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlFzAs_at_inport_0[2];/* '<S325>/Mux' */
  real_T ImpAsg_InsertedFor_VehFzAs_at_inport_0[2];/* '<S325>/Mux1' */
  real_T ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[2];/* '<S325>/Sum2' */
  real_T ImpAsg_InsertedFor_WhlFz_at_inport_0_p[2];/* '<S283>/Suspension' */
  real_T ImpAsg_InsertedFor_WhlAng_at_inport_0_m[6];/* '<S283>/Suspension' */
  real_T ImpAsg_InsertedFor_VehM_at_inport_0_n[6];/* '<S283>/Suspension' */
  real_T ImpAsg_InsertedFor_VehFz_at_inport_0_g[2];/* '<S283>/Suspension' */
  real_T ImpAsg_InsertedFor_Info_at_inport_0_n[16];/* '<S283>/Suspension' */
  real_T ImpAsg_InsertedFor_p_at_inport_0;
                                     /* '<S282>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_cgV_at_inport_0[3];
                                     /* '<S282>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_cgP_at_inport_0[3];
                                     /* '<S282>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_DCM_at_inport_0[9];
                                     /* '<S282>/Solid Axle 2 DOF CG Dynamics' */
  real_T ImpAsg_InsertedFor_WhlV_at_inport_0[6];/* '<S281>/Sum2' */
  real_T ImpAsg_InsertedFor_WhlP_at_inport_0[6];/* '<S281>/Sum4' */
  real_T ImpAsg_InsertedFor_SuspV_at_inport_0[6];/* '<S281>/Sum3' */
  real_T ImpAsg_InsertedFor_SuspP_at_inport_0[6];/* '<S281>/Sum1' */
  real_T Vehicle_Spd_kph;              /* '<S109>/Data Type Conversion11' */
  real_T Clock1_b;                     /* '<S109>/Clock1' */
  real_T US1_n;                        /* '<S109>/US1' */
  real_T RateLimiter_f;                /* '<S109>/Rate Limiter' */
  real_T Gain2_m;                      /* '<S109>/Gain2' */
  real_T Memory4;                      /* '<S109>/Memory4' */
  real_T Clock_h;                      /* '<S109>/Clock' */
  real_T Switch1_o;                    /* '<S109>/Switch1' */
  real_T tCycle;                       /* '<S109>/Add1' */
  real_T uDLookupTable_a;              /* '<S109>/1-D Lookup Table' */
  real_T Add2_mh;                      /* '<S109>/Add2' */
  real_T Gain3_o;                      /* '<S109>/Gain3' */
  real_T RefSpdkmhr;                   /* '<S109>/Multiport Switch1' */
  real_T Memory1_d;                    /* '<S151>/Memory1' */
  real_T RefSpdkmhr_o;                 /* '<S151>/SpdSat' */
  real_T Memory1_ld;                   /* '<S157>/Memory1' */
  real_T Memory_p;                     /* '<S157>/Memory' */
  real_T Add_cy;                       /* '<S157>/Add' */
  real_T Driver_IntTerm_pct;           /* '<S157>/Limits [-50,50]' */
  real_T VS_Flt;                       /* '<S157>/Lowpass Filter' */
  real_T SpdErr;                       /* '<S157>/Sum' */
  real_T Kp;                           /* '<S157>/Kp' */
  real_T Memory2_e;                    /* '<S151>/Memory2' */
  real_T ProportionalGainScheduling; /* '<S157>/Proportional Gain Scheduling' */
  real_T Driver_PropTerm_pct;          /* '<S157>/Product1' */
  real_T Feedforward1;                 /* '<S157>/Feedforward1' */
  real_T Driver_FdfwdTerm_pct;         /* '<S157>/FeedforwardGain' */
  real_T Driver_TotCmdPreLim_pct;      /* '<S157>/Sum1' */
  real_T AcceleratorPedalPosPre_pct;   /* '<S157>/Switch1' */
  real_T AcceleratorPedalPos_pct;      /* '<S157>/Switch3' */
  real_T Gain_a;                       /* '<S155>/Gain' */
  real_T Kffvnom;                      /* '<S172>/Kff//vnom' */
  real_T Integrator_o4;                /* '<S180>/Integrator' */
  real_T Kpvnom;                       /* '<S172>/Kp//vnom' */
  real_T Integrator1_is;               /* '<S172>/Integrator1' */
  real_T Kg;                           /* '<S172>/Kg' */
  real_T Sum1_a;                       /* '<S172>/Sum1' */
  real_T uto1;                         /* '<S172>/-1 to 1 ' */
  real_T Accel;                        /* '<S173>/0~1' */
  real_T ut;                           /* '<S173>/Switch' */
  real_T Switch1_p;                    /* '<S173>/Switch1' */
  real_T Saturation_hh;                /* '<S173>/Saturation' */
  real_T AcceleratorPedalPos_pct_i;    /* '<S155>/Switch3' */
  real_T AccelPedalCmd;                /* '<S155>/Gain2' */
  real_T Gain_gh;                      /* '<S156>/Gain' */
  real_T Integrator1_j;                /* '<S193>/Integrator1' */
  real_T Accel_a;                      /* '<S192>/0~1' */
  real_T ut_j;                         /* '<S192>/Switch' */
  real_T Switch1_k;                    /* '<S192>/Switch1' */
  real_T Saturation_lj;                /* '<S192>/Saturation' */
  real_T AcceleratorPedalPos_pct_h;    /* '<S156>/Switch3' */
  real_T AccelPedalCmd_j;              /* '<S156>/Gain4' */
  real_T AccelPedalCmd_o;              /* '<S153>/Multiport Switch1' */
  real_T Gain_fk;                      /* '<S157>/Gain' */
  real_T BrakePedalPosPre_pct;         /* '<S157>/Switch2' */
  real_T BrakePedalPos_pct;            /* '<S157>/Switch4' */
  real_T u0;                           /* '<S174>/-1~0' */
  real_T Decel;                        /* '<S174>/Unary Minus' */
  real_T ut_k;                         /* '<S174>/Switch' */
  real_T Switch1_c;                    /* '<S174>/Switch1' */
  real_T Saturation_fs;                /* '<S174>/Saturation' */
  real_T BrakePedalPos_pct_h;          /* '<S155>/Switch4' */
  real_T BrakePedalCmd;                /* '<S155>/Gain3' */
  real_T u0_c;                         /* '<S194>/-1~0' */
  real_T Decel_g;                      /* '<S194>/Unary Minus' */
  real_T ut_f;                         /* '<S194>/Switch' */
  real_T Switch1_ey;                   /* '<S194>/Switch1' */
  real_T Saturation_nn;                /* '<S194>/Saturation' */
  real_T BrakePedalPos_pct_g;          /* '<S156>/Switch4' */
  real_T BrakePedalCmd_g;              /* '<S156>/Gain5' */
  real_T BrakePedalCmd_n;              /* '<S153>/Multiport Switch2' */
  real_T Drive;                        /* '<S151>/Constant' */
  real_T Ki;                           /* '<S157>/Ki' */
  real_T Feedforward;                  /* '<S157>/Feedforward' */
  real_T VS_Flt_o;                     /* '<S155>/Lowpass Filter' */
  real_T Gain1_og;                     /* '<S155>/Gain1' */
  real_T Decel_m;                      /* '<S174>/Unary Minus1' */
  real_T Sum5;                         /* '<S172>/Sum5' */
  real_T Kaw;                          /* '<S172>/Kaw' */
  real_T Kivnom;                       /* '<S172>/Ki//vnom' */
  real_T Sum8;                         /* '<S172>/Sum8' */
  real_T Divide_jn;                    /* '<S179>/Divide' */
  real_T Sum7;                         /* '<S167>/Sum7' */
  real_T Switch_a;                     /* '<S167>/Switch' */
  real_T Sum_o;                        /* '<S180>/Sum' */
  real_T Product_ip;                   /* '<S180>/Product' */
  real_T UnitConversion_g;             /* '<S170>/Unit Conversion' */
  real_T Integrator2;                  /* '<S181>/Integrator2' */
  real_T Product_cl;                   /* '<S181>/Product' */
  real_T UnitDelay[2];                 /* '<S181>/Unit Delay' */
  real_T VectorConcatenate1_h[2];      /* '<S181>/Vector Concatenate1' */
  real_T Switch_cz[2];                 /* '<S181>/Switch' */
  real_T Constant_a;                   /* '<S182>/Constant' */
  real_T Constant1_l;                  /* '<S182>/Constant1' */
  real_T VS_Flt_b;                     /* '<S156>/Lowpass Filter' */
  real_T Gain1_m;                      /* '<S156>/Gain1' */
  real_T UnitConversion1_l;            /* '<S191>/Unit Conversion1' */
  real_T UnitConversion_hm;            /* '<S191>/Unit Conversion' */
  real_T UnitConversion2;              /* '<S191>/Unit Conversion2' */
  real_T Product1_m;                   /* '<S191>/Product1' */
  real_T Product_lt;                   /* '<S191>/Product' */
  real_T ytT;                          /* '<S191>/Add2' */
  real_T etT;                          /* '<S191>/Add4' */
  real_T Divide_eq;                    /* '<S191>/Divide' */
  real_T u_ot;                         /* '<S191>/Add1' */
  real_T Sum_j2;                       /* '<S193>/Sum' */
  real_T Divide_oj;                    /* '<S193>/Divide' */
  real_T Decel_k;                      /* '<S194>/Unary Minus1' */
  real_T Sum7_n;                       /* '<S186>/Sum7' */
  real_T UnitConversion_o;             /* '<S189>/Unit Conversion' */
  real_T Integrator2_o;                /* '<S204>/Integrator2' */
  real_T Product_mn;                   /* '<S204>/Product' */
  real_T UnitDelay_p[2];               /* '<S204>/Unit Delay' */
  real_T VectorConcatenate1_p[2];      /* '<S204>/Vector Concatenate1' */
  real_T Switch_h0[2];                 /* '<S204>/Switch' */
  real_T Constant_l;                   /* '<S205>/Constant' */
  real_T Constant1_p;                  /* '<S205>/Constant1' */
  real_T Switch_b;                     /* '<S186>/Switch' */
  real_T RefSpdmph;                    /* '<S151>/km to miles' */
  real_T Vehicle_Spd_mph;              /* '<S109>/Gain' */
  real_T Vehicle_SpdDmd_mph;           /* '<S109>/Gain1' */
  real_T Clock1_j;                     /* '<S152>/Clock1' */
  real_T Memory4_d;                    /* '<S152>/Memory4' */
  real_T DataTypeConversion;           /* '<S152>/Data Type Conversion' */
  real_T Clock2;                       /* '<S152>/Clock2' */
  real_T Switch_mr;                    /* '<S152>/Switch' */
  real_T tCycle_e;                     /* '<S152>/Add1' */
  real_T Add_j;                        /* '<S109>/Add' */
  real_T ManSpd;                       /* '<S109>/ManSpd' */
  real_T x[2];                         /* '<S191>/Vehicle' */
  real_T F_bo[4];                      /* '<S191>/Setup' */
  real_T G[2];                         /* '<S191>/Setup' */
  real_T a_star;                       /* '<S191>/Setup' */
  real_T b_star[2];                    /* '<S191>/Setup' */
  real_T Sum3_l;                       /* '<S132>/Sum3' */
  real_T Sum3_lq;                      /* '<S131>/Sum3' */
  real_T ControlWord;                  /* '<S117>/MOOG State Machine' */
  real_T PositionCmd;                  /* '<S117>/MOOG State Machine' */
  real_T Actuator_Ready;               /* '<S117>/MOOG State Machine' */
  real_T FaultCntr;                    /* '<S117>/MOOG State Machine' */
  real_T MoogSMState;                  /* '<S117>/MOOG State Machine' */
  real_T InitComplete;                 /* '<S117>/MOOG State Machine' */
  real_T MoogOpMode;                   /* '<S117>/MOOG State Machine' */
  real_T LearningComplete;             /* '<S117>/MOOG State Machine' */
  real_T MaxStop;                      /* '<S117>/MOOG State Machine' */
  real_T MinStop;                      /* '<S117>/MOOG State Machine' */
  real_T CycleMode;                    /* '<S1>/RealSimHILCycle' */
  real_T SpeedCmd;                     /* '<S1>/RealSimHILCycle' */
  real_T timeSimulationOut;            /* '<S4>/generateTimeStepTrigger' */
  real_T timeInOut_d;                  /* '<S4>/generateTimeStepTrigger' */
  real_T timeStepTrigger;              /* '<S4>/generateTimeStepTrigger' */
  real_T nMsgSend;                     /* '<S4>/RealSimPack' */
  real_T RealSimInterpSpeed_o1;        /* '<S4>/RealSimInterpSpeed' */
  real_T accelerationDesired;          /* '<S4>/RealSimInterpSpeed' */
  real_T timeSimulator;                /* '<S4>/RealSimInterpSpeed' */
  real_T timeStepSimulator;            /* '<S4>/RealSimInterpSpeed' */
  real_T timeStepTrigger_c;            /* '<S4>/RealSimInterpSpeed' */
  real_T RealSimInterpSpeed_o6;        /* '<S4>/RealSimInterpSpeed' */
  real_T simStateIn;                   /* '<S4>/RealSimDepack' */
  real_T tIn;                          /* '<S4>/RealSimDepack' */
  real_T nVehIn;                       /* '<S4>/RealSimDepack' */
  real_T isVehicleInNetwork;           /* '<S4>/RealSimDepack' */
  real_T intVal;                       /* '<S4>/MATLAB Function' */
  uint32_T PowertrainData_10_HS1Counter;/* '<S2>/Data Inport S-Fcn' */
  uint32_T WheelSpeed_HS1Counter;      /* '<S2>/Data Inport S-Fcn' */
  uint32_T BrakeSysFeatures_HS1Counter;/* '<S2>/Data Inport S-Fcn' */
  uint32_T EngVehicleSpThrottle_HS1Counter;/* '<S2>/Data Inport S-Fcn' */
  uint32_T BrakeSnData_4_HS1Counter;   /* '<S2>/Data Inport S-Fcn' */
  uint32_T ReceivedBytes;              /* '<S15>/Data Inport S-Fcn' */
  uint32_T AvailableBytes;             /* '<S15>/Data Inport S-Fcn' */
  uint32_T DataTypeConversion4;        /* '<S10>/Data Type Conversion4' */
  uint32_T sendByteSize_RealSimPack;   /* '<S10>/switch for 0.1 send out' */
  uint32_T SentBytes;                  /* '<S18>/Data Inport S-Fcn' */
  uint32_T TPDO3Counter;               /* '<S127>/Data Inport S-Fcn' */
  int32_T ConnectionState_f;           /* '<S17>/Data Inport S-Fcn' */
  int32_T ActualPositionValue;         /* '<S121>/Data Inport S-Fcn' */
  int32_T TargetPosition_Cnt_Final;    /* '<S114>/Data Type Conversion2' */
  uint16_T BrkTot_Tq_RqArbValue;       /* '<S2>/Data Inport S-Fcn' */
  uint16_T BrkTot_Tq_ActlValue;        /* '<S2>/Data Inport S-Fcn' */
  int16_T Modeofoperation;             /* '<S117>/Gain1' */
  int16_T StatusWord0Value;            /* '<S123>/Data Inport S-Fcn' */
  int16_T ControlWord_Final;           /* '<S114>/Data Type Conversion1' */
  uint8_T TrnRng_D_RqValue;            /* '<S2>/Data Inport S-Fcn' */
  uint8_T TrnPrkSys_D_ActlValue;       /* '<S2>/Data Inport S-Fcn' */
  uint8_T GearLvr_D_ActlDrvValue;      /* '<S2>/Data Inport S-Fcn' */
  uint8_T GearPos_No_CsValue;          /* '<S2>/Data Inport S-Fcn' */
  uint8_T GearPos_D_TrgValue;          /* '<S2>/Data Inport S-Fcn' */
  uint8_T GearPos_No_CntValue;         /* '<S2>/Data Inport S-Fcn' */
  uint8_T GearPos_D_ActlValue;         /* '<S2>/Data Inport S-Fcn' */
  uint8_T VehVActlBrk_No_CsValue;      /* '<S2>/Data Inport S-Fcn' */
  uint8_T VehVActlBrk_No_CntValue;     /* '<S2>/Data Inport S-Fcn' */
  uint8_T VehVActlBrk_D_QfValue;       /* '<S2>/Data Inport S-Fcn' */
  uint8_T BrkTotTqRqArb_No_CsValue;    /* '<S2>/Data Inport S-Fcn' */
  uint8_T BrkTotTqRqArb_No_CntValue;   /* '<S2>/Data Inport S-Fcn' */
  uint8_T HsaStat_D_ActlValue;         /* '<S2>/Data Inport S-Fcn' */
  uint8_T DataVector[1024];            /* '<S15>/Data Inport S-Fcn' */
  uint8_T DataTypeConversion1_j[1024]; /* '<S9>/Data Type Conversion1' */
  uint8_T signalLightId[50];           /* '<S4>/Bus Selector2' */
  uint8_T ByteSend[200];               /* '<S4>/RealSimPack' */
  int8_T DataTypeConversion_h;         /* '<S114>/Data Type Conversion' */
  int8_T ModesOfOperationDisplayValue; /* '<S122>/Data Inport S-Fcn' */
  boolean_T PowertrainData_10_HS1State;/* '<S2>/Data Inport S-Fcn' */
  boolean_T WheelSpeed_HS1State;       /* '<S2>/Data Inport S-Fcn' */
  boolean_T BrakeSysFeatures_HS1State; /* '<S2>/Data Inport S-Fcn' */
  boolean_T EngVehicleSpThrottle_HS1State;/* '<S2>/Data Inport S-Fcn' */
  boolean_T BrakeSnData_4_HS1State;    /* '<S2>/Data Inport S-Fcn' */
  boolean_T Status;                    /* '<S15>/Data Inport S-Fcn' */
  boolean_T Compare;                   /* '<S21>/Compare' */
  boolean_T Compare_g;                 /* '<S20>/Compare' */
  boolean_T LogicalOperator;           /* '<S12>/Logical Operator' */
  boolean_T Memory_pz;                 /* '<S280>/Memory' */
  boolean_T DataTypeConversion5;       /* '<S4>/Data Type Conversion5' */
  boolean_T DataTypeConversion_c;      /* '<S10>/Data Type Conversion' */
  boolean_T Status_p;                  /* '<S18>/Data Inport S-Fcn' */
  boolean_T Compare_j;                 /* '<S244>/Compare' */
  boolean_T Compare_i;                 /* '<S247>/Compare' */
  boolean_T Compare_e;                 /* '<S251>/Compare' */
  boolean_T Compare_f;                 /* '<S252>/Compare' */
  boolean_T LogicalOperator_c;         /* '<S246>/Logical Operator' */
  boolean_T LowerRelop1;               /* '<S253>/LowerRelop1' */
  boolean_T UpperRelop;                /* '<S253>/UpperRelop' */
  boolean_T Compare_p;                 /* '<S256>/Compare' */
  boolean_T Compare_jz;                /* '<S257>/Compare' */
  boolean_T LogicalOperator_g;         /* '<S250>/Logical Operator' */
  boolean_T LowerRelop1_a;             /* '<S255>/LowerRelop1' */
  boolean_T UpperRelop_e;              /* '<S255>/UpperRelop' */
  boolean_T Compare_l;                 /* '<S414>/Compare' */
  boolean_T Compare_fi;                /* '<S415>/Compare' */
  boolean_T LogicalOperator_k;         /* '<S413>/Logical Operator' */
  boolean_T Compare_b;                 /* '<S456>/Compare' */
  boolean_T Compare_fw;                /* '<S457>/Compare' */
  boolean_T LogicalOperator_h;         /* '<S455>/Logical Operator' */
  boolean_T DataTypeConversion3;       /* '<S116>/Data Type Conversion3' */
  boolean_T RelationalOperator2;       /* '<S124>/Relational Operator2' */
  boolean_T Compare_a;                 /* '<S128>/Compare' */
  boolean_T BrakeSwitch;               /* '<S107>/Relational Operator' */
  boolean_T Compare_aw;                /* '<S158>/Compare' */
  boolean_T Compare_o;                 /* '<S159>/Compare' */
  boolean_T Compare_d;                 /* '<S163>/Compare' */
  boolean_T Uk1;                       /* '<S161>/Delay Input1' */
  boolean_T FixPtRelationalOperator;   /* '<S161>/FixPt Relational Operator' */
  boolean_T Compare_a0;                /* '<S162>/Compare' */
  boolean_T Uk1_n;                     /* '<S160>/Delay Input1' */
  boolean_T FixPtRelationalOperator_h; /* '<S160>/FixPt Relational Operator' */
  boolean_T LogicalOperator_f;         /* '<S157>/Logical Operator' */
  boolean_T Compare_ol;                /* '<S164>/Compare' */
  boolean_T LogicalOperator1;          /* '<S173>/Logical Operator1' */
  boolean_T LogicalOperator2;          /* '<S173>/Logical Operator2' */
  boolean_T LogicalOperator3;          /* '<S173>/Logical Operator3' */
  boolean_T NOT;                       /* '<S175>/NOT' */
  boolean_T Compare_ee;                /* '<S183>/Compare' */
  boolean_T LogicalOperator1_o;        /* '<S192>/Logical Operator1' */
  boolean_T LogicalOperator2_f;        /* '<S192>/Logical Operator2' */
  boolean_T LogicalOperator3_h;        /* '<S192>/Logical Operator3' */
  boolean_T NOT_g;                     /* '<S198>/NOT' */
  boolean_T LogicalOperator1_d;        /* '<S174>/Logical Operator1' */
  boolean_T LogicalOperator2_k;        /* '<S174>/Logical Operator2' */
  boolean_T LogicalOperator3_c;        /* '<S174>/Logical Operator3' */
  boolean_T NOT_e;                     /* '<S177>/NOT' */
  boolean_T LogicalOperator1_l;        /* '<S194>/Logical Operator1' */
  boolean_T LogicalOperator2_o;        /* '<S194>/Logical Operator2' */
  boolean_T LogicalOperator3_m;        /* '<S194>/Logical Operator3' */
  boolean_T NOT_a;                     /* '<S200>/NOT' */
  boolean_T BrakeSwitch_h;             /* '<S157>/Relational Operator' */
  boolean_T BrakeSwitch_d;             /* '<S155>/Relational Operator' */
  boolean_T BrakeSwitch_a;             /* '<S156>/Relational Operator' */
  boolean_T BrakeSwitch_i;             /* '<S153>/Multiport Switch3' */
  boolean_T LogicalOperator_kc;        /* '<S167>/Logical Operator' */
  boolean_T VectorConcatenate_ln[2];   /* '<S181>/Vector Concatenate' */
  boolean_T LogicalOperator_cf;        /* '<S186>/Logical Operator' */
  boolean_T VectorConcatenate_dw[2];   /* '<S204>/Vector Concatenate' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_ca_T CoreSubsys_pna[4];/* '<S474>/Wheel to Body Transform' */
  B_MATLABFunction_CAVE_MachE_dSPACE_250912_T sf_MATLABFunction3;/* '<S481>/MATLAB Function3' */
  B_MATLABFunction_CAVE_MachE_dSPACE_250912_T sf_MATLABFunction2;/* '<S481>/MATLAB Function2' */
  B_MATLABFunction_CAVE_MachE_dSPACE_250912_T sf_MATLABFunction1;/* '<S481>/MATLAB Function1' */
  B_MATLABFunction_CAVE_MachE_dSPACE_250912_T sf_MATLABFunction_n;/* '<S481>/MATLAB Function' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_b1_T CoreSubsys_k[1];/* '<S717>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_c_T sf_MagicTireConstInput_gz;/* '<S708>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_g_T CoreSubsys_d[1];/* '<S689>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_c_T sf_MagicTireConstInput_f;/* '<S680>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_ha_T CoreSubsys_e[1];/* '<S661>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_c_T sf_MagicTireConstInput_a;/* '<S652>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_ey_T CoreSubsys_od[1];/* '<S633>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_c_T sf_MagicTireConstInput_e;/* '<S624>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_f_T CoreSubsys_n[1];/* '<S605>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput_m;/* '<S596>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_k_T CoreSubsys_cu[1];/* '<S577>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput_d;/* '<S568>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T CoreSubsys_c[1];/* '<S549>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput_g;/* '<S540>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_b_T CoreSubsys_h[1];/* '<S521>/Clutch Scalar Parameters' */
  B_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput;/* '<S512>/Magic Tire Const Input' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T CoreSubsys_pn[4];/* '<S472>/Wheel to Body Transform' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T CoreSubsys_b[2];
  /* '<S279>/For each track and axle combination calculate suspension forces and moments' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_hf_T CoreSubsys_p[1];
                                     /* '<S324>/For Each Axle With Anti-Sway' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T CoreSubsys_o[2];
  /* '<S274>/For each track and axle combination calculate suspension forces and moments' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T CoreSubsys_g[1];
         /* '<S274>/For each axle calculate axle cg positions and velocities' */
  B_CoreSubsys_CAVE_MachE_dSPACE_250912_T CoreSubsys[2];
  /* '<S274>/For each axle and track calculate suspension and wheel positions and velocities' */
  B_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T IfActionSubsystem1_a;/* '<S227>/If Action Subsystem1' */
  B_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T IfActionSubsystem1;/* '<S216>/If Action Subsystem1' */
  B_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough_e;/* '<S200>/Pass Through' */
  B_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough_k;/* '<S198>/Pass Through' */
  B_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough_i;/* '<S177>/Pass Through' */
  B_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough;/* '<S175>/Pass Through' */
} B_CAVE_MachE_dSPACE_250912_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  RealSimDepack_CAVE_MachE_dSPACE_250912_T obj;/* '<S4>/RealSimDepack' */
  RealSimPack_CAVE_MachE_dSPACE_250912_T obj_a;/* '<S4>/RealSimPack' */
  RealSimInterpSpeed_CAVE_MachE_dSPACE_250912_T obj_i;/* '<S4>/RealSimInterpSpeed' */
  real_T UnitDelay_DSTATE[2];          /* '<S181>/Unit Delay' */
  real_T UnitDelay_DSTATE_g[2];        /* '<S204>/Unit Delay' */
  real_T Memory1_PreviousInput;        /* '<S206>/Memory1' */
  real_T Memory_PreviousInput;         /* '<S111>/Memory' */
  real_T Memory_PreviousInput_n;       /* '<S4>/Memory' */
  real_T Memory_PreviousInput_g;       /* '<Root>/Memory' */
  real_T Memory_PreviousInput_h;       /* '<S206>/Memory' */
  real_T PrevY;                        /* '<S269>/Backlash' */
  real_T Memory1_PreviousInput_h[2];   /* '<S280>/Memory1' */
  real_T Memory1_PreviousInput_i;      /* '<Root>/Memory1' */
  real_T timeSimulation_PreviousInput; /* '<S4>/timeSimulation' */
  real_T timeInOut_PreviousInput;      /* '<S4>/timeInOut' */
  real_T PrevY_k;                      /* '<S508>/Rate Limiter' */
  real_T LastMajorTime;                /* '<S508>/Rate Limiter' */
  real_T PrevY_d;                      /* '<S508>/Rate Limiter1' */
  real_T LastMajorTime_m;              /* '<S508>/Rate Limiter1' */
  real_T PrevY_e;                      /* '<S508>/Rate Limiter2' */
  real_T LastMajorTime_ms;             /* '<S508>/Rate Limiter2' */
  real_T PrevY_n;                      /* '<S508>/Rate Limiter3' */
  real_T LastMajorTime_i;              /* '<S508>/Rate Limiter3' */
  real_T Memory2_PreviousInput;        /* '<S124>/Memory2' */
  real_T Memory_PreviousInput_a;       /* '<S133>/Memory' */
  real_T Memory_PreviousInput_f;       /* '<S134>/Memory' */
  real_T PrevY_i;                      /* '<S114>/Rate Limiter' */
  real_T Memory_PreviousInput_k;       /* '<S117>/Memory' */
  real_T Memory2_PreviousInput_h;      /* '<S117>/Memory2' */
  real_T Memory1_PreviousInput_c;      /* '<S117>/Memory1' */
  real_T Sum2_DWORK1;                  /* '<S117>/Sum2' */
  real_T PrevY_g;                      /* '<S115>/Rate Limiter' */
  real_T PrevY_j;                      /* '<S109>/Rate Limiter' */
  real_T LastMajorTime_a;              /* '<S109>/Rate Limiter' */
  real_T Memory4_PreviousInput;        /* '<S109>/Memory4' */
  real_T Memory1_PreviousInput_n;      /* '<S151>/Memory1' */
  real_T Memory1_PreviousInput_nk;     /* '<S157>/Memory1' */
  real_T Memory_PreviousInput_m;       /* '<S157>/Memory' */
  real_T Memory2_PreviousInput_l;      /* '<S151>/Memory2' */
  real_T Memory4_PreviousInput_g;      /* '<S152>/Memory4' */
  void *DataInportSFcn_PWORK[44];      /* '<S2>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_n[4];     /* '<S15>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_f;        /* '<S17>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_p;        /* '<S14>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_h[2];     /* '<S18>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_k;        /* '<S127>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_a;        /* '<S123>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_g;        /* '<S121>/Data Inport S-Fcn' */
  void *DataInportSFcn_PWORK_i;        /* '<S122>/Data Inport S-Fcn' */
  uint32_T m_bpIndex;                /* '<S157>/Proportional Gain Scheduling' */
  uint32_T temporalCounter_i1;         /* '<S1>/RealSimHILCycle' */
  int_T Integrator_IWORK;              /* '<S467>/Integrator' */
  int_T IntegratorLimited_IWORK;       /* '<S144>/Integrator Limited' */
  int_T Integrator_IWORK_i;            /* '<S384>/Integrator' */
  int_T IntegratorSecondOrder_MODE;    /* '<S519>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_b;  /* '<S547>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_i;  /* '<S575>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_n;  /* '<S603>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_g;  /* '<S631>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_k;  /* '<S659>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_o;  /* '<S687>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_e;  /* '<S715>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_oh; /* '<S501>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_p;  /* '<S502>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_f;  /* '<S503>/Integrator, Second-Order' */
  int_T IntegratorSecondOrder_MODE_d;  /* '<S504>/Integrator, Second-Order' */
  uint16_T temporalCounter_i1_l;       /* '<S117>/MOOG State Machine' */
  uint16_T temporalCounter_i2;         /* '<S117>/MOOG State Machine' */
  uint16_T temporalCounter_i3;         /* '<S117>/MOOG State Machine' */
  uint16_T temporalCounter_i4;         /* '<S117>/MOOG State Machine' */
  uint16_T temporalCounter_i5;         /* '<S117>/MOOG State Machine' */
  boolean_T DelayInput1_DSTATE;        /* '<S161>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_a;      /* '<S160>/Delay Input1' */
  int8_T If_ActiveSubsystem;           /* '<S216>/If' */
  int8_T If_ActiveSubsystem_i;         /* '<S227>/If' */
  uint8_T is_active_c11_CAVE_MachE_dSPACE_250912;/* '<S117>/MOOG State Machine' */
  uint8_T is_MoogStateMachine;         /* '<S117>/MOOG State Machine' */
  uint8_T is_Initialisation;           /* '<S117>/MOOG State Machine' */
  uint8_T is_PositionControl;          /* '<S117>/MOOG State Machine' */
  uint8_T is_MainStateMachine;         /* '<S117>/MOOG State Machine' */
  uint8_T is_Learning;                 /* '<S117>/MOOG State Machine' */
  uint8_T is_LearnMaxStop;             /* '<S117>/MOOG State Machine' */
  uint8_T is_LearnMinStop;             /* '<S117>/MOOG State Machine' */
  uint8_T is_NormalOperation;          /* '<S117>/MOOG State Machine' */
  uint8_T is_active_c10_CAVE_MachE_dSPACE_250912;/* '<S1>/RealSimHILCycle' */
  uint8_T is_c10_CAVE_MachE_dSPACE_250912;/* '<S1>/RealSimHILCycle' */
  uint8_T is_CycleOn;                  /* '<S1>/RealSimHILCycle' */
  boolean_T IntegratorSecondOrder_DWORK1;/* '<S519>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_b;/* '<S547>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_k;/* '<S575>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_f;/* '<S603>/Integrator, Second-Order' */
  boolean_T Memory_PreviousInput_mh;   /* '<S280>/Memory' */
  boolean_T IntegratorSecondOrder_DWORK1_c;/* '<S631>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_h;/* '<S659>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_n;/* '<S687>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_a;/* '<S715>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_m;/* '<S501>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_i;/* '<S502>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_e;/* '<S503>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_DWORK1_kf;/* '<S504>/Integrator, Second-Order' */
  boolean_T PrevLimited;               /* '<S508>/Rate Limiter' */
  boolean_T PrevLimited_j;             /* '<S508>/Rate Limiter1' */
  boolean_T PrevLimited_l;             /* '<S508>/Rate Limiter2' */
  boolean_T PrevLimited_i;             /* '<S508>/Rate Limiter3' */
  boolean_T PrevLimited_e;             /* '<S109>/Rate Limiter' */
  boolean_T objisempty;                /* '<S4>/RealSimPack' */
  boolean_T objisempty_p;              /* '<S4>/RealSimInterpSpeed' */
  boolean_T objisempty_e;              /* '<S4>/RealSimDepack' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_nf_T CoreSubsys_k[1];/* '<S717>/Clutch Scalar Parameters' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T CoreSubsys_d[1];/* '<S689>/Clutch Scalar Parameters' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T CoreSubsys_e[1];/* '<S661>/Clutch Scalar Parameters' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_m1_T CoreSubsys_od[1];/* '<S633>/Clutch Scalar Parameters' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_m_T CoreSubsys_n[1];/* '<S605>/Clutch Scalar Parameters' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T CoreSubsys_cu[1];/* '<S577>/Clutch Scalar Parameters' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_o_T CoreSubsys_c[1];/* '<S549>/Clutch Scalar Parameters' */
  DW_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T CoreSubsys_h[1];/* '<S521>/Clutch Scalar Parameters' */
  DW_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough_e;/* '<S200>/Pass Through' */
  DW_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough_k;/* '<S198>/Pass Through' */
  DW_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough_i;/* '<S177>/Pass Through' */
  DW_PassThrough_CAVE_MachE_dSPACE_250912_T PassThrough;/* '<S175>/Pass Through' */
} DW_CAVE_MachE_dSPACE_250912_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE[4];         /* '<S467>/Integrator' */
  real_T Integrator_CSTATE_e;          /* '<S216>/Integrator' */
  real_T IntegratorLimited_CSTATE;     /* '<S144>/Integrator Limited' */
  real_T Integrator_CSTATE_l;          /* '<S100>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<S100>/Integrator1' */
  real_T Integrator_CSTATE_a;          /* '<S518>/Integrator' */
  real_T Integrator_CSTATE_at;         /* '<S515>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE[2];/* '<S519>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_n;          /* '<S546>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S543>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_e[2];/* '<S547>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_po;         /* '<S574>/Integrator' */
  real_T Integrator_CSTATE_ew;         /* '<S571>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_l[2];/* '<S575>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_i;          /* '<S602>/Integrator' */
  real_T Integrator_CSTATE_pq;         /* '<S599>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_l0[2];/* '<S603>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_b;          /* '<S516>/Integrator' */
  real_T Integrator_CSTATE_c;          /* '<S544>/Integrator' */
  real_T Integrator_CSTATE_ca;         /* '<S572>/Integrator' */
  real_T Integrator_CSTATE_f;          /* '<S600>/Integrator' */
  real_T Integrator1_CSTATE_h[4];      /* '<S469>/Integrator1' */
  real_T Integrator1_CSTATE_e[12];     /* '<S470>/Integrator1' */
  real_T Integrator_CSTATE_fa[2];      /* '<S384>/Integrator' */
  real_T Integrator_CSTATE_o;          /* '<S106>/Integrator' */
  real_T TransferFcn_CSTATE;           /* '<S507>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S507>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S507>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE;          /* '<S507>/Transfer Fcn3' */
  real_T Integrator_CSTATE_bf;         /* '<S227>/Integrator' */
  real_T TransferFcn3_CSTATE_f;        /* '<S259>/Transfer Fcn3' */
  real_T TransferFcn1_CSTATE_f;        /* '<S259>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_a;        /* '<S259>/Transfer Fcn2' */
  real_T TransferFcn4_CSTATE;          /* '<S259>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_n;         /* '<S259>/Transfer Fcn' */
  real_T TransferFcn9_CSTATE;          /* '<S259>/Transfer Fcn9' */
  real_T TransferFcn10_CSTATE;         /* '<S259>/Transfer Fcn10' */
  real_T TransferFcn11_CSTATE;         /* '<S259>/Transfer Fcn11' */
  real_T TransferFcn5_CSTATE;          /* '<S259>/Transfer Fcn5' */
  real_T TransferFcn6_CSTATE;          /* '<S259>/Transfer Fcn6' */
  real_T TransferFcn7_CSTATE;          /* '<S259>/Transfer Fcn7' */
  real_T TransferFcn8_CSTATE;          /* '<S259>/Transfer Fcn8' */
  real_T Integrator_CSTATE_ab;         /* '<S630>/Integrator' */
  real_T Integrator_CSTATE_fy;         /* '<S627>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_p[2];/* '<S631>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_ln;         /* '<S628>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S658>/Integrator' */
  real_T Integrator_CSTATE_d;          /* '<S655>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_h[2];/* '<S659>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_ng;         /* '<S656>/Integrator' */
  real_T Integrator_CSTATE_my;         /* '<S686>/Integrator' */
  real_T Integrator_CSTATE_g;          /* '<S683>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_eb[2];/* '<S687>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_k;          /* '<S684>/Integrator' */
  real_T Integrator_CSTATE_n2;         /* '<S714>/Integrator' */
  real_T Integrator_CSTATE_ia;         /* '<S711>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_ew[2];/* '<S715>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_cw;         /* '<S712>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_f[2];/* '<S501>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_pv[2];/* '<S502>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_m[2];/* '<S503>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_k[2];/* '<S504>/Integrator, Second-Order' */
  real_T TransferFcn_CSTATE_j;         /* '<S508>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_a;        /* '<S508>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_d;        /* '<S508>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_i;        /* '<S508>/Transfer Fcn3' */
  real_T Integrator_CSTATE_el;         /* '<S129>/Integrator' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T CoreSubsys_k[1];/* '<S721>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_i_T CoreSubsys_d[1];/* '<S693>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_o_T CoreSubsys_e[1];/* '<S665>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T CoreSubsys_od[1];/* '<S637>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_nc_T CoreSubsys_n[1];/* '<S609>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_g_T CoreSubsys_cu[1];/* '<S581>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_m_T CoreSubsys_c[1];/* '<S553>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T CoreSubsys_h[1];/* '<S525>/CoreSubsys' */
  X_CoreSubsys_CAVE_MachE_dSPACE_250912_k_T CoreSubsys_g[1];/* '<S282>/CoreSubsys' */
  real_T Limits5050_CSTATE;            /* '<S157>/Limits [-50,50]' */
  real_T LowpassFilter_CSTATE;         /* '<S157>/Lowpass Filter' */
  real_T Integrator_CSTATE_cd;         /* '<S180>/Integrator' */
  real_T Integrator1_CSTATE_f;         /* '<S172>/Integrator1' */
  real_T Integrator1_CSTATE_b;         /* '<S193>/Integrator1' */
  real_T LowpassFilter_CSTATE_p;       /* '<S155>/Lowpass Filter' */
  real_T Integrator2_CSTATE;           /* '<S181>/Integrator2' */
  real_T LowpassFilter_CSTATE_k;       /* '<S156>/Lowpass Filter' */
  real_T Integrator2_CSTATE_e;         /* '<S204>/Integrator2' */
} X_CAVE_MachE_dSPACE_250912_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE[4];         /* '<S467>/Integrator' */
  real_T Integrator_CSTATE_e;          /* '<S216>/Integrator' */
  real_T IntegratorLimited_CSTATE;     /* '<S144>/Integrator Limited' */
  real_T Integrator_CSTATE_l;          /* '<S100>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<S100>/Integrator1' */
  real_T Integrator_CSTATE_a;          /* '<S518>/Integrator' */
  real_T Integrator_CSTATE_at;         /* '<S515>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE[2];/* '<S519>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_n;          /* '<S546>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S543>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_e[2];/* '<S547>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_po;         /* '<S574>/Integrator' */
  real_T Integrator_CSTATE_ew;         /* '<S571>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_l[2];/* '<S575>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_i;          /* '<S602>/Integrator' */
  real_T Integrator_CSTATE_pq;         /* '<S599>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_l0[2];/* '<S603>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_b;          /* '<S516>/Integrator' */
  real_T Integrator_CSTATE_c;          /* '<S544>/Integrator' */
  real_T Integrator_CSTATE_ca;         /* '<S572>/Integrator' */
  real_T Integrator_CSTATE_f;          /* '<S600>/Integrator' */
  real_T Integrator1_CSTATE_h[4];      /* '<S469>/Integrator1' */
  real_T Integrator1_CSTATE_e[12];     /* '<S470>/Integrator1' */
  real_T Integrator_CSTATE_fa[2];      /* '<S384>/Integrator' */
  real_T Integrator_CSTATE_o;          /* '<S106>/Integrator' */
  real_T TransferFcn_CSTATE;           /* '<S507>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S507>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S507>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE;          /* '<S507>/Transfer Fcn3' */
  real_T Integrator_CSTATE_bf;         /* '<S227>/Integrator' */
  real_T TransferFcn3_CSTATE_f;        /* '<S259>/Transfer Fcn3' */
  real_T TransferFcn1_CSTATE_f;        /* '<S259>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_a;        /* '<S259>/Transfer Fcn2' */
  real_T TransferFcn4_CSTATE;          /* '<S259>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_n;         /* '<S259>/Transfer Fcn' */
  real_T TransferFcn9_CSTATE;          /* '<S259>/Transfer Fcn9' */
  real_T TransferFcn10_CSTATE;         /* '<S259>/Transfer Fcn10' */
  real_T TransferFcn11_CSTATE;         /* '<S259>/Transfer Fcn11' */
  real_T TransferFcn5_CSTATE;          /* '<S259>/Transfer Fcn5' */
  real_T TransferFcn6_CSTATE;          /* '<S259>/Transfer Fcn6' */
  real_T TransferFcn7_CSTATE;          /* '<S259>/Transfer Fcn7' */
  real_T TransferFcn8_CSTATE;          /* '<S259>/Transfer Fcn8' */
  real_T Integrator_CSTATE_ab;         /* '<S630>/Integrator' */
  real_T Integrator_CSTATE_fy;         /* '<S627>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_p[2];/* '<S631>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_ln;         /* '<S628>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S658>/Integrator' */
  real_T Integrator_CSTATE_d;          /* '<S655>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_h[2];/* '<S659>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_ng;         /* '<S656>/Integrator' */
  real_T Integrator_CSTATE_my;         /* '<S686>/Integrator' */
  real_T Integrator_CSTATE_g;          /* '<S683>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_eb[2];/* '<S687>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_k;          /* '<S684>/Integrator' */
  real_T Integrator_CSTATE_n2;         /* '<S714>/Integrator' */
  real_T Integrator_CSTATE_ia;         /* '<S711>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_ew[2];/* '<S715>/Integrator, Second-Order' */
  real_T Integrator_CSTATE_cw;         /* '<S712>/Integrator' */
  real_T IntegratorSecondOrder_CSTATE_f[2];/* '<S501>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_pv[2];/* '<S502>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_m[2];/* '<S503>/Integrator, Second-Order' */
  real_T IntegratorSecondOrder_CSTATE_k[2];/* '<S504>/Integrator, Second-Order' */
  real_T TransferFcn_CSTATE_j;         /* '<S508>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_a;        /* '<S508>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_d;        /* '<S508>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_i;        /* '<S508>/Transfer Fcn3' */
  real_T Integrator_CSTATE_el;         /* '<S129>/Integrator' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T CoreSubsys_k[1];/* '<S721>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_p1_T CoreSubsys_d[1];/* '<S693>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T CoreSubsys_e[1];/* '<S665>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_m_T CoreSubsys_od[1];/* '<S637>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_o_T CoreSubsys_n[1];/* '<S609>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T CoreSubsys_cu[1];/* '<S581>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T CoreSubsys_c[1];/* '<S553>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T CoreSubsys_h[1];/* '<S525>/CoreSubsys' */
  XDot_CoreSubsys_CAVE_MachE_dSPACE_250912_p_T CoreSubsys_g[1];/* '<S282>/CoreSubsys' */
  real_T Limits5050_CSTATE;            /* '<S157>/Limits [-50,50]' */
  real_T LowpassFilter_CSTATE;         /* '<S157>/Lowpass Filter' */
  real_T Integrator_CSTATE_cd;         /* '<S180>/Integrator' */
  real_T Integrator1_CSTATE_f;         /* '<S172>/Integrator1' */
  real_T Integrator1_CSTATE_b;         /* '<S193>/Integrator1' */
  real_T LowpassFilter_CSTATE_p;       /* '<S155>/Lowpass Filter' */
  real_T Integrator2_CSTATE;           /* '<S181>/Integrator2' */
  real_T LowpassFilter_CSTATE_k;       /* '<S156>/Lowpass Filter' */
  real_T Integrator2_CSTATE_e;         /* '<S204>/Integrator2' */
} XDot_CAVE_MachE_dSPACE_250912_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE[4];      /* '<S467>/Integrator' */
  boolean_T Integrator_CSTATE_e;       /* '<S216>/Integrator' */
  boolean_T IntegratorLimited_CSTATE;  /* '<S144>/Integrator Limited' */
  boolean_T Integrator_CSTATE_l;       /* '<S100>/Integrator' */
  boolean_T Integrator1_CSTATE;        /* '<S100>/Integrator1' */
  boolean_T Integrator_CSTATE_a;       /* '<S518>/Integrator' */
  boolean_T Integrator_CSTATE_at;      /* '<S515>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE[2];/* '<S519>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_n;       /* '<S546>/Integrator' */
  boolean_T Integrator_CSTATE_p;       /* '<S543>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_e[2];/* '<S547>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_po;      /* '<S574>/Integrator' */
  boolean_T Integrator_CSTATE_ew;      /* '<S571>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_l[2];/* '<S575>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_i;       /* '<S602>/Integrator' */
  boolean_T Integrator_CSTATE_pq;      /* '<S599>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_l0[2];/* '<S603>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_b;       /* '<S516>/Integrator' */
  boolean_T Integrator_CSTATE_c;       /* '<S544>/Integrator' */
  boolean_T Integrator_CSTATE_ca;      /* '<S572>/Integrator' */
  boolean_T Integrator_CSTATE_f;       /* '<S600>/Integrator' */
  boolean_T Integrator1_CSTATE_h[4];   /* '<S469>/Integrator1' */
  boolean_T Integrator1_CSTATE_e[12];  /* '<S470>/Integrator1' */
  boolean_T Integrator_CSTATE_fa[2];   /* '<S384>/Integrator' */
  boolean_T Integrator_CSTATE_o;       /* '<S106>/Integrator' */
  boolean_T TransferFcn_CSTATE;        /* '<S507>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S507>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE;       /* '<S507>/Transfer Fcn2' */
  boolean_T TransferFcn3_CSTATE;       /* '<S507>/Transfer Fcn3' */
  boolean_T Integrator_CSTATE_bf;      /* '<S227>/Integrator' */
  boolean_T TransferFcn3_CSTATE_f;     /* '<S259>/Transfer Fcn3' */
  boolean_T TransferFcn1_CSTATE_f;     /* '<S259>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE_a;     /* '<S259>/Transfer Fcn2' */
  boolean_T TransferFcn4_CSTATE;       /* '<S259>/Transfer Fcn4' */
  boolean_T TransferFcn_CSTATE_n;      /* '<S259>/Transfer Fcn' */
  boolean_T TransferFcn9_CSTATE;       /* '<S259>/Transfer Fcn9' */
  boolean_T TransferFcn10_CSTATE;      /* '<S259>/Transfer Fcn10' */
  boolean_T TransferFcn11_CSTATE;      /* '<S259>/Transfer Fcn11' */
  boolean_T TransferFcn5_CSTATE;       /* '<S259>/Transfer Fcn5' */
  boolean_T TransferFcn6_CSTATE;       /* '<S259>/Transfer Fcn6' */
  boolean_T TransferFcn7_CSTATE;       /* '<S259>/Transfer Fcn7' */
  boolean_T TransferFcn8_CSTATE;       /* '<S259>/Transfer Fcn8' */
  boolean_T Integrator_CSTATE_ab;      /* '<S630>/Integrator' */
  boolean_T Integrator_CSTATE_fy;      /* '<S627>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_p[2];/* '<S631>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_ln;      /* '<S628>/Integrator' */
  boolean_T Integrator_CSTATE_m;       /* '<S658>/Integrator' */
  boolean_T Integrator_CSTATE_d;       /* '<S655>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_h[2];/* '<S659>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_ng;      /* '<S656>/Integrator' */
  boolean_T Integrator_CSTATE_my;      /* '<S686>/Integrator' */
  boolean_T Integrator_CSTATE_g;       /* '<S683>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_eb[2];/* '<S687>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_k;       /* '<S684>/Integrator' */
  boolean_T Integrator_CSTATE_n2;      /* '<S714>/Integrator' */
  boolean_T Integrator_CSTATE_ia;      /* '<S711>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_ew[2];/* '<S715>/Integrator, Second-Order' */
  boolean_T Integrator_CSTATE_cw;      /* '<S712>/Integrator' */
  boolean_T IntegratorSecondOrder_CSTATE_f[2];/* '<S501>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_pv[2];/* '<S502>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_m[2];/* '<S503>/Integrator, Second-Order' */
  boolean_T IntegratorSecondOrder_CSTATE_k[2];/* '<S504>/Integrator, Second-Order' */
  boolean_T TransferFcn_CSTATE_j;      /* '<S508>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE_a;     /* '<S508>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE_d;     /* '<S508>/Transfer Fcn2' */
  boolean_T TransferFcn3_CSTATE_i;     /* '<S508>/Transfer Fcn3' */
  boolean_T Integrator_CSTATE_el;      /* '<S129>/Integrator' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_pq_T CoreSubsys_k[1];/* '<S721>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_nd_T CoreSubsys_d[1];/* '<S693>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_p_T CoreSubsys_e[1];/* '<S665>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T CoreSubsys_od[1];/* '<S637>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_js_T CoreSubsys_n[1];/* '<S609>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_g_T CoreSubsys_cu[1];/* '<S581>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T CoreSubsys_c[1];/* '<S553>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_i_T CoreSubsys_h[1];/* '<S525>/CoreSubsys' */
  XDis_CoreSubsys_CAVE_MachE_dSPACE_250912_n_T CoreSubsys_g[1];/* '<S282>/CoreSubsys' */
  boolean_T Limits5050_CSTATE;         /* '<S157>/Limits [-50,50]' */
  boolean_T LowpassFilter_CSTATE;      /* '<S157>/Lowpass Filter' */
  boolean_T Integrator_CSTATE_cd;      /* '<S180>/Integrator' */
  boolean_T Integrator1_CSTATE_f;      /* '<S172>/Integrator1' */
  boolean_T Integrator1_CSTATE_b;      /* '<S193>/Integrator1' */
  boolean_T LowpassFilter_CSTATE_p;    /* '<S155>/Lowpass Filter' */
  boolean_T Integrator2_CSTATE;        /* '<S181>/Integrator2' */
  boolean_T LowpassFilter_CSTATE_k;    /* '<S156>/Lowpass Filter' */
  boolean_T Integrator2_CSTATE_e;      /* '<S204>/Integrator2' */
} XDis_CAVE_MachE_dSPACE_250912_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Integrator_Reset_ZCE;     /* '<S129>/Integrator' */
  ZCSigState Limits5050_Reset_ZCE;     /* '<S157>/Limits [-50,50]' */
} PrevZCX_CAVE_MachE_dSPACE_250912_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Parameters for system: '<S216>/If Action Subsystem1' */
struct P_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T_ {
  real_T Interpolatedzerocrossing_tableData[2];/* Expression: [-1 1]
                                                * Referenced by: '<S220>/Interpolated zero-crossing'
                                                */
  real_T Interpolatedzerocrossing_bp01Data[2];/* Expression: [-1 1]
                                               * Referenced by: '<S220>/Interpolated zero-crossing'
                                               */
};

/* Parameters for system: '<S274>/For each axle and track calculate suspension and wheel positions and velocities' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_T_ {
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S281>/Constant1'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S281>/Gain'
                                        */
  real_T Constant1_Value_g;            /* Expression: 1
                                        * Referenced by: '<S285>/Constant1'
                                        */
  real_T DCMStaringRow_Gain;           /* Expression: 3
                                        * Referenced by: '<S285>/DCM Staring Row'
                                        */
};

/* Parameters for system: '<S274>/For each axle calculate axle cg positions and velocities' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_f_T_ {
  real_T SelectAxleMassByAxle_AxleNums;
                                /* Mask Parameter: SelectAxleMassByAxle_AxleNums
                                 * Referenced by: '<S294>/Axle Numbers'
                                 */
  real_T SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums;
            /* Mask Parameter: SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums
             * Referenced by: '<S295>/Axle Numbers'
             */
  real_T SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums;
            /* Mask Parameter: SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums
             * Referenced by: '<S292>/Axle Numbers'
             */
  real_T SelectAxleMassByAxle_AxleNums_l;
                              /* Mask Parameter: SelectAxleMassByAxle_AxleNums_l
                               * Referenced by: '<S291>/Axle Numbers'
                               */
  real_T SuspensionMomentDirectionOnSolidAxle_Gain;/* Expression: -1
                                                    * Referenced by: '<S288>/Suspension Moment Direction On Solid Axle'
                                                    */
  real_T SuspensionForceDirectionOnSolidAxle_Gain;/* Expression: -1
                                                   * Referenced by: '<S288>/Suspension Force Direction On Solid Axle'
                                                   */
  real_T Trackcoordinatesinaxlebodyframe1_Value[3];/* Expression: [1 0 0]
                                                    * Referenced by: '<S293>/Track coordinates in axle body frame1'
                                                    */
  real_T Trackcoordinatesinaxlebodyframe2_Value;/* Expression: 0
                                                 * Referenced by: '<S293>/Track coordinates in axle body frame2'
                                                 */
  real_T _IC;                          /* Expression: 0
                                        * Referenced by: '<S293>/ '
                                        */
  real_T _UpperSat;                    /* Expression: 0.99*pi/2
                                        * Referenced by: '<S293>/ '
                                        */
  real_T _LowerSat;                    /* Expression: -0.99*pi/2
                                        * Referenced by: '<S293>/ '
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S293>/Gain'
                                        */
  real_T cgcoordinates_IC;             /* Expression: 0
                                        * Referenced by: '<S289>/cg coordinates'
                                        */
  real_T Fy1_Value;                    /* Expression: 0
                                        * Referenced by: '<S287>/Fy1'
                                        */
  real_T Vz_IC;                        /* Expression: 0
                                        * Referenced by: '<S287>/Vz'
                                        */
  real_T Vy1_IC;                       /* Expression: 0
                                        * Referenced by: '<S287>/Vy1'
                                        */
  real_T Vy1_UpperSat;                 /* Expression: 0.99*pi/2
                                        * Referenced by: '<S287>/Vy1'
                                        */
  real_T Vy1_LowerSat;                 /* Expression: -0.99*pi/2
                                        * Referenced by: '<S287>/Vy1'
                                        */
  real_T Fy_Value;                     /* Expression: 0
                                        * Referenced by: '<S287>/Fy'
                                        */
  real_T Vy_IC;                        /* Expression: 0
                                        * Referenced by: '<S287>/Vy'
                                        */
  real_T gEarth_Value;                 /* Expression: 9.807
                                        * Referenced by: '<S287>/g (Earth)'
                                        */
};

/* Parameters for system: '<S314>/Min stop reached' */
struct P_Minstopreached_CAVE_MachE_dSPACE_250912_T_ {
  real_T Gain5_Gain;                   /* Expression: 4
                                        * Referenced by: '<S320>/Gain5'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<S320>/Gain4'
                                        */
  real_T Constant_Value;               /* Expression: 3
                                        * Referenced by: '<S320>/Constant'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 4
                                        * Referenced by: '<S320>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S320>/Saturation'
                                        */
  real_T Gain_Gain;                    /* Expression: 3
                                        * Referenced by: '<S320>/Gain'
                                        */
  real_T LowerHardStopBlendMult_tableData[3];/* Expression: [1 1 0]
                                              * Referenced by: '<S320>/Lower Hard Stop Blend Mult'
                                              */
  real_T LowerHardStopBlendMult_bp01Data[3];/* Expression: [-0.02 -0.01 0]
                                             * Referenced by: '<S320>/Lower Hard Stop Blend Mult'
                                             */
};

/* Parameters for system: '<S314>/Max stop reached' */
struct P_Maxstopreached_CAVE_MachE_dSPACE_250912_T_ {
  real_T Gain5_Gain;                   /* Expression: 4
                                        * Referenced by: '<S319>/Gain5'
                                        */
  real_T Gain4_Gain;                   /* Expression: 0.05
                                        * Referenced by: '<S319>/Gain4'
                                        */
  real_T Constant_Value;               /* Expression: 3
                                        * Referenced by: '<S319>/Constant'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 4
                                        * Referenced by: '<S319>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S319>/Saturation'
                                        */
  real_T Gain_Gain;                    /* Expression: 3
                                        * Referenced by: '<S319>/Gain'
                                        */
  real_T UpperHardStopBlendMult_tableData[3];/* Expression: [0 1 1]
                                              * Referenced by: '<S319>/Upper Hard Stop Blend Mult'
                                              */
  real_T UpperHardStopBlendMult_bp01Data[3];/* Expression: [0 0.01 0.02]
                                             * Referenced by: '<S319>/Upper Hard Stop Blend Mult'
                                             */
};

/* Parameters for system: '<S274>/For each track and axle combination calculate suspension forces and moments' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T_ {
  real_T SelectCamberSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCamberSteeringCenter_AxleNums
                           * Referenced by: '<S306>/Axle Numbers'
                           */
  real_T SelectCamberHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCamberHeightSlope_AxleNums
                              * Referenced by: '<S305>/Axle Numbers'
                              */
  real_T Constrainedspringdampercombination_AxleNums;
                  /* Mask Parameter: Constrainedspringdampercombination_AxleNums
                   * Referenced by:
                   *   '<S315>/Axle Numbers'
                   *   '<S316>/Axle Numbers'
                   *   '<S317>/Axle Numbers'
                   *   '<S318>/Axle Numbers'
                   */
  real_T SelectCasterSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCasterSteeringCenter_AxleNums
                           * Referenced by: '<S308>/Axle Numbers'
                           */
  real_T SelectCasterHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCasterHeightSlope_AxleNums
                              * Referenced by: '<S307>/Axle Numbers'
                              */
  real_T SelectToeSteeringCenter_AxleNums;
                             /* Mask Parameter: SelectToeSteeringCenter_AxleNums
                              * Referenced by: '<S310>/Axle Numbers'
                              */
  real_T SelectRollSteerSlope_AxleNums;
                                /* Mask Parameter: SelectRollSteerSlope_AxleNums
                                 * Referenced by: '<S309>/Axle Numbers'
                                 */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S321>/Constant'
                                        */
  real_T Constant_Value_k;             /* Expression: 0
                                        * Referenced by: '<S297>/Constant'
                                        */
  real_T VehicleVehicleWheelOffset3_Value[2];/* Expression: [0 cumsum(NumWhlsByAxl)]
                                              * Referenced by: '<S296>/Vehicle Vehicle Wheel Offset3'
                                              */
  real_T VehicleVehicleWheelOffset1_Value;/* Expression: StrgEnByAxl
                                           * Referenced by: '<S297>/Vehicle Vehicle Wheel Offset1'
                                           */
  real_T HeightSignConvention_Gain;    /* Expression: -1
                                        * Referenced by: '<S312>/Height Sign Convention'
                                        */
  real_T VehicleVehicleWheelOffset3_Value_c;/* Expression: NumWhlsByAxl
                                             * Referenced by: '<S304>/Vehicle Vehicle Wheel Offset3'
                                             */
  real_T Constant_Value_kq;            /* Expression: 1
                                        * Referenced by: '<S304>/Constant'
                                        */
  real_T Gain_Gain;                    /* Expression: 1/2
                                        * Referenced by: '<S304>/Gain'
                                        */
  real_T VehicleVehicleWheelOffset3_Value_cw;/* Expression: NumWhlsByAxl
                                              * Referenced by: '<S303>/Vehicle Vehicle Wheel Offset3'
                                              */
  real_T Constant_Value_a;             /* Expression: 1
                                        * Referenced by: '<S303>/Constant'
                                        */
  real_T Gain_Gain_l;                  /* Expression: 1/2
                                        * Referenced by: '<S303>/Gain'
                                        */
  real_T VehicleForceSign_Gain;        /* Expression: -1
                                        * Referenced by: '<S311>/Vehicle Force Sign'
                                        */
  real_T Signconvention_Gain;          /* Expression: -1
                                        * Referenced by: '<S311>/Sign convention'
                                        */
  real_T Constant_Value_l;             /* Expression: 0
                                        * Referenced by: '<S299>/Constant'
                                        */
  real_T VehicleVehicleWheelOffset3_Value_cr;/* Expression: NumWhlsByAxl
                                              * Referenced by: '<S302>/Vehicle Vehicle Wheel Offset3'
                                              */
  real_T Constant_Value_c;             /* Expression: 1
                                        * Referenced by: '<S302>/Constant'
                                        */
  real_T Gain_Gain_g;                  /* Expression: 1/2
                                        * Referenced by: '<S302>/Gain'
                                        */
  P_Maxstopreached_CAVE_MachE_dSPACE_250912_T Maxstopreached;/* '<S314>/Max stop reached' */
  P_Minstopreached_CAVE_MachE_dSPACE_250912_T Minstopreached;/* '<S314>/Min stop reached' */
};

/* Parameters for system: '<S324>/For Each Axle With Anti-Sway' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T_ {
  real_T AntiSwayArmRadiusByAxle_AxleNums;
                             /* Mask Parameter: AntiSwayArmRadiusByAxle_AxleNums
                              * Referenced by: '<S328>/Axle Numbers'
                              */
  real_T AntiSwayArmNeutralAngleByAxle_AxleNums;
                       /* Mask Parameter: AntiSwayArmNeutralAngleByAxle_AxleNums
                        * Referenced by: '<S327>/Axle Numbers'
                        */
  real_T AntiSwayBarTorsionSpringConstantByAxle_AxleNums;
              /* Mask Parameter: AntiSwayBarTorsionSpringConstantByAxle_AxleNums
               * Referenced by: '<S329>/Axle Numbers'
               */
  real_T VehicleVehicleWheelOffset3_Value[2];/* Expression: [0 cumsum(NumWhlsByAxl)]
                                              * Referenced by: '<S325>/Vehicle Vehicle Wheel Offset3'
                                              */
  real_T Constant_Value[2];            /* Expression: 1:2
                                        * Referenced by: '<S325>/Constant'
                                        */
  real_T AngleTangentLimit_tableData[2];/* Expression: [-1 1]
                                         * Referenced by: '<S326>/Angle Tangent Limit'
                                         */
  real_T AngleTangentLimit_bp01Data[2];/* Expression: [-1 1]
                                        * Referenced by: '<S326>/Angle Tangent Limit'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S326>/Gain'
                                        */
};

/* Parameters for system: '<S279>/For each track and axle combination calculate suspension forces and moments' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T_ {
  real_T SelectCamberSteeringSlope_AxleNums;
                           /* Mask Parameter: SelectCamberSteeringSlope_AxleNums
                            * Referenced by: '<S341>/Axle Numbers'
                            */
  real_T SelectCamberSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCamberSteeringCenter_AxleNums
                           * Referenced by: '<S340>/Axle Numbers'
                           */
  real_T SelectCamberHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCamberHeightSlope_AxleNums
                              * Referenced by: '<S339>/Axle Numbers'
                              */
  real_T SteeringHeightSlopeBySteeredAxle_AxleNums;
                    /* Mask Parameter: SteeringHeightSlopeBySteeredAxle_AxleNums
                     * Referenced by: '<S359>/Axle Numbers'
                     */
  real_T Constrainedspringdampercombination_AxleNums;
                  /* Mask Parameter: Constrainedspringdampercombination_AxleNums
                   * Referenced by:
                   *   '<S352>/Axle Numbers'
                   *   '<S353>/Axle Numbers'
                   *   '<S354>/Axle Numbers'
                   *   '<S355>/Axle Numbers'
                   */
  real_T SelectCasterSteeringSlope_AxleNums;
                           /* Mask Parameter: SelectCasterSteeringSlope_AxleNums
                            * Referenced by: '<S344>/Axle Numbers'
                            */
  real_T SelectCasterSteeringCenter_AxleNums;
                          /* Mask Parameter: SelectCasterSteeringCenter_AxleNums
                           * Referenced by: '<S343>/Axle Numbers'
                           */
  real_T SelectCasterHeightSlope_AxleNums;
                             /* Mask Parameter: SelectCasterHeightSlope_AxleNums
                              * Referenced by: '<S342>/Axle Numbers'
                              */
  real_T SelectToeSteeringSlope_AxleNums;
                              /* Mask Parameter: SelectToeSteeringSlope_AxleNums
                               * Referenced by: '<S347>/Axle Numbers'
                               */
  real_T SelectToeSteeringCenter_AxleNums;
                             /* Mask Parameter: SelectToeSteeringCenter_AxleNums
                              * Referenced by: '<S346>/Axle Numbers'
                              */
  real_T SelectRollSteerSlope_AxleNums;
                                /* Mask Parameter: SelectRollSteerSlope_AxleNums
                                 * Referenced by: '<S345>/Axle Numbers'
                                 */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S331>/Constant'
                                        */
  real_T VehicleVehicleWheelOffset3_Value[2];/* Expression: [0 cumsum(NumWhlsByAxl)]
                                              * Referenced by: '<S330>/Vehicle Vehicle Wheel Offset3'
                                              */
  real_T VehicleVehicleWheelOffset1_Value;/* Expression: StrgEnByAxl
                                           * Referenced by: '<S331>/Vehicle Vehicle Wheel Offset1'
                                           */
  real_T HeightSignConvention_Gain;    /* Expression: -1
                                        * Referenced by: '<S349>/Height Sign Convention'
                                        */
  real_T VehicleVehicleWheelOffset3_Value_g;/* Expression: NumWhlsByAxl
                                             * Referenced by: '<S338>/Vehicle Vehicle Wheel Offset3'
                                             */
  real_T Constant_Value_g;             /* Expression: 1
                                        * Referenced by: '<S338>/Constant'
                                        */
  real_T Gain_Gain;                    /* Expression: 1/2
                                        * Referenced by: '<S338>/Gain'
                                        */
  real_T VehicleVehicleWheelOffset3_Value_l;/* Expression: NumWhlsByAxl
                                             * Referenced by: '<S337>/Vehicle Vehicle Wheel Offset3'
                                             */
  real_T Constant_Value_a;             /* Expression: 1
                                        * Referenced by: '<S337>/Constant'
                                        */
  real_T Gain_Gain_e;                  /* Expression: 1/2
                                        * Referenced by: '<S337>/Gain'
                                        */
  real_T VehicleForceSign_Gain;        /* Expression: -1
                                        * Referenced by: '<S348>/Vehicle Force Sign'
                                        */
  real_T Signconvention_Gain;          /* Expression: -1
                                        * Referenced by: '<S348>/Sign convention'
                                        */
  real_T Constant_Value_m;             /* Expression: 0
                                        * Referenced by: '<S333>/Constant'
                                        */
  real_T VehicleVehicleWheelOffset3_Value_o;/* Expression: NumWhlsByAxl
                                             * Referenced by: '<S336>/Vehicle Vehicle Wheel Offset3'
                                             */
  real_T Constant_Value_d;             /* Expression: 1
                                        * Referenced by: '<S336>/Constant'
                                        */
  real_T Gain_Gain_o;                  /* Expression: 1/2
                                        * Referenced by: '<S336>/Gain'
                                        */
  P_Maxstopreached_CAVE_MachE_dSPACE_250912_T Maxstopreached;/* '<S351>/Max stop reached' */
  P_Minstopreached_CAVE_MachE_dSPACE_250912_T Minstopreached;/* '<S351>/Min stop reached' */
};

/* Parameters for system: '<S512>/Magic Tire Const Input' */
struct P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T_ {
  real_T MagicTireConstInput_vdynMF[279];/* Expression: vdynMF
                                          * Referenced by: '<S512>/Magic Tire Const Input'
                                          */
};

/* Parameters for system: '<S526>/detectSlip' */
struct P_detectSlip_CAVE_MachE_dSPACE_250912_T_ {
  boolean_T yn_Y0;                     /* Computed Parameter: yn_Y0
                                        * Referenced by: '<S530>/yn'
                                        */
};

/* Parameters for system: '<S526>/detectLockup' */
struct P_detectLockup_CAVE_MachE_dSPACE_250912_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S529>/Constant'
                                        */
  boolean_T yn_Y0;                     /* Computed Parameter: yn_Y0
                                        * Referenced by: '<S529>/yn'
                                        */
  boolean_T UnitDelay_InitialCondition;
                               /* Computed Parameter: UnitDelay_InitialCondition
                                * Referenced by: '<S534>/Unit Delay'
                                */
  boolean_T CombinatorialLogic_table[8];
                                 /* Computed Parameter: CombinatorialLogic_table
                                  * Referenced by: '<S534>/Combinatorial  Logic'
                                  */
};

/* Parameters for system: '<S526>/Locked' */
struct P_Locked_CAVE_MachE_dSPACE_250912_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S527>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S527>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<S527>/Constant2'
                                        */
};

/* Parameters for system: '<S526>/Slipping' */
struct P_Slipping_CAVE_MachE_dSPACE_250912_T_ {
  real_T u_Gain;                       /* Expression: -4
                                        * Referenced by: '<S528>/-4'
                                        */
};

/* Parameters for system: '<S525>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S526>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S526>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S526>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S526>/detectSlip' */
};

/* Parameters for system: '<S521>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S525>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_T sf_Clutch;/* '<S525>/Clutch' */
};

/* Parameters for system: '<S553>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_i_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S554>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S554>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S554>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S554>/detectSlip' */
};

/* Parameters for system: '<S549>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_ls_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S553>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_i_T sf_Clutch;/* '<S553>/Clutch' */
};

/* Parameters for system: '<S581>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_a_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S582>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S582>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S582>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S582>/detectSlip' */
};

/* Parameters for system: '<S577>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S581>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_a_T sf_Clutch;/* '<S581>/Clutch' */
};

/* Parameters for system: '<S609>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_l_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S610>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S610>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S610>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S610>/detectSlip' */
};

/* Parameters for system: '<S605>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S609>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_l_T sf_Clutch;/* '<S609>/Clutch' */
};

/* Parameters for system: '<S624>/Magic Tire Const Input' */
struct P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_i_T_ {
  real_T MagicTireConstInput_vdynMF[279];/* Expression: vdynMF
                                          * Referenced by: '<S624>/Magic Tire Const Input'
                                          */
};

/* Parameters for system: '<S637>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_h_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S638>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S638>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S638>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S638>/detectSlip' */
};

/* Parameters for system: '<S633>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_jo_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S637>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_h_T sf_Clutch;/* '<S637>/Clutch' */
};

/* Parameters for system: '<S665>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_p_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S666>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S666>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S666>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S666>/detectSlip' */
};

/* Parameters for system: '<S661>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_dg_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S665>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_p_T sf_Clutch;/* '<S665>/Clutch' */
};

/* Parameters for system: '<S693>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_g_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S694>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S694>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S694>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S694>/detectSlip' */
};

/* Parameters for system: '<S689>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S693>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_g_T sf_Clutch;/* '<S693>/Clutch' */
};

/* Parameters for system: '<S721>/Clutch' */
struct P_Clutch_CAVE_MachE_dSPACE_250912_pj_T_ {
  P_Slipping_CAVE_MachE_dSPACE_250912_T Slipping;/* '<S722>/Slipping' */
  P_Locked_CAVE_MachE_dSPACE_250912_T Locked;/* '<S722>/Locked' */
  P_detectLockup_CAVE_MachE_dSPACE_250912_T detectLockup;/* '<S722>/detectLockup' */
  P_detectSlip_CAVE_MachE_dSPACE_250912_T detectSlip;/* '<S722>/detectSlip' */
};

/* Parameters for system: '<S717>/Clutch Scalar Parameters' */
struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_f4_T_ {
  real_T Clutch_OmegaTol;              /* Mask Parameter: Clutch_OmegaTol
                                        * Referenced by: '<S721>/Clutch'
                                        */
  P_Clutch_CAVE_MachE_dSPACE_250912_pj_T sf_Clutch;/* '<S721>/Clutch' */
};

/* Parameters (default storage) */
struct P_CAVE_MachE_dSPACE_250912_T_ {
  struct_H6OzwuYMZP7H9bFCBCIB9C US06;  /* Variable: US06
                                        * Referenced by:
                                        *   '<Root>/US1'
                                        *   '<S109>/US1'
                                        */
  struct_RLXElOtgrzvJMUIzjpiLaD RealSimPara;/* Variable: RealSimPara
                                             * Referenced by:
                                             *   '<S1>/Constant'
                                             *   '<S4>/Constant3'
                                             *   '<S4>/RealSimInterpSpeed'
                                             */
  struct_m2VjwNiXoluKspK4Fr7zNG InitStates[34];/* Variable: InitStates
                                                * Referenced by: '<S140>/Constant1'
                                                */
  real_T BattChargeLimitMaxPwr;        /* Variable: BattChargeLimitMaxPwr
                                        * Referenced by: '<S240>/MaxChrg'
                                        */
  real_T BattChargeLimitSocBpts[12];   /* Variable: BattChargeLimitSocBpts
                                        * Referenced by:
                                        *   '<S240>/ChrgLmt'
                                        *   '<S242>/ChrgLmt'
                                        */
  real_T BattChargeLimitTbl[12];       /* Variable: BattChargeLimitTbl
                                        * Referenced by:
                                        *   '<S240>/ChrgLmt'
                                        *   '<S242>/ChrgLmt'
                                        */
  real_T BattChrgCapcty;               /* Variable: BattChrgCapcty
                                        * Referenced by:
                                        *   '<S140>/Constant1'
                                        *   '<S144>/Constant1'
                                        *   '<S144>/Integrator Limited'
                                        *   '<S144>/Switch'
                                        *   '<S145>/Constant1'
                                        */
  real_T BattDischargeLimitSocBpts[11];/* Variable: BattDischargeLimitSocBpts
                                        * Referenced by: '<S240>/DischrgLmt'
                                        */
  real_T BattDischargeLimitTbl[11];    /* Variable: BattDischargeLimitTbl
                                        * Referenced by: '<S240>/DischrgLmt'
                                        */
  real_T BattDischargeMaxPwr;          /* Variable: BattDischargeMaxPwr
                                        * Referenced by: '<S240>/MaxDischrg'
                                        */
  real_T BattNumCellsParallel;         /* Variable: BattNumCellsParallel
                                        * Referenced by:
                                        *   '<S144>/Gain1'
                                        *   '<S146>/Gain2'
                                        *   '<S146>/Gain4'
                                        */
  real_T BattNumCellsSeries;           /* Variable: BattNumCellsSeries
                                        * Referenced by:
                                        *   '<S146>/Gain1'
                                        *   '<S146>/Gain3'
                                        */
  real_T BattOpenVoltCapBpts[11];      /* Variable: BattOpenVoltCapBpts
                                        * Referenced by: '<S146>/Em'
                                        */
  real_T BattOpenVoltTbl[11];          /* Variable: BattOpenVoltTbl
                                        * Referenced by: '<S146>/Em'
                                        */
  real_T BattResistSocBpts[6];         /* Variable: BattResistSocBpts
                                        * Referenced by: '<S146>/R'
                                        */
  real_T BattResistTbl[42];            /* Variable: BattResistTbl
                                        * Referenced by: '<S146>/R'
                                        */
  real_T BattResistTempBpts[7];        /* Variable: BattResistTempBpts
                                        * Referenced by: '<S146>/R'
                                        */
  real_T BrakeMaxPrs;                  /* Variable: BrakeMaxPrs
                                        * Referenced by:
                                        *   '<S260>/Gain'
                                        *   '<S260>/Saturation'
                                        *   '<S260>/Saturation1'
                                        *   '<S260>/Saturation2'
                                        *   '<S260>/Saturation3'
                                        */
  real_T BrakeMaxTrq;                  /* Variable: BrakeMaxTrq
                                        * Referenced by:
                                        *   '<S242>/Gain1'
                                        *   '<S242>/Gain2'
                                        */
  real_T DiffRatio;                    /* Variable: DiffRatio
                                        * Referenced by:
                                        *   '<S206>/Gain2'
                                        *   '<S206>/Gain3'
                                        *   '<S242>/MotTrqReflectedToWheels'
                                        *   '<S242>/WhlTrqReflectedToMot'
                                        */
  real_T EnvPrs;                       /* Variable: EnvPrs
                                        * Referenced by: '<S103>/Constant'
                                        */
  real_T EnvTemp;                      /* Variable: EnvTemp
                                        * Referenced by:
                                        *   '<S103>/Constant1'
                                        *   '<S137>/Constant'
                                        */
  real_T MtrEffSpdBpts[12];            /* Variable: MtrEffSpdBpts
                                        * Referenced by: '<S248>/Eff Map'
                                        */
  real_T MtrEffTbl[180];               /* Variable: MtrEffTbl
                                        * Referenced by: '<S248>/Eff Map'
                                        */
  real_T MtrEffTrqBpts[15];            /* Variable: MtrEffTrqBpts
                                        * Referenced by: '<S248>/Eff Map'
                                        */
  real_T MtrPwrMax;                    /* Variable: MtrPwrMax
                                        * Referenced by:
                                        *   '<S227>/If Action Subsystem1'
                                        *   '<S227>/Constant'
                                        *   '<S243>/Constant'
                                        *   '<S258>/Constant'
                                        *   '<S254>/Constant'
                                        */
  real_T MtrTrqMax;                    /* Variable: MtrTrqMax
                                        * Referenced by:
                                        *   '<S227>/If Action Subsystem'
                                        *   '<S227>/Constant1'
                                        *   '<S243>/Saturation'
                                        *   '<S258>/Saturation'
                                        *   '<S254>/Saturation'
                                        */
  real_T MtrTrqTimeCnst;               /* Variable: MtrTrqTimeCnst
                                        * Referenced by: '<S227>/Gain1'
                                        */
  real_T Ref_speed[2017];              /* Variable: Ref_speed
                                        * Referenced by: '<S109>/1-D Lookup Table'
                                        */
  real_T Ref_time[2017];               /* Variable: Ref_time
                                        * Referenced by: '<S109>/1-D Lookup Table'
                                        */
  real_T SupvsryCtrlRegenBrkCutOffTbl[2];/* Variable: SupvsryCtrlRegenBrkCutOffTbl
                                          * Referenced by: '<S242>/RegenBrakingCutoff'
                                          */
  real_T SupvsryCtrlRegenSpdBpts[2];   /* Variable: SupvsryCtrlRegenSpdBpts
                                        * Referenced by: '<S242>/RegenBrakingCutoff'
                                        */
  real_T TestScnrioCycleNum;           /* Variable: TestScnrioCycleNum
                                        * Referenced by: '<S140>/Constant1'
                                        */
  real_T VehicleMessageFieldDefInputVec[29];
                                     /* Variable: VehicleMessageFieldDefInputVec
                                      * Referenced by:
                                      *   '<S4>/RealSimDepack'
                                      *   '<S4>/RealSimPack'
                                      */
  real_T CombinedSlipWheel2DOF_ALPMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_ALPMAX
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_ALPMAX
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_ALPMAX
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_ALPMAX
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_ALPMAX
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_ALPMAX
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_ALPMAX
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_ALPMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_ALPMAX
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF_ALPMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_ALPMIN
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_ALPMIN
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_ALPMIN
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_ALPMIN
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_ALPMIN
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_ALPMIN
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_ALPMIN
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_ALPMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_ALPMIN
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T VehicleBody3DOFDualTrack_Af;
                                  /* Mask Parameter: VehicleBody3DOFDualTrack_Af
                                   * Referenced by: '<S377>/.5.*A.*Pabs.//R.//T'
                                   */
  real_T independentSuspensionsMacPherson_AntiSwayNtrlAng;
             /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayNtrlAng
              * Referenced by: '<S326>/Constant2'
              */
  real_T independentSuspensionsMacPherson_AntiSwayR;
                   /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayR
                    * Referenced by: '<S326>/Constant1'
                    */
  real_T independentSuspensionsMacPherson_AntiSwayTrsK;
                /* Mask Parameter: independentSuspensionsMacPherson_AntiSwayTrsK
                 * Referenced by: '<S326>/Constant3'
                 */
  real_T SolidAxleSuspensionCoilSpring_AxlIxx;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxlIxx
                          * Referenced by:
                          *   '<S287>/Axle I'
                          *   '<S290>/Axle I1'
                          */
  real_T SolidAxleSuspensionCoilSpring_AxlM;
                           /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxlM
                            * Referenced by:
                            *   '<S287>/Axle M'
                            *   '<S290>/Axle M'
                            */
  real_T SolidAxleSuspensionCoilSpring_AxleNumVec[2];
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_AxleNumVec
                      * Referenced by:
                      *   '<S274>/Axle Number'
                      *   '<S274>/Axle Number2'
                      *   '<S288>/Constant2'
                      *   '<S290>/Constant2'
                      */
  real_T independentSuspensionsMacPherson_AxleNumVec[2];
                  /* Mask Parameter: independentSuspensionsMacPherson_AxleNumVec
                   * Referenced by: '<S279>/Axle Number'
                   */
  real_T CombinedSlipWheel2DOF_BOTTOM_OFFST;
                           /* Mask Parameter: CombinedSlipWheel2DOF_BOTTOM_OFFST
                            * Referenced by: '<S512>/Magic Tire Const Input'
                            */
  real_T CombinedSlipWheel2DOF1_BOTTOM_OFFST;
                          /* Mask Parameter: CombinedSlipWheel2DOF1_BOTTOM_OFFST
                           * Referenced by: '<S540>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF2_BOTTOM_OFFST;
                          /* Mask Parameter: CombinedSlipWheel2DOF2_BOTTOM_OFFST
                           * Referenced by: '<S568>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF3_BOTTOM_OFFST;
                          /* Mask Parameter: CombinedSlipWheel2DOF3_BOTTOM_OFFST
                           * Referenced by: '<S596>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF4_BOTTOM_OFFST;
                          /* Mask Parameter: CombinedSlipWheel2DOF4_BOTTOM_OFFST
                           * Referenced by: '<S624>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF5_BOTTOM_OFFST;
                          /* Mask Parameter: CombinedSlipWheel2DOF5_BOTTOM_OFFST
                           * Referenced by: '<S652>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF6_BOTTOM_OFFST;
                          /* Mask Parameter: CombinedSlipWheel2DOF6_BOTTOM_OFFST
                           * Referenced by: '<S680>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF7_BOTTOM_OFFST;
                          /* Mask Parameter: CombinedSlipWheel2DOF7_BOTTOM_OFFST
                           * Referenced by: '<S708>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF_BOTTOM_STIFF;
                           /* Mask Parameter: CombinedSlipWheel2DOF_BOTTOM_STIFF
                            * Referenced by: '<S512>/Magic Tire Const Input'
                            */
  real_T CombinedSlipWheel2DOF1_BOTTOM_STIFF;
                          /* Mask Parameter: CombinedSlipWheel2DOF1_BOTTOM_STIFF
                           * Referenced by: '<S540>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF2_BOTTOM_STIFF;
                          /* Mask Parameter: CombinedSlipWheel2DOF2_BOTTOM_STIFF
                           * Referenced by: '<S568>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF3_BOTTOM_STIFF;
                          /* Mask Parameter: CombinedSlipWheel2DOF3_BOTTOM_STIFF
                           * Referenced by: '<S596>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF4_BOTTOM_STIFF;
                          /* Mask Parameter: CombinedSlipWheel2DOF4_BOTTOM_STIFF
                           * Referenced by: '<S624>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF5_BOTTOM_STIFF;
                          /* Mask Parameter: CombinedSlipWheel2DOF5_BOTTOM_STIFF
                           * Referenced by: '<S652>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF6_BOTTOM_STIFF;
                          /* Mask Parameter: CombinedSlipWheel2DOF6_BOTTOM_STIFF
                           * Referenced by: '<S680>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF7_BOTTOM_STIFF;
                          /* Mask Parameter: CombinedSlipWheel2DOF7_BOTTOM_STIFF
                           * Referenced by: '<S708>/Magic Tire Const Input'
                           */
  real_T CombinedSlipWheel2DOF_BREFF;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_BREFF
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_BREFF
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_BREFF
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_BREFF
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_BREFF
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_BREFF
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_BREFF
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_BREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_BREFF
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_CAMMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_CAMMAX
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_CAMMAX
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_CAMMAX
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_CAMMAX
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_CAMMAX
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_CAMMAX
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_CAMMAX
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_CAMMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_CAMMAX
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF_CAMMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_CAMMIN
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_CAMMIN
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_CAMMIN
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_CAMMIN
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_CAMMIN
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_CAMMIN
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_CAMMIN
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_CAMMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_CAMMIN
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T SolidAxleSuspensionCoilSpring_Camber;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_Camber
                          * Referenced by: '<S301>/Constant4'
                          */
  real_T independentSuspensionsMacPherson_Camber;
                      /* Mask Parameter: independentSuspensionsMacPherson_Camber
                       * Referenced by: '<S335>/Constant4'
                       */
  real_T SolidAxleSuspensionCoilSpring_CamberHslp;
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_CamberHslp
                      * Referenced by: '<S301>/Constant5'
                      */
  real_T independentSuspensionsMacPherson_CamberHslp;
                  /* Mask Parameter: independentSuspensionsMacPherson_CamberHslp
                   * Referenced by: '<S335>/Constant5'
                   */
  real_T independentSuspensionsMacPherson_CamberStrgSlp;
               /* Mask Parameter: independentSuspensionsMacPherson_CamberStrgSlp
                * Referenced by: '<S335>/Constant3'
                */
  real_T SolidAxleSuspensionCoilSpring_Caster;
                         /* Mask Parameter: SolidAxleSuspensionCoilSpring_Caster
                          * Referenced by: '<S301>/Constant7'
                          */
  real_T independentSuspensionsMacPherson_Caster;
                      /* Mask Parameter: independentSuspensionsMacPherson_Caster
                       * Referenced by: '<S335>/Constant7'
                       */
  real_T SolidAxleSuspensionCoilSpring_CasterHslp;
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_CasterHslp
                      * Referenced by: '<S301>/Constant8'
                      */
  real_T independentSuspensionsMacPherson_CasterHslp;
                  /* Mask Parameter: independentSuspensionsMacPherson_CasterHslp
                   * Referenced by: '<S335>/Constant8'
                   */
  real_T independentSuspensionsMacPherson_CasterStrgSlp;
               /* Mask Parameter: independentSuspensionsMacPherson_CasterStrgSlp
                * Referenced by: '<S335>/Constant6'
                */
  real_T VehicleBody3DOFDualTrack_Cd;
                                  /* Mask Parameter: VehicleBody3DOFDualTrack_Cd
                                   * Referenced by: '<S377>/Constant'
                                   */
  real_T VehicleBody3DOFDualTrack_Cl;
                                  /* Mask Parameter: VehicleBody3DOFDualTrack_Cl
                                   * Referenced by: '<S377>/Constant1'
                                   */
  real_T VehicleBody3DOFDualTrack_Cpm;
                                 /* Mask Parameter: VehicleBody3DOFDualTrack_Cpm
                                  * Referenced by: '<S377>/Constant2'
                                  */
  real_T VehicleBody3DOFDualTrack_Cs[11];
                                  /* Mask Parameter: VehicleBody3DOFDualTrack_Cs
                                   * Referenced by: '<S377>/Cs'
                                   */
  real_T VehicleBody3DOFDualTrack_Cym[11];
                                 /* Mask Parameter: VehicleBody3DOFDualTrack_Cym
                                  * Referenced by: '<S377>/Cym'
                                  */
  real_T SolidAxleSuspensionCoilSpring_Cz;
                             /* Mask Parameter: SolidAxleSuspensionCoilSpring_Cz
                              * Referenced by: '<S312>/Constant2'
                              */
  real_T independentSuspensionsMacPherson_Cz;
                          /* Mask Parameter: independentSuspensionsMacPherson_Cz
                           * Referenced by: '<S349>/Constant2'
                           */
  real_T SolidAxleSuspensionCoilSpring_CzWhlAxl;
                       /* Mask Parameter: SolidAxleSuspensionCoilSpring_CzWhlAxl
                        * Referenced by: '<S284>/Carrier to Axle Damping'
                        */
  real_T CombinedSlipWheel2DOF_DREFF;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_DREFF
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_DREFF
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_DREFF
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_DREFF
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_DREFF
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_DREFF
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_DREFF
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_DREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_DREFF
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T KinematicSteering_Db;         /* Mask Parameter: KinematicSteering_Db
                                        * Referenced by: '<S269>/Backlash'
                                        */
  real_T SolidAxleSuspensionCoilSpring_F0z;
                            /* Mask Parameter: SolidAxleSuspensionCoilSpring_F0z
                             * Referenced by: '<S312>/Constant1'
                             */
  real_T independentSuspensionsMacPherson_F0z;
                         /* Mask Parameter: independentSuspensionsMacPherson_F0z
                          * Referenced by: '<S349>/Constant1'
                          */
  real_T SolidAxleSuspensionCoilSpring_F0zWhlAxl;
                      /* Mask Parameter: SolidAxleSuspensionCoilSpring_F0zWhlAxl
                       * Referenced by: '<S284>/Preload'
                       */
  real_T CombinedSlipWheel2DOF_FNOMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_FNOMIN
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_FNOMIN
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_FNOMIN
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_FNOMIN
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_FNOMIN
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_FNOMIN
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_FNOMIN
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_FNOMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_FNOMIN
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF_FREFF;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_FREFF
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FREFF
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FREFF
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FREFF
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FREFF
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_FREFF
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_FREFF
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_FREFF;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_FREFF
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_FZMAX;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_FZMAX
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FZMAX
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FZMAX
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FZMAX
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FZMAX
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_FZMAX
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_FZMAX
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_FZMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_FZMAX
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_FZMIN;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_FZMIN
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_FZMIN
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_FZMIN
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_FZMIN
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_FZMIN
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_FZMIN
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_FZMIN
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_FZMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_FZMIN
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_GRAVITY;
                                /* Mask Parameter: CombinedSlipWheel2DOF_GRAVITY
                                 * Referenced by: '<S519>/Fg'
                                 */
  real_T CombinedSlipWheel2DOF1_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_GRAVITY
                                * Referenced by: '<S547>/Fg'
                                */
  real_T CombinedSlipWheel2DOF2_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_GRAVITY
                                * Referenced by: '<S575>/Fg'
                                */
  real_T CombinedSlipWheel2DOF3_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_GRAVITY
                                * Referenced by: '<S603>/Fg'
                                */
  real_T CombinedSlipWheel2DOF4_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_GRAVITY
                                * Referenced by: '<S631>/Fg'
                                */
  real_T CombinedSlipWheel2DOF5_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF5_GRAVITY
                                * Referenced by: '<S659>/Fg'
                                */
  real_T CombinedSlipWheel2DOF6_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF6_GRAVITY
                                * Referenced by: '<S687>/Fg'
                                */
  real_T CombinedSlipWheel2DOF7_GRAVITY;
                               /* Mask Parameter: CombinedSlipWheel2DOF7_GRAVITY
                                * Referenced by: '<S715>/Fg'
                                */
  real_T SolidAxleSuspensionCoilSpring_Hmax;
                           /* Mask Parameter: SolidAxleSuspensionCoilSpring_Hmax
                            * Referenced by:
                            *   '<S312>/Constant3'
                            *   '<S314>/Max stop reached'
                            *   '<S314>/Min stop reached'
                            */
  real_T independentSuspensionsMacPherson_Hmax;
                        /* Mask Parameter: independentSuspensionsMacPherson_Hmax
                         * Referenced by:
                         *   '<S349>/Constant3'
                         *   '<S351>/Max stop reached'
                         *   '<S351>/Min stop reached'
                         */
  real_T SignalHold_IC;                /* Mask Parameter: SignalHold_IC
                                        * Referenced by: '<S175>/Pass Through'
                                        */
  real_T SignalHold_IC_k;              /* Mask Parameter: SignalHold_IC_k
                                        * Referenced by: '<S198>/Pass Through'
                                        */
  real_T SignalHold_IC_kz;             /* Mask Parameter: SignalHold_IC_kz
                                        * Referenced by: '<S177>/Pass Through'
                                        */
  real_T SignalHold_IC_m;              /* Mask Parameter: SignalHold_IC_m
                                        * Referenced by: '<S200>/Pass Through'
                                        */
  real_T CombinedSlipWheel2DOF_IYY; /* Mask Parameter: CombinedSlipWheel2DOF_IYY
                                     * Referenced by: '<S525>/Clutch'
                                     */
  real_T CombinedSlipWheel2DOF1_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF1_IYY
                                    * Referenced by: '<S553>/Clutch'
                                    */
  real_T CombinedSlipWheel2DOF2_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF2_IYY
                                    * Referenced by: '<S581>/Clutch'
                                    */
  real_T CombinedSlipWheel2DOF3_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF3_IYY
                                    * Referenced by: '<S609>/Clutch'
                                    */
  real_T CombinedSlipWheel2DOF4_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF4_IYY
                                    * Referenced by: '<S637>/Clutch'
                                    */
  real_T CombinedSlipWheel2DOF5_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF5_IYY
                                    * Referenced by: '<S665>/Clutch'
                                    */
  real_T CombinedSlipWheel2DOF6_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF6_IYY
                                    * Referenced by: '<S693>/Clutch'
                                    */
  real_T CombinedSlipWheel2DOF7_IYY;
                                   /* Mask Parameter: CombinedSlipWheel2DOF7_IYY
                                    * Referenced by: '<S721>/Clutch'
                                    */
  real_T VehicleBody3DOFDualTrack_Izz;
                                 /* Mask Parameter: VehicleBody3DOFDualTrack_Izz
                                  * Referenced by:
                                  *   '<S362>/vehicle model'
                                  *   '<S383>/Constant2'
                                  */
  real_T CombinedSlipWheel2DOF_KPUMAX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_KPUMAX
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_KPUMAX
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_KPUMAX
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_KPUMAX
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_KPUMAX
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_KPUMAX
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_KPUMAX
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_KPUMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_KPUMAX
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF_KPUMIN;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_KPUMIN
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_KPUMIN
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_KPUMIN
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_KPUMIN
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_KPUMIN
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_KPUMIN
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_KPUMIN
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_KPUMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_KPUMIN
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T LongitudinalDriver_Kaw;       /* Mask Parameter: LongitudinalDriver_Kaw
                                        * Referenced by: '<S172>/Kaw'
                                        */
  real_T LongitudinalDriver_Kff;       /* Mask Parameter: LongitudinalDriver_Kff
                                        * Referenced by: '<S172>/Kff//vnom'
                                        */
  real_T LongitudinalDriver_Kg;        /* Mask Parameter: LongitudinalDriver_Kg
                                        * Referenced by: '<S172>/Kg'
                                        */
  real_T LongitudinalDriver_Ki;        /* Mask Parameter: LongitudinalDriver_Ki
                                        * Referenced by: '<S172>/Ki//vnom'
                                        */
  real_T LongitudinalDriver_Kp;        /* Mask Parameter: LongitudinalDriver_Kp
                                        * Referenced by: '<S172>/Kp//vnom'
                                        */
  real_T LongitudinalDriver2_Kpt;     /* Mask Parameter: LongitudinalDriver2_Kpt
                                       * Referenced by: '<S191>/Setup'
                                       */
  real_T SolidAxleSuspensionCoilSpring_Kz;
                             /* Mask Parameter: SolidAxleSuspensionCoilSpring_Kz
                              * Referenced by: '<S312>/Constant'
                              */
  real_T independentSuspensionsMacPherson_Kz;
                          /* Mask Parameter: independentSuspensionsMacPherson_Kz
                           * Referenced by: '<S349>/Constant'
                           */
  real_T SolidAxleSuspensionCoilSpring_KzWhlAxl;
                       /* Mask Parameter: SolidAxleSuspensionCoilSpring_KzWhlAxl
                        * Referenced by: '<S284>/Carrier to Axle Compliance'
                        */
  real_T LongitudinalDriver2_L;        /* Mask Parameter: LongitudinalDriver2_L
                                        * Referenced by: '<S191>/Setup'
                                        */
  real_T CombinedSlipWheel2DOF_LATERAL_STIFFNESS;
                      /* Mask Parameter: CombinedSlipWheel2DOF_LATERAL_STIFFNESS
                       * Referenced by: '<S512>/Magic Tire Const Input'
                       */
  real_T CombinedSlipWheel2DOF1_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF1_LATERAL_STIFFNESS
                      * Referenced by: '<S540>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF2_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF2_LATERAL_STIFFNESS
                      * Referenced by: '<S568>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF3_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF3_LATERAL_STIFFNESS
                      * Referenced by: '<S596>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF4_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF4_LATERAL_STIFFNESS
                      * Referenced by: '<S624>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF5_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF5_LATERAL_STIFFNESS
                      * Referenced by: '<S652>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF6_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF6_LATERAL_STIFFNESS
                      * Referenced by: '<S680>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF7_LATERAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF7_LATERAL_STIFFNESS
                      * Referenced by: '<S708>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF_LONGITUDINAL_STIFFNESS;
                 /* Mask Parameter: CombinedSlipWheel2DOF_LONGITUDINAL_STIFFNESS
                  * Referenced by: '<S512>/Magic Tire Const Input'
                  */
  real_T CombinedSlipWheel2DOF1_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF1_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S540>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF2_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF2_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S568>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF3_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF3_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S596>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF4_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF4_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S624>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF5_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF5_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S652>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF6_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF6_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S680>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF7_LONGITUDINAL_STIFFNESS;
                /* Mask Parameter: CombinedSlipWheel2DOF7_LONGITUDINAL_STIFFNESS
                 * Referenced by: '<S708>/Magic Tire Const Input'
                 */
  real_T CombinedSlipWheel2DOF_LONGVL;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_LONGVL
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_LONGVL
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_LONGVL
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_LONGVL
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_LONGVL
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_LONGVL
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_LONGVL
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_LONGVL;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_LONGVL
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF_MASS;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_MASS
                                    * Referenced by:
                                    *   '<S519>/Fg'
                                    *   '<S519>/Gain1'
                                    */
  real_T CombinedSlipWheel2DOF1_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_MASS
                                   * Referenced by:
                                   *   '<S547>/Fg'
                                   *   '<S547>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF2_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_MASS
                                   * Referenced by:
                                   *   '<S575>/Fg'
                                   *   '<S575>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF3_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_MASS
                                   * Referenced by:
                                   *   '<S603>/Fg'
                                   *   '<S603>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF4_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_MASS
                                   * Referenced by:
                                   *   '<S631>/Fg'
                                   *   '<S631>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF5_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_MASS
                                   * Referenced by:
                                   *   '<S659>/Fg'
                                   *   '<S659>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF6_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_MASS
                                   * Referenced by:
                                   *   '<S687>/Fg'
                                   *   '<S687>/Gain1'
                                   */
  real_T CombinedSlipWheel2DOF7_MASS;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_MASS
                                   * Referenced by:
                                   *   '<S715>/Fg'
                                   *   '<S715>/Gain1'
                                   */
  real_T VehicleBody3DOFDualTrack_NF;
                                  /* Mask Parameter: VehicleBody3DOFDualTrack_NF
                                   * Referenced by: '<S362>/vehicle model'
                                   */
  real_T CombinedSlipWheel2DOF_NOMPRES;
                                /* Mask Parameter: CombinedSlipWheel2DOF_NOMPRES
                                 * Referenced by: '<S512>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_NOMPRES
                                * Referenced by: '<S540>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_NOMPRES
                                * Referenced by: '<S568>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_NOMPRES
                                * Referenced by: '<S596>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_NOMPRES
                                * Referenced by: '<S624>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF5_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF5_NOMPRES
                                * Referenced by: '<S652>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF6_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF6_NOMPRES
                                * Referenced by: '<S680>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF7_NOMPRES;
                               /* Mask Parameter: CombinedSlipWheel2DOF7_NOMPRES
                                * Referenced by: '<S708>/Magic Tire Const Input'
                                */
  real_T VehicleBody3DOFDualTrack_NR;
                                  /* Mask Parameter: VehicleBody3DOFDualTrack_NR
                                   * Referenced by: '<S362>/vehicle model'
                                   */
  real_T CombinedSlipWheel2DOF_PCFX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PCFX1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PCFX1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PCFX1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PCFX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PCFX1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PCFX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PCFX2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PCFX2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PCFX2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PCFX2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PCFX2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PCFX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PCFX3
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFX3
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFX3
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFX3
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFX3
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PCFX3
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PCFX3
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PCFX3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PCFX3
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PCFY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PCFY1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PCFY1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PCFY1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PCFY1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PCFY1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PCFY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PCFY2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PCFY2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PCFY2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PCFY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PCFY2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PCFY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PCFY3
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PCFY3
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PCFY3
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PCFY3
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PCFY3
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PCFY3
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PCFY3
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PCFY3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PCFY3
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PCX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PCX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PCX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PCX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PCX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PCX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PCX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PCX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PCX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PCY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PCY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PCY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PCY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PCY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PCY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PCY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PCY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PCY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PDX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PDX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PDX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PDX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PDX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PDX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PDX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PDX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PDX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PDX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PDX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PDX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PDX3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PDX3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDX3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDX3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDX3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDX3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PDX3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PDX3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PDX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PDX3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PDXP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PDXP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PDXP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PDXP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PDXP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PDXP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PDXP2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PDXP2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PDXP2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PDXP2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PDXP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PDXP2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PDXP3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PDXP3
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDXP3
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDXP3
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDXP3
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDXP3
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PDXP3
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PDXP3
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PDXP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PDXP3
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PDY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PDY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PDY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PDY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PDY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PDY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PDY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PDY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PDY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PDY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PDY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PDY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PDY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PDY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PDY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PDY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PDY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PDY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PDY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PDY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PDY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PDY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PDYP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PDYP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PDYP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PDYP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PDYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PDYP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PDYP2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PDYP2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PDYP2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PDYP2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PDYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PDYP2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PDYP3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PDYP3
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP3
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP3
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP3
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP3
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PDYP3
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PDYP3
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PDYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PDYP3
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PDYP4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PDYP4
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PDYP4
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PDYP4
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PDYP4
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PDYP4
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PDYP4
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PDYP4
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PDYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PDYP4
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PECP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PECP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PECP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PECP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PECP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PECP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PECP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PECP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PECP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PECP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PECP2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PECP2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PECP2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PECP2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PECP2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PECP2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PECP2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PECP2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PECP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PECP2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PEX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEX3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEX3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEX3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEX3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEX3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEX4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEX4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEX4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEX4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEX4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEX4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEX4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEX4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEX4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEY4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEY4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEY4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEY4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEY4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PEY5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PEY5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PEY5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PEY5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PEY5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PEY5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PEY5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PEY5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PEY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PEY5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PFZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PFZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PFZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PFZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PFZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PFZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PFZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PFZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PFZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PFZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PHX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PHX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PHX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PHX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PHX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PHX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PHX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PHX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PHX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PHX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PHX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PHY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PHY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PHY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PHY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PHY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PHY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PHY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PHY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PHY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PHY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PHY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PHY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PHY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PHY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PHYP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PHYP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PHYP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PHYP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PHYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PHYP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PHYP2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PHYP2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PHYP2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PHYP2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PHYP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PHYP2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PHYP3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PHYP3
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP3
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP3
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP3
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP3
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PHYP3
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PHYP3
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PHYP3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PHYP3
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PHYP4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PHYP4
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PHYP4
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PHYP4
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PHYP4
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PHYP4
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PHYP4
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PHYP4
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PHYP4;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PHYP4
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PKX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKX3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKX3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKX3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKX3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKX3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKX3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKX3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKX3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKX3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKY4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKY4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKY4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKY4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKY4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKY5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKY5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKY5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKY5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKY5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKY6;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKY6
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY6
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY6
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY6
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY6
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKY6
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKY6
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKY6
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKY7;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PKY7
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PKY7
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PKY7
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PKY7
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PKY7
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PKY7
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PKY7
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PKY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PKY7
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PKYP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PKYP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PKYP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PKYP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PKYP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PKYP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PKYP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PKYP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PKYP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PKYP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PPMX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_PPMX1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_PPMX1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_PPMX1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_PPMX1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_PPMX1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_PPMX1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_PPMX1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_PPMX1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_PPMX1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_PPX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPX3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPX3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPX3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPX3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPX3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPX4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPX4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPX4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPX4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPX4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPX4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPX4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPX4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPX4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPY4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPY4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPY4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPY4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPY4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPY5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPY5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPY5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPY5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPY5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPY5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPY5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPY5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPY5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PPZ2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PPZ2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PPZ2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PPZ2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PPZ2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PPZ2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PPZ2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PPZ2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PPZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PPZ2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PRESMAX;
                                /* Mask Parameter: CombinedSlipWheel2DOF_PRESMAX
                                 * Referenced by: '<S512>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_PRESMAX
                                * Referenced by: '<S540>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_PRESMAX
                                * Referenced by: '<S568>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_PRESMAX
                                * Referenced by: '<S596>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_PRESMAX
                                * Referenced by: '<S624>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF5_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF5_PRESMAX
                                * Referenced by: '<S652>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF6_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF6_PRESMAX
                                * Referenced by: '<S680>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF7_PRESMAX;
                               /* Mask Parameter: CombinedSlipWheel2DOF7_PRESMAX
                                * Referenced by: '<S708>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF_PRESMIN;
                                /* Mask Parameter: CombinedSlipWheel2DOF_PRESMIN
                                 * Referenced by: '<S512>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF1_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF1_PRESMIN
                                * Referenced by: '<S540>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF2_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF2_PRESMIN
                                * Referenced by: '<S568>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF3_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF3_PRESMIN
                                * Referenced by: '<S596>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF4_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF4_PRESMIN
                                * Referenced by: '<S624>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF5_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF5_PRESMIN
                                * Referenced by: '<S652>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF6_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF6_PRESMIN
                                * Referenced by: '<S680>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF7_PRESMIN;
                               /* Mask Parameter: CombinedSlipWheel2DOF7_PRESMIN
                                * Referenced by: '<S708>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF_PVX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PVX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PVX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PVX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PVX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PVX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PVX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PVX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PVX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PVX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PVX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PVX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PVY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PVY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PVY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PVY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PVY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PVY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PVY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PVY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PVY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PVY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PVY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PVY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PVY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PVY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PVY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_PVY4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_PVY4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_PVY4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_PVY4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_PVY4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_PVY4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_PVY4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_PVY4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_PVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_PVY4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T VehicleBody3DOFDualTrack_Pabs;
                                /* Mask Parameter: VehicleBody3DOFDualTrack_Pabs
                                 * Referenced by: '<S377>/.5.*A.*Pabs.//R.//T'
                                 */
  real_T CombinedSlipWheel2DOF_QBRP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QBRP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QBRP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QBRP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QBRP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QBRP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QBRP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QBRP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QBRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QBRP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QBZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QBZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QBZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QBZ10;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QBZ10
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ10
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ10
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ10
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ10
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ10
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ10
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QBZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ10
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QBZ2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QBZ2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QBZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QBZ3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QBZ3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QBZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QBZ4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QBZ4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QBZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QBZ5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QBZ5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QBZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QBZ6;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QBZ6
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ6
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ6
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ6
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ6
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ6
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ6
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QBZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ6
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QBZ9;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QBZ9
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QBZ9
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QBZ9
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QBZ9
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QBZ9
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QBZ9
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QBZ9
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QBZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QBZ9
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QCRP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QCRP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QCRP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QCRP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QCRP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QCRP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QCRP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QCRP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QCRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QCRP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QCRP2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QCRP2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QCRP2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QCRP2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QCRP2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QCRP2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QCRP2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QCRP2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QCRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QCRP2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QCZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QCZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QCZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QCZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QCZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QCZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QCZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QCZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QCZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QCZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDRP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QDRP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDRP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDRP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDRP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDRP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QDRP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QDRP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QDRP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QDRP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QDRP2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QDRP2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDRP2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDRP2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDRP2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDRP2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QDRP2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QDRP2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QDRP2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QDRP2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QDTP1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QDTP1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDTP1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDTP1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDTP1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDTP1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QDTP1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QDTP1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QDTP1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QDTP1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QDZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDZ10;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QDZ10
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ10
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ10
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ10
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ10
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ10
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ10
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QDZ10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ10
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QDZ11;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QDZ11
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ11
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ11
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ11
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ11
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ11
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ11
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QDZ11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ11
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QDZ2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDZ3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDZ4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDZ6;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ6
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ6
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ6
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ6
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ6
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ6
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ6
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ6
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDZ7;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ7
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ7
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ7
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ7
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ7
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ7
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ7
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ7
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDZ8;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ8
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ8
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ8
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ8
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ8
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ8
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ8
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ8
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QDZ9;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QDZ9
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QDZ9
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QDZ9
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QDZ9
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QDZ9
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QDZ9
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QDZ9
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QDZ9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QDZ9
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QEZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QEZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QEZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QEZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QEZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QEZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QEZ2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QEZ2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QEZ2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QEZ2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QEZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QEZ2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QEZ3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QEZ3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QEZ3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QEZ3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QEZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QEZ3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QEZ4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QEZ4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QEZ4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QEZ4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QEZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QEZ4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QEZ5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QEZ5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QEZ5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QEZ5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QEZ5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QEZ5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QEZ5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QEZ5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QEZ5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QEZ5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QHZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QHZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QHZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QHZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QHZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QHZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QHZ2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QHZ2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QHZ2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QHZ2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QHZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QHZ2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QHZ3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QHZ3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QHZ3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QHZ3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QHZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QHZ3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QHZ4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QHZ4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QHZ4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QHZ4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QHZ4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QHZ4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QHZ4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QHZ4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QHZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QHZ4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX10;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QSX10
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX10
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX10
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX10
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX10
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QSX10
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QSX10
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QSX10;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QSX10
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QSX11;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QSX11
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX11
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX11
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX11
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX11
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QSX11
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QSX11
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QSX11;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QSX11
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QSX12;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QSX12
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX12
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX12
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX12
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX12
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QSX12
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QSX12
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QSX12;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QSX12
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QSX13;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QSX13
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX13
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX13
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX13
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX13
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QSX13
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QSX13
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QSX13;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QSX13
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QSX14;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_QSX14
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_QSX14
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_QSX14
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_QSX14
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_QSX14
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_QSX14
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_QSX14
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_QSX14;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_QSX14
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_QSX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX6;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX6
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX6
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX6
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX6
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX6
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX6
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX6
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX6
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX7;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX7
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX7
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX7
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX7
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX7
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX7
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX7
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX7
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX8;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX8
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX8
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX8
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX8
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX8
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX8
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX8
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX8
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSX9;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSX9
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSX9
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSX9
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSX9
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSX9
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSX9
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSX9
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSX9;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSX9
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY6;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY6
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY6
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY6
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY6
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY6
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY6
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY6
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY6
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY7;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY7
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY7
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY7
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY7
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY7
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY7
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY7
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY7;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY7
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_QSY8;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_QSY8
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_QSY8
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_QSY8
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_QSY8
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_QSY8
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_QSY8
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_QSY8
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_QSY8;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_QSY8
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_Q_FCX;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_FCX
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCX
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCX
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCX
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCX
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_FCX
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_FCX
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_FCX;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_FCX
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_FCY;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_FCY
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCY
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCY
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCY
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCY
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_FCY
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_FCY
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_FCY;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_FCY
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_FCY2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_Q_FCY2
                                  * Referenced by: '<S512>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF1_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FCY2
                                 * Referenced by: '<S540>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF2_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FCY2
                                 * Referenced by: '<S568>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF3_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FCY2
                                 * Referenced by: '<S596>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF4_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FCY2
                                 * Referenced by: '<S624>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF5_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_Q_FCY2
                                 * Referenced by: '<S652>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF6_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_Q_FCY2
                                 * Referenced by: '<S680>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF7_Q_FCY2;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_Q_FCY2
                                 * Referenced by: '<S708>/Magic Tire Const Input'
                                 */
  real_T CombinedSlipWheel2DOF_Q_FZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_FZ1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_FZ1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_FZ1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_FZ1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_FZ1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_FZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_FZ2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_FZ2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_FZ2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_FZ2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_FZ2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_FZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_FZ3
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_FZ3
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_FZ3
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_FZ3
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_FZ3
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_FZ3
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_FZ3
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_FZ3;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_FZ3
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_RA1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_RA1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RA1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RA1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RA1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RA1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_RA1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_RA1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_RA1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_RA1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_RA2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_RA2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RA2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RA2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RA2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RA2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_RA2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_RA2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_RA2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_RA2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_RB1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_RB1
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RB1
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RB1
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RB1
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RB1
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_RB1
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_RB1
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_RB1;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_RB1
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_RB2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_RB2
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RB2
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RB2
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RB2
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RB2
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_RB2
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_RB2
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_RB2;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_RB2
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_RE0;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_Q_RE0
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_Q_RE0
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_Q_RE0
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_Q_RE0
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_Q_RE0
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_Q_RE0
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_Q_RE0
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_Q_RE0;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_Q_RE0
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_Q_V1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_Q_V1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_Q_V1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_Q_V1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_Q_V1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_Q_V1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_Q_V1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_Q_V1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_Q_V1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_Q_V1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_Q_V2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_Q_V2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_Q_V2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_Q_V2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_Q_V2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_Q_V2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_Q_V2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_Q_V2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_Q_V2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_Q_V2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T DragForce_R;                  /* Mask Parameter: DragForce_R
                                        * Referenced by: '<S377>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T CombinedSlipWheel2DOF_RBX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RBX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RBX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RBX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RBX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RBX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RBX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RBX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RBX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RBX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RBX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RBX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RBX3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RBX3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBX3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBX3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBX3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBX3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RBX3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RBX3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RBX3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RBX3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RBY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RBY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RBY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RBY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RBY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RBY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RBY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RBY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RBY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RBY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RBY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RBY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RBY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RBY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RBY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RBY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RBY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RBY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RBY4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RBY4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RBY4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RBY4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RBY4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RBY4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RBY4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RBY4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RBY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RBY4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RCX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RCX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RCX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RCX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RCX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RCX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RCX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RCX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RCX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RCX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RCY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RCY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RCY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RCY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RCY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RCY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RCY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RCY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RCY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RCY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_REX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_REX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_REX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_REX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_REX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_REX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_REX2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_REX2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REX2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REX2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REX2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REX2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_REX2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_REX2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_REX2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_REX2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_REY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_REY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_REY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_REY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_REY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_REY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_REY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_REY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_REY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_REY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_REY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_REY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_REY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_REY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_REY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_REY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RHX1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RHX1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHX1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHX1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHX1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHX1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RHX1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RHX1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RHX1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RHX1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RHY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RHY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RHY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RHY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RHY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RHY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RHY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RHY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RHY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RHY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RHY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RHY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RHY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RHY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RHY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RHY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RIM_RADIUS;
                             /* Mask Parameter: CombinedSlipWheel2DOF_RIM_RADIUS
                              * Referenced by: '<S512>/Magic Tire Const Input'
                              */
  real_T CombinedSlipWheel2DOF1_RIM_RADIUS;
                            /* Mask Parameter: CombinedSlipWheel2DOF1_RIM_RADIUS
                             * Referenced by: '<S540>/Magic Tire Const Input'
                             */
  real_T CombinedSlipWheel2DOF2_RIM_RADIUS;
                            /* Mask Parameter: CombinedSlipWheel2DOF2_RIM_RADIUS
                             * Referenced by: '<S568>/Magic Tire Const Input'
                             */
  real_T CombinedSlipWheel2DOF3_RIM_RADIUS;
                            /* Mask Parameter: CombinedSlipWheel2DOF3_RIM_RADIUS
                             * Referenced by: '<S596>/Magic Tire Const Input'
                             */
  real_T CombinedSlipWheel2DOF4_RIM_RADIUS;
                            /* Mask Parameter: CombinedSlipWheel2DOF4_RIM_RADIUS
                             * Referenced by: '<S624>/Magic Tire Const Input'
                             */
  real_T CombinedSlipWheel2DOF5_RIM_RADIUS;
                            /* Mask Parameter: CombinedSlipWheel2DOF5_RIM_RADIUS
                             * Referenced by: '<S652>/Magic Tire Const Input'
                             */
  real_T CombinedSlipWheel2DOF6_RIM_RADIUS;
                            /* Mask Parameter: CombinedSlipWheel2DOF6_RIM_RADIUS
                             * Referenced by: '<S680>/Magic Tire Const Input'
                             */
  real_T CombinedSlipWheel2DOF7_RIM_RADIUS;
                            /* Mask Parameter: CombinedSlipWheel2DOF7_RIM_RADIUS
                             * Referenced by: '<S708>/Magic Tire Const Input'
                             */
  real_T CombinedSlipWheel2DOF_RVY1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RVY1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RVY1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RVY1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RVY1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RVY1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RVY2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RVY2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RVY2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RVY2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RVY2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RVY2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RVY3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RVY3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RVY3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RVY3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RVY3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RVY3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RVY4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RVY4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RVY4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RVY4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RVY4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RVY4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RVY5;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RVY5
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY5
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY5
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY5
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY5
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RVY5
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RVY5
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RVY5;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RVY5
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_RVY6;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_RVY6
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_RVY6
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_RVY6
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_RVY6
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_RVY6
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_RVY6
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_RVY6
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_RVY6;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_RVY6
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_Rm;   /* Mask Parameter: CombinedSlipWheel2DOF_Rm
                                      * Referenced by: '<S524>/Torque Conversion'
                                      */
  real_T CombinedSlipWheel2DOF1_Rm; /* Mask Parameter: CombinedSlipWheel2DOF1_Rm
                                     * Referenced by: '<S552>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF2_Rm; /* Mask Parameter: CombinedSlipWheel2DOF2_Rm
                                     * Referenced by: '<S580>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF3_Rm; /* Mask Parameter: CombinedSlipWheel2DOF3_Rm
                                     * Referenced by: '<S608>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF4_Rm; /* Mask Parameter: CombinedSlipWheel2DOF4_Rm
                                     * Referenced by: '<S636>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF5_Rm; /* Mask Parameter: CombinedSlipWheel2DOF5_Rm
                                     * Referenced by: '<S664>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF6_Rm; /* Mask Parameter: CombinedSlipWheel2DOF6_Rm
                                     * Referenced by: '<S692>/Torque Conversion'
                                     */
  real_T CombinedSlipWheel2DOF7_Rm; /* Mask Parameter: CombinedSlipWheel2DOF7_Rm
                                     * Referenced by: '<S720>/Torque Conversion'
                                     */
  real_T SolidAxleSuspensionCoilSpring_RollStrgSlp;
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_RollStrgSlp
                     * Referenced by: '<S301>/Constant2'
                     */
  real_T independentSuspensionsMacPherson_RollStrgSlp;
                 /* Mask Parameter: independentSuspensionsMacPherson_RollStrgSlp
                  * Referenced by: '<S335>/Constant2'
                  */
  real_T CombinedSlipWheel2DOF_SSZ1;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_SSZ1
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ1
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ1
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ1
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ1
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_SSZ1
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_SSZ1
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_SSZ1;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_SSZ1
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_SSZ2;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_SSZ2
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ2
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ2
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ2
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ2
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_SSZ2
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_SSZ2
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_SSZ2;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_SSZ2
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_SSZ3;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_SSZ3
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ3
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ3
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ3
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ3
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_SSZ3
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_SSZ3
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_SSZ3;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_SSZ3
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF_SSZ4;
                                   /* Mask Parameter: CombinedSlipWheel2DOF_SSZ4
                                    * Referenced by: '<S512>/Magic Tire Const Input'
                                    */
  real_T CombinedSlipWheel2DOF1_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF1_SSZ4
                                   * Referenced by: '<S540>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF2_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF2_SSZ4
                                   * Referenced by: '<S568>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF3_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF3_SSZ4
                                   * Referenced by: '<S596>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF4_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF4_SSZ4
                                   * Referenced by: '<S624>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF5_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF5_SSZ4
                                   * Referenced by: '<S652>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF6_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF6_SSZ4
                                   * Referenced by: '<S680>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF7_SSZ4;
                                  /* Mask Parameter: CombinedSlipWheel2DOF7_SSZ4
                                   * Referenced by: '<S708>/Magic Tire Const Input'
                                   */
  real_T independentSuspensionsMacPherson_StrgHgtSlp;
                  /* Mask Parameter: independentSuspensionsMacPherson_StrgHgtSlp
                   * Referenced by: '<S358>/Constant'
                   */
  real_T KinematicSteering_StrgRatio;
                                  /* Mask Parameter: KinematicSteering_StrgRatio
                                   * Referenced by:
                                   *   '<S270>/Constant'
                                   *   '<S270>/Gain'
                                   *   '<S270>/Gain1'
                                   *   '<S270>/Gain2'
                                   */
  real_T KinematicSteering_StrgRng; /* Mask Parameter: KinematicSteering_StrgRng
                                     * Referenced by: '<S269>/Saturation'
                                     */
  real_T SolidAxleSuspensionCoilSpring_SuspCoords[6];
                     /* Mask Parameter: SolidAxleSuspensionCoilSpring_SuspCoords
                      * Referenced by:
                      *   '<S281>/Suspension axle connection coordinates in axle body frame'
                      *   '<S288>/Suspension connection point coordinates in axle body frame'
                      */
  real_T VehicleBody3DOFDualTrack_Tair;
                                /* Mask Parameter: VehicleBody3DOFDualTrack_Tair
                                 * Referenced by: '<S362>/AirTempConstant'
                                 */
  real_T MachEMotor_Tc;                /* Mask Parameter: MachEMotor_Tc
                                        * Referenced by: '<S216>/Gain1'
                                        */
  real_T CANdiagnosticCounter_TimeOutDuration;
                         /* Mask Parameter: CANdiagnosticCounter_TimeOutDuration
                          * Referenced by: '<S128>/Constant'
                          */
  real_T SolidAxleSuspensionCoilSpring_Toe;
                            /* Mask Parameter: SolidAxleSuspensionCoilSpring_Toe
                             * Referenced by: '<S301>/Constant1'
                             */
  real_T independentSuspensionsMacPherson_Toe;
                         /* Mask Parameter: independentSuspensionsMacPherson_Toe
                          * Referenced by: '<S335>/Constant1'
                          */
  real_T independentSuspensionsMacPherson_ToeStrgSlp;
                  /* Mask Parameter: independentSuspensionsMacPherson_ToeStrgSlp
                   * Referenced by: '<S335>/Constant'
                   */
  real_T SolidAxleSuspensionCoilSpring_TrackCoords[6];
                    /* Mask Parameter: SolidAxleSuspensionCoilSpring_TrackCoords
                     * Referenced by:
                     *   '<S281>/Track coordinates in axle body frame'
                     *   '<S290>/Track coordinates in axle body frame'
                     */
  real_T CombinedSlipWheel2DOF_UNLOADED_RADIUS;
                        /* Mask Parameter: CombinedSlipWheel2DOF_UNLOADED_RADIUS
                         * Referenced by:
                         *   '<S512>/Magic Tire Const Input'
                         *   '<S514>/Constant9'
                         */
  real_T CombinedSlipWheel2DOF1_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF1_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S540>/Magic Tire Const Input'
                        *   '<S542>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF2_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF2_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S568>/Magic Tire Const Input'
                        *   '<S570>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF3_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF3_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S596>/Magic Tire Const Input'
                        *   '<S598>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF4_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF4_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S624>/Magic Tire Const Input'
                        *   '<S626>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF5_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF5_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S652>/Magic Tire Const Input'
                        *   '<S654>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF6_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF6_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S680>/Magic Tire Const Input'
                        *   '<S682>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF7_UNLOADED_RADIUS;
                       /* Mask Parameter: CombinedSlipWheel2DOF7_UNLOADED_RADIUS
                        * Referenced by:
                        *   '<S708>/Magic Tire Const Input'
                        *   '<S710>/Constant9'
                        */
  real_T CombinedSlipWheel2DOF_VERTICAL_DAMPING;
                       /* Mask Parameter: CombinedSlipWheel2DOF_VERTICAL_DAMPING
                        * Referenced by: '<S519>/Gain2'
                        */
  real_T CombinedSlipWheel2DOF1_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF1_VERTICAL_DAMPING
                       * Referenced by: '<S547>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF2_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF2_VERTICAL_DAMPING
                       * Referenced by: '<S575>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF3_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF3_VERTICAL_DAMPING
                       * Referenced by: '<S603>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF4_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF4_VERTICAL_DAMPING
                       * Referenced by: '<S631>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF5_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF5_VERTICAL_DAMPING
                       * Referenced by: '<S659>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF6_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF6_VERTICAL_DAMPING
                       * Referenced by: '<S687>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF7_VERTICAL_DAMPING;
                      /* Mask Parameter: CombinedSlipWheel2DOF7_VERTICAL_DAMPING
                       * Referenced by: '<S715>/Gain2'
                       */
  real_T CombinedSlipWheel2DOF_VERTICAL_STIFFNESS;
                     /* Mask Parameter: CombinedSlipWheel2DOF_VERTICAL_STIFFNESS
                      * Referenced by: '<S512>/Magic Tire Const Input'
                      */
  real_T CombinedSlipWheel2DOF1_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF1_VERTICAL_STIFFNESS
                     * Referenced by: '<S540>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF2_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF2_VERTICAL_STIFFNESS
                     * Referenced by: '<S568>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF3_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF3_VERTICAL_STIFFNESS
                     * Referenced by: '<S596>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF4_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF4_VERTICAL_STIFFNESS
                     * Referenced by: '<S624>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF5_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF5_VERTICAL_STIFFNESS
                     * Referenced by: '<S652>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF6_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF6_VERTICAL_STIFFNESS
                     * Referenced by: '<S680>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF7_VERTICAL_STIFFNESS;
                    /* Mask Parameter: CombinedSlipWheel2DOF7_VERTICAL_STIFFNESS
                     * Referenced by: '<S708>/Magic Tire Const Input'
                     */
  real_T CombinedSlipWheel2DOF_VXLOW;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_VXLOW
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_VXLOW
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_VXLOW
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_VXLOW
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_VXLOW
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_VXLOW
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_VXLOW
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_VXLOW;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_VXLOW
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF_WIDTH;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_WIDTH
                                   * Referenced by: '<S512>/Magic Tire Const Input'
                                   */
  real_T CombinedSlipWheel2DOF1_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_WIDTH
                                  * Referenced by: '<S540>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF2_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_WIDTH
                                  * Referenced by: '<S568>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF3_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_WIDTH
                                  * Referenced by: '<S596>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF4_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_WIDTH
                                  * Referenced by: '<S624>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF5_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_WIDTH
                                  * Referenced by: '<S652>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF6_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_WIDTH
                                  * Referenced by: '<S680>/Magic Tire Const Input'
                                  */
  real_T CombinedSlipWheel2DOF7_WIDTH;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_WIDTH
                                  * Referenced by: '<S708>/Magic Tire Const Input'
                                  */
  real_T independentSuspensionsMacPherson_WhlNumVec[2];
                   /* Mask Parameter: independentSuspensionsMacPherson_WhlNumVec
                    * Referenced by: '<S279>/Wheel Number'
                    */
  real_T SolidAxleSuspensionCoilSpring_WhlNumVec[2];
                      /* Mask Parameter: SolidAxleSuspensionCoilSpring_WhlNumVec
                       * Referenced by: '<S274>/Wheel Number'
                       */
  real_T VehicleBody3DOFDualTrack_X_o;
                                 /* Mask Parameter: VehicleBody3DOFDualTrack_X_o
                                  * Referenced by: '<S362>/X_oConstant'
                                  */
  real_T VehicleBody3DOFDualTrack_Y_o;
                                 /* Mask Parameter: VehicleBody3DOFDualTrack_Y_o
                                  * Referenced by: '<S362>/Y_oConstant'
                                  */
  real_T VehicleBody3DOFDualTrack_a;
                                   /* Mask Parameter: VehicleBody3DOFDualTrack_a
                                    * Referenced by:
                                    *   '<S362>/vehicle model'
                                    *   '<S377>/Constant3'
                                    *   '<S385>/a'
                                    *   '<S386>/a'
                                    */
  real_T LongitudinalDriver2_aR;       /* Mask Parameter: LongitudinalDriver2_aR
                                        * Referenced by: '<S191>/Setup'
                                        */
  real_T VehicleBody3DOFDualTrack_b;
                                   /* Mask Parameter: VehicleBody3DOFDualTrack_b
                                    * Referenced by:
                                    *   '<S362>/vehicle model'
                                    *   '<S377>/Constant3'
                                    *   '<S388>/b'
                                    *   '<S389>/b'
                                    */
  real_T LongitudinalDriver2_bR;       /* Mask Parameter: LongitudinalDriver2_bR
                                        * Referenced by: '<S191>/Setup'
                                        */
  real_T VehicleBody3DOFDualTrack_beta_w[11];
                              /* Mask Parameter: VehicleBody3DOFDualTrack_beta_w
                               * Referenced by:
                               *   '<S377>/Cs'
                               *   '<S377>/Cym'
                               */
  real_T CombinedSlipWheel2DOF_br;   /* Mask Parameter: CombinedSlipWheel2DOF_br
                                      * Referenced by: '<S525>/Clutch'
                                      */
  real_T CombinedSlipWheel2DOF1_br; /* Mask Parameter: CombinedSlipWheel2DOF1_br
                                     * Referenced by: '<S553>/Clutch'
                                     */
  real_T CombinedSlipWheel2DOF2_br; /* Mask Parameter: CombinedSlipWheel2DOF2_br
                                     * Referenced by: '<S581>/Clutch'
                                     */
  real_T CombinedSlipWheel2DOF3_br; /* Mask Parameter: CombinedSlipWheel2DOF3_br
                                     * Referenced by: '<S609>/Clutch'
                                     */
  real_T CombinedSlipWheel2DOF4_br; /* Mask Parameter: CombinedSlipWheel2DOF4_br
                                     * Referenced by: '<S637>/Clutch'
                                     */
  real_T CombinedSlipWheel2DOF5_br; /* Mask Parameter: CombinedSlipWheel2DOF5_br
                                     * Referenced by: '<S665>/Clutch'
                                     */
  real_T CombinedSlipWheel2DOF6_br; /* Mask Parameter: CombinedSlipWheel2DOF6_br
                                     * Referenced by: '<S693>/Clutch'
                                     */
  real_T CombinedSlipWheel2DOF7_br; /* Mask Parameter: CombinedSlipWheel2DOF7_br
                                     * Referenced by: '<S721>/Clutch'
                                     */
  real_T VerticalWheelandUnsprungMassResponse_bz;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse_bz
                       * Referenced by: '<S501>/Gain2'
                       */
  real_T VerticalWheelandUnsprungMassResponse1_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_bz
                      * Referenced by: '<S502>/Gain2'
                      */
  real_T VerticalWheelandUnsprungMassResponse2_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_bz
                      * Referenced by: '<S503>/Gain2'
                      */
  real_T VerticalWheelandUnsprungMassResponse3_bz;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_bz
                      * Referenced by: '<S504>/Gain2'
                      */
  real_T LongitudinalDriver2_cR;       /* Mask Parameter: LongitudinalDriver2_cR
                                        * Referenced by: '<S191>/Setup'
                                        */
  real_T CompareToConstant1_const;   /* Mask Parameter: CompareToConstant1_const
                                      * Referenced by: '<S158>/Constant'
                                      */
  real_T CompareToConstant3_const;   /* Mask Parameter: CompareToConstant3_const
                                      * Referenced by: '<S159>/Constant'
                                      */
  real_T CompareToConstant1_const_n;
                                   /* Mask Parameter: CompareToConstant1_const_n
                                    * Referenced by: '<S164>/Constant'
                                    */
  real_T CompareToConstant1_const_nq;
                                  /* Mask Parameter: CompareToConstant1_const_nq
                                   * Referenced by: '<S183>/Constant'
                                   */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S20>/Constant'
                                       */
  real_T Pressure_const;               /* Mask Parameter: Pressure_const
                                        * Referenced by: '<S471>/Pressure'
                                        */
  real_T CompareToConstant_const_e; /* Mask Parameter: CompareToConstant_const_e
                                     * Referenced by: '<S244>/Constant'
                                     */
  real_T CompareToConstant_const_k; /* Mask Parameter: CompareToConstant_const_k
                                     * Referenced by: '<S247>/Constant'
                                     */
  real_T VehicleBody3DOFDualTrack_d;
                                   /* Mask Parameter: VehicleBody3DOFDualTrack_d
                                    * Referenced by:
                                    *   '<S362>/vehicle model'
                                    *   '<S385>/d'
                                    *   '<S386>/d'
                                    *   '<S388>/d'
                                    *   '<S389>/d'
                                    *   '<S390>/Constant1'
                                    */
  real_T CombinedSlipWheel2DOF_disk_abore;
                             /* Mask Parameter: CombinedSlipWheel2DOF_disk_abore
                              * Referenced by: '<S524>/Disk brake actuator bore'
                              */
  real_T CombinedSlipWheel2DOF1_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF1_disk_abore
                             * Referenced by: '<S552>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF2_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF2_disk_abore
                             * Referenced by: '<S580>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF3_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF3_disk_abore
                             * Referenced by: '<S608>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF4_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF4_disk_abore
                             * Referenced by: '<S636>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF5_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF5_disk_abore
                             * Referenced by: '<S664>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF6_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF6_disk_abore
                             * Referenced by: '<S692>/Disk brake actuator bore'
                             */
  real_T CombinedSlipWheel2DOF7_disk_abore;
                            /* Mask Parameter: CombinedSlipWheel2DOF7_disk_abore
                             * Referenced by: '<S720>/Disk brake actuator bore'
                             */
  real_T LongitudinalDriver2_g;        /* Mask Parameter: LongitudinalDriver2_g
                                        * Referenced by: '<S191>/Setup'
                                        */
  real_T VehicleBody3DOFDualTrack_g;
                                   /* Mask Parameter: VehicleBody3DOFDualTrack_g
                                    * Referenced by:
                                    *   '<S362>/vehicle model'
                                    *   '<S383>/Constant'
                                    */
  real_T VerticalWheelandUnsprungMassResponse_g;
                       /* Mask Parameter: VerticalWheelandUnsprungMassResponse_g
                        * Referenced by: '<S501>/Fg'
                        */
  real_T VerticalWheelandUnsprungMassResponse1_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_g
                       * Referenced by: '<S502>/Fg'
                       */
  real_T VerticalWheelandUnsprungMassResponse2_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_g
                       * Referenced by: '<S503>/Fg'
                       */
  real_T VerticalWheelandUnsprungMassResponse3_g;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_g
                       * Referenced by: '<S504>/Fg'
                       */
  real_T VehicleBody3DOFDualTrack_h;
                                   /* Mask Parameter: VehicleBody3DOFDualTrack_h
                                    * Referenced by:
                                    *   '<S362>/vehicle model'
                                    *   '<S385>/h'
                                    *   '<S386>/h'
                                    *   '<S388>/h'
                                    *   '<S389>/h'
                                    *   '<S390>/Constant2'
                                    */
  real_T VehicleBody3DOFDualTrack_latOff;
                              /* Mask Parameter: VehicleBody3DOFDualTrack_latOff
                               * Referenced by: '<S387>/latOff'
                               */
  real_T VehicleBody3DOFDualTrack_longOff;
                             /* Mask Parameter: VehicleBody3DOFDualTrack_longOff
                              * Referenced by: '<S387>/longOff'
                              */
  real_T LongitudinalDriver2_m;        /* Mask Parameter: LongitudinalDriver2_m
                                        * Referenced by: '<S191>/Setup'
                                        */
  real_T VehicleBody3DOFDualTrack_m;
                                   /* Mask Parameter: VehicleBody3DOFDualTrack_m
                                    * Referenced by:
                                    *   '<S362>/vehicle model'
                                    *   '<S383>/Constant'
                                    *   '<S383>/Constant1'
                                    */
  real_T VerticalWheelandUnsprungMassResponse_m;
                       /* Mask Parameter: VerticalWheelandUnsprungMassResponse_m
                        * Referenced by:
                        *   '<S501>/Fg'
                        *   '<S501>/Gain1'
                        */
  real_T VerticalWheelandUnsprungMassResponse1_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_m
                       * Referenced by:
                       *   '<S502>/Fg'
                       *   '<S502>/Gain1'
                       */
  real_T VerticalWheelandUnsprungMassResponse2_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_m
                       * Referenced by:
                       *   '<S503>/Fg'
                       *   '<S503>/Gain1'
                       */
  real_T VerticalWheelandUnsprungMassResponse3_m;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_m
                       * Referenced by:
                       *   '<S504>/Fg'
                       *   '<S504>/Gain1'
                       */
  real_T CombinedSlipWheel2DOF_mu_kinetic;
                             /* Mask Parameter: CombinedSlipWheel2DOF_mu_kinetic
                              * Referenced by:
                              *   '<S522>/Ratio of static to kinetic'
                              *   '<S522>/Ratio of static to kinetic1'
                              *   '<S524>/Torque Conversion'
                              */
  real_T CombinedSlipWheel2DOF1_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF1_mu_kinetic
                             * Referenced by:
                             *   '<S550>/Ratio of static to kinetic'
                             *   '<S550>/Ratio of static to kinetic1'
                             *   '<S552>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF2_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF2_mu_kinetic
                             * Referenced by:
                             *   '<S578>/Ratio of static to kinetic'
                             *   '<S578>/Ratio of static to kinetic1'
                             *   '<S580>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF3_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF3_mu_kinetic
                             * Referenced by:
                             *   '<S606>/Ratio of static to kinetic'
                             *   '<S606>/Ratio of static to kinetic1'
                             *   '<S608>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF4_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF4_mu_kinetic
                             * Referenced by:
                             *   '<S634>/Ratio of static to kinetic'
                             *   '<S634>/Ratio of static to kinetic1'
                             *   '<S636>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF5_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF5_mu_kinetic
                             * Referenced by:
                             *   '<S662>/Ratio of static to kinetic'
                             *   '<S662>/Ratio of static to kinetic1'
                             *   '<S664>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF6_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF6_mu_kinetic
                             * Referenced by:
                             *   '<S690>/Ratio of static to kinetic'
                             *   '<S690>/Ratio of static to kinetic1'
                             *   '<S692>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF7_mu_kinetic;
                            /* Mask Parameter: CombinedSlipWheel2DOF7_mu_kinetic
                             * Referenced by:
                             *   '<S718>/Ratio of static to kinetic'
                             *   '<S718>/Ratio of static to kinetic1'
                             *   '<S720>/Torque Conversion'
                             */
  real_T CombinedSlipWheel2DOF_mu_static;
                              /* Mask Parameter: CombinedSlipWheel2DOF_mu_static
                               * Referenced by: '<S522>/Ratio of static to kinetic'
                               */
  real_T CombinedSlipWheel2DOF1_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF1_mu_static
                              * Referenced by: '<S550>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF2_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF2_mu_static
                              * Referenced by: '<S578>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF3_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF3_mu_static
                              * Referenced by: '<S606>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF4_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF4_mu_static
                              * Referenced by: '<S634>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF5_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF5_mu_static
                              * Referenced by: '<S662>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF6_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF6_mu_static
                              * Referenced by: '<S690>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF7_mu_static;
                             /* Mask Parameter: CombinedSlipWheel2DOF7_mu_static
                              * Referenced by: '<S718>/Ratio of static to kinetic'
                              */
  real_T CombinedSlipWheel2DOF_num_pads;
                               /* Mask Parameter: CombinedSlipWheel2DOF_num_pads
                                * Referenced by: '<S524>/Number of brake pads'
                                */
  real_T CombinedSlipWheel2DOF1_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF1_num_pads
                               * Referenced by: '<S552>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF2_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF2_num_pads
                               * Referenced by: '<S580>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF3_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF3_num_pads
                               * Referenced by: '<S608>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF4_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF4_num_pads
                               * Referenced by: '<S636>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF5_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF5_num_pads
                               * Referenced by: '<S664>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF6_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF6_num_pads
                               * Referenced by: '<S692>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF7_num_pads;
                              /* Mask Parameter: CombinedSlipWheel2DOF7_num_pads
                               * Referenced by: '<S720>/Number of brake pads'
                               */
  real_T CombinedSlipWheel2DOF_omegao;
                                 /* Mask Parameter: CombinedSlipWheel2DOF_omegao
                                  * Referenced by: '<S525>/Clutch'
                                  */
  real_T CombinedSlipWheel2DOF1_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF1_omegao
                                 * Referenced by: '<S553>/Clutch'
                                 */
  real_T CombinedSlipWheel2DOF2_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF2_omegao
                                 * Referenced by: '<S581>/Clutch'
                                 */
  real_T CombinedSlipWheel2DOF3_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF3_omegao
                                 * Referenced by: '<S609>/Clutch'
                                 */
  real_T CombinedSlipWheel2DOF4_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF4_omegao
                                 * Referenced by: '<S637>/Clutch'
                                 */
  real_T CombinedSlipWheel2DOF5_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF5_omegao
                                 * Referenced by: '<S665>/Clutch'
                                 */
  real_T CombinedSlipWheel2DOF6_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF6_omegao
                                 * Referenced by: '<S693>/Clutch'
                                 */
  real_T CombinedSlipWheel2DOF7_omegao;
                                /* Mask Parameter: CombinedSlipWheel2DOF7_omegao
                                 * Referenced by: '<S721>/Clutch'
                                 */
  real_T MachEMotor_power_max;         /* Mask Parameter: MachEMotor_power_max
                                        * Referenced by:
                                        *   '<S216>/If Action Subsystem1'
                                        *   '<S216>/Constant'
                                        */
  real_T VehicleBody3DOFDualTrack_psi_o;
                               /* Mask Parameter: VehicleBody3DOFDualTrack_psi_o
                                * Referenced by: '<S362>/psi_oConstant'
                                */
  real_T VehicleBody3DOFDualTrack_r_o;
                                 /* Mask Parameter: VehicleBody3DOFDualTrack_r_o
                                  * Referenced by: '<S362>/r_oConstant'
                                  */
  real_T LongitudinalDriver2_tau;     /* Mask Parameter: LongitudinalDriver2_tau
                                       * Referenced by:
                                       *   '<S191>/Setup'
                                       *   '<S193>/Constant'
                                       */
  real_T LongitudinalDriver_tauerr; /* Mask Parameter: LongitudinalDriver_tauerr
                                     * Referenced by: '<S179>/Constant'
                                     */
  real_T div0protectpoly_thresh;       /* Mask Parameter: div0protectpoly_thresh
                                        * Referenced by:
                                        *   '<S256>/Constant'
                                        *   '<S257>/Constant'
                                        */
  real_T MachEMotor_torque_max;        /* Mask Parameter: MachEMotor_torque_max
                                        * Referenced by:
                                        *   '<S216>/If Action Subsystem'
                                        *   '<S216>/Constant1'
                                        */
  real_T CombinedSlipWheel2DOF_turnslip;
                               /* Mask Parameter: CombinedSlipWheel2DOF_turnslip
                                * Referenced by: '<S512>/Magic Tire Const Input'
                                */
  real_T CombinedSlipWheel2DOF1_turnslip;
                              /* Mask Parameter: CombinedSlipWheel2DOF1_turnslip
                               * Referenced by: '<S540>/Magic Tire Const Input'
                               */
  real_T CombinedSlipWheel2DOF2_turnslip;
                              /* Mask Parameter: CombinedSlipWheel2DOF2_turnslip
                               * Referenced by: '<S568>/Magic Tire Const Input'
                               */
  real_T CombinedSlipWheel2DOF3_turnslip;
                              /* Mask Parameter: CombinedSlipWheel2DOF3_turnslip
                               * Referenced by: '<S596>/Magic Tire Const Input'
                               */
  real_T CombinedSlipWheel2DOF4_turnslip;
                              /* Mask Parameter: CombinedSlipWheel2DOF4_turnslip
                               * Referenced by: '<S624>/Magic Tire Const Input'
                               */
  real_T CombinedSlipWheel2DOF5_turnslip;
                              /* Mask Parameter: CombinedSlipWheel2DOF5_turnslip
                               * Referenced by: '<S652>/Magic Tire Const Input'
                               */
  real_T CombinedSlipWheel2DOF6_turnslip;
                              /* Mask Parameter: CombinedSlipWheel2DOF6_turnslip
                               * Referenced by: '<S680>/Magic Tire Const Input'
                               */
  real_T CombinedSlipWheel2DOF7_turnslip;
                              /* Mask Parameter: CombinedSlipWheel2DOF7_turnslip
                               * Referenced by: '<S708>/Magic Tire Const Input'
                               */
  real_T VehicleBody3DOFDualTrack_vertOff;
                             /* Mask Parameter: VehicleBody3DOFDualTrack_vertOff
                              * Referenced by: '<S387>/vertOff'
                              */
  real_T LongitudinalDriver_vnom;     /* Mask Parameter: LongitudinalDriver_vnom
                                       * Referenced by:
                                       *   '<S172>/Kff//vnom'
                                       *   '<S172>/Ki//vnom'
                                       *   '<S172>/Kp//vnom'
                                       */
  real_T VehicleBody3DOFDualTrack_w[2];
                                   /* Mask Parameter: VehicleBody3DOFDualTrack_w
                                    * Referenced by:
                                    *   '<S362>/vehicle model'
                                    *   '<S385>/w'
                                    *   '<S386>/w'
                                    *   '<S388>/w'
                                    *   '<S389>/w'
                                    */
  real_T ContLPF_wc;                   /* Mask Parameter: ContLPF_wc
                                        * Referenced by: '<S469>/Constant'
                                        */
  real_T ContLPF1_wc;                  /* Mask Parameter: ContLPF1_wc
                                        * Referenced by: '<S470>/Constant'
                                        */
  real_T VehicleBody3DOFDualTrack_xdot_o;
                              /* Mask Parameter: VehicleBody3DOFDualTrack_xdot_o
                               * Referenced by: '<S362>/xdot_oConstant'
                               */
  real_T VehicleBody3DOFDualTrack_xdot_tol;
                            /* Mask Parameter: VehicleBody3DOFDualTrack_xdot_tol
                             * Referenced by:
                             *   '<S362>/vehicle model'
                             *   '<S456>/Constant'
                             *   '<S457>/Constant'
                             *   '<S414>/Constant'
                             *   '<S415>/Constant'
                             */
  real_T VehicleBody3DOFDualTrack_ydot_o;
                              /* Mask Parameter: VehicleBody3DOFDualTrack_ydot_o
                               * Referenced by: '<S362>/ydot_oConstant'
                               */
  real_T CombinedSlipWheel2DOF_zdoto;
                                  /* Mask Parameter: CombinedSlipWheel2DOF_zdoto
                                   * Referenced by: '<S519>/Integrator, Second-Order'
                                   */
  real_T CombinedSlipWheel2DOF1_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF1_zdoto
                                  * Referenced by: '<S547>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF2_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF2_zdoto
                                  * Referenced by: '<S575>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF3_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF3_zdoto
                                  * Referenced by: '<S603>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF4_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF4_zdoto
                                  * Referenced by: '<S631>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF5_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF5_zdoto
                                  * Referenced by: '<S659>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF6_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF6_zdoto
                                  * Referenced by: '<S687>/Integrator, Second-Order'
                                  */
  real_T CombinedSlipWheel2DOF7_zdoto;
                                 /* Mask Parameter: CombinedSlipWheel2DOF7_zdoto
                                  * Referenced by: '<S715>/Integrator, Second-Order'
                                  */
  real_T VerticalWheelandUnsprungMassResponse_zdoto;
                   /* Mask Parameter: VerticalWheelandUnsprungMassResponse_zdoto
                    * Referenced by: '<S501>/Integrator, Second-Order'
                    */
  real_T VerticalWheelandUnsprungMassResponse1_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_zdoto
                   * Referenced by: '<S502>/Integrator, Second-Order'
                   */
  real_T VerticalWheelandUnsprungMassResponse2_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_zdoto
                   * Referenced by: '<S503>/Integrator, Second-Order'
                   */
  real_T VerticalWheelandUnsprungMassResponse3_zdoto;
                  /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_zdoto
                   * Referenced by: '<S504>/Integrator, Second-Order'
                   */
  real_T CombinedSlipWheel2DOF_zo;   /* Mask Parameter: CombinedSlipWheel2DOF_zo
                                      * Referenced by: '<S519>/Integrator, Second-Order'
                                      */
  real_T CombinedSlipWheel2DOF1_zo; /* Mask Parameter: CombinedSlipWheel2DOF1_zo
                                     * Referenced by: '<S547>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF2_zo; /* Mask Parameter: CombinedSlipWheel2DOF2_zo
                                     * Referenced by: '<S575>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF3_zo; /* Mask Parameter: CombinedSlipWheel2DOF3_zo
                                     * Referenced by: '<S603>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF4_zo; /* Mask Parameter: CombinedSlipWheel2DOF4_zo
                                     * Referenced by: '<S631>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF5_zo; /* Mask Parameter: CombinedSlipWheel2DOF5_zo
                                     * Referenced by: '<S659>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF6_zo; /* Mask Parameter: CombinedSlipWheel2DOF6_zo
                                     * Referenced by: '<S687>/Integrator, Second-Order'
                                     */
  real_T CombinedSlipWheel2DOF7_zo; /* Mask Parameter: CombinedSlipWheel2DOF7_zo
                                     * Referenced by: '<S715>/Integrator, Second-Order'
                                     */
  real_T VerticalWheelandUnsprungMassResponse_zo;
                      /* Mask Parameter: VerticalWheelandUnsprungMassResponse_zo
                       * Referenced by: '<S501>/Integrator, Second-Order'
                       */
  real_T VerticalWheelandUnsprungMassResponse1_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse1_zo
                      * Referenced by: '<S502>/Integrator, Second-Order'
                      */
  real_T VerticalWheelandUnsprungMassResponse2_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse2_zo
                      * Referenced by: '<S503>/Integrator, Second-Order'
                      */
  real_T VerticalWheelandUnsprungMassResponse3_zo;
                     /* Mask Parameter: VerticalWheelandUnsprungMassResponse3_zo
                      * Referenced by: '<S504>/Integrator, Second-Order'
                      */
  boolean_T DetectRisePositive_vinit;/* Mask Parameter: DetectRisePositive_vinit
                                      * Referenced by: '<S161>/Delay Input1'
                                      */
  boolean_T DetectFallNegative_vinit;/* Mask Parameter: DetectFallNegative_vinit
                                      * Referenced by: '<S160>/Delay Input1'
                                      */
  real_T RealSimInterpSpeed_speedUpperBound;/* Expression: 30
                                             * Referenced by: '<S4>/RealSimInterpSpeed'
                                             */
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S131>/Out1'
                                        */
  real_T MaxStopOffset_Value;          /* Expression: -5500
                                        * Referenced by: '<S131>/MaxStopOffset'
                                        */
  real_T Out1_Y0_p;                    /* Expression: [0]
                                        * Referenced by: '<S132>/Out1'
                                        */
  real_T MinStopOffset_Value;          /* Expression: 3500
                                        * Referenced by: '<S132>/MinStopOffset'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S162>/Constant'
                                        */
  real_T Constant_Value_b;             /* Expression: 0
                                        * Referenced by: '<S163>/Constant'
                                        */
  real_T ManSpdSel1_Value;             /* Expression: 2
                                        * Referenced by: '<S109>/ManSpdSel1'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: 5
                                        * Referenced by: '<S109>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -5
                                        * Referenced by: '<S109>/Rate Limiter'
                                        */
  real_T Gain2_Gain;                   /* Expression: 3.6
                                        * Referenced by: '<S109>/Gain2'
                                        */
  real_T Memory4_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S109>/Memory4'
                                        */
  real_T Switch1_Threshold;            /* Expression: 3
                                        * Referenced by: '<S109>/Switch1'
                                        */
  real_T Gain3_Gain;                   /* Expression: 3.6
                                        * Referenced by: '<S109>/Gain3'
                                        */
  real_T Constant_Value_e;             /* Expression: 0
                                        * Referenced by: '<S109>/Constant'
                                        */
  real_T Controlselection_Value;       /* Expression: 1
                                        * Referenced by: '<S153>/Control selection'
                                        */
  real_T Constant_Value_bj;            /* Expression: 0
                                        * Referenced by: '<S157>/Constant'
                                        */
  real_T Memory1_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S151>/Memory1'
                                        */
  real_T SpdSat_UpperSat;              /* Expression: 190
                                        * Referenced by: '<S151>/SpdSat'
                                        */
  real_T SpdSat_LowerSat;              /* Expression: -5
                                        * Referenced by: '<S151>/SpdSat'
                                        */
  real_T Memory1_InitialCondition_e;   /* Expression: 0
                                        * Referenced by: '<S157>/Memory1'
                                        */
  real_T Memory_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S157>/Memory'
                                        */
  real_T Limits5050_IC;                /* Expression: 0
                                        * Referenced by: '<S157>/Limits [-50,50]'
                                        */
  real_T Limits5050_UpperSat;          /* Expression: 50
                                        * Referenced by: '<S157>/Limits [-50,50]'
                                        */
  real_T Limits5050_LowerSat;          /* Expression: -50
                                        * Referenced by: '<S157>/Limits [-50,50]'
                                        */
  real_T LowpassFilter_A;              /* Computed Parameter: LowpassFilter_A
                                        * Referenced by: '<S157>/Lowpass Filter'
                                        */
  real_T LowpassFilter_C;              /* Computed Parameter: LowpassFilter_C
                                        * Referenced by: '<S157>/Lowpass Filter'
                                        */
  real_T Kp_Gain;                      /* Expression: 10
                                        * Referenced by: '<S157>/Kp'
                                        */
  real_T Memory2_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S151>/Memory2'
                                        */
  real_T ProportionalGainScheduling_tableData[3];/* Expression: [1 1 1]
                                                  * Referenced by: '<S157>/Proportional Gain Scheduling'
                                                  */
  real_T ProportionalGainScheduling_bp01Data[3];/* Expression: [ 0 30 70]
                                                 * Referenced by: '<S157>/Proportional Gain Scheduling'
                                                 */
  real_T Feedforward1_tableData[17];
  /* Expression: [0 2.6021    4.7790    6.8575    8.1555    9.4176   10.7914   12.3245   14.1570   15.9699   17.4605   19.0355   21.3079   22.8407   25.3373   26.9453   28.6975]
   * Referenced by: '<S157>/Feedforward1'
   */
  real_T Feedforward1_bp01Data[17];
  /* Expression: [0 10    20    30    40    50    60    70    80    90   100   110   120   130   140   150   160]
   * Referenced by: '<S157>/Feedforward1'
   */
  real_T FeedforwardGain_Gain;         /* Expression: 1
                                        * Referenced by: '<S157>/FeedforwardGain'
                                        */
  real_T Switch1_Threshold_d;          /* Expression: 0
                                        * Referenced by: '<S157>/Switch1'
                                        */
  real_T Constant_Value_l;             /* Expression: 0
                                        * Referenced by: '<S155>/Constant'
                                        */
  real_T Gain_Gain;                    /* Expression: 1/3.6
                                        * Referenced by: '<S155>/Gain'
                                        */
  real_T Constant_Value_ea;            /* Expression: 0
                                        * Referenced by: '<S173>/Constant'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S180>/Integrator'
                                        */
  real_T Integrator1_IC;               /* Expression: 0
                                        * Referenced by: '<S172>/Integrator1'
                                        */
  real_T Integrator1_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S172>/Integrator1'
                                        */
  real_T Integrator1_LowerSat;         /* Expression: -1
                                        * Referenced by: '<S172>/Integrator1'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S155>/Constant1'
                                        */
  real_T uto1_UpperSat;                /* Expression: 1
                                        * Referenced by: '<S172>/-1 to 1 '
                                        */
  real_T uto1_LowerSat;                /* Expression: -1
                                        * Referenced by: '<S172>/-1 to 1 '
                                        */
  real_T u1_UpperSat;                  /* Expression: 1
                                        * Referenced by: '<S173>/0~1'
                                        */
  real_T u1_LowerSat;                  /* Expression: 0
                                        * Referenced by: '<S173>/0~1'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 1
                                        * Referenced by: '<S173>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S173>/Saturation'
                                        */
  real_T Gain2_Gain_c;                 /* Expression: 100
                                        * Referenced by: '<S155>/Gain2'
                                        */
  real_T Constant_Value_i;             /* Expression: 0
                                        * Referenced by: '<S156>/Constant'
                                        */
  real_T Gain_Gain_i;                  /* Expression: 1/3.6
                                        * Referenced by: '<S156>/Gain'
                                        */
  real_T Constant_Value_k;             /* Expression: 0
                                        * Referenced by: '<S192>/Constant'
                                        */
  real_T Integrator1_IC_a;             /* Expression: 0
                                        * Referenced by: '<S193>/Integrator1'
                                        */
  real_T u1_UpperSat_f;                /* Expression: 1
                                        * Referenced by: '<S192>/0~1'
                                        */
  real_T u1_LowerSat_c;                /* Expression: 0
                                        * Referenced by: '<S192>/0~1'
                                        */
  real_T Saturation_UpperSat_h;        /* Expression: 1
                                        * Referenced by: '<S192>/Saturation'
                                        */
  real_T Saturation_LowerSat_b;        /* Expression: 0
                                        * Referenced by: '<S192>/Saturation'
                                        */
  real_T Gain4_Gain;                   /* Expression: 100
                                        * Referenced by: '<S156>/Gain4'
                                        */
  real_T standstillbrake0100_Value;    /* Expression: 50
                                        * Referenced by: '<S157>/standstill brake [0,100]'
                                        */
  real_T Gain_Gain_k;                  /* Expression: -1
                                        * Referenced by: '<S157>/Gain'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0
                                        * Referenced by: '<S157>/Switch2'
                                        */
  real_T standstillbrake01_Value;      /* Expression: 0.5
                                        * Referenced by: '<S155>/standstill brake [0,1]'
                                        */
  real_T Constant_Value_if;            /* Expression: 0
                                        * Referenced by: '<S174>/Constant'
                                        */
  real_T u0_UpperSat;                  /* Expression: 0
                                        * Referenced by: '<S174>/-1~0'
                                        */
  real_T u0_LowerSat;                  /* Expression: -1
                                        * Referenced by: '<S174>/-1~0'
                                        */
  real_T Saturation_UpperSat_b;        /* Expression: 1
                                        * Referenced by: '<S174>/Saturation'
                                        */
  real_T Saturation_LowerSat_bp;       /* Expression: 0
                                        * Referenced by: '<S174>/Saturation'
                                        */
  real_T Gain3_Gain_c;                 /* Expression: 100
                                        * Referenced by: '<S155>/Gain3'
                                        */
  real_T standstillbrake01_Value_m;    /* Expression: 0.5
                                        * Referenced by: '<S156>/standstill brake [0,1]'
                                        */
  real_T Constant_Value_a;             /* Expression: 0
                                        * Referenced by: '<S194>/Constant'
                                        */
  real_T u0_UpperSat_n;                /* Expression: 0
                                        * Referenced by: '<S194>/-1~0'
                                        */
  real_T u0_LowerSat_c;                /* Expression: -1
                                        * Referenced by: '<S194>/-1~0'
                                        */
  real_T Saturation_UpperSat_d;        /* Expression: 1
                                        * Referenced by: '<S194>/Saturation'
                                        */
  real_T Saturation_LowerSat_g;        /* Expression: 0
                                        * Referenced by: '<S194>/Saturation'
                                        */
  real_T Gain5_Gain;                   /* Expression: 100
                                        * Referenced by: '<S156>/Gain5'
                                        */
  real_T MinthresholdforBrakeswitch_Value;/* Expression: 0
                                           * Referenced by: '<S157>/Min threshold for Brake switch'
                                           */
  real_T MinthresholdforBrakeswitch_Value_k;/* Expression: 0
                                             * Referenced by: '<S155>/Min threshold for Brake switch'
                                             */
  real_T MinthresholdforBrakeswitch_Value_e;/* Expression: 0
                                             * Referenced by: '<S156>/Min threshold for Brake switch'
                                             */
  real_T Constant_Value_f;             /* Expression: 1
                                        * Referenced by: '<S151>/Constant'
                                        */
  real_T Ki_Gain;                      /* Expression: 1
                                        * Referenced by: '<S157>/Ki'
                                        */
  real_T Feedforward_tableData[17];
       /* Expression: [0	3.5	12.5	16	20	21	22	24.5	26	29.5	32	33	34	36	37	38	39]
        * Referenced by: '<S157>/Feedforward'
        */
  real_T Feedforward_bp01Data[17];
  /* Expression: [0 10    20    30    40    50    60    70    80    90   100   110   120   130   140   150   160]
   * Referenced by: '<S157>/Feedforward'
   */
  real_T LowpassFilter_A_o;            /* Computed Parameter: LowpassFilter_A_o
                                        * Referenced by: '<S155>/Lowpass Filter'
                                        */
  real_T LowpassFilter_C_f;            /* Computed Parameter: LowpassFilter_C_f
                                        * Referenced by: '<S155>/Lowpass Filter'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1/3.6
                                        * Referenced by: '<S155>/Gain1'
                                        */
  real_T Zero_Value;                   /* Expression: 0
                                        * Referenced by: '<S167>/Zero'
                                        */
  real_T Integrator2_IC;               /* Expression: 0
                                        * Referenced by: '<S181>/Integrator2'
                                        */
  real_T UnitDelay_InitialCondition[2];/* Expression: [0,0]
                                        * Referenced by: '<S181>/Unit Delay'
                                        */
  real_T Constant_Value_e3;            /* Expression: 1
                                        * Referenced by: '<S182>/Constant'
                                        */
  real_T Constant1_Value_o;            /* Expression: 0
                                        * Referenced by: '<S182>/Constant1'
                                        */
  real_T Constant1_Value_p;            /* Expression: 0
                                        * Referenced by: '<S156>/Constant1'
                                        */
  real_T LowpassFilter_A_l;            /* Computed Parameter: LowpassFilter_A_l
                                        * Referenced by: '<S156>/Lowpass Filter'
                                        */
  real_T LowpassFilter_C_d;            /* Computed Parameter: LowpassFilter_C_d
                                        * Referenced by: '<S156>/Lowpass Filter'
                                        */
  real_T Gain1_Gain_j;                 /* Expression: 1/3.6
                                        * Referenced by: '<S156>/Gain1'
                                        */
  real_T Integrator2_IC_o;             /* Expression: 0
                                        * Referenced by: '<S204>/Integrator2'
                                        */
  real_T UnitDelay_InitialCondition_l[2];/* Expression: [0,0]
                                          * Referenced by: '<S204>/Unit Delay'
                                          */
  real_T Constant_Value_p;             /* Expression: 1
                                        * Referenced by: '<S205>/Constant'
                                        */
  real_T Constant1_Value_b;            /* Expression: 0
                                        * Referenced by: '<S205>/Constant1'
                                        */
  real_T Zero_Value_m;                 /* Expression: 0
                                        * Referenced by: '<S186>/Zero'
                                        */
  real_T kmtomiles_Gain;               /* Expression: 1/1.609344
                                        * Referenced by: '<S151>/km to miles'
                                        */
  real_T Gain_Gain_a;                  /* Expression: 1/1.609
                                        * Referenced by: '<S109>/Gain'
                                        */
  real_T Gain1_Gain_d;                 /* Expression: 1/1.609
                                        * Referenced by: '<S109>/Gain1'
                                        */
  real_T driverlookaheadseconds_Value; /* Expression: 0.5
                                        * Referenced by: '<S109>/driver look ahead seconds'
                                        */
  real_T Memory4_InitialCondition_m;   /* Expression: 0
                                        * Referenced by: '<S152>/Memory4'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S152>/Switch'
                                        */
  real_T ManSpd_Value;                 /* Expression: 0
                                        * Referenced by: '<S109>/ManSpd'
                                        */
  real_T vehiclemodel_Fxtire_sat;      /* Expression: Fxtire_sat
                                        * Referenced by: '<S362>/vehicle model'
                                        */
  real_T vehiclemodel_Fytire_sat;      /* Expression: Fytire_sat
                                        * Referenced by: '<S362>/vehicle model'
                                        */
  real_T vehiclemodel_Fznom;           /* Expression: Fznom
                                        * Referenced by: '<S362>/vehicle model'
                                        */
  real_T Constant_Value_l4;            /* Expression: 0
                                        * Referenced by: '<S21>/Constant'
                                        */
  real_T Constant_Value_e1;            /* Expression: 0
                                        * Referenced by: '<S251>/Constant'
                                        */
  real_T Constant_Value_e0;            /* Expression: 0
                                        * Referenced by: '<S252>/Constant'
                                        */
  real_T Brake_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/Brake'
                                        */
  real_T Memory1_InitialCondition_d;   /* Expression: 0
                                        * Referenced by: '<S206>/Memory1'
                                        */
  real_T Integrator_IC_k;              /* Expression: 0
                                        * Referenced by: '<S216>/Integrator'
                                        */
  real_T Merge_InitialOutput;          /* Expression: 0
                                        * Referenced by: '<S216>/Merge'
                                        */
  real_T IntegratorLimited_LowerSat;   /* Expression: 0
                                        * Referenced by: '<S144>/Integrator Limited'
                                        */
  real_T Memory_InitialCondition_b;    /* Expression: 0
                                        * Referenced by: '<S111>/Memory'
                                        */
  real_T Integrator_IC_m;              /* Expression: 0
                                        * Referenced by: '<S100>/Integrator'
                                        */
  real_T mtomile_Gain;                 /* Expression: 0.000621371
                                        * Referenced by: '<S100>/m to mile'
                                        */
  real_T Integrator1_IC_p;             /* Expression: 0
                                        * Referenced by: '<S100>/Integrator1'
                                        */
  real_T m3toUSGal_Gain;               /* Expression: 264.172
                                        * Referenced by: '<S100>/m^3 to US Gal'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: Inf
                                        * Referenced by: '<S100>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: 1e-3
                                        * Referenced by: '<S100>/Saturation1'
                                        */
  real_T PedalRobotControlSelection_Value;/* Expression: 2
                                           * Referenced by: '<S107>/Pedal Robot Control Selection'
                                           */
  real_T APP_Value;                    /* Expression: 0
                                        * Referenced by: '<S107>/APP'
                                        */
  real_T Gain_Gain_g;                  /* Expression: 3.6
                                        * Referenced by: '<S101>/Gain'
                                        */
  real_T DriveCycle_trigger_Value;     /* Expression: 0
                                        * Referenced by: '<S101>/DriveCycle_trigger'
                                        */
  real_T SUMO_speed_level_Value;       /* Expression: 0
                                        * Referenced by: '<S101>/SUMO_speed_level'
                                        */
  real_T RS_actualSpeedSourceSelector_Value;/* Expression: 3
                                             * Referenced by: '<S4>/RS_actualSpeedSourceSelector'
                                             */
  real_T Memory_InitialCondition_p;    /* Expression: 0
                                        * Referenced by: '<S4>/Memory'
                                        */
  real_T Memory_InitialCondition_e;    /* Expression: 0
                                        * Referenced by: '<Root>/Memory'
                                        */
  real_T Constant2_Value;              /* Expression: 3
                                        * Referenced by: '<S12>/Constant2'
                                        */
  real_T SimState_Value;               /* Expression: 10
                                        * Referenced by: '<S1>/SimState'
                                        */
  real_T DistanceTravelm_Value;        /* Expression: 0
                                        * Referenced by: '<S1>/DistanceTravel (m)'
                                        */
  real_T initialWaitTime_Value;        /* Expression: 10000
                                        * Referenced by: '<S1>/initialWaitTime'
                                        */
  real_T totalDistancem_Value;         /* Expression: 1000000
                                        * Referenced by: '<S1>/totalDistance (m)'
                                        */
  real_T totalTimesec_Value;           /* Expression: 10000
                                        * Referenced by: '<S1>/totalTime (sec)'
                                        */
  real_T RampDownFlag_Value;           /* Expression: 0
                                        * Referenced by: '<S1>/RampDownFlag'
                                        */
  real_T RampUpFlag_Value;             /* Expression: 0
                                        * Referenced by: '<S1>/RampUpFlag'
                                        */
  real_T Gain2_Gain_i;                 /* Expression: 1/3.6
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T Gain_Gain_e;                  /* Expression: 100
                                        * Referenced by: '<S107>/Gain'
                                        */
  real_T Constant_Value_m;             /* Expression: 0
                                        * Referenced by: '<S107>/Constant'
                                        */
  real_T AR_limit_UpperSat;            /* Expression: 65
                                        * Referenced by: '<S107>/AR_limit'
                                        */
  real_T AR_limit_LowerSat;            /* Expression: 0
                                        * Referenced by: '<S107>/AR_limit'
                                        */
  real_T Gain_Gain_e0;                 /* Expression: 0.01
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T BR_command_0to100_Value;      /* Expression: 0
                                        * Referenced by: '<S107>/BR_command_0to100'
                                        */
  real_T Brake_ref_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Brake_ref'
                                        */
  real_T Gain1_Gain_a;                 /* Expression: 100
                                        * Referenced by: '<S107>/Gain1'
                                        */
  real_T Constant1_Value_h;            /* Expression: 0
                                        * Referenced by: '<S107>/Constant1'
                                        */
  real_T BR_limit_UpperSat;            /* Expression: 70
                                        * Referenced by: '<S107>/BR_limit'
                                        */
  real_T BR_limit_LowerSat;            /* Expression: 0
                                        * Referenced by: '<S107>/BR_limit'
                                        */
  real_T Gain1_Gain_l;                 /* Expression: 0.01
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Steer_ref_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Steer_ref'
                                        */
  real_T Constant_Value_j;             /* Expression: 0
                                        * Referenced by: '<S462>/Constant'
                                        */
  real_T Constant_Value_at;            /* Expression: 0
                                        * Referenced by: '<S465>/Constant'
                                        */
  real_T Constant_Value_ed;            /* Expression: mu
                                        * Referenced by: '<S460>/Constant'
                                        */
  real_T Constant_Value_fq;            /* Expression: 0
                                        * Referenced by: '<S481>/Constant'
                                        */
  real_T Memory_InitialCondition_g;    /* Expression: 0
                                        * Referenced by: '<S206>/Memory'
                                        */
  real_T Gain4_Gain_n;                 /* Expression: 0.25
                                        * Referenced by: '<S206>/Gain4'
                                        */
  real_T Gain4_Gain_g[4];              /* Expression: [1;-1;1;-1]*0+1
                                        * Referenced by: '<S474>/Gain4'
                                        */
  real_T Integrator_IC_n;              /* Expression: 0
                                        * Referenced by: '<S518>/Integrator'
                                        */
  real_T Integrator_IC_d;              /* Expression: 0
                                        * Referenced by: '<S515>/Integrator'
                                        */
  real_T GroundZLevel_Value[4];        /* Expression: zeros(4,1)
                                        * Referenced by: '<S98>/Ground Z Level'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: inf
                                        * Referenced by: '<S519>/Saturation'
                                        */
  real_T Saturation_LowerSat_h;        /* Expression: 0
                                        * Referenced by: '<S519>/Saturation'
                                        */
  real_T Saturation_UpperSat_ej;       /* Expression: inf
                                        * Referenced by: '<S514>/Saturation'
                                        */
  real_T Saturation_LowerSat_o;        /* Expression: 0
                                        * Referenced by: '<S514>/Saturation'
                                        */
  real_T DriveOption_Value;            /* Expression: 1
                                        * Referenced by: '<Root>/DriveOption'
                                        */
  real_T Constant6_Value;              /* Expression: 0.5
                                        * Referenced by: '<S3>/Constant6'
                                        */
  real_T Switch1_Threshold_dj;         /* Expression: 0
                                        * Referenced by: '<S3>/Switch1'
                                        */
  real_T Saturation_UpperSat_n;        /* Expression: 1
                                        * Referenced by: '<S242>/Saturation'
                                        */
  real_T Saturation_LowerSat_m;        /* Expression: 0
                                        * Referenced by: '<S242>/Saturation'
                                        */
  real_T Saturation1_UpperSat_g;       /* Expression: inf
                                        * Referenced by: '<S258>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_a;       /* Expression: 1
                                        * Referenced by: '<S258>/Saturation1'
                                        */
  real_T Saturation_LowerSat_k;        /* Expression: 0
                                        * Referenced by: '<S258>/Saturation'
                                        */
  real_T Saturation1_UpperSat_n;       /* Expression: 1
                                        * Referenced by: '<S242>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_c;       /* Expression: 0
                                        * Referenced by: '<S242>/Saturation1'
                                        */
  real_T Gain2_Gain_p;                 /* Expression: 0.5
                                        * Referenced by: '<S260>/Gain2'
                                        */
  real_T Gain3_Gain_o;                 /* Expression: 0.5
                                        * Referenced by: '<S260>/Gain3'
                                        */
  real_T Saturation_LowerSat_m4;       /* Expression: 0
                                        * Referenced by: '<S260>/Saturation'
                                        */
  real_T Saturation1_LowerSat_l;       /* Expression: 0
                                        * Referenced by: '<S260>/Saturation1'
                                        */
  real_T Gain1_Gain_je;                /* Expression: 0.5
                                        * Referenced by: '<S260>/Gain1'
                                        */
  real_T Gain4_Gain_gs;                /* Expression: 0.5
                                        * Referenced by: '<S260>/Gain4'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S260>/Saturation2'
                                        */
  real_T Saturation3_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S260>/Saturation3'
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
  real_T Switch_Threshold_d;           /* Expression: 0
                                        * Referenced by: '<S481>/Switch'
                                        */
  real_T Constant1_Value_k;            /* Expression: 0
                                        * Referenced by: '<S481>/Constant1'
                                        */
  real_T Integrator_IC_kz;             /* Expression: 0
                                        * Referenced by: '<S546>/Integrator'
                                        */
  real_T Integrator_IC_df;             /* Expression: 0
                                        * Referenced by: '<S543>/Integrator'
                                        */
  real_T Saturation_UpperSat_em;       /* Expression: inf
                                        * Referenced by: '<S547>/Saturation'
                                        */
  real_T Saturation_LowerSat_ky;       /* Expression: 0
                                        * Referenced by: '<S547>/Saturation'
                                        */
  real_T Saturation_UpperSat_i;        /* Expression: inf
                                        * Referenced by: '<S542>/Saturation'
                                        */
  real_T Saturation_LowerSat_mm;       /* Expression: 0
                                        * Referenced by: '<S542>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_o;     /* Expression: pi/4
                                        * Referenced by: '<S552>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_a;/* Expression: inf
                                                 * Referenced by: '<S552>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_n;/* Expression: eps
                                                 * Referenced by: '<S552>/Disallow Negative Brake Torque'
                                                 */
  real_T Switch1_Threshold_n;          /* Expression: 0
                                        * Referenced by: '<S481>/Switch1'
                                        */
  real_T Constant2_Value_e;            /* Expression: 0
                                        * Referenced by: '<S481>/Constant2'
                                        */
  real_T Integrator_IC_p;              /* Expression: 0
                                        * Referenced by: '<S574>/Integrator'
                                        */
  real_T Integrator_IC_k5;             /* Expression: 0
                                        * Referenced by: '<S571>/Integrator'
                                        */
  real_T Saturation_UpperSat_g;        /* Expression: inf
                                        * Referenced by: '<S575>/Saturation'
                                        */
  real_T Saturation_LowerSat_i;        /* Expression: 0
                                        * Referenced by: '<S575>/Saturation'
                                        */
  real_T Saturation_UpperSat_nr;       /* Expression: inf
                                        * Referenced by: '<S570>/Saturation'
                                        */
  real_T Saturation_LowerSat_n;        /* Expression: 0
                                        * Referenced by: '<S570>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_b;     /* Expression: pi/4
                                        * Referenced by: '<S580>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_p;/* Expression: inf
                                                 * Referenced by: '<S580>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_f;/* Expression: eps
                                                 * Referenced by: '<S580>/Disallow Negative Brake Torque'
                                                 */
  real_T Switch2_Threshold_a;          /* Expression: 0
                                        * Referenced by: '<S481>/Switch2'
                                        */
  real_T Constant3_Value;              /* Expression: 0
                                        * Referenced by: '<S481>/Constant3'
                                        */
  real_T Integrator_IC_o;              /* Expression: 0
                                        * Referenced by: '<S602>/Integrator'
                                        */
  real_T Integrator_IC_na;             /* Expression: 0
                                        * Referenced by: '<S599>/Integrator'
                                        */
  real_T Saturation_UpperSat_k;        /* Expression: inf
                                        * Referenced by: '<S603>/Saturation'
                                        */
  real_T Saturation_LowerSat_h0;       /* Expression: 0
                                        * Referenced by: '<S603>/Saturation'
                                        */
  real_T Saturation_UpperSat_be;       /* Expression: inf
                                        * Referenced by: '<S598>/Saturation'
                                        */
  real_T Saturation_LowerSat_d;        /* Expression: 0
                                        * Referenced by: '<S598>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_c;     /* Expression: pi/4
                                        * Referenced by: '<S608>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_m;/* Expression: inf
                                                 * Referenced by: '<S608>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_p;/* Expression: eps
                                                 * Referenced by: '<S608>/Disallow Negative Brake Torque'
                                                 */
  real_T Switch3_Threshold;            /* Expression: 0
                                        * Referenced by: '<S481>/Switch3'
                                        */
  real_T DeadZone2_Start;              /* Expression: -5
                                        * Referenced by: '<S472>/Dead Zone2'
                                        */
  real_T DeadZone2_End;                /* Expression: 5
                                        * Referenced by: '<S472>/Dead Zone2'
                                        */
  real_T Integrator_IC_e;              /* Expression: 0
                                        * Referenced by: '<S516>/Integrator'
                                        */
  real_T Integrator_IC_es;             /* Expression: 0
                                        * Referenced by: '<S544>/Integrator'
                                        */
  real_T Integrator_IC_a;              /* Expression: 0
                                        * Referenced by: '<S572>/Integrator'
                                        */
  real_T Integrator_IC_ke;             /* Expression: 0
                                        * Referenced by: '<S600>/Integrator'
                                        */
  real_T DeadZone3_Start;              /* Expression: -10
                                        * Referenced by: '<S472>/Dead Zone3'
                                        */
  real_T DeadZone3_End;                /* Expression: 10
                                        * Referenced by: '<S472>/Dead Zone3'
                                        */
  real_T Integrator1_IC_l;             /* Expression: 0
                                        * Referenced by: '<S469>/Integrator1'
                                        */
  real_T Saturation_UpperSat_b2;       /* Expression: 10*9.81*2040
                                        * Referenced by: '<S267>/Saturation'
                                        */
  real_T Saturation_LowerSat_mi;       /* Expression: -10*9.81*2040
                                        * Referenced by: '<S267>/Saturation'
                                        */
  real_T Constant3_Value_a[4];         /* Expression: zeros(4,1)
                                        * Referenced by: '<S267>/Constant3'
                                        */
  real_T Integrator1_IC_b;             /* Expression: 0
                                        * Referenced by: '<S470>/Integrator1'
                                        */
  real_T Constant1_Value_l;            /* Expression: pi
                                        * Referenced by: '<S477>/Constant1'
                                        */
  real_T Constant3_Value_h[4];         /* Expression: ones(1,4).*0
                                        * Referenced by: '<S477>/Constant3'
                                        */
  real_T Constant2_Value_m[4];     /* Expression: [pi;0;pi;0].*0+[0;pi;0;pi].*.0
                                    * Referenced by: '<S477>/Constant2'
                                    */
  real_T Constant_Value_ld;            /* Expression: 0
                                        * Referenced by: '<S384>/Constant'
                                        */
  real_T Gain_Gain_d;                  /* Expression: pi
                                        * Referenced by: '<S454>/Gain'
                                        */
  real_T Constant2_Value_mc;           /* Expression: pi
                                        * Referenced by: '<S454>/Constant2'
                                        */
  real_T Constant3_Value_a5;           /* Expression: 2
                                        * Referenced by: '<S454>/Constant3'
                                        */
  real_T Switch_Threshold_m;           /* Expression: 0
                                        * Referenced by: '<S454>/Switch'
                                        */
  real_T Constant7_Value;              /* Expression: 0
                                        * Referenced by: '<S384>/Constant7'
                                        */
  real_T Constant2_Value_eq;           /* Expression: 0
                                        * Referenced by: '<S384>/Constant2'
                                        */
  real_T Gain_Gain_f;                  /* Expression: -1/2
                                        * Referenced by: '<S385>/Gain'
                                        */
  real_T Gain_Gain_ir;                 /* Expression: 1/2
                                        * Referenced by: '<S386>/Gain'
                                        */
  real_T Gain_Gain_b;                  /* Expression: -1/2
                                        * Referenced by: '<S388>/Gain'
                                        */
  real_T Gain_Gain_n;                  /* Expression: 1/2
                                        * Referenced by: '<S389>/Gain'
                                        */
  real_T InertialFrameCGtoAxleOffset_Value[12];
                                     /* Expression: [zeros(2,4);0.134*ones(1,4)]
                                      * Referenced by: '<S277>/Inertial Frame CG to Axle Offset'
                                      */
  real_T Constant5_Value;              /* Expression: 0
                                        * Referenced by: '<S384>/Constant5'
                                        */
  real_T Constant4_Value;              /* Expression: 0
                                        * Referenced by: '<S384>/Constant4'
                                        */
  real_T Constant6_Value_e;            /* Expression: 0
                                        * Referenced by: '<S384>/Constant6'
                                        */
  real_T Steer_Value;                  /* Expression: 0
                                        * Referenced by: '<S3>/Steer'
                                        */
  real_T Switch2_Threshold_e;          /* Expression: 0
                                        * Referenced by: '<S3>/Switch2'
                                        */
  real_T EVBoltSteer_tableData[183];
  /* Expression: [-9.10000000000000	-9	-8.90000000000000	-8.80000000000000	-8.70000000000000	-8.60000000000000	-8.50000000000000	-8.40000000000000	-8.30000000000000	-8.20000000000000	-8.10000000000000	-8	-7.90000000000000	-7.80000000000000	-7.70000000000000	-7.60000000000000	-7.50000000000000	-7.40000000000000	-7.30000000000000	-7.20000000000000	-7.10000000000000	-7	-6.90000000000000	-6.80000000000000	-6.70000000000000	-6.60000000000000	-6.50000000000000	-6.40000000000000	-6.30000000000000	-6.20000000000000	-6.10000000000000	-6	-5.90000000000000	-5.80000000000000	-5.70000000000000	-5.60000000000000	-5.50000000000000	-5.40000000000000	-5.30000000000000	-5.20000000000000	-5.10000000000000	-5	-4.90000000000000	-4.80000000000000	-4.70000000000000	-4.60000000000000	-4.50000000000000	-4.40000000000000	-4.30000000000000	-4.20000000000000	-4.10000000000000	-4	-3.90000000000000	-3.80000000000000	-3.70000000000000	-3.60000000000000	-3.50000000000000	-3.40000000000000	-3.30000000000000	-3.20000000000000	-3.10000000000000	-3	-2.90000000000000	-2.80000000000000	-2.70000000000000	-2.60000000000000	-2.50000000000000	-2.40000000000000	-2.30000000000000	-2.20000000000000	-2.10000000000000	-2	-1.90000000000000	-1.80000000000000	-1.70000000000000	-1.60000000000000	-1.50000000000000	-1.40000000000000	-1.30000000000000	-1.20000000000000	-1.10000000000000	-1	-0.900000000000000	-0.800000000000000	-0.700000000000000	-0.600000000000000	-0.500000000000000	-0.400000000000000	-0.300000000000000	-0.200000000000000	-0.100000000000000	0	0.100000000000000	0.200000000000000	0.300000000000000	0.400000000000000	0.500000000000000	0.600000000000000	0.700000000000000	0.800000000000000	0.900000000000000	1	1.10000000000000	1.20000000000000	1.30000000000000	1.40000000000000	1.50000000000000	1.60000000000000	1.70000000000000	1.80000000000000	1.90000000000000	2	2.10000000000000	2.20000000000000	2.30000000000000	2.40000000000000	2.50000000000000	2.60000000000000	2.70000000000000	2.80000000000000	2.90000000000000	3	3.10000000000000	3.20000000000000	3.30000000000000	3.40000000000000	3.50000000000000	3.60000000000000	3.70000000000000	3.80000000000000	3.90000000000000	4	4.10000000000000	4.20000000000000	4.30000000000000	4.40000000000000	4.50000000000000	4.60000000000000	4.70000000000000	4.80000000000000	4.90000000000000	5	5.10000000000000	5.20000000000000	5.30000000000000	5.40000000000000	5.50000000000000	5.60000000000000	5.70000000000000	5.80000000000000	5.90000000000000	6	6.10000000000000	6.20000000000000	6.30000000000000	6.40000000000000	6.50000000000000	6.60000000000000	6.70000000000000	6.80000000000000	6.90000000000000	7	7.10000000000000	7.20000000000000	7.30000000000000	7.40000000000000	7.50000000000000	7.60000000000000	7.70000000000000	7.80000000000000	7.90000000000000	8	8.10000000000000	8.20000000000000	8.30000000000000	8.40000000000000	8.50000000000000	8.60000000000000	8.70000000000000	8.80000000000000	8.90000000000000	9	9.10000000000000]
   * Referenced by: '<S3>/EV Bolt Steer'
   */
  real_T EVBoltSteer_bp01Data[183];
  /* Expression: [-0.541571931581492	-0.535619550629112	-0.529667169676731	-0.523714788724350	-0.517762407771969	-0.511810026819588	-0.505857645867207	-0.499905264914826	-0.493952883962445	-0.488000503010064	-0.482048122057683	-0.476095741105302	-0.470143360152921	-0.464190979200541	-0.458238598248160	-0.452286217295779	-0.446333836343398	-0.440381455391017	-0.434429074438636	-0.428476693486255	-0.422524312533874	-0.416571931581493	-0.410619550629112	-0.404667169676731	-0.398714788724350	-0.392762407771969	-0.386810026819588	-0.380857645867207	-0.374905264914826	-0.368952883962445	-0.363000503010064	-0.357048122057683	-0.351095741105302	-0.345143360152921	-0.339190979200541	-0.333238598248160	-0.327286217295779	-0.321333836343398	-0.315381455391017	-0.309429074438636	-0.303476693486255	-0.297524312533874	-0.291571931581493	-0.285619550629112	-0.279667169676731	-0.273714788724350	-0.267762407771969	-0.261810026819588	-0.255857645867207	-0.249905264914826	-0.243952883962445	-0.238000503010065	-0.232048122057684	-0.226095741105303	-0.220143360152922	-0.214190979200541	-0.208238598248160	-0.202286217295779	-0.196333836343398	-0.190381455391017	-0.184429074438636	-0.178476693486255	-0.172524312533874	-0.166571931581493	-0.160619550629112	-0.154667169676731	-0.148714788724350	-0.142762407771969	-0.136810026819588	-0.130857645867207	-0.124905264914826	-0.118952883962445	-0.113000503010064	-0.107048122057683	-0.101095741105303	-0.0951433601529216	-0.0891909792005406	-0.0832385982481596	-0.0772862172957787	-0.0713338363433977	-0.0653814553910168	-0.0594290744386358	-0.0534766934862548	-0.0475243125338739	-0.0415719315814929	-0.0356195506291120	-0.0296671696767310	-0.0237147887243500	-0.0177624077719691	-0.0118100268195881	-0.00585764586720720	0	0.00585764586720720	0.0118100268195881	0.0177624077719691	0.0237147887243500	0.0296671696767310	0.0356195506291120	0.0415719315814929	0.0475243125338739	0.0534766934862548	0.0594290744386358	0.0653814553910168	0.0713338363433977	0.0772862172957787	0.0832385982481596	0.0891909792005406	0.0951433601529216	0.101095741105303	0.107048122057683	0.113000503010064	0.118952883962445	0.124905264914826	0.130857645867207	0.136810026819588	0.142762407771969	0.148714788724350	0.154667169676731	0.160619550629112	0.166571931581493	0.172524312533874	0.178476693486255	0.184429074438636	0.190381455391017	0.196333836343398	0.202286217295779	0.208238598248160	0.214190979200541	0.220143360152922	0.226095741105303	0.232048122057684	0.238000503010065	0.243952883962445	0.249905264914826	0.255857645867207	0.261810026819588	0.267762407771969	0.273714788724350	0.279667169676731	0.285619550629112	0.291571931581493	0.297524312533874	0.303476693486255	0.309429074438636	0.315381455391017	0.321333836343398	0.327286217295779	0.333238598248160	0.339190979200541	0.345143360152921	0.351095741105302	0.357048122057683	0.363000503010064	0.368952883962445	0.374905264914826	0.380857645867207	0.386810026819588	0.392762407771969	0.398714788724350	0.404667169676731	0.410619550629112	0.416571931581493	0.422524312533874	0.428476693486255	0.434429074438636	0.440381455391017	0.446333836343398	0.452286217295779	0.458238598248160	0.464190979200541	0.470143360152921	0.476095741105302	0.482048122057683	0.488000503010064	0.493952883962445	0.499905264914826	0.505857645867207	0.511810026819588	0.517762407771969	0.523714788724350	0.529667169676731	0.535619550629112	0.541571931581492]
   * Referenced by: '<S3>/EV Bolt Steer'
   */
  real_T Backlash_InitialOutput;       /* Expression: 0
                                        * Referenced by: '<S269>/Backlash'
                                        */
  real_T index_Value;                  /* Expression: 1
                                        * Referenced by: '<S268>/index'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0
                                        * Referenced by: '<S268>/Switch'
                                        */
  real_T Switch1_Threshold_i;          /* Expression: 0
                                        * Referenced by: '<S268>/Switch1'
                                        */
  real_T Constant_Value_o;             /* Expression: 0
                                        * Referenced by: '<S261>/Constant'
                                        */
  real_T AxleNumber3_Value[2];         /* Expression: zeros(1,length(WhlNumVec))
                                        * Referenced by: '<S274>/Axle Number3'
                                        */
  real_T Constant1_Value_d;            /* Expression: zeros(1,NumAxl)
                                        * Referenced by: '<S280>/Constant1'
                                        */
  real_T Memory1_InitialCondition_di;  /* Expression: 0
                                        * Referenced by: '<S280>/Memory1'
                                        */
  real_T MeanWheelPosition_Gain;       /* Expression: 1/2
                                        * Referenced by: '<S280>/Mean Wheel Position'
                                        */
  real_T WheelNumber2_Value[2];        /* Expression: 1:length(WhlNumVec)
                                        * Referenced by: '<S274>/Wheel Number2'
                                        */
  real_T Constant1_Value_i;            /* Expression: pi
                                        * Referenced by: '<S735>/Constant1'
                                        */
  real_T Constant3_Value_n[4];         /* Expression: ones(1,4).*0
                                        * Referenced by: '<S735>/Constant3'
                                        */
  real_T Constant2_Value_d[4];      /* Expression: [pi;0;pi;0].*0+[0;pi;0;pi].*0
                                     * Referenced by: '<S735>/Constant2'
                                     */
  real_T SteerRates_Value[4];          /* Expression: zeros(1,4)
                                        * Referenced by: '<S275>/SteerRates'
                                        */
  real_T Constant_Value_ko[4];         /* Expression: zeros(1,4)
                                        * Referenced by: '<S275>/Constant'
                                        */
  real_T ones2_Value[4];               /* Expression: ones(1,numWheels)
                                        * Referenced by: '<S475>/ones2'
                                        */
  real_T Friction_Value[4];            /* Expression: ones(4,1).*1
                                        * Referenced by: '<S98>/Friction'
                                        */
  real_T u_Value[4];                   /* Expression: [zeros(1,numWheels)]
                                        * Referenced by: '<S475>/0'
                                        */
  real_T ones_Value[92];               /* Expression: [ones(23,numWheels)]
                                        * Referenced by: '<S475>/ones'
                                        */
  real_T Constant1_Value_n;            /* Expression: 0
                                        * Referenced by: '<S519>/Constant1'
                                        */
  real_T vertType_Value;               /* Expression: vertType
                                        * Referenced by: '<S482>/vertType'
                                        */
  real_T Constant1_Value_lx;           /* Expression: 0
                                        * Referenced by: '<S547>/Constant1'
                                        */
  real_T vertType_Value_c;             /* Expression: vertType
                                        * Referenced by: '<S483>/vertType'
                                        */
  real_T Constant1_Value_i2;           /* Expression: 0
                                        * Referenced by: '<S575>/Constant1'
                                        */
  real_T vertType_Value_cq;            /* Expression: vertType
                                        * Referenced by: '<S484>/vertType'
                                        */
  real_T Constant1_Value_ir;           /* Expression: 0
                                        * Referenced by: '<S603>/Constant1'
                                        */
  real_T vertType_Value_h;             /* Expression: vertType
                                        * Referenced by: '<S485>/vertType'
                                        */
  real_T AxlesUsingAntiSway_Value;     /* Expression: find(AntiSwayEnByAxl==1)
                                        * Referenced by: '<S324>/Axles Using Anti-Sway'
                                        */
  real_T Constant_Value_c;             /* Expression: 0
                                        * Referenced by: '<S364>/Constant'
                                        */
  real_T Constant2_Value_c;            /* Expression: 0
                                        * Referenced by: '<S103>/Constant2'
                                        */
  real_T Constant3_Value_o;            /* Expression: 0
                                        * Referenced by: '<S103>/Constant3'
                                        */
  real_T Constant4_Value_a;            /* Expression: 0
                                        * Referenced by: '<S103>/Constant4'
                                        */
  real_T u_Gain[3];                    /* Expression: [4.*ones(2,1); 0]
                                        * Referenced by: '<S377>/4'
                                        */
  real_T Constant4_Value_aq[3];        /* Expression: [0; 0; 1]
                                        * Referenced by: '<S377>/Constant4'
                                        */
  real_T Crm_tableData[2];             /* Expression: [0 0]
                                        * Referenced by: '<S377>/Crm'
                                        */
  real_T Crm_bp01Data[2];              /* Expression: [-1 1]
                                        * Referenced by: '<S377>/Crm'
                                        */
  real_T Cyf_Value;                    /* Expression: Cy_f
                                        * Referenced by: '<S376>/Cyf'
                                        */
  real_T Cyr_Value;                    /* Expression: Cy_r
                                        * Referenced by: '<S376>/Cyr'
                                        */
  real_T Gain3_Gain_b;                 /* Expression: 3.6
                                        * Referenced by: '<S1>/Gain3'
                                        */
  real_T RS_connectRequest_Value;      /* Expression: 0
                                        * Referenced by: '<S4>/RS_connectRequest'
                                        */
  real_T Constant2_Value_o;            /* Expression: 1
                                        * Referenced by: '<S4>/Constant2'
                                        */
  real_T Memory1_InitialCondition_l;   /* Expression: 0
                                        * Referenced by: '<Root>/Memory1'
                                        */
  real_T timeSimulation_InitialCondition;/* Expression: 0
                                          * Referenced by: '<S4>/timeSimulation'
                                          */
  real_T timeInOut_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<S4>/timeInOut'
                                        */
  real_T simState_Value;               /* Expression: 1
                                        * Referenced by: '<S4>/simState'
                                        */
  real_T rgba_Value[4];                /* Expression: [255,0,0,255]
                                        * Referenced by: '<S4>/rgba'
                                        */
  real_T rgbabitoperation_Value[4];    /* Expression: [2^24,2^16,2^8,2^0]
                                        * Referenced by: '<S13>/rgba bit operation'
                                        */
  real_T activeLaneChange_Value;       /* Expression: 0
                                        * Referenced by: '<S4>/activeLaneChange'
                                        */
  real_T Constant1_Value_bm;           /* Expression: 0
                                        * Referenced by: '<S4>/Constant1'
                                        */
  real_T Constant1_Value_e;            /* Expression: 0
                                        * Referenced by: '<S10>/Constant1'
                                        */
  real_T switchfor01sendout_Threshold; /* Expression: 0
                                        * Referenced by: '<S10>/switch for 0.1 send out'
                                        */
  real_T Constant_Value_c5;            /* Expression: 1
                                        * Referenced by: '<S10>/Constant'
                                        */
  real_T SimTime_Value;                /* Expression: 10
                                        * Referenced by: '<S1>/SimTime'
                                        */
  real_T Integrator_IC_i;              /* Expression: 0
                                        * Referenced by: '<S106>/Integrator'
                                        */
  real_T Step_Time;                    /* Expression: 5
                                        * Referenced by: '<S3>/Step'
                                        */
  real_T Step_Y0;                      /* Expression: 0
                                        * Referenced by: '<S3>/Step'
                                        */
  real_T Step_YFinal;                  /* Expression: 0
                                        * Referenced by: '<S3>/Step'
                                        */
  real_T Switch_Threshold_ks;          /* Expression: 0
                                        * Referenced by: '<S3>/Switch'
                                        */
  real_T Constant_Value_oy;            /* Expression: 0
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Constant2_Value_p;            /* Expression: 1
                                        * Referenced by: '<S3>/Constant2'
                                        */
  real_T Constant3_Value_i;            /* Expression: 0
                                        * Referenced by: '<S3>/Constant3'
                                        */
  real_T Constant4_Value_b;            /* Expression: 1
                                        * Referenced by: '<S3>/Constant4'
                                        */
  real_T wperkw_Value;                 /* Expression: 1000
                                        * Referenced by: '<S100>/w per kw'
                                        */
  real_T USEPAkwhUSgalequivalent_Value;/* Expression: 33.7
                                        * Referenced by: '<S100>/US EPA kwh//USgal equivalent'
                                        */
  real_T sperh_Value;                  /* Expression: 3600
                                        * Referenced by: '<S100>/s per h'
                                        */
  real_T mto100Km_Gain;                /* Expression: 1/1000/100
                                        * Referenced by: '<S100>/m to 100Km'
                                        */
  real_T Saturation_UpperSat_l;        /* Expression: Inf
                                        * Referenced by: '<S100>/Saturation'
                                        */
  real_T Saturation_LowerSat_k1;       /* Expression: 0.001
                                        * Referenced by: '<S100>/Saturation'
                                        */
  real_T m3toL_Gain;                   /* Expression: 1000
                                        * Referenced by: '<S100>/m^3 to L'
                                        */
  real_T m3pergal_Gain;                /* Expression: 0.00378541
                                        * Referenced by: '<S100>/m^3 per gal'
                                        */
  real_T VehicleSimulationType_Value;  /* Expression: 0
                                        * Referenced by: '<S101>/VehicleSimulationType'
                                        */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S507>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S507>/Transfer Fcn'
                                        */
  real_T Constant_Value_af;            /* Expression: 0
                                        * Referenced by: '<S507>/Constant'
                                        */
  real_T Switch1_Threshold_dj1;        /* Expression: 0.11
                                        * Referenced by: '<S507>/Switch1'
                                        */
  real_T TransferFcn1_A;               /* Computed Parameter: TransferFcn1_A
                                        * Referenced by: '<S507>/Transfer Fcn1'
                                        */
  real_T TransferFcn1_C;               /* Computed Parameter: TransferFcn1_C
                                        * Referenced by: '<S507>/Transfer Fcn1'
                                        */
  real_T Switch2_Threshold_l;          /* Expression: 0.11
                                        * Referenced by: '<S507>/Switch2'
                                        */
  real_T TransferFcn2_A;               /* Computed Parameter: TransferFcn2_A
                                        * Referenced by: '<S507>/Transfer Fcn2'
                                        */
  real_T TransferFcn2_C;               /* Computed Parameter: TransferFcn2_C
                                        * Referenced by: '<S507>/Transfer Fcn2'
                                        */
  real_T Switch3_Threshold_k;          /* Expression: 0.11
                                        * Referenced by: '<S507>/Switch3'
                                        */
  real_T TransferFcn3_A;               /* Computed Parameter: TransferFcn3_A
                                        * Referenced by: '<S507>/Transfer Fcn3'
                                        */
  real_T TransferFcn3_C;               /* Computed Parameter: TransferFcn3_C
                                        * Referenced by: '<S507>/Transfer Fcn3'
                                        */
  real_T Switch4_Threshold;            /* Expression: 0.11
                                        * Referenced by: '<S507>/Switch4'
                                        */
  real_T Gain_Gain_p;                  /* Expression: -1
                                        * Referenced by: '<S141>/Gain'
                                        */
  real_T Gain1_Gain_b;                 /* Expression: -1
                                        * Referenced by: '<S141>/Gain1'
                                        */
  real_T Gain1_Gain_dr;                /* Expression: 0.25
                                        * Referenced by: '<S206>/Gain1'
                                        */
  real_T Constant_Value_cc;            /* Expression: 0
                                        * Referenced by: '<S207>/Constant'
                                        */
  real_T Constant1_Value_c;            /* Expression: 0
                                        * Referenced by: '<S207>/Constant1'
                                        */
  real_T Constant2_Value_g;            /* Expression: 0
                                        * Referenced by: '<S207>/Constant2'
                                        */
  real_T Constant_Value_p0;            /* Expression: 0
                                        * Referenced by: '<S214>/Constant'
                                        */
  real_T uDLookupTable_tableData[667]; /* Expression: x_losses_mat
                                        * Referenced by: '<S218>/2-D Lookup Table'
                                        */
  real_T uDLookupTable_bp01Data[23];   /* Expression: x_w_eff_vec
                                        * Referenced by: '<S218>/2-D Lookup Table'
                                        */
  real_T uDLookupTable_bp02Data[29];   /* Expression: x_T_eff_vec
                                        * Referenced by: '<S218>/2-D Lookup Table'
                                        */
  real_T Saturation_UpperSat_kg;       /* Expression: Inf
                                        * Referenced by: '<S215>/Saturation'
                                        */
  real_T Saturation_LowerSat_hs;       /* Expression: 0.0001
                                        * Referenced by: '<S215>/Saturation'
                                        */
  real_T Saturation1_UpperSat_p;       /* Expression: inf
                                        * Referenced by: '<S254>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_m;       /* Expression: 1
                                        * Referenced by: '<S254>/Saturation1'
                                        */
  real_T Saturation_LowerSat_e;        /* Expression: 0
                                        * Referenced by: '<S254>/Saturation'
                                        */
  real_T Saturation1_UpperSat_f;       /* Expression: inf
                                        * Referenced by: '<S243>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_m3;      /* Expression: 1
                                        * Referenced by: '<S243>/Saturation1'
                                        */
  real_T Saturation_LowerSat_l;        /* Expression: 0
                                        * Referenced by: '<S243>/Saturation'
                                        */
  real_T Gain_Gain_ij;                 /* Expression: -1
                                        * Referenced by: '<S242>/Gain'
                                        */
  real_T rads_to_rpm_Gain;             /* Expression: 30/pi
                                        * Referenced by: '<S248>/rads_to_rpm'
                                        */
  real_T Gain1_Gain_ad;                /* Expression: 1/100
                                        * Referenced by: '<S248>/Gain1'
                                        */
  real_T Constant1_Value_bt;           /* Expression: -1
                                        * Referenced by: '<S248>/Constant1'
                                        */
  real_T Constant2_Value_l;            /* Expression: 1
                                        * Referenced by: '<S248>/Constant2'
                                        */
  real_T Switch2_Threshold_ah;         /* Expression: 0
                                        * Referenced by: '<S248>/Switch2'
                                        */
  real_T Constant_Value_ja;            /* Expression: 1
                                        * Referenced by: '<S250>/Constant'
                                        */
  real_T Switch1_Threshold_i4;         /* Expression: 0
                                        * Referenced by: '<S250>/Switch1'
                                        */
  real_T Gain_Gain_ah;                 /* Expression: -1
                                        * Referenced by: '<S249>/Gain'
                                        */
  real_T Gain_Gain_j;                  /* Expression: -1
                                        * Referenced by: '<S217>/Gain'
                                        */
  real_T Gain1_Gain_c;                 /* Expression: 1
                                        * Referenced by: '<S217>/Gain1'
                                        */
  real_T Constant_Value_n;             /* Expression: 0
                                        * Referenced by: '<S225>/Constant'
                                        */
  real_T Integrator_IC_nf;             /* Expression: 0
                                        * Referenced by: '<S227>/Integrator'
                                        */
  real_T Merge_InitialOutput_e;        /* Expression: 0
                                        * Referenced by: '<S227>/Merge'
                                        */
  real_T uDLookupTable_tableData_p[667];/* Expression: x_losses_mat
                                         * Referenced by: '<S229>/2-D Lookup Table'
                                         */
  real_T uDLookupTable_bp01Data_k[23]; /* Expression: x_w_eff_vec
                                        * Referenced by: '<S229>/2-D Lookup Table'
                                        */
  real_T uDLookupTable_bp02Data_a[29]; /* Expression: x_T_eff_vec
                                        * Referenced by: '<S229>/2-D Lookup Table'
                                        */
  real_T Saturation_UpperSat_gz;       /* Expression: Inf
                                        * Referenced by: '<S226>/Saturation'
                                        */
  real_T Saturation_LowerSat_mg;       /* Expression: 0.0001
                                        * Referenced by: '<S226>/Saturation'
                                        */
  real_T Gain_Gain_m;                  /* Expression: -1
                                        * Referenced by: '<S228>/Gain'
                                        */
  real_T Gain1_Gain_ak;                /* Expression: 1
                                        * Referenced by: '<S228>/Gain1'
                                        */
  real_T Switch4_Threshold_i;          /* Expression: 0
                                        * Referenced by: '<S481>/Switch4'
                                        */
  real_T Switch5_Threshold;            /* Expression: 0
                                        * Referenced by: '<S481>/Switch5'
                                        */
  real_T Switch6_Threshold;            /* Expression: 0
                                        * Referenced by: '<S481>/Switch6'
                                        */
  real_T Switch7_Threshold;            /* Expression: 0
                                        * Referenced by: '<S481>/Switch7'
                                        */
  real_T TransferFcn3_A_g;             /* Computed Parameter: TransferFcn3_A_g
                                        * Referenced by: '<S259>/Transfer Fcn3'
                                        */
  real_T TransferFcn3_C_n;             /* Computed Parameter: TransferFcn3_C_n
                                        * Referenced by: '<S259>/Transfer Fcn3'
                                        */
  real_T TransferFcn1_A_m;             /* Computed Parameter: TransferFcn1_A_m
                                        * Referenced by: '<S259>/Transfer Fcn1'
                                        */
  real_T TransferFcn1_C_l;             /* Computed Parameter: TransferFcn1_C_l
                                        * Referenced by: '<S259>/Transfer Fcn1'
                                        */
  real_T TransferFcn2_A_h;             /* Computed Parameter: TransferFcn2_A_h
                                        * Referenced by: '<S259>/Transfer Fcn2'
                                        */
  real_T TransferFcn2_C_j;             /* Computed Parameter: TransferFcn2_C_j
                                        * Referenced by: '<S259>/Transfer Fcn2'
                                        */
  real_T TransferFcn4_A;               /* Computed Parameter: TransferFcn4_A
                                        * Referenced by: '<S259>/Transfer Fcn4'
                                        */
  real_T TransferFcn4_C;               /* Computed Parameter: TransferFcn4_C
                                        * Referenced by: '<S259>/Transfer Fcn4'
                                        */
  real_T TransferFcn_A_p;              /* Computed Parameter: TransferFcn_A_p
                                        * Referenced by: '<S259>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_c;              /* Computed Parameter: TransferFcn_C_c
                                        * Referenced by: '<S259>/Transfer Fcn'
                                        */
  real_T TransferFcn9_A;               /* Computed Parameter: TransferFcn9_A
                                        * Referenced by: '<S259>/Transfer Fcn9'
                                        */
  real_T TransferFcn9_C;               /* Computed Parameter: TransferFcn9_C
                                        * Referenced by: '<S259>/Transfer Fcn9'
                                        */
  real_T TransferFcn10_A;              /* Computed Parameter: TransferFcn10_A
                                        * Referenced by: '<S259>/Transfer Fcn10'
                                        */
  real_T TransferFcn10_C;              /* Computed Parameter: TransferFcn10_C
                                        * Referenced by: '<S259>/Transfer Fcn10'
                                        */
  real_T TransferFcn11_A;              /* Computed Parameter: TransferFcn11_A
                                        * Referenced by: '<S259>/Transfer Fcn11'
                                        */
  real_T TransferFcn11_C;              /* Computed Parameter: TransferFcn11_C
                                        * Referenced by: '<S259>/Transfer Fcn11'
                                        */
  real_T Constant_Value_g;             /* Expression: 0
                                        * Referenced by: '<S269>/Constant'
                                        */
  real_T PerAckInConstant_Value;       /* Expression: 1
                                        * Referenced by: '<S268>/PerAckInConstant'
                                        */
  real_T Gain_Gain_jt;                 /* Expression: -1
                                        * Referenced by: '<S274>/Gain'
                                        */
  real_T Zero_Value_f;                 /* Expression: 0
                                        * Referenced by: '<S481>/Zero'
                                        */
  real_T Zero1_Value;                  /* Expression: 0
                                        * Referenced by: '<S481>/Zero1'
                                        */
  real_T AxleNumber1_Value;            /* Expression: 1:NumAxl
                                        * Referenced by: '<S274>/Axle Number1'
                                        */
  real_T TransferFcn5_A;               /* Computed Parameter: TransferFcn5_A
                                        * Referenced by: '<S259>/Transfer Fcn5'
                                        */
  real_T TransferFcn5_C;               /* Computed Parameter: TransferFcn5_C
                                        * Referenced by: '<S259>/Transfer Fcn5'
                                        */
  real_T TransferFcn6_A;               /* Computed Parameter: TransferFcn6_A
                                        * Referenced by: '<S259>/Transfer Fcn6'
                                        */
  real_T TransferFcn6_C;               /* Computed Parameter: TransferFcn6_C
                                        * Referenced by: '<S259>/Transfer Fcn6'
                                        */
  real_T TransferFcn7_A;               /* Computed Parameter: TransferFcn7_A
                                        * Referenced by: '<S259>/Transfer Fcn7'
                                        */
  real_T TransferFcn7_C;               /* Computed Parameter: TransferFcn7_C
                                        * Referenced by: '<S259>/Transfer Fcn7'
                                        */
  real_T TransferFcn8_A;               /* Computed Parameter: TransferFcn8_A
                                        * Referenced by: '<S259>/Transfer Fcn8'
                                        */
  real_T TransferFcn8_C;               /* Computed Parameter: TransferFcn8_C
                                        * Referenced by: '<S259>/Transfer Fcn8'
                                        */
  real_T Constant8_Value;              /* Expression: 0
                                        * Referenced by: '<S384>/Constant8'
                                        */
  real_T Constant_Value_eu;            /* Expression: dh
                                        * Referenced by: '<S390>/Constant'
                                        */
  real_T Constant3_Value_j;            /* Expression: hh
                                        * Referenced by: '<S390>/Constant3'
                                        */
  real_T Constant4_Value_m;            /* Expression: hl
                                        * Referenced by: '<S390>/Constant4'
                                        */
  real_T Constant1_Value_m;            /* Expression: 0
                                        * Referenced by: '<S384>/Constant1'
                                        */
  real_T Constant10_Value;             /* Expression: 0
                                        * Referenced by: '<S384>/Constant10'
                                        */
  real_T Constant3_Value_e;            /* Expression: 0
                                        * Referenced by: '<S384>/Constant3'
                                        */
  real_T Constant9_Value;              /* Expression: 0
                                        * Referenced by: '<S384>/Constant9'
                                        */
  real_T Integrator_IC_nl;             /* Expression: 0
                                        * Referenced by: '<S630>/Integrator'
                                        */
  real_T Integrator_IC_ng;             /* Expression: 0
                                        * Referenced by: '<S627>/Integrator'
                                        */
  real_T Saturation_UpperSat_bo;       /* Expression: inf
                                        * Referenced by: '<S631>/Saturation'
                                        */
  real_T Saturation_LowerSat_gf;       /* Expression: 0
                                        * Referenced by: '<S631>/Saturation'
                                        */
  real_T Saturation_UpperSat_c;        /* Expression: inf
                                        * Referenced by: '<S626>/Saturation'
                                        */
  real_T Saturation_LowerSat_c;        /* Expression: 0
                                        * Referenced by: '<S626>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_e;     /* Expression: pi/4
                                        * Referenced by: '<S636>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_n;/* Expression: inf
                                                 * Referenced by: '<S636>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_e;/* Expression: eps
                                                 * Referenced by: '<S636>/Disallow Negative Brake Torque'
                                                 */
  real_T Saturation1_UpperSat_j;       /* Expression: inf
                                        * Referenced by: '<S515>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_k;       /* Expression: 1e-2
                                        * Referenced by: '<S515>/Saturation1'
                                        */
  real_T Saturation_UpperSat_a;        /* Expression: inf
                                        * Referenced by: '<S515>/Saturation'
                                        */
  real_T Saturation_LowerSat_oo;       /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S515>/Saturation'
                                        */
  real_T Saturation1_UpperSat_d;       /* Expression: inf
                                        * Referenced by: '<S516>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_h;       /* Expression: 1e-2
                                        * Referenced by: '<S516>/Saturation1'
                                        */
  real_T Saturation_UpperSat_b21;      /* Expression: inf
                                        * Referenced by: '<S516>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S516>/Saturation'
                                        */
  real_T Saturation1_UpperSat_b;       /* Expression: inf
                                        * Referenced by: '<S518>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_b;       /* Expression: 1e-2
                                        * Referenced by: '<S518>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ly;       /* Expression: inf
                                        * Referenced by: '<S518>/Saturation'
                                        */
  real_T Saturation_LowerSat_gb;       /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S518>/Saturation'
                                        */
  real_T Constant_Value_h;             /* Expression: 0
                                        * Referenced by: '<S519>/Constant'
                                        */
  real_T Switch_Threshold_e;           /* Expression: 0
                                        * Referenced by: '<S519>/Switch'
                                        */
  real_T Saturation1_UpperSat_fk;      /* Expression: inf
                                        * Referenced by: '<S543>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_le;      /* Expression: 1e-2
                                        * Referenced by: '<S543>/Saturation1'
                                        */
  real_T Saturation_UpperSat_dh;       /* Expression: inf
                                        * Referenced by: '<S543>/Saturation'
                                        */
  real_T Saturation_LowerSat_fb;       /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S543>/Saturation'
                                        */
  real_T Saturation1_UpperSat_c;       /* Expression: inf
                                        * Referenced by: '<S544>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_d;       /* Expression: 1e-2
                                        * Referenced by: '<S544>/Saturation1'
                                        */
  real_T Saturation_UpperSat_p;        /* Expression: inf
                                        * Referenced by: '<S544>/Saturation'
                                        */
  real_T Saturation_LowerSat_ee;       /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S544>/Saturation'
                                        */
  real_T Saturation1_UpperSat_pj;      /* Expression: inf
                                        * Referenced by: '<S546>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_hk;      /* Expression: 1e-2
                                        * Referenced by: '<S546>/Saturation1'
                                        */
  real_T Saturation_UpperSat_bb;       /* Expression: inf
                                        * Referenced by: '<S546>/Saturation'
                                        */
  real_T Saturation_LowerSat_o0;       /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S546>/Saturation'
                                        */
  real_T Constant_Value_f2;            /* Expression: 0
                                        * Referenced by: '<S547>/Constant'
                                        */
  real_T Switch_Threshold_e4;          /* Expression: 0
                                        * Referenced by: '<S547>/Switch'
                                        */
  real_T Saturation1_UpperSat_cf;      /* Expression: inf
                                        * Referenced by: '<S571>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_i;       /* Expression: 1e-2
                                        * Referenced by: '<S571>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ds;       /* Expression: inf
                                        * Referenced by: '<S571>/Saturation'
                                        */
  real_T Saturation_LowerSat_j;        /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S571>/Saturation'
                                        */
  real_T Saturation1_UpperSat_i;       /* Expression: inf
                                        * Referenced by: '<S572>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_f;       /* Expression: 1e-2
                                        * Referenced by: '<S572>/Saturation1'
                                        */
  real_T Saturation_UpperSat_e0;       /* Expression: inf
                                        * Referenced by: '<S572>/Saturation'
                                        */
  real_T Saturation_LowerSat_hb;       /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S572>/Saturation'
                                        */
  real_T Saturation1_UpperSat_gn;      /* Expression: inf
                                        * Referenced by: '<S574>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_g;       /* Expression: 1e-2
                                        * Referenced by: '<S574>/Saturation1'
                                        */
  real_T Saturation_UpperSat_j;        /* Expression: inf
                                        * Referenced by: '<S574>/Saturation'
                                        */
  real_T Saturation_LowerSat_ga;       /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S574>/Saturation'
                                        */
  real_T Constant_Value_kc;            /* Expression: 0
                                        * Referenced by: '<S575>/Constant'
                                        */
  real_T Switch_Threshold_a;           /* Expression: 0
                                        * Referenced by: '<S575>/Switch'
                                        */
  real_T Saturation1_UpperSat_gnn;     /* Expression: inf
                                        * Referenced by: '<S599>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_in;      /* Expression: 1e-2
                                        * Referenced by: '<S599>/Saturation1'
                                        */
  real_T Saturation_UpperSat_jf;       /* Expression: inf
                                        * Referenced by: '<S599>/Saturation'
                                        */
  real_T Saturation_LowerSat_kd;       /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S599>/Saturation'
                                        */
  real_T Saturation1_UpperSat_e;       /* Expression: inf
                                        * Referenced by: '<S600>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_p;       /* Expression: 1e-2
                                        * Referenced by: '<S600>/Saturation1'
                                        */
  real_T Saturation_UpperSat_f;        /* Expression: inf
                                        * Referenced by: '<S600>/Saturation'
                                        */
  real_T Saturation_LowerSat_ix;       /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S600>/Saturation'
                                        */
  real_T Saturation1_UpperSat_de;      /* Expression: inf
                                        * Referenced by: '<S602>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_fn;      /* Expression: 1e-2
                                        * Referenced by: '<S602>/Saturation1'
                                        */
  real_T Saturation_UpperSat_bd;       /* Expression: inf
                                        * Referenced by: '<S602>/Saturation'
                                        */
  real_T Saturation_LowerSat_cr;       /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S602>/Saturation'
                                        */
  real_T Constant_Value_m2;            /* Expression: 0
                                        * Referenced by: '<S603>/Constant'
                                        */
  real_T Switch_Threshold_ai;          /* Expression: 0
                                        * Referenced by: '<S603>/Switch'
                                        */
  real_T Integrator_IC_ns;             /* Expression: 0
                                        * Referenced by: '<S628>/Integrator'
                                        */
  real_T Constant1_Value_ds;           /* Expression: 0
                                        * Referenced by: '<S631>/Constant1'
                                        */
  real_T vertType_Value_a;             /* Expression: vertType
                                        * Referenced by: '<S486>/vertType'
                                        */
  real_T Saturation1_UpperSat_jm;      /* Expression: inf
                                        * Referenced by: '<S627>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_dt;      /* Expression: 1e-2
                                        * Referenced by: '<S627>/Saturation1'
                                        */
  real_T Saturation_UpperSat_m;        /* Expression: inf
                                        * Referenced by: '<S627>/Saturation'
                                        */
  real_T Saturation_LowerSat_o3;       /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S627>/Saturation'
                                        */
  real_T Saturation1_UpperSat_k;       /* Expression: inf
                                        * Referenced by: '<S628>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_e;       /* Expression: 1e-2
                                        * Referenced by: '<S628>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ns;       /* Expression: inf
                                        * Referenced by: '<S628>/Saturation'
                                        */
  real_T Saturation_LowerSat_jf;       /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S628>/Saturation'
                                        */
  real_T Saturation1_UpperSat_et;      /* Expression: inf
                                        * Referenced by: '<S630>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_ed;      /* Expression: 1e-2
                                        * Referenced by: '<S630>/Saturation1'
                                        */
  real_T Saturation_UpperSat_pu;       /* Expression: inf
                                        * Referenced by: '<S630>/Saturation'
                                        */
  real_T Saturation_LowerSat_i3;       /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S630>/Saturation'
                                        */
  real_T Constant_Value_mj;            /* Expression: 0
                                        * Referenced by: '<S631>/Constant'
                                        */
  real_T Switch_Threshold_dl;          /* Expression: 0
                                        * Referenced by: '<S631>/Switch'
                                        */
  real_T Integrator_IC_kf;             /* Expression: 0
                                        * Referenced by: '<S658>/Integrator'
                                        */
  real_T Integrator_IC_mm;             /* Expression: 0
                                        * Referenced by: '<S655>/Integrator'
                                        */
  real_T Saturation_UpperSat_ap;       /* Expression: inf
                                        * Referenced by: '<S659>/Saturation'
                                        */
  real_T Saturation_LowerSat_nu;       /* Expression: 0
                                        * Referenced by: '<S659>/Saturation'
                                        */
  real_T Saturation_UpperSat_hg;       /* Expression: inf
                                        * Referenced by: '<S654>/Saturation'
                                        */
  real_T Saturation_LowerSat_nd;       /* Expression: 0
                                        * Referenced by: '<S654>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_k;     /* Expression: pi/4
                                        * Referenced by: '<S664>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_g;/* Expression: inf
                                                 * Referenced by: '<S664>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_nc;/* Expression: eps
                                                  * Referenced by: '<S664>/Disallow Negative Brake Torque'
                                                  */
  real_T Integrator_IC_ep;             /* Expression: 0
                                        * Referenced by: '<S656>/Integrator'
                                        */
  real_T Constant1_Value_kb;           /* Expression: 0
                                        * Referenced by: '<S659>/Constant1'
                                        */
  real_T vertType_Value_j;             /* Expression: vertType
                                        * Referenced by: '<S487>/vertType'
                                        */
  real_T Saturation1_UpperSat_ij;      /* Expression: inf
                                        * Referenced by: '<S655>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_gs;      /* Expression: 1e-2
                                        * Referenced by: '<S655>/Saturation1'
                                        */
  real_T Saturation_UpperSat_by;       /* Expression: inf
                                        * Referenced by: '<S655>/Saturation'
                                        */
  real_T Saturation_LowerSat_lv;       /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S655>/Saturation'
                                        */
  real_T Saturation1_UpperSat_h;       /* Expression: inf
                                        * Referenced by: '<S656>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_px;      /* Expression: 1e-2
                                        * Referenced by: '<S656>/Saturation1'
                                        */
  real_T Saturation_UpperSat_fk;       /* Expression: inf
                                        * Referenced by: '<S656>/Saturation'
                                        */
  real_T Saturation_LowerSat_p;        /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S656>/Saturation'
                                        */
  real_T Saturation1_UpperSat_cp;      /* Expression: inf
                                        * Referenced by: '<S658>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_gh;      /* Expression: 1e-2
                                        * Referenced by: '<S658>/Saturation1'
                                        */
  real_T Saturation_UpperSat_pk;       /* Expression: inf
                                        * Referenced by: '<S658>/Saturation'
                                        */
  real_T Saturation_LowerSat_oq;       /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S658>/Saturation'
                                        */
  real_T Constant_Value_bw;            /* Expression: 0
                                        * Referenced by: '<S659>/Constant'
                                        */
  real_T Switch_Threshold_a3;          /* Expression: 0
                                        * Referenced by: '<S659>/Switch'
                                        */
  real_T Integrator_IC_ae;             /* Expression: 0
                                        * Referenced by: '<S686>/Integrator'
                                        */
  real_T Integrator_IC_g;              /* Expression: 0
                                        * Referenced by: '<S683>/Integrator'
                                        */
  real_T Saturation_UpperSat_bv;       /* Expression: inf
                                        * Referenced by: '<S687>/Saturation'
                                        */
  real_T Saturation_LowerSat_a;        /* Expression: 0
                                        * Referenced by: '<S687>/Saturation'
                                        */
  real_T Saturation_UpperSat_cs;       /* Expression: inf
                                        * Referenced by: '<S682>/Saturation'
                                        */
  real_T Saturation_LowerSat_cy;       /* Expression: 0
                                        * Referenced by: '<S682>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_g;     /* Expression: pi/4
                                        * Referenced by: '<S692>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_pm;/* Expression: inf
                                                  * Referenced by: '<S692>/Disallow Negative Brake Torque'
                                                  */
  real_T DisallowNegativeBrakeTorque_LowerSat_a;/* Expression: eps
                                                 * Referenced by: '<S692>/Disallow Negative Brake Torque'
                                                 */
  real_T Integrator_IC_at;             /* Expression: 0
                                        * Referenced by: '<S684>/Integrator'
                                        */
  real_T Constant1_Value_ez;           /* Expression: 0
                                        * Referenced by: '<S687>/Constant1'
                                        */
  real_T vertType_Value_n;             /* Expression: vertType
                                        * Referenced by: '<S488>/vertType'
                                        */
  real_T Saturation1_UpperSat_d4;      /* Expression: inf
                                        * Referenced by: '<S683>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_cp;      /* Expression: 1e-2
                                        * Referenced by: '<S683>/Saturation1'
                                        */
  real_T Saturation_UpperSat_kh;       /* Expression: inf
                                        * Referenced by: '<S683>/Saturation'
                                        */
  real_T Saturation_LowerSat_lu;       /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S683>/Saturation'
                                        */
  real_T Saturation1_UpperSat_hs;      /* Expression: inf
                                        * Referenced by: '<S684>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_fa;      /* Expression: 1e-2
                                        * Referenced by: '<S684>/Saturation1'
                                        */
  real_T Saturation_UpperSat_ia;       /* Expression: inf
                                        * Referenced by: '<S684>/Saturation'
                                        */
  real_T Saturation_LowerSat_me;       /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S684>/Saturation'
                                        */
  real_T Saturation1_UpperSat_a;       /* Expression: inf
                                        * Referenced by: '<S686>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_bg;      /* Expression: 1e-2
                                        * Referenced by: '<S686>/Saturation1'
                                        */
  real_T Saturation_UpperSat_kf;       /* Expression: inf
                                        * Referenced by: '<S686>/Saturation'
                                        */
  real_T Saturation_LowerSat_ct;       /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S686>/Saturation'
                                        */
  real_T Constant_Value_aa;            /* Expression: 0
                                        * Referenced by: '<S687>/Constant'
                                        */
  real_T Switch_Threshold_aj;          /* Expression: 0
                                        * Referenced by: '<S687>/Switch'
                                        */
  real_T Integrator_IC_pr;             /* Expression: 0
                                        * Referenced by: '<S714>/Integrator'
                                        */
  real_T Integrator_IC_ny;             /* Expression: 0
                                        * Referenced by: '<S711>/Integrator'
                                        */
  real_T Saturation_UpperSat_hz;       /* Expression: inf
                                        * Referenced by: '<S715>/Saturation'
                                        */
  real_T Saturation_LowerSat_oy;       /* Expression: 0
                                        * Referenced by: '<S715>/Saturation'
                                        */
  real_T Saturation_UpperSat_di;       /* Expression: inf
                                        * Referenced by: '<S710>/Saturation'
                                        */
  real_T Saturation_LowerSat_gg;       /* Expression: 0
                                        * Referenced by: '<S710>/Saturation'
                                        */
  real_T TorqueConversion1_Gain_m;     /* Expression: pi/4
                                        * Referenced by: '<S720>/Torque Conversion1'
                                        */
  real_T DisallowNegativeBrakeTorque_UpperSat_d;/* Expression: inf
                                                 * Referenced by: '<S720>/Disallow Negative Brake Torque'
                                                 */
  real_T DisallowNegativeBrakeTorque_LowerSat_c;/* Expression: eps
                                                 * Referenced by: '<S720>/Disallow Negative Brake Torque'
                                                 */
  real_T Integrator_IC_a3;             /* Expression: 0
                                        * Referenced by: '<S712>/Integrator'
                                        */
  real_T Constant1_Value_f;            /* Expression: 0
                                        * Referenced by: '<S715>/Constant1'
                                        */
  real_T vertType_Value_ny;            /* Expression: vertType
                                        * Referenced by: '<S489>/vertType'
                                        */
  real_T Saturation1_UpperSat_f2;      /* Expression: inf
                                        * Referenced by: '<S711>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_j;       /* Expression: 1e-2
                                        * Referenced by: '<S711>/Saturation1'
                                        */
  real_T Saturation_UpperSat_hk;       /* Expression: inf
                                        * Referenced by: '<S711>/Saturation'
                                        */
  real_T Saturation_LowerSat_n4;       /* Expression: FxRelFreqLwrLim
                                        * Referenced by: '<S711>/Saturation'
                                        */
  real_T Saturation1_UpperSat_gj;      /* Expression: inf
                                        * Referenced by: '<S712>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_jk;      /* Expression: 1e-2
                                        * Referenced by: '<S712>/Saturation1'
                                        */
  real_T Saturation_UpperSat_nb;       /* Expression: inf
                                        * Referenced by: '<S712>/Saturation'
                                        */
  real_T Saturation_LowerSat_miq;      /* Expression: FyRelFreqLwrLim
                                        * Referenced by: '<S712>/Saturation'
                                        */
  real_T Saturation1_UpperSat_o;       /* Expression: inf
                                        * Referenced by: '<S714>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_pd;      /* Expression: 1e-2
                                        * Referenced by: '<S714>/Saturation1'
                                        */
  real_T Saturation_UpperSat_g4;       /* Expression: inf
                                        * Referenced by: '<S714>/Saturation'
                                        */
  real_T Saturation_LowerSat_jf2;      /* Expression: MyRelFreqLwrLim
                                        * Referenced by: '<S714>/Saturation'
                                        */
  real_T Constant_Value_kv;            /* Expression: 0
                                        * Referenced by: '<S715>/Constant'
                                        */
  real_T Switch_Threshold_ky;          /* Expression: 0
                                        * Referenced by: '<S715>/Switch'
                                        */
  real_T Constant_Value_ai;            /* Expression: 0
                                        * Referenced by: '<S501>/Constant'
                                        */
  real_T Constant1_Value_ig;           /* Expression: 0
                                        * Referenced by: '<S501>/Constant1'
                                        */
  real_T Switch_Threshold_l;           /* Expression: 0
                                        * Referenced by: '<S501>/Switch'
                                        */
  real_T Saturation_UpperSat_nh;       /* Expression: inf
                                        * Referenced by: '<S501>/Saturation'
                                        */
  real_T Saturation_LowerSat_b5;       /* Expression: 0
                                        * Referenced by: '<S501>/Saturation'
                                        */
  real_T Constant_Value_ic;            /* Expression: 0
                                        * Referenced by: '<S502>/Constant'
                                        */
  real_T Constant1_Value_fv;           /* Expression: 0
                                        * Referenced by: '<S502>/Constant1'
                                        */
  real_T Switch_Threshold_n;           /* Expression: 0
                                        * Referenced by: '<S502>/Switch'
                                        */
  real_T Saturation_UpperSat_ay;       /* Expression: inf
                                        * Referenced by: '<S502>/Saturation'
                                        */
  real_T Saturation_LowerSat_i0;       /* Expression: 0
                                        * Referenced by: '<S502>/Saturation'
                                        */
  real_T Constant_Value_bo;            /* Expression: 0
                                        * Referenced by: '<S503>/Constant'
                                        */
  real_T Constant1_Value_mn;           /* Expression: 0
                                        * Referenced by: '<S503>/Constant1'
                                        */
  real_T Switch_Threshold_p;           /* Expression: 0
                                        * Referenced by: '<S503>/Switch'
                                        */
  real_T Saturation_UpperSat_n3;       /* Expression: inf
                                        * Referenced by: '<S503>/Saturation'
                                        */
  real_T Saturation_LowerSat_gh;       /* Expression: 0
                                        * Referenced by: '<S503>/Saturation'
                                        */
  real_T Constant_Value_bp;            /* Expression: 0
                                        * Referenced by: '<S504>/Constant'
                                        */
  real_T Constant1_Value_ow;           /* Expression: 0
                                        * Referenced by: '<S504>/Constant1'
                                        */
  real_T Switch_Threshold_o;           /* Expression: 0
                                        * Referenced by: '<S504>/Switch'
                                        */
  real_T Saturation_UpperSat_lyi;      /* Expression: inf
                                        * Referenced by: '<S504>/Saturation'
                                        */
  real_T Saturation_LowerSat_e0;       /* Expression: 0
                                        * Referenced by: '<S504>/Saturation'
                                        */
  real_T TransferFcn_A_e;              /* Computed Parameter: TransferFcn_A_e
                                        * Referenced by: '<S508>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_o;              /* Computed Parameter: TransferFcn_C_o
                                        * Referenced by: '<S508>/Transfer Fcn'
                                        */
  real_T TransferFcn1_A_d;             /* Computed Parameter: TransferFcn1_A_d
                                        * Referenced by: '<S508>/Transfer Fcn1'
                                        */
  real_T TransferFcn1_C_lm;            /* Computed Parameter: TransferFcn1_C_lm
                                        * Referenced by: '<S508>/Transfer Fcn1'
                                        */
  real_T TransferFcn2_A_j;             /* Computed Parameter: TransferFcn2_A_j
                                        * Referenced by: '<S508>/Transfer Fcn2'
                                        */
  real_T TransferFcn2_C_l;             /* Computed Parameter: TransferFcn2_C_l
                                        * Referenced by: '<S508>/Transfer Fcn2'
                                        */
  real_T TransferFcn3_A_a;             /* Computed Parameter: TransferFcn3_A_a
                                        * Referenced by: '<S508>/Transfer Fcn3'
                                        */
  real_T TransferFcn3_C_o;             /* Computed Parameter: TransferFcn3_C_o
                                        * Referenced by: '<S508>/Transfer Fcn3'
                                        */
  real_T Constant_Value_od;            /* Expression: 0
                                        * Referenced by: '<S508>/Constant'
                                        */
  real_T RateLimiter_RisingLim_k;      /* Expression: 5
                                        * Referenced by: '<S508>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim_i;     /* Expression: -5
                                        * Referenced by: '<S508>/Rate Limiter'
                                        */
  real_T RateLimiter1_RisingLim;       /* Expression: 5
                                        * Referenced by: '<S508>/Rate Limiter1'
                                        */
  real_T RateLimiter1_FallingLim;      /* Expression: -5
                                        * Referenced by: '<S508>/Rate Limiter1'
                                        */
  real_T RateLimiter2_RisingLim;       /* Expression: 5
                                        * Referenced by: '<S508>/Rate Limiter2'
                                        */
  real_T RateLimiter2_FallingLim;      /* Expression: -5
                                        * Referenced by: '<S508>/Rate Limiter2'
                                        */
  real_T RateLimiter3_RisingLim;       /* Expression: 5
                                        * Referenced by: '<S508>/Rate Limiter3'
                                        */
  real_T RateLimiter3_FallingLim;      /* Expression: -5
                                        * Referenced by: '<S508>/Rate Limiter3'
                                        */
  real_T Switch1_Threshold_np;         /* Expression: 0.11
                                        * Referenced by: '<S508>/Switch1'
                                        */
  real_T Switch2_Threshold_f;          /* Expression: 0.11
                                        * Referenced by: '<S508>/Switch2'
                                        */
  real_T Switch3_Threshold_d;          /* Expression: 0.11
                                        * Referenced by: '<S508>/Switch3'
                                        */
  real_T Switch4_Threshold_j;          /* Expression: 0.11
                                        * Referenced by: '<S508>/Switch4'
                                        */
  real_T constant_Value;               /* Expression: 1
                                        * Referenced by: '<S114>/constant'
                                        */
  real_T ControlWordbypassOverridevalue_Value;/* Expression: 0
                                               * Referenced by: '<S114>/ControlWord bypass Override value'
                                               */
  real_T ControlWordbypassEnable_Value;/* Expression: 0
                                        * Referenced by: '<S114>/ControlWord bypass Enable'
                                        */
  real_T RPDO3Enable_Value;            /* Expression: 1
                                        * Referenced by: '<S116>/RPDO3 Enable '
                                        */
  real_T RPDO3period_Value;            /* Expression: 0.001
                                        * Referenced by: '<S116>/RPDO3 period'
                                        */
  real_T Memory2_InitialCondition_k;   /* Expression: 0
                                        * Referenced by: '<S124>/Memory2'
                                        */
  real_T Integrator_IC_j;              /* Expression: 0
                                        * Referenced by: '<S129>/Integrator'
                                        */
  real_T Integrator_UpperSat;          /* Expression: inf
                                        * Referenced by: '<S129>/Integrator'
                                        */
  real_T Integrator_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S129>/Integrator'
                                        */
  real_T MoogResetCmd_Value;           /* Expression: 0
                                        * Referenced by: '<S117>/MoogResetCmd'
                                        */
  real_T Memory_InitialCondition_h;    /* Expression: 0
                                        * Referenced by: '<S133>/Memory'
                                        */
  real_T PositionLearningTrigger_Value;/* Expression: 0
                                        * Referenced by: '<S117>/PositionLearningTrigger'
                                        */
  real_T Memory_InitialCondition_o;    /* Expression: 0
                                        * Referenced by: '<S134>/Memory'
                                        */
  real_T SineWave_Amp;                 /* Expression: 50
                                        * Referenced by: '<S114>/Sine Wave'
                                        */
  real_T SineWave_Bias;                /* Expression: 50
                                        * Referenced by: '<S114>/Sine Wave'
                                        */
  real_T SineWave_Freq;                /* Expression: 1
                                        * Referenced by: '<S114>/Sine Wave'
                                        */
  real_T SineWave_Phase;               /* Expression: 0
                                        * Referenced by: '<S114>/Sine Wave'
                                        */
  real_T traceselection_Value;         /* Expression: 0
                                        * Referenced by: '<S114>/trace selection'
                                        */
  real_T BrakePedalTargetPositionManualBypass_Value;/* Expression: 0
                                                     * Referenced by: '<S114>/BrakePedal Target Position Manual Bypass'
                                                     */
  real_T RateLimiter_RisingLim_e;      /* Expression: 50
                                        * Referenced by: '<S114>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim_l;     /* Expression: -50
                                        * Referenced by: '<S114>/Rate Limiter'
                                        */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<S114>/Rate Limiter'
                                        */
  real_T Switch_Threshold_j;           /* Expression: 0
                                        * Referenced by: '<S114>/Switch'
                                        */
  real_T BrakePedalTargetPositionBypassEnable_Value;/* Expression: 0
                                                     * Referenced by: '<S114>/BrakePedal TargetPosition Bypass Enable'
                                                     */
  real_T bped2br_N_tableData[4];       /* Expression: [0 0 25 100]
                                        * Referenced by: '<S107>/bped2br_N'
                                        */
  real_T bped2br_N_bp01Data[4];        /* Expression: [0 0.1 0.2 100]
                                        * Referenced by: '<S107>/bped2br_N'
                                        */
  real_T Switch2_Threshold_e5;         /* Expression: 0
                                        * Referenced by: '<S114>/Switch2'
                                        */
  real_T Gain_Gain_ka;                 /* Expression: 1
                                        * Referenced by: '<S114>/Gain'
                                        */
  real_T Constant2_Value_m4;           /* Expression: 100
                                        * Referenced by: '<S117>/Constant2'
                                        */
  real_T Memory_InitialCondition_i;    /* Expression: 0
                                        * Referenced by: '<S117>/Memory'
                                        */
  real_T Memory2_InitialCondition_f;   /* Expression: 0
                                        * Referenced by: '<S117>/Memory2'
                                        */
  real_T Memory1_InitialCondition_o;   /* Expression: 0
                                        * Referenced by: '<S117>/Memory1'
                                        */
  real_T ActuatorMinTravelForReady_Value;/* Expression: 35000
                                          * Referenced by: '<S117>/ActuatorMinTravelForReady'
                                          */
  real_T Switch1_Threshold_a;          /* Expression: 0
                                        * Referenced by: '<S114>/Switch1'
                                        */
  real_T Gain2_Gain_m;                 /* Expression: 1
                                        * Referenced by: '<S117>/Gain2'
                                        */
  real_T Constant_Value_ak;            /* Expression: 1
                                        * Referenced by: '<S129>/Constant'
                                        */
  real_T Constant1_Value_a;            /* Expression: 100
                                        * Referenced by: '<S117>/Constant1'
                                        */
  real_T AccelPositionManualBypass_Value;/* Expression: 0
                                          * Referenced by: '<S115>/AccelPosition Manual Bypass [%]'
                                          */
  real_T RateLimiter_RisingLim_p;      /* Expression: 50
                                        * Referenced by: '<S115>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim_l4;    /* Expression: -50
                                        * Referenced by: '<S115>/Rate Limiter'
                                        */
  real_T RateLimiter_IC_j;             /* Expression: 0
                                        * Referenced by: '<S115>/Rate Limiter'
                                        */
  real_T AccelPositionBypassEnable_Value;/* Expression: 0
                                          * Referenced by: '<S115>/AccelPosition Bypass Enable'
                                          */
  real_T Switch2_Threshold_m;          /* Expression: 0
                                        * Referenced by: '<S115>/Switch2'
                                        */
  real_T APP1Tbl_tableData[2];         /* Expression: [16 78.8]
                                        * Referenced by: '<S115>/APP1Tbl'
                                        */
  real_T APP1Tbl_bp01Data[2];          /* Expression: [0 100]
                                        * Referenced by: '<S115>/APP1Tbl'
                                        */
  real_T Gain3_Gain_g;                 /* Expression: 1/100
                                        * Referenced by: '<S115>/Gain3'
                                        */
  real_T VRef_Value;                   /* Expression: 5.01
                                        * Referenced by: '<S115>/VRef'
                                        */
  real_T Gain1_Gain_k;                 /* Expression: 1
                                        * Referenced by: '<S115>/Gain1'
                                        */
  real_T APP2Tbl_tableData[2];         /* Expression: [8 39.4]
                                        * Referenced by: '<S115>/APP2Tbl'
                                        */
  real_T APP2Tbl_bp01Data[2];          /* Expression: [0 100]
                                        * Referenced by: '<S115>/APP2Tbl'
                                        */
  real_T Gain4_Gain_p;                 /* Expression: 1/100
                                        * Referenced by: '<S115>/Gain4'
                                        */
  real_T Gain_Gain_c;                  /* Expression: 1
                                        * Referenced by: '<S115>/Gain'
                                        */
  real_T Gain2_Gain_a;                 /* Expression: 1
                                        * Referenced by: '<S115>/Gain2'
                                        */
  real_T MinthresholdforBrakeswitch_Value_o;/* Expression: 0
                                             * Referenced by: '<S107>/Min threshold for Brake switch'
                                             */
  real_T Ackermansteer_tableData[189];
  /* Expression: [-9.40000000000000	-9.30000000000000	-9.20000000000000	-9.10000000000000	-9	-8.90000000000000	-8.80000000000000	-8.70000000000000	-8.60000000000000	-8.50000000000000	-8.40000000000000	-8.30000000000000	-8.20000000000000	-8.10000000000000	-8	-7.90000000000000	-7.80000000000000	-7.70000000000000	-7.60000000000000	-7.50000000000000	-7.40000000000000	-7.30000000000000	-7.20000000000000	-7.10000000000000	-7	-6.90000000000000	-6.80000000000000	-6.70000000000000	-6.60000000000000	-6.50000000000000	-6.40000000000000	-6.30000000000000	-6.20000000000000	-6.10000000000000	-6	-5.90000000000000	-5.80000000000000	-5.70000000000000	-5.60000000000000	-5.50000000000000	-5.40000000000000	-5.30000000000000	-5.20000000000000	-5.10000000000000	-5	-4.90000000000000	-4.80000000000000	-4.70000000000000	-4.60000000000000	-4.50000000000000	-4.40000000000000	-4.30000000000000	-4.20000000000000	-4.10000000000000	-4	-3.90000000000000	-3.80000000000000	-3.70000000000000	-3.60000000000000	-3.50000000000000	-3.40000000000000	-3.30000000000000	-3.20000000000000	-3.10000000000000	-3	-2.90000000000000	-2.80000000000000	-2.70000000000000	-2.60000000000000	-2.50000000000000	-2.40000000000000	-2.30000000000000	-2.20000000000000	-2.10000000000000	-2	-1.90000000000000	-1.80000000000000	-1.70000000000000	-1.60000000000000	-1.50000000000000	-1.40000000000000	-1.30000000000000	-1.20000000000000	-1.10000000000000	-1	-0.900000000000000	-0.800000000000000	-0.700000000000000	-0.600000000000000	-0.500000000000000	-0.400000000000000	-0.300000000000000	-0.200000000000000	-0.100000000000000	0	0.100000000000000	0.200000000000000	0.300000000000000	0.400000000000000	0.500000000000000	0.600000000000000	0.700000000000000	0.800000000000000	0.900000000000000	1	1.10000000000000	1.20000000000000	1.30000000000000	1.40000000000000	1.50000000000000	1.60000000000000	1.70000000000000	1.80000000000000	1.90000000000000	2	2.10000000000000	2.20000000000000	2.30000000000000	2.40000000000000	2.50000000000000	2.60000000000000	2.70000000000000	2.80000000000000	2.90000000000000	3	3.10000000000000	3.20000000000000	3.30000000000000	3.40000000000000	3.50000000000000	3.60000000000000	3.70000000000000	3.80000000000000	3.90000000000000	4	4.10000000000000	4.20000000000000	4.30000000000000	4.40000000000000	4.50000000000000	4.60000000000000	4.70000000000000	4.80000000000000	4.90000000000000	5	5.10000000000000	5.20000000000000	5.30000000000000	5.40000000000000	5.50000000000000	5.60000000000000	5.70000000000000	5.80000000000000	5.90000000000000	6	6.10000000000000	6.20000000000000	6.30000000000000	6.40000000000000	6.50000000000000	6.60000000000000	6.70000000000000	6.80000000000000	6.90000000000000	7	7.10000000000000	7.20000000000000	7.30000000000000	7.40000000000000	7.50000000000000	7.60000000000000	7.70000000000000	7.80000000000000	7.90000000000000	8	8.10000000000000	8.20000000000000	8.30000000000000	8.40000000000000	8.50000000000000	8.60000000000000	8.70000000000000	8.80000000000000	8.90000000000000	9	9.10000000000000	9.20000000000000	9.30000000000000	9.40000000000000]
   * Referenced by: '<S3>/Ackerman steer'
   */
  real_T Ackermansteer_bp01Data[189];
  /* Expression: [-0.446316297916244	-0.442150786830951	-0.437977161945005	-0.433795286969319	-0.429605025209166	-0.425406239561593	-0.421198792513030	-0.416982546137136	-0.412757362092861	-0.408523101622746	-0.404279625551461	-0.400026794284603	-0.395764467807732	-0.391492505685692	-0.387210767062193	-0.382919110659678	-0.378617394779480	-0.374305477302279	-0.369983215688866	-0.365650466981227	-0.361307087803951	-0.356952934365975	-0.352587862462675	-0.348211727478321	-0.343824384388886	-0.339425687765245	-0.335015491776758	-0.330593650195247	-0.326160016399396	-0.321714443379563	-0.317256783743034	-0.312786889719717	-0.308304613168298	-0.303809805582871	-0.299302318100048	-0.294782001506567	-0.290248706247410	-0.285702282434450	-0.281142579855627	-0.276569447984685	-0.271982735991464	-0.267382292752788	-0.262767966863931	-0.258139606650706	-0.253497060182174	-0.248840175283994	-0.244168799552432	-0.239482780369037	-0.234781964916014	-0.230066200192295	-0.225335333030331	-0.220589210113630	-0.215827677995039	-0.211050583115807	-0.206257771825433	-0.201449090402323	-0.196624385075265	-0.191783502045761	-0.186926287511198	-0.182052587688919	-0.177162248841170	-0.172255117300976	-0.167331039498945	-0.162389861991014	-0.157431431487183	-0.152455594881217	-0.147462199281363	-0.142451092042095	-0.137422120796894	-0.132375133492088	-0.127309978421782	-0.122226504263869	-0.117124560117169	-0.112003995539695	-0.106864660588066	-0.101706405858090	-0.0965290825265349	-0.0913325423940914	-0.0861166379295599	-0.0808812223152671	-0.0756261494937309	-0.0703512742155897	-0.0650564520888085	-0.0597415396291770	-0.0544063943121130	-0.0490508746257825	-0.0436748401255510	-0.0382781514897739	-0.0328606705769398	-0.0274222604841732	-0.0219627856071077	-0.0164821117011354	-0.0109801059440403	-0.00545663700002112	0	0.00547767577609933	0.0110656090794470	0.0166755107772618	0.0223075038233658	0.0279617092740365	0.0336382462142580	0.0393372316825870	0.0450587805946413	0.0508030056652199	0.0565700173290652	0.0623599236602815	0.0681728302904245	0.0740088403252790	0.0798680542603446	0.0857505698950509	0.0916564822457268	0.0975858834573503	0.103538862714108	0.109515506148794	0.115515896751090	0.121540114274751	0.127588235143750	0.133660332357415	0.139756475394610	0.145876730117007	0.152021158671496	0.158189819391799	0.164382766699339	0.170600051003425	0.176841718600822	0.183107811574771	0.189398367693541	0.195713420308563	0.202052998252265	0.208417125735647	0.214805822245717	0.221219102442855	0.227656976058216	0.234119447791246	0.240606517207442	0.247118178636427	0.253654421070478	0.260215228063589	0.266800577631217	0.273410442150791	0.280044788263140	0.286703576774936	0.293386762562311	0.300094294475738	0.306826115246357	0.313582161393837	0.320362363135953	0.327166644299990	0.333994922236136	0.340847107733006	0.347723104935443	0.354622811264752	0.361546117341510	0.368492906911125	0.375463056772267	0.382456436708368	0.389472909422305	0.396512330474460	0.403574548224288	0.410659403775562	0.417766730925451	0.424896356117582	0.432048098399246	0.439221769382891	0.446417173212069	0.453634106531970	0.460872358464695	0.468131710589418	0.475411936927561	0.482712803933137	0.490034070488372	0.497375487904758	0.504736799929627	0.512117742758400	0.519518045052588	0.526937427963674	0.534375605162959	0.541832282877473	0.549307159932028	0.556799927797496	0.564310270645373	0.571837865408689	0.579382381849329	0.586943482631776	0.594520823403350	0.602114052880920	0.609722812944144	0.617346738735206	0.624985458765058]
   * Referenced by: '<S3>/Ackerman steer'
   */
  real_T Steer_Value_b;                /* Expression: 0
                                        * Referenced by: '<Root>/Steer'
                                        */
  real_T Throttle_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/Throttle'
                                        */
  int32_T Gain_Gain_jp;                /* Computed Parameter: Gain_Gain_jp
                                        * Referenced by: '<S117>/Gain'
                                        */
  int32_T Gain1_Gain_cz;               /* Computed Parameter: Gain1_Gain_cz
                                        * Referenced by: '<S114>/Gain1'
                                        */
  uint32_T R_maxIndex[2];              /* Computed Parameter: R_maxIndex
                                        * Referenced by: '<S146>/R'
                                        */
  uint32_T uDLookupTable_maxIndex[2];
                                   /* Computed Parameter: uDLookupTable_maxIndex
                                    * Referenced by: '<S218>/2-D Lookup Table'
                                    */
  uint32_T EffMap_maxIndex[2];         /* Computed Parameter: EffMap_maxIndex
                                        * Referenced by: '<S248>/Eff Map'
                                        */
  uint32_T uDLookupTable_maxIndex_p[2];
                                 /* Computed Parameter: uDLookupTable_maxIndex_p
                                  * Referenced by: '<S229>/2-D Lookup Table'
                                  */
  boolean_T Memory_InitialCondition_h5;/* Expression: false
                                        * Referenced by: '<S280>/Memory'
                                        */
  boolean_T Constant_Value_j0;         /* Expression: true
                                        * Referenced by: '<S280>/Constant'
                                        */
  int8_T Gain1_Gain_i;                 /* Computed Parameter: Gain1_Gain_i
                                        * Referenced by: '<S117>/Gain1'
                                        */
  uint8_T ManualSwitch2_CurrentSetting;
                             /* Computed Parameter: ManualSwitch2_CurrentSetting
                              * Referenced by: '<S4>/Manual Switch2'
                              */
  uint8_T ManualSwitch_CurrentSetting;
                              /* Computed Parameter: ManualSwitch_CurrentSetting
                               * Referenced by: '<S1>/Manual Switch'
                               */
  uint8_T ManualSwitch1_CurrentSetting;
                             /* Computed Parameter: ManualSwitch1_CurrentSetting
                              * Referenced by: '<S1>/Manual Switch1'
                              */
  uint8_T ManualSwitch_CurrentSetting_i;
                            /* Computed Parameter: ManualSwitch_CurrentSetting_i
                             * Referenced by: '<Root>/Manual Switch'
                             */
  uint8_T ManualSwitch6_CurrentSetting;
                             /* Computed Parameter: ManualSwitch6_CurrentSetting
                              * Referenced by: '<S267>/Manual Switch6'
                              */
  uint8_T ManualSwitch_CurrentSetting_in;
                           /* Computed Parameter: ManualSwitch_CurrentSetting_in
                            * Referenced by: '<S4>/Manual Switch'
                            */
  uint8_T manualTriggerSW_CurrentSetting;
                           /* Computed Parameter: manualTriggerSW_CurrentSetting
                            * Referenced by: '<S4>/manualTriggerSW'
                            */
  uint8_T ManualSwitch3_CurrentSetting;
                             /* Computed Parameter: ManualSwitch3_CurrentSetting
                              * Referenced by: '<S4>/Manual Switch3'
                              */
  uint8_T ManualSwitch1_CurrentSetting_b;
                           /* Computed Parameter: ManualSwitch1_CurrentSetting_b
                            * Referenced by: '<S4>/Manual Switch1'
                            */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_f4_T CoreSubsys_k;/* '<S717>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_i_T sf_MagicTireConstInput_gz;/* '<S708>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T CoreSubsys_d;/* '<S689>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_i_T sf_MagicTireConstInput_f;/* '<S680>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_dg_T CoreSubsys_e;/* '<S661>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_i_T sf_MagicTireConstInput_a;/* '<S652>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_jo_T CoreSubsys_od;/* '<S633>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_i_T sf_MagicTireConstInput_e;/* '<S624>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T CoreSubsys_n;/* '<S605>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput_m;/* '<S596>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T CoreSubsys_cu;/* '<S577>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput_d;/* '<S568>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_ls_T CoreSubsys_c;/* '<S549>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput_g;/* '<S540>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T CoreSubsys_h;/* '<S521>/Clutch Scalar Parameters' */
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T sf_MagicTireConstInput;/* '<S512>/Magic Tire Const Input' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T CoreSubsys_b;
  /* '<S279>/For each track and axle combination calculate suspension forces and moments' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T CoreSubsys_p;
                                     /* '<S324>/For Each Axle With Anti-Sway' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T CoreSubsys_o;
  /* '<S274>/For each track and axle combination calculate suspension forces and moments' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_f_T CoreSubsys_g;
         /* '<S274>/For each axle calculate axle cg positions and velocities' */
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_T CoreSubsys;
  /* '<S274>/For each axle and track calculate suspension and wheel positions and velocities' */
  P_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T IfActionSubsystem1_a;/* '<S227>/If Action Subsystem1' */
  P_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T IfActionSubsystem1;/* '<S216>/If Action Subsystem1' */
};

/* Real-time Model Data Structure */
struct tag_RTM_CAVE_MachE_dSPACE_250912_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_CAVE_MachE_dSPACE_250912_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_CAVE_MachE_dSPACE_250912_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[121];
  real_T odeF[4][121];
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
    struct {
      uint8_T TID[3];
    } TaskCounters;

    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* Block parameters (default storage) */
extern P_CAVE_MachE_dSPACE_250912_T CAVE_MachE_dSPACE_250912_P;

/* Block signals (default storage) */
extern B_CAVE_MachE_dSPACE_250912_T CAVE_MachE_dSPACE_250912_B;

/* Continuous states (default storage) */
extern X_CAVE_MachE_dSPACE_250912_T CAVE_MachE_dSPACE_250912_X;

/* Disabled states (default storage) */
extern XDis_CAVE_MachE_dSPACE_250912_T CAVE_MachE_dSPACE_250912_XDis;

/* Block states (default storage) */
extern DW_CAVE_MachE_dSPACE_250912_T CAVE_MachE_dSPACE_250912_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_CAVE_MachE_dSPACE_250912_T CAVE_MachE_dSPACE_250912_PrevZCX;

/* Model entry point functions */
extern void CAVE_MachE_dSPACE_250912_initialize(void);
extern void CAVE_MachE_dSPACE_250912_output(void);
extern void CAVE_MachE_dSPACE_250912_update(void);
extern void CAVE_MachE_dSPACE_250912_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CAVE_MachE_dSPACE_250912_T *const CAVE_MachE_dSPACE_250912_M;

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
 * '<Root>' : 'CAVE_MachE_dSPACE_250912'
 * '<S1>'   : 'CAVE_MachE_dSPACE_250912/FIXS'
 * '<S2>'   : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In'
 * '<S3>'   : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2'
 * '<S4>'   : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core'
 * '<S5>'   : 'CAVE_MachE_dSPACE_250912/FIXS/RealSimHILCycle'
 * '<S6>'   : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/MATLAB Function'
 * '<S7>'   : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/RealSim CreatBus'
 * '<S8>'   : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/Status//Control_Out [TCP (1)]'
 * '<S9>'   : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In'
 * '<S10>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP Out'
 * '<S11>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/generateTimeStepTrigger'
 * '<S12>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/speedCmd_gen'
 * '<S13>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/RealSim CreatBus/rgba2uint'
 * '<S14>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Ethernet Setup'
 * '<S15>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Receive_In [TCP (1)]'
 * '<S16>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Receive_Out [TCP (1)]'
 * '<S17>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Status//Control_In [TCP (1)]'
 * '<S18>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Transmit_In [TCP (1)]'
 * '<S19>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP Out/Transmit_Out [TCP (1)]'
 * '<S20>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/speedCmd_gen/Compare To Constant'
 * '<S21>'  : 'CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/speedCmd_gen/Compare To Zero'
 * '<S22>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad'
 * '<S23>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad'
 * '<S24>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad'
 * '<S25>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad'
 * '<S26>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1'
 * '<S27>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2'
 * '<S28>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad'
 * '<S29>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S30>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S31>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1'
 * '<S32>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2'
 * '<S33>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3'
 * '<S34>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S35>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad1'
 * '<S36>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad2'
 * '<S37>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad3'
 * '<S38>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad4'
 * '<S39>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad5'
 * '<S40>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad6'
 * '<S41>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad7'
 * '<S42>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad'
 * '<S43>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad'
 * '<S44>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad'
 * '<S45>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad'
 * '<S46>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad4/Ad'
 * '<S47>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad5/Ad'
 * '<S48>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad6/Ad'
 * '<S49>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad/Ad7/Ad'
 * '<S50>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad'
 * '<S51>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad1'
 * '<S52>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad2'
 * '<S53>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad3'
 * '<S54>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad4'
 * '<S55>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad/Ad'
 * '<S56>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad1/Ad'
 * '<S57>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad2/Ad'
 * '<S58>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad3/Ad'
 * '<S59>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad1/Ad4/Ad'
 * '<S60>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad'
 * '<S61>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad1'
 * '<S62>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad2'
 * '<S63>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad3'
 * '<S64>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad4'
 * '<S65>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad/Ad'
 * '<S66>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad1/Ad'
 * '<S67>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad2/Ad'
 * '<S68>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad3/Ad'
 * '<S69>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad2/Ad4/Ad'
 * '<S70>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad'
 * '<S71>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad1'
 * '<S72>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad2'
 * '<S73>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad/Ad'
 * '<S74>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad1/Ad'
 * '<S75>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad/Ad/Ad/Ad3/Ad2/Ad'
 * '<S76>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad'
 * '<S77>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad'
 * '<S78>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad'
 * '<S79>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad'
 * '<S80>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad1'
 * '<S81>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad2'
 * '<S82>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad3'
 * '<S83>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad4'
 * '<S84>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad5'
 * '<S85>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad/Ad'
 * '<S86>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad1/Ad'
 * '<S87>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad2/Ad'
 * '<S88>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad3/Ad'
 * '<S89>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad4/Ad'
 * '<S90>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad1/Ad/Ad/Ad/Ad5/Ad'
 * '<S91>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad'
 * '<S92>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad'
 * '<S93>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad'
 * '<S94>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad'
 * '<S95>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad1'
 * '<S96>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad/Ad'
 * '<S97>'  : 'CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In/Ad/Ad/Ad/Ad2/Ad/Ad/Ad/Ad1/Ad'
 * '<S98>'  : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Environment'
 * '<S99>'  : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Monitor'
 * '<S100>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Performance Calculations'
 * '<S101>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle'
 * '<S102>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Environment/Ground Feedback'
 * '<S103>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Environment/Subsystem'
 * '<S104>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Environment/Subsystem1'
 * '<S105>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Environment/Ground Feedback/Constant'
 * '<S106>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Monitor/Calculate Powertrain Energy'
 * '<S107>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs'
 * '<S108>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery'
 * '<S109>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle'
 * '<S110>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Front Wheel Drive'
 * '<S111>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor'
 * '<S112>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller '
 * '<S113>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics'
 * '<S114>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator'
 * '<S115>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output'
 * '<S116>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings'
 * '<S117>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface'
 * '<S118>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\ControlWord0\ISignal Value\ControlWord0 Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S119>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\ModesOfOperation\ISignal Value\ModesOfOperation Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S120>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\TargetPosition\ISignal Value\TargetPosition Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S121>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\ActualPosition\ISignal Value\ActualPosition Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S122>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\ModesOfOperationDisplay\ISignal Value\ModesOfOperationDisplay Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S123>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\StatusWord0\ISignal Value\StatusWord0 Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S124>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/ CAN diagnostic Counter'
 * '<S125>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/RPDO3\PDU Features\PDU Cyclic Timing Control\RPDO3 Timing Control Period [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S126>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/RPDO3\PDU Features\PDU Enable\RPDO3 Enable [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs]'
 * '<S127>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/TPDO3\PDU Features\PDU RX Status\TPDO3 Counter [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs]'
 * '<S128>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/ CAN diagnostic Counter/Compare To Constant3'
 * '<S129>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/ CAN diagnostic Counter/Timer'
 * '<S130>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/MOOG State Machine'
 * '<S131>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Max Stop Adjustment'
 * '<S132>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Min Stop Adjustment'
 * '<S133>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Rising edge'
 * '<S134>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/MOOG Interface/Rising edge1'
 * '<S135>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output/Voltage Out (1)'
 * '<S136>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output/Voltage Out (2)'
 * '<S137>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery'
 * '<S138>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Subsystem1'
 * '<S139>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1'
 * '<S140>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal'
 * '<S141>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus'
 * '<S142>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Output Passthrough'
 * '<S143>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery'
 * '<S144>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/Charge Model'
 * '<S145>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/State of Charge Capacity'
 * '<S146>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Datasheet Battery Internal/Datasheet Battery/Voltage and Power Calculation'
 * '<S147>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator'
 * '<S148>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S149>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrStored Input'
 * '<S150>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Battery/Battery/Lithium Ion Battery Pack1/Info Bus/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S151>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control'
 * '<S152>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Get Cycle Clock Time'
 * '<S153>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set'
 * '<S154>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE'
 * '<S155>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF'
 * '<S156>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive'
 * '<S157>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE/In-house Speed Controller'
 * '<S158>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE/In-house Speed Controller/Compare To Constant1'
 * '<S159>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE/In-house Speed Controller/Compare To Constant3'
 * '<S160>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE/In-house Speed Controller/Detect Fall Negative'
 * '<S161>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE/In-house Speed Controller/Detect Rise Positive'
 * '<S162>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE/In-house Speed Controller/Detect Fall Negative/Negative'
 * '<S163>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/In-house CAVE/In-house Speed Controller/Detect Rise Positive/Positive'
 * '<S164>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Compare To Constant1'
 * '<S165>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver'
 * '<S166>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/External Action Routing'
 * '<S167>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver'
 * '<S168>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control'
 * '<S169>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/LPF'
 * '<S170>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Routing'
 * '<S171>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Shift'
 * '<S172>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control/Scalar'
 * '<S173>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control/Scalar/Accel Override'
 * '<S174>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control/Scalar/Deccel Override'
 * '<S175>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control/Scalar/Accel Override/Signal Hold'
 * '<S176>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control/Scalar/Accel Override/Signal Hold/Pass Through'
 * '<S177>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control/Scalar/Deccel Override/Signal Hold'
 * '<S178>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Control/Scalar/Deccel Override/Signal Hold/Pass Through'
 * '<S179>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/LPF/LPF'
 * '<S180>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/LPF/LPF/Error LPF'
 * '<S181>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Routing/Error Metrics'
 * '<S182>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/PIF/Longitudinal Driver/Longitudinal Driver/Shift/None'
 * '<S183>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Compare To Constant1'
 * '<S184>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2'
 * '<S185>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/External Action Routing'
 * '<S186>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver'
 * '<S187>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control'
 * '<S188>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/LPF'
 * '<S189>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Routing'
 * '<S190>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Shift'
 * '<S191>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive'
 * '<S192>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Accel Override'
 * '<S193>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Cont LPF'
 * '<S194>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Deccel Override'
 * '<S195>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Powertrain Response'
 * '<S196>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Setup'
 * '<S197>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Vehicle'
 * '<S198>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Accel Override/Signal Hold'
 * '<S199>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Accel Override/Signal Hold/Pass Through'
 * '<S200>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Deccel Override/Signal Hold'
 * '<S201>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Deccel Override/Signal Hold/Pass Through'
 * '<S202>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Control/Predictive/Powertrain Response/Unfiltered'
 * '<S203>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/LPF/pass'
 * '<S204>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Routing/Error Metrics'
 * '<S205>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Driver and Drive Cycle/Driver and PI with FF Control/Speed controller set/Predictive/Longitudinal Driver2/Longitudinal Driver/Shift/None'
 * '<S206>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Front Wheel Drive/Ideal Diff AWD'
 * '<S207>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Front Wheel Drive/Ideal Diff AWD/Power Bus'
 * '<S208>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Front Wheel Drive/Ideal Diff AWD/Power Bus/Power Accounting Bus Creator'
 * '<S209>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Front Wheel Drive/Ideal Diff AWD/Power Bus/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S210>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Front Wheel Drive/Ideal Diff AWD/Power Bus/Power Accounting Bus Creator/PwrStored Input'
 * '<S211>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Front Wheel Drive/Ideal Diff AWD/Power Bus/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S212>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor'
 * '<S213>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor'
 * '<S214>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2'
 * '<S215>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Electrical Current'
 * '<S216>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Maximum Torque and Power '
 * '<S217>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Motor Units'
 * '<S218>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Tabular Power Loss Data'
 * '<S219>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem'
 * '<S220>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem1'
 * '<S221>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator'
 * '<S222>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S223>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrStored Input'
 * '<S224>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mach-E Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S225>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2'
 * '<S226>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Electrical Current'
 * '<S227>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power '
 * '<S228>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units'
 * '<S229>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Tabular Power Loss Data'
 * '<S230>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem'
 * '<S231>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Maximum Torque and Power /If Action Subsystem1'
 * '<S232>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator'
 * '<S233>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S234>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrStored Input'
 * '<S235>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Motor/Mapped Motor/Mapped Motor Core Speed 2/Motor Units/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S236>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command'
 * '<S237>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Subsystem1'
 * '<S238>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Subsystem4'
 * '<S239>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Accel Pedal to Traction Wheel Torque Request1'
 * '<S240>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Battery Management System'
 * '<S241>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management'
 * '<S242>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Series Regen Braking'
 * '<S243>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Accel Pedal to Traction Wheel Torque Request1/Max Motor Torque'
 * '<S244>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Compare To Constant'
 * '<S245>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management'
 * '<S246>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits'
 * '<S247>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Compare To Constant'
 * '<S248>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Mech to Elec Power Estimate'
 * '<S249>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit'
 * '<S250>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly'
 * '<S251>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Compare To Zero'
 * '<S252>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Compare To Zero1'
 * '<S253>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Check Within Limits/Saturation Dynamic1'
 * '<S254>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit/Max Motor Torque'
 * '<S255>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/Torque Limit/Saturation Dynamic'
 * '<S256>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly/Compare To Constant'
 * '<S257>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Motor Torque Arbitration and Power Management/Power Management/div0protect - poly/Compare To Constant1'
 * '<S258>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Powertrain Supervisory Controller /Engine Torque Command/Series Regen Braking/Max Motor Torque'
 * '<S259>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF'
 * '<S260>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Brake Pressure'
 * '<S261>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering'
 * '<S262>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Subsystem1'
 * '<S263>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension'
 * '<S264>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle'
 * '<S265>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle Routing1'
 * '<S266>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheel Routing1'
 * '<S267>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires'
 * '<S268>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering'
 * '<S269>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput'
 * '<S270>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput/ParalConstRatio'
 * '<S271>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Steering/Kinematic Steering/AngInput/ParalConstRatio/Parallel'
 * '<S272>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Subsystem1/Subsystem'
 * '<S273>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension'
 * '<S274>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring'
 * '<S275>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Steer Rate Adapter'
 * '<S276>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Steering Adapter'
 * '<S277>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Vehicle Adapter'
 * '<S278>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Wheel  Adapter'
 * '<S279>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson'
 * '<S280>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/Add Axle Offsets'
 * '<S281>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle and track calculate suspension and wheel positions and velocities'
 * '<S282>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities'
 * '<S283>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments'
 * '<S284>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/Wheel Carrier to Axle Interface Compliance'
 * '<S285>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle and track calculate suspension and wheel positions and velocities/Select DCM'
 * '<S286>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics'
 * '<S287>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics'
 * '<S288>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Suspension Forces and Moments'
 * '<S289>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Track location transforms'
 * '<S290>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments'
 * '<S291>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics/Select Axle Mass By Axle'
 * '<S292>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/2 DOF axle dynamics/Select X Axis Axle Mass Moment of Inertia By Axle'
 * '<S293>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Track location transforms/DCM Transpose'
 * '<S294>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments/Select Axle Mass By Axle'
 * '<S295>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each axle calculate axle cg positions and velocities/Solid Axle 2 DOF CG Dynamics/Wheel Forces and Moments/Select Y Axis Axle Mass Moment of Inertia By Axle'
 * '<S296>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension'
 * '<S297>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Steering Delta Select'
 * '<S298>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations'
 * '<S299>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Vehicle Moments From X and Y Forces'
 * '<S300>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic'
 * '<S301>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled'
 * '<S302>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Adjust Camber Sign For Track'
 * '<S303>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Convert From Steer To Toe'
 * '<S304>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Convert from Toe To Steer'
 * '<S305>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Camber Height Slope'
 * '<S306>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Camber Steering Center'
 * '<S307>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Caster Height Slope'
 * '<S308>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Caster Steering Center'
 * '<S309>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Roll Steer Slope'
 * '<S310>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Disabled/Select Toe Steering Center'
 * '<S311>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension'
 * '<S312>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination'
 * '<S313>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope'
 * '<S314>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force'
 * '<S315>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select C By Axle'
 * '<S316>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select F0 By Axle'
 * '<S317>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select K By Axle'
 * '<S318>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select Max Travel By Axle'
 * '<S319>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Max stop reached'
 * '<S320>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Min stop reached'
 * '<S321>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/Solid Axle Suspension – Coil Spring/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Disabled'
 * '<S322>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force'
 * '<S323>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments'
 * '<S324>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force'
 * '<S325>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway'
 * '<S326>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta'
 * '<S327>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Arm Neutral Angle By Axle'
 * '<S328>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Arm Radius By Axle'
 * '<S329>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/Anti-Sway Force/Anti-Sway Force/For Each Axle With Anti-Sway/Anti-Sway Force Delta/Anti-Sway Bar Torsion Spring Constant By Axle'
 * '<S330>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension'
 * '<S331>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Steering Delta Select'
 * '<S332>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations'
 * '<S333>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Vehicle Moments From X and Y Forces'
 * '<S334>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic'
 * '<S335>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled'
 * '<S336>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Adjust Camber Sign For Track'
 * '<S337>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Convert From Steer To Toe'
 * '<S338>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Convert From Toe To Steer'
 * '<S339>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Height Slope'
 * '<S340>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Steering Center'
 * '<S341>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Camber Steering Slope'
 * '<S342>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Height Slope'
 * '<S343>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Steering Center'
 * '<S344>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Caster Steering Slope'
 * '<S345>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Roll Steer Slope'
 * '<S346>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Toe Steering Center'
 * '<S347>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Suspension Angle Calculations/Ideal Steering Enabled/Select Toe Steering Slope'
 * '<S348>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension'
 * '<S349>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination'
 * '<S350>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope'
 * '<S351>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force'
 * '<S352>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select C By Axle'
 * '<S353>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select F0 By Axle'
 * '<S354>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select K By Axle'
 * '<S355>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Select Max Travel By Axle'
 * '<S356>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Max stop reached'
 * '<S357>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Constrained spring damper combination/Hardstop Feedback Force/Min stop reached'
 * '<S358>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Enabled'
 * '<S359>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Suspension/MacPherson Front Suspension Solid Axle Rear Suspension/independent Suspensions - MacPherson/For each track and axle combination calculate suspension forces and moments/Suspension/Z axis suspension characteristic/Ideal Suspension/Steering Height Slope/Steering Enabled/Steering Height Slope By Steered Axle'
 * '<S360>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem'
 * '<S361>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF'
 * '<S362>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track'
 * '<S363>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Cy'
 * '<S364>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Drag'
 * '<S365>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing'
 * '<S366>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/friction'
 * '<S367>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/front forces'
 * '<S368>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/front steer'
 * '<S369>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/hitch geometry parameters'
 * '<S370>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/rear forces'
 * '<S371>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/rear steer'
 * '<S372>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/sigma'
 * '<S373>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/state'
 * '<S374>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/vehicle model'
 * '<S375>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/wind'
 * '<S376>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Cy/Cy const dual'
 * '<S377>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Drag/Drag Force'
 * '<S378>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Drag/inertial2body'
 * '<S379>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual'
 * '<S380>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Forces 3DOF'
 * '<S381>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF'
 * '<S382>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Moments'
 * '<S383>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Power'
 * '<S384>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus'
 * '<S385>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left'
 * '<S386>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right'
 * '<S387>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric'
 * '<S388>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left'
 * '<S389>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right'
 * '<S390>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform'
 * '<S391>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement'
 * '<S392>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S393>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S394>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S395>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S396>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S397>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S398>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S399>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement'
 * '<S400>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S401>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S402>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S403>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S404>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S405>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S406>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Front Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S407>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta'
 * '<S408>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip'
 * '<S409>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Rotation Angles to Direction Cosine Matrix'
 * '<S410>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/transform to Inertial axes'
 * '<S411>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/transform to Inertial axes1'
 * '<S412>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR'
 * '<S413>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly'
 * '<S414>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly/Compare To Constant'
 * '<S415>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Body Slip/div0protect - abs poly/Compare To Constant1'
 * '<S416>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S417>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR/Subsystem'
 * '<S418>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Geometric/Hard Point Coordinate Transform External Displacement Beta/wxR/Subsystem1'
 * '<S419>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement'
 * '<S420>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S421>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S422>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S423>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S424>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S425>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S426>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Left/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S427>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement'
 * '<S428>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S429>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S430>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S431>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S432>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S433>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S434>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hard Point Coordinate Transform Rear Right/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S435>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement'
 * '<S436>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix'
 * '<S437>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement/transform to Inertial axes'
 * '<S438>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement/transform to Inertial axes1'
 * '<S439>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement/wxR'
 * '<S440>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S441>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement/wxR/Subsystem'
 * '<S442>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Lateral 3DOF/Hitch Coordinate Transform/Hard Point Coordinate Transform External Displacement/wxR/Subsystem1'
 * '<S443>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Power/Power Accounting Bus Creator'
 * '<S444>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Power/xdot mode'
 * '<S445>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Power/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S446>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Power/Power Accounting Bus Creator/PwrStored Input'
 * '<S447>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Power/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S448>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/Power/xdot mode/FxIn'
 * '<S449>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Angle Wrap'
 * '<S450>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Body Slip'
 * '<S451>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/COMB2I'
 * '<S452>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Rotation Angles to Direction Cosine Matrix'
 * '<S453>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/xddot2ax'
 * '<S454>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Angle Wrap/Wrap'
 * '<S455>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Body Slip/div0protect - abs poly'
 * '<S456>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Body Slip/div0protect - abs poly/Compare To Constant'
 * '<S457>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Body Slip/div0protect - abs poly/Compare To Constant1'
 * '<S458>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S459>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/Signal Routing/Signal Routing Dual/state2bus/xddot2ax/m^22gn'
 * '<S460>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/friction/mu int dual'
 * '<S461>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/front forces/ext dual'
 * '<S462>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/front steer/delta int dual'
 * '<S463>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/hitch geometry parameters/hitch inactive'
 * '<S464>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/rear forces/ext dual'
 * '<S465>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/rear steer/delta int dual'
 * '<S466>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/sigma/no sigma dual'
 * '<S467>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/state/xdot int'
 * '<S468>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Vehicle/Variant Subsystem/3DOF/Vehicle Body 3DOF Dual Track/wind/wind ext'
 * '<S469>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/Cont LPF'
 * '<S470>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/Cont LPF1'
 * '<S471>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS'
 * '<S472>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh'
 * '<S473>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires'
 * '<S474>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire'
 * '<S475>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/scale factors with friction'
 * '<S476>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Routiong'
 * '<S477>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel Angles'
 * '<S478>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform'
 * '<S479>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix Retro: ZYX'
 * '<S480>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tire2Veh/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix Retro: ZYX/Subsystem'
 * '<S481>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires'
 * '<S482>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF'
 * '<S483>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1'
 * '<S484>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2'
 * '<S485>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3'
 * '<S486>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4'
 * '<S487>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5'
 * '<S488>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6'
 * '<S489>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7'
 * '<S490>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fx'
 * '<S491>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fy'
 * '<S492>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Fz'
 * '<S493>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/MATLAB Function'
 * '<S494>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/MATLAB Function1'
 * '<S495>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/MATLAB Function2'
 * '<S496>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/MATLAB Function3'
 * '<S497>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Mx'
 * '<S498>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/My'
 * '<S499>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Mz'
 * '<S500>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Re'
 * '<S501>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response'
 * '<S502>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response1'
 * '<S503>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response2'
 * '<S504>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Vertical Wheel and Unsprung Mass Response3'
 * '<S505>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/alpha'
 * '<S506>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/kappa'
 * '<S507>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/omega'
 * '<S508>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/omega1'
 * '<S509>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/z'
 * '<S510>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/zdot'
 * '<S511>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Bus Routing'
 * '<S512>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Magic Tire Const Input'
 * '<S513>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Vertical DOF'
 * '<S514>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module'
 * '<S515>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Magic Tire Const Input/Fx Relaxation'
 * '<S516>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Magic Tire Const Input/Fy Relaxation'
 * '<S517>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Magic Tire Const Input/Magic Tire Const Input'
 * '<S518>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Magic Tire Const Input/My Relaxation'
 * '<S519>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S520>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Brakes'
 * '<S521>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch'
 * '<S522>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Friction Model'
 * '<S523>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Brakes/Disk Brake'
 * '<S524>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S525>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S526>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S527>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S528>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S529>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S530>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S531>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S532>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S533>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S534>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S535>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S536>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S537>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S538>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S539>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Bus Routing'
 * '<S540>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input'
 * '<S541>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Vertical DOF'
 * '<S542>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module'
 * '<S543>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Fx Relaxation'
 * '<S544>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Fy Relaxation'
 * '<S545>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/Magic Tire Const Input'
 * '<S546>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Magic Tire Const Input/My Relaxation'
 * '<S547>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S548>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes'
 * '<S549>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch'
 * '<S550>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Friction Model'
 * '<S551>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes/Disk Brake'
 * '<S552>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S553>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S554>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S555>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S556>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S557>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S558>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S559>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S560>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S561>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S562>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S563>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S564>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S565>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S566>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF1/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S567>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Bus Routing'
 * '<S568>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input'
 * '<S569>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Vertical DOF'
 * '<S570>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module'
 * '<S571>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Fx Relaxation'
 * '<S572>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Fy Relaxation'
 * '<S573>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/Magic Tire Const Input'
 * '<S574>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Magic Tire Const Input/My Relaxation'
 * '<S575>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S576>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes'
 * '<S577>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch'
 * '<S578>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Friction Model'
 * '<S579>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes/Disk Brake'
 * '<S580>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S581>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S582>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S583>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S584>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S585>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S586>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S587>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S588>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S589>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S590>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S591>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S592>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S593>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S594>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF2/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S595>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Bus Routing'
 * '<S596>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input'
 * '<S597>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Vertical DOF'
 * '<S598>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module'
 * '<S599>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Fx Relaxation'
 * '<S600>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Fy Relaxation'
 * '<S601>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/Magic Tire Const Input'
 * '<S602>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Magic Tire Const Input/My Relaxation'
 * '<S603>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S604>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes'
 * '<S605>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch'
 * '<S606>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Friction Model'
 * '<S607>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes/Disk Brake'
 * '<S608>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S609>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S610>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S611>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S612>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S613>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S614>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S615>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S616>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S617>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S618>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S619>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S620>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S621>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S622>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF3/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S623>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Bus Routing'
 * '<S624>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input'
 * '<S625>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Vertical DOF'
 * '<S626>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module'
 * '<S627>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Fx Relaxation'
 * '<S628>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Fy Relaxation'
 * '<S629>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/Magic Tire Const Input'
 * '<S630>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Magic Tire Const Input/My Relaxation'
 * '<S631>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S632>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes'
 * '<S633>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch'
 * '<S634>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Friction Model'
 * '<S635>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes/Disk Brake'
 * '<S636>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S637>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S638>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S639>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S640>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S641>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S642>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S643>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S644>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S645>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S646>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S647>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S648>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S649>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S650>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF4/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S651>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Bus Routing'
 * '<S652>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Magic Tire Const Input'
 * '<S653>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Vertical DOF'
 * '<S654>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module'
 * '<S655>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Magic Tire Const Input/Fx Relaxation'
 * '<S656>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Magic Tire Const Input/Fy Relaxation'
 * '<S657>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Magic Tire Const Input/Magic Tire Const Input'
 * '<S658>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Magic Tire Const Input/My Relaxation'
 * '<S659>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S660>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Brakes'
 * '<S661>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch'
 * '<S662>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Friction Model'
 * '<S663>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Brakes/Disk Brake'
 * '<S664>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S665>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S666>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S667>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S668>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S669>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S670>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S671>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S672>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S673>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S674>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S675>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S676>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S677>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S678>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF5/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S679>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Bus Routing'
 * '<S680>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Magic Tire Const Input'
 * '<S681>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Vertical DOF'
 * '<S682>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module'
 * '<S683>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Magic Tire Const Input/Fx Relaxation'
 * '<S684>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Magic Tire Const Input/Fy Relaxation'
 * '<S685>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Magic Tire Const Input/Magic Tire Const Input'
 * '<S686>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Magic Tire Const Input/My Relaxation'
 * '<S687>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S688>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Brakes'
 * '<S689>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch'
 * '<S690>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Friction Model'
 * '<S691>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Brakes/Disk Brake'
 * '<S692>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S693>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S694>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S695>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S696>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S697>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S698>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S699>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S700>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S701>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S702>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S703>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S704>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S705>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S706>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF6/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S707>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Bus Routing'
 * '<S708>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Magic Tire Const Input'
 * '<S709>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Vertical DOF'
 * '<S710>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module'
 * '<S711>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Magic Tire Const Input/Fx Relaxation'
 * '<S712>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Magic Tire Const Input/Fy Relaxation'
 * '<S713>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Magic Tire Const Input/Magic Tire Const Input'
 * '<S714>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Magic Tire Const Input/My Relaxation'
 * '<S715>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Vertical DOF/Vertical Wheel and Unsprung Mass Response'
 * '<S716>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Brakes'
 * '<S717>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch'
 * '<S718>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Friction Model'
 * '<S719>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Brakes/Disk Brake'
 * '<S720>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Brakes/Disk Brake/Disk Brake'
 * '<S721>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters'
 * '<S722>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch'
 * '<S723>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Locked'
 * '<S724>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/Slipping'
 * '<S725>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup'
 * '<S726>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip'
 * '<S727>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic'
 * '<S728>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Break Apart Detection'
 * '<S729>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection'
 * '<S730>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup FSM'
 * '<S731>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Requisite Friction'
 * '<S732>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Friction Calc'
 * '<S733>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectLockup/Friction Mode Logic/Lockup Detection/Required Friction for Lockup'
 * '<S734>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Tires/MF Tires/Combined Slip Wheel 2DOF7/Wheel Module/Clutch/Clutch Scalar Parameters/Clutch/detectSlip/Break Apart Detection'
 * '<S735>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel Angles'
 * '<S736>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform'
 * '<S737>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix Retro: ZYX'
 * '<S738>' : 'CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Vehicle Dynamics/Vehicle 14DOF/Wheels and Tires/VDBS/Veh2Tire/Wheel to Body Transform/Rotation Angles to Direction Cosine Matrix Retro: ZYX/Subsystem'
 */
#endif                                 /* CAVE_MachE_dSPACE_250912_h_ */
