/**
  **************************************************************************************
  * @file    rov2016_Filter.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains macros and extern function prototypes for rov2016_Filter.c
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/
//#define BLOCK_SIZE 10
#define DESIMATION_FACTOR 10
#define NUM_TAPS 64


/* Extern function prototypes ----------------------------------------------------------*/
extern void FIR_decimate(float *pSrc, float *pDst, float *pState);







