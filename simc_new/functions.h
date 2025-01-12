//*********************************************************************
// ~NAME : functions.h
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Mathematical functions header
//**********************************************************************
#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <time.h>

#include "datatypes.h"

// ----------------------------------------------------------------
// trnAxis enum
// ----------------------------------------------------------------
typedef enum CartesianCoordsEnum
{
	X_AXIS,
	Y_AXIS,
	Z_AXIS
} crtCoordsEnum;

// ----------------------------------------------------------------
// rotAxis enum
// ----------------------------------------------------------------
typedef enum EulerAngsEnum
{
	PHI_AXIS,
	THT_AXIS,
	PSI_AXIS
} eulerAngsEnum;

// ----------------------------------------------------------------
// Input enum
// ----------------------------------------------------------------
typedef enum InputEnum
{
	NOT_SPEC,
	PRCNT,
	PRCNT_NEG,
	MILLIMETER,
	DEGREE,
	VIANO,
	POSITIVE_INPUT
} inputEnum;

struct tm timeinfo;

int sgn(double x);
r3 rotateOnAnAxis(r3 in, crtCoordsEnum axis, double angle_in_deg);
void dateTimeForm(char date_time[21], char string[21], struct tm timeinfo);

void print_r3(const int a, const int b, const r3 r3var);
void print_r3x3(const int a, const int b, const r3x3 r3x3var);

// ----------------------------------------------------------------
// input functions
// ----------------------------------------------------------------
double inputDouble(inputEnum inputType);
char inputMenu(void);

#endif //_FUNCTIONS_H_
