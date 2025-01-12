//*********************************************************************
// ~NAME : functions.c
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Useful functions
//**********************************************************************

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "functions.h"
#include "test.h"

// ----------------------------------------------------------------
// sign function
// ----------------------------------------------------------------
int sgn(double x)
{
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

r3 rotateOnAnAxis(r3 in, crtCoordsEnum axis, double angle_in_deg)
{
	double angle;
	r3x3 rot_mat = { 0.0 };

	angle = angle_in_deg * _DEG_TO_RAD;

	switch (axis)
	{
	case X_AXIS:
		rot_mat = set_r3x3(	1.0	, 0.0			, 0.0			,
							0.0	, cos(angle)	, -sin(angle)	,
							0.0	, sin(angle)	, cos(angle)	);
		break;
	case Y_AXIS:
		rot_mat = set_r3x3(	cos(angle)	, 0.0	, sin(angle)	,
							0.0			, 1.0	, 0.0			,
							-sin(angle)	, 0.0	, cos(angle)	);
		break;
	case Z_AXIS:
		rot_mat = set_r3x3(	cos(angle)	, -sin(angle)	, 0.0	,
							sin(angle)	, cos(angle)	, 0.0	,
							0.0			, 0.0			, 1.0	);
		break;
	default:
		printf("\nInvalid axis enum!");
		break;
	}

	return mul_r3x3_and_r3(rot_mat, in);
}

// ----------------------------------------------------------------
// dateTimeForm
// ----------------------------------------------------------------
void dateTimeForm(char date_time[21], char string[21], struct tm timeinfo)
{
	char year[5];
	char mon[3];
	char mday[3];
	char hour[3];
	char min[3];
	char sec[3];

	sprintf_s(year, 5, "%d", timeinfo.tm_year + 1900);

	if (timeinfo.tm_mon + 1 >= 10)
		sprintf_s(mon, 3, "%d", timeinfo.tm_mon + 1);
	else
		sprintf_s(mon, 3, "0%d", timeinfo.tm_mon + 1);

	if (timeinfo.tm_mday >= 10)
		sprintf_s(mday, 3, "%d", timeinfo.tm_mday);
	else
		sprintf_s(mday, 3, "0%d", timeinfo.tm_mday);

	if (timeinfo.tm_hour >= 10)
		sprintf_s(hour, 3, "%d", timeinfo.tm_hour);
	else
		sprintf_s(hour, 3, "0%d", timeinfo.tm_hour);

	if (timeinfo.tm_min >= 10)
		sprintf_s(min, 3, "%d", timeinfo.tm_min);
	else
		sprintf_s(min, 3, "0%d", timeinfo.tm_min);

	if (timeinfo.tm_sec >= 10)
		sprintf_s(sec, 3, "%d", timeinfo.tm_sec);
	else
		sprintf_s(sec, 3, "0%d", timeinfo.tm_sec);

	sprintf_s(date_time, 21, string, year, mon, mday, hour, min, sec);
}

void print_r3(const int a, const int b, const r3 r3var)
{
	int i;

	for (i = 0; i < 3; i++)
	{
		printf("\n%*.*f", a, b, r3var.data[i]);
	}
	printf("\n");
}

void print_r3x3(const int a, const int b, const r3x3 r3x3var)
{
	int i;

	for (i = 0; i < 3; i++)
	{
		printf("\n%*.*f %*.*f %*.*f", a, b, r3x3var.data[i][0], a, b, r3x3var.data[i][1], a, b, r3x3var.data[i][2]);
	}
	printf("\n");
}

// ----------------------------------------------------------------
// inputDouble
// ----------------------------------------------------------------
double inputDouble(inputEnum inputType)
{
	char userInput[64];
	int cond_check;
	double result;
	unsigned int strlenMin, strlenMax;
	double rangeMin, rangeMax;

	switch (inputType)
	{
	case PRCNT:
		strlenMin = 2;
		strlenMax = 4;
		rangeMin = 0;
		rangeMax = 100;
		break;
	case PRCNT_NEG:
		strlenMin = 2;
		strlenMax = 5;
		rangeMin = -100;
		rangeMax = 100;
		break;
	case MILLIMETER:
		strlenMin = 2;
		strlenMax = 8;
		rangeMin = -1e4;
		rangeMax = 1e4;
		break;
	case DEGREE:
		strlenMin = 2;
		strlenMax = 8;
		rangeMin = -1e3;
		rangeMax = 1e3;
		break;
	default:
		strlenMin = 2;
		strlenMax = 8;
		rangeMin = -1e7;
		rangeMax = 1e7;
		break;
	}

	while (1)
	{
		cond_check = 0;

		fgets(userInput, 63, stdin);

		if (strlen(userInput) < strlenMin || strlen(userInput) > strlenMax)
		{
			printf("\nstrlen failed. \n");
			cond_check++;
		}

		if (sscanf_s(userInput, " %lf", &result) != 1)
		{
			printf("\nsscanf_s failed. \n");
			cond_check++;
		}

		if (result < rangeMin || result >= rangeMax)
		{
			printf("\nout of range. \n");
			cond_check++;
		}

		if (cond_check == 0)
		{
			break;
		}
		else
		{
			printf("\ntry again: ");
		}
	}

	if ((inputType == PRCNT) || (inputType == PRCNT_NEG))
		result = result * 0.01;
	else if (inputType == MILLIMETER)
		result = result * _MM_TO_M;
	else if (inputType == DEGREE)
		result = result * _DEG_TO_RAD;

	return result;
}

// ----------------------------------------------------------------
// inputMenu
// ----------------------------------------------------------------
char inputMenu(void)
{
	char	userInput[64];
	int		cond_check;
	char	result;

	while (1)
	{
		cond_check = 0;

		fgets(userInput, 63, stdin);

		if (strlen(userInput) != 2)
		{
			printf("\nstrlen failed. \n");
			cond_check++;
		}

		if (sscanf_s(userInput, " %c", &result, 64) != 1)
		{
			printf("\nsscanf_s failed. \n");
			cond_check++;
		}

		if (cond_check == 0)
		{
			break;
		}
		else
		{
			printf("\n\t Selection ? ");
		}
	}

	return result;
}
