//*********************************************************************
// ~NAME : printdata.h
// ~INSTITUTION : ITU
// ~DESCRIPTION : Data export function
//**********************************************************************
#ifndef _PRINTDATA_H_
#define _PRINTDATA_H_

#include <stdbool.h>

#define MAX_ARRAY_SAMPLE (50000)
#define WIN_ROOT_CHAR_NB 13

typedef enum _record
{
	STOP_RECORD,
	START_RECORD
} record;

typedef enum _recordOnOff
{
	RECORD_OFF,
	RECORD_ON
} recordOnOff;

// ----------------------------------------------------------------
// _PrintData structure
// ----------------------------------------------------------------
typedef struct _PrintData
{
	unsigned int	cycleNb, testNb;
	record			rec;
	bool			unprinted_test; // flag for showing print data entry in the main menu
	//------------------------------------------------//
	float q_posFbk[MAX_ARRAY_SAMPLE][6];
	float q_velFbk[MAX_ARRAY_SAMPLE][6];
	float q_torCmd[MAX_ARRAY_SAMPLE][6];
	float q_torCtr[MAX_ARRAY_SAMPLE][6];
	float q_torFrc[MAX_ARRAY_SAMPLE][6];
	float q_torSpr[MAX_ARRAY_SAMPLE][6];
} PrintData;
PrintData _pd;

void printAlgoData(int testNb);
void printMotionInfo(void);
void printMotionData(void);
void recordData(void);


#endif //_PRINTDATA_H_
