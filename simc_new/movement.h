//*********************************************************************
// ~NAME : movement.h
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Available movement types for the robot
//**********************************************************************
#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#include "datatypes.h"

// ----------------------------------------------------------------
// SpaceType enum
// ----------------------------------------------------------------
typedef enum SpaceType
{
	NO_SPC,
	JOINT_SPC,
	TASK_SPC
} spaceType;

// ----------------------------------------------------------------
// JointEntering enum
// ----------------------------------------------------------------
typedef enum JointEntering
{
	ABSOLUTE_JNT,
	INCRMNTL_JNT
} jointEntering;

// ----------------------------------------------------------------
// MotionFrame enum
// ----------------------------------------------------------------
typedef enum MotionFrame
{
	BASE_FRAME,
	TASK_FRAME
} motionFrame;

// ----------------------------------------------------------------
// Parametric enum
// ----------------------------------------------------------------
typedef enum Parametric
{
	NO_PARA,
	CIRCULAR,
	EIGHT
} parametric;

// ----------------------------------------------------------------
// Sinusoidal enum
// ----------------------------------------------------------------
typedef enum Sinusoidal
{
	DIST_PRD,
	VEL_PRD_AMAX,
	VEL_PRD_PMAX
} sinusoidal;

// ----------------------------------------------------------------
// TaskMotType enum
// ----------------------------------------------------------------
typedef enum TaskMotType
{
	TRN_TSK,
	ROT_TSK,
	ALL_TSK
} taskMotType;

// ----------------------------------------------------------------
// Polynomial enum
// ----------------------------------------------------------------
typedef enum Polynomial
{
	NO_POLY,
	LINEAR,
	CUBIC,
	QUINTIC,
	BANGBANG,
	TRAPEZOIDAL,
	STRAPEZOIDAL
} polynomial;

// ----------------------------------------------------------------
// Trajectory enum
// ----------------------------------------------------------------
typedef enum Trajectory
{
	NO_TRJ,
	PNT2PNT,
	VIAPNTS,
	SINUSOIDAL,
	PARAMETRIC
} trajectory;

// ----------------------------------------------------------------
// motionData structure
// ----------------------------------------------------------------
typedef struct __MotData
{
	double tf;
	rn q_posIni;

	spaceType		trjSpace;
	trajectory		trjType;
	polynomial		p2pType;
	parametric		paraType;
	sinusoidal		snsdlType;
	taskMotType		tskMotType;
	motionFrame		motFrame;
	jointEntering	jntEnterType;
} _MotData;
_MotData	_md;

void _moveMenu(void);

#endif //_MOVEMENT_H_
