//*********************************************************************
// ~NAME : test.h
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION :
//**********************************************************************
#ifndef _TEST_H_
#define _TEST_H_

// constants
#define MAX_NB_JNT			6
#define MAX_NB_DOF			6
#define TRN_NB_DOF			3
#define ROT_NB_DOF			3
#define PI					3.1415926535897932384626433832795

#define _RAD_TO_DEG			(180.0 / PI)
#define _DEG_TO_RAD			(PI / 180.0)
#define _M_TO_MM			(1000.0)
#define _MM_TO_M			(0.001)

#define S_SMALL_FLOAT		(1.e-5)
#define S_VERY_SMALL_FLOAT	(1.e-10)
#define S_BIG_FLOAT			(1.e10)

#define GRAV				(9.81)

// ----------------------------------------------------------------
// robotType
// ----------------------------------------------------------------
typedef enum RobotModel
{
	TX90,
	RX160,
	RX160L
} robotModel;

// ----------------------------------------------------------------
// eeAttachments
// ----------------------------------------------------------------
typedef enum EeAttachments
{
	NO_EE,
	SENSOR,
	GRIPPER,
	BALL_SPRING
} eeAttachments;

// ----------------------------------------------------------------
// enableState
// ----------------------------------------------------------------
typedef enum EnableState
{
	DISABLED,
	ENABLED
} enableState;

// ----------------------------------------------------------------
// _RobotData structure
// ----------------------------------------------------------------
typedef struct __RobotData
{
	robotModel		robot_model;
	eeAttachments	ee_attachments;
	enableState		enable_state;
} _RobotData;
_RobotData	_rd;

// ----------------------------------------------------------------
// _SimData structure
// ----------------------------------------------------------------
typedef struct __SimData
{
	double cycle_time;
	double sample_time;
	double stop_time;
} _SimData;
_SimData	_sd;

//unsigned int	g_jntNb;
double			g_cycleTime;

#endif //_TEST_H_
