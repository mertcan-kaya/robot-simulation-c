//*********************************************************************
// ~NAME : kinematics.h
// ~INSTITUTION : ITU
// ~DESCRIPTION : Computing and storing of the kinematic paramaters
//**********************************************************************
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "datatypes.h"

// ----------------------------------------------------------------
// eulerAngles enum
// ----------------------------------------------------------------
typedef enum EulerAngles
{
	ZXZ,
	ZYZ,
	ZYX,
	XYZ
} eulerAngles;

// ----------------------------------------------------------------
// rotationRep enum
// ----------------------------------------------------------------
typedef enum RotationRep
{
	ROT_MAT,
	EULER_ANG,
	ANGLE_AXIS,
	QUATERNION
} rotationRep;

// ----------------------------------------------------------------
// kinematicsData structure
// ----------------------------------------------------------------
typedef struct __KinData
{
	double	q_posLim[N_DOF][2];		// (rad)
	double	q_velLim[N_DOF];		// (rad/s)
	double	q_accLim[N_DOF];		// (rad/s^2)
	double	t_posLim[N_DOF][2];		// (m)

	double	a[N_DOF];				// (m)
	double	alpha[N_DOF];			// (rad)
	double	d[N_DOF];				// (m)
	double	theta[N_DOF];			// (rad)

	double	d_e;						// end-effector length (m)

	double	sensor_mount_ang;
	double  sensor_length, tool_length;

	eulerAngles	eulerSet;
	rotationRep	rotRepresent;

	r3 ph_i[N_DOF], zi_i;
	r3 rh_h_i[N_DOF + 1];
	r3x3 Rh_i[N_DOF];
	r6xn J_e;
	se3 T0_n;
	screw zzi_i;
	st XXi_h[N_DOF], XXR0_n;
} _KinData;
_KinData _kd;

void initKinPara(void);

void getDHparameters(void);
void getEEparameters(void);

se3 getPose(const rn q_pos);
void getFwdKin(const rn q_pos, const rn q_vel);

r3x3 get_Rh_i(r3x3 Rh_i, const unsigned int i, const double q_pos);
r3 get_ph_i(r3 ph_i, const unsigned int i);

rnxn0 compute_regressor(const screw Ji_ij[N_DOF][N_DOF], const screwx10 Psii_i[N_DOF]);

//void getTh_imatrix(mdarray *Th_i, const unsigned int i, const mdarray *q_pos);
//r66 getJ_eMatrix(const double q_pos[6], const double q_vel[6]);

#endif //_KINEMATICS_H_
