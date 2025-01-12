//*********************************************************************
// ~NAME : kinematics.c
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Computing and storing of the kinematic paramaters
//**********************************************************************
// RX160/RX160L
//-------------------------------DH Table------------------------------//
// i	a_i(mm)		Alpha_i(rad)		  d_i(mm)		Theta_i(rad)
// 1		  0				   0			  550			 q1 
// 2		150		       -pi/2			   d2			 q2-pi/2 
// 3		825		           0			    0			 q3+pi/2 
// 4		  0		        pi/2		  625/925			 q4 
// 5		  0		       -pi/2			    0			 q5 
// 6		  0		        pi/2			  110			 q6 
//---------------------------------------------------------------------//
// Th_i = 
// [          c(thetai)          -s(thetai)          0            ai]
// [s(thetai)*c(alphai) c(thetai)*c(alphai) -s(alphai) -s(alphai)*di]
// [s(thetai)*s(alphai) c(thetai)*s(alphai)  c(alphai)  c(alphai)*di]
// [                  0                   0          0             1]
//---------------------------------------------------------------------//

#include <math.h>
#include <stdio.h>

#include "kinematics.h"
#include "test.h"
#include "functions.h"
#include "datatypes.h"
#include "algo.h"

// ----------------------------------------------------------------
// initKinPara
// ----------------------------------------------------------------
void initKinPara(void)
{
	// joint position limit +/-
	if (_rd.robot_model == TX90)
	{
		// joint pos limits (deg to rad)
		_kd.q_posLim[0][0] = -180.0 * _DEG_TO_RAD;
		_kd.q_posLim[1][0] = -130.0 * _DEG_TO_RAD;
		_kd.q_posLim[2][0] = -145.0 * _DEG_TO_RAD;
		_kd.q_posLim[3][0] = -270.0 * _DEG_TO_RAD;
		_kd.q_posLim[4][0] = -115.0 * _DEG_TO_RAD;
		_kd.q_posLim[5][0] = -270.0 * _DEG_TO_RAD;

		_kd.q_posLim[0][1] = 180.0 * _DEG_TO_RAD;
		_kd.q_posLim[1][1] = 147.5 * _DEG_TO_RAD;
		_kd.q_posLim[2][1] = 145.0 * _DEG_TO_RAD;
		_kd.q_posLim[3][1] = 270.0 * _DEG_TO_RAD;
		_kd.q_posLim[4][1] = 140.0 * _DEG_TO_RAD;
		_kd.q_posLim[5][1] = 270.0 * _DEG_TO_RAD;

		// task space limits (m)
		_kd.t_posLim[0][0] = 1.000;
		_kd.t_posLim[1][0] = 1.000;
		_kd.t_posLim[2][0] = 1.428;

		_kd.t_posLim[0][1] = -0.900;
		_kd.t_posLim[1][1] = -1.000;
		_kd.t_posLim[2][1] = -0.600;

		// joint abs vel limits (rad/s)
		_kd.q_velLim[0] = 400 * _DEG_TO_RAD;
		_kd.q_velLim[1] = 400 * _DEG_TO_RAD;
		_kd.q_velLim[2] = 430 * _DEG_TO_RAD;
		_kd.q_velLim[3] = 540 * _DEG_TO_RAD;
		_kd.q_velLim[4] = 475 * _DEG_TO_RAD;
		_kd.q_velLim[5] = 760 * _DEG_TO_RAD;
	}
	else
	{
		// joint pos limits (deg to rad)
		_kd.q_posLim[0][0] = -160.0 * _DEG_TO_RAD;
		_kd.q_posLim[1][0] = -137.5 * _DEG_TO_RAD;
		_kd.q_posLim[2][0] = -150.0 * _DEG_TO_RAD;
		_kd.q_posLim[3][0] = -270.0 * _DEG_TO_RAD;
		_kd.q_posLim[4][0] = -105.0 * _DEG_TO_RAD;
		_kd.q_posLim[5][0] = -270.0 * _DEG_TO_RAD;

		_kd.q_posLim[0][1] = 160.0 * _DEG_TO_RAD;
		_kd.q_posLim[1][1] = 137.5 * _DEG_TO_RAD;
		_kd.q_posLim[2][1] = 150.0 * _DEG_TO_RAD;
		_kd.q_posLim[3][1] = 270.0 * _DEG_TO_RAD;
		_kd.q_posLim[4][1] = 120.0 * _DEG_TO_RAD;
		_kd.q_posLim[5][1] = 270.0 * _DEG_TO_RAD;

		// joint abs vel limits (rad/s)
		_kd.q_velLim[0] = 200 * _DEG_TO_RAD;
		_kd.q_velLim[1] = 200 * _DEG_TO_RAD;
		_kd.q_velLim[2] = 255 * _DEG_TO_RAD;
		_kd.q_velLim[3] = 315 * _DEG_TO_RAD;
		_kd.q_velLim[4] = 360 * _DEG_TO_RAD;
		_kd.q_velLim[5] = 870 * _DEG_TO_RAD;

		// task space limits (m)
		if (_rd.robot_model == RX160)
		{
			_kd.t_posLim[0][0] = 1.710;
			_kd.t_posLim[1][0] = 1.710;
			_kd.t_posLim[2][0] = 2.110;

			_kd.t_posLim[0][1] = -1.410;
			_kd.t_posLim[1][1] = -1.710;
			_kd.t_posLim[2][1] = -0.600;
		}
		else
		{
			_kd.t_posLim[0][0] = 2.010;
			_kd.t_posLim[1][0] = 2.010;
			_kd.t_posLim[2][0] = 2.410;

			_kd.t_posLim[0][1] = -1.710;
			_kd.t_posLim[1][1] = -2.010;
			_kd.t_posLim[2][1] = -0.600;
		}
	}

	// joint abs acc limits (rad/s^2)
	_kd.q_accLim[0] = 7.9000;
	_kd.q_accLim[1] = 6.5000;
	_kd.q_accLim[2] = 10.5000;
	_kd.q_accLim[3] = 25.2000;
	_kd.q_accLim[4] = 19.6000;
	_kd.q_accLim[5] = 41.7000;

	getEEparameters();
	getDHparameters();

	_kd.zi_i = set_r3(0.0, 0.0, 1.0);
	_kd.zzi_i = set_screw(get_r3_zero(), _kd.zi_i);

	for (int i = 0; i < N_DOF; i++)
		_kd.rh_h_i[i] = get_ph_i(_kd.rh_h_i[i], i);
	_kd.rh_h_i[N_DOF] = get_r3_zero();

}

// ----------------------------------------------------------------
// getEEmatrix
// ----------------------------------------------------------------
void getEEparameters(void)
{
	// f/t sensor parameters
	if (_rd.ee_attachments == NO_EE)
	{
		_kd.sensor_length = 0.0; // m
		_kd.sensor_mount_ang = 0;
	}
	else
	{
		_kd.sensor_length = 0.0333; // m
		if ((_rd.robot_model == RX160) || (_rd.robot_model == TX90))
			_kd.sensor_mount_ang = 45;
		else
			_kd.sensor_mount_ang = 0;
	}

	if (_rd.ee_attachments == GRIPPER)
	{
		_kd.tool_length = 0.144;
	}
	else if (_rd.ee_attachments == BALL_SPRING)
	{
		_kd.tool_length = 0.18;
	}
	else
	{
		_kd.tool_length = 0.0;
	}

	/*mdarray_set_identity(_kd.R6_E);
	mdarray1_set3(_kd.p6_E, 0, 0, _kd.sensor_length + _kd.tool_length);*/
}

// ----------------------------------------------------------------
// getDHmatrix
// ----------------------------------------------------------------
void getDHparameters(void)
{
	_kd.a[0] = 0.0;
	if (_rd.robot_model == TX90)
	{	// TX family
		_kd.a[1] = 0.050;
		_kd.a[2] = 0.425;
	}
	else
	{	// RX family
		_kd.a[1] = 0.150;
		_kd.a[2] = 0.825;
	}
	_kd.a[3] = 0.0;
	_kd.a[4] = 0.0;
	_kd.a[5] = 0.0;

	_kd.alpha[0] = 0.0;
	_kd.alpha[1] = -(PI / 2.0);
	_kd.alpha[2] = 0.0;
	_kd.alpha[3] = (PI / 2.0);
	_kd.alpha[4] = -(PI / 2.0);
	_kd.alpha[5] = (PI / 2.0);

	// DH parameters (m, rad)
	if (_rd.robot_model == TX90)
	{	// TX family
		_kd.d[0] = 0.478;
		_kd.d[1] = 0.050;
		_kd.d[2] = 0.0;
		_kd.d[3] = 0.425;
		_kd.d[4] = 0.0;
		_kd.d[5] = 0.100;
	}
	else
	{	// RX family
		_kd.d[0] = 0.550;
		_kd.d[1] = 0.0;
		_kd.d[2] = 0.0;
		if (_rd.robot_model == RX160)
			_kd.d[3] = 0.625;
		else
			_kd.d[3] = 0.925;
		_kd.d[4] = 0.0;
		_kd.d[5] = 0.110;
	}
	if (_rd.ee_attachments != NO_EE)
		_kd.d_e = _kd.sensor_length + _kd.tool_length;
	else
		_kd.d_e = 0.0;

	_kd.theta[0] = 0.0;
	_kd.theta[1] = -(PI / 2.0);
	_kd.theta[2] = (PI / 2.0);
	_kd.theta[3] = 0.0;
	_kd.theta[4] = 0.0;
	_kd.theta[5] = 0.0;
}

r3x3 get_Rh_i(r3x3 Rh_i, const unsigned int i, const double q_pos)
{
	double CA_h, SA_h, CT_h, ST_h;

	CA_h = cos(_kd.alpha[i]);
	SA_h = sin(_kd.alpha[i]);
	CT_h = cos(_kd.theta[i] + q_pos);
	ST_h = sin(_kd.theta[i] + q_pos);

	Rh_i.data[X_AXIS][X_AXIS] = CT_h		; Rh_i.data[X_AXIS][Y_AXIS] = -ST_h			; Rh_i.data[X_AXIS][Z_AXIS] = 0.0;
	Rh_i.data[Y_AXIS][X_AXIS] = CA_h * ST_h	; Rh_i.data[Y_AXIS][Y_AXIS] = CA_h * CT_h	; Rh_i.data[Y_AXIS][Z_AXIS] = -SA_h;
	Rh_i.data[Z_AXIS][X_AXIS] = SA_h * ST_h	; Rh_i.data[Z_AXIS][Y_AXIS] = SA_h * CT_h	; Rh_i.data[Z_AXIS][Z_AXIS] = CA_h;

	return Rh_i;
}

r3 get_ph_i(r3 ph_i, const unsigned int i)
{
	ph_i.data[X_AXIS] = _kd.a[i];
	ph_i.data[Y_AXIS] = -_kd.d[i] * sin(_kd.alpha[i]);
	ph_i.data[Z_AXIS] = _kd.d[i] * cos(_kd.alpha[i]);

	return ph_i;
}

se3 get_T0_n(se3 T0_n, const r3x3 Rh_i, const r3 ph_i)
{
	T0_n.trn = add_r3(mul_r3x3_and_r3(T0_n.rot, ph_i), T0_n.trn);
	T0_n.rot = mul_r3x3(T0_n.rot, Rh_i);

	return T0_n;
}

se3 getPose(const rn q_pos)
{
	int i;
	r3 ph_i = { 0.0 };
	r3x3 Rh_i = { 0.0 };
	se3 x_pos = { 0.0 };

	x_pos.rot = get_r3x3_idnt();
	x_pos.trn = get_r3_zero();

	for (i = 0; i < 6; i++)
	{
		Rh_i = get_Rh_i(Rh_i, i, q_pos.data[i]);
		ph_i = get_ph_i(ph_i, i);
		x_pos = get_T0_n(x_pos, Rh_i, ph_i);
	}

	return x_pos;
}

// ----------------------------------------------------------------
// getFwdKin
// ----------------------------------------------------------------
void getFwdKin(const rn q_pos, const rn q_vel)
{
	unsigned int i;
	screw vvh_h = { 0.0 };

	_kd.T0_n.rot = get_r3x3_idnt();
	_kd.T0_n.trn = get_r3_zero();

	for (i = 0; i < 6; i++)
	{
		_kd.Rh_i[i] = get_Rh_i(_kd.Rh_i[i], i, q_pos.data[i]);
		_kd.ph_i[i] = get_ph_i(_kd.ph_i[i], i);
		_kd.XXi_h[i] = set_tr_wrench_st_from_se(_kd.Rh_i[i], _kd.ph_i[i]);
		_kd.T0_n = get_T0_n(_kd.T0_n, _kd.Rh_i[i], _kd.ph_i[i]);
		vvh_h = add_screw(mul_st_and_screw(_kd.XXi_h[i], vvh_h), mul_double_and_screw(q_vel.data[i], _kd.zzi_i));
	}
	_kd.XXR0_n = set_st(_kd.T0_n.rot, get_r3x3_zero(), get_r3x3_zero(), _kd.T0_n.rot);
	_ad.x_velFbk = mul_st_and_screw(_kd.XXR0_n, vvh_h);
}

rnxn0 compute_regressor(const screw Ji_ij[N_DOF][N_DOF], const screwx10 Psii_i[N_DOF])
{
	r10 yT_ij[N_DOF][N_DOF] = { 0.0 };
	rnxn0 Y = { 0.0 };
	int i, j;

	for (i = 0; i < 6; i++)
	{
		for (j = i; j < 6; j++)
			yT_ij[j][i] = mul_transpose_screwx10_and_screw(Psii_i[j], Ji_ij[i][j]);
	}

	Y = set_rnxn0_from_r10s(yT_ij);

	return Y;
}

//// ----------------------------------------------------------------
//// get Jacobian matrix for end-effector w.r.t. base frame
//// ----------------------------------------------------------------
//r66 getJ_eMatrix(const double q_pos[6])
//{
//	r66 J_e = { 0.0 };
//	double a2, a3, d2, d4, d6;
//	double C1, C2, C3, C4, C5, S1, S2, S3, S4, S5, C2p3, S2p3;
//
//	a2 = _kd.a[1];
//	a3 = _kd.a[2];
//	d2 = _kd.d[1];
//	d4 = _kd.d[3];
//	d6 = _kd.d[5];
//
//	C1 = cos(q_pos[0]);
//	C2 = cos(q_pos[1]);
//	C3 = cos(q_pos[2]);
//	C4 = cos(q_pos[3]);
//	C5 = cos(q_pos[4]);
//
//	S1 = sin(q_pos[0]);
//	S2 = sin(q_pos[1]);
//	S3 = sin(q_pos[2]);
//	S4 = sin(q_pos[3]);
//	S5 = sin(q_pos[4]);
//
//	C2p3 = cos(q_pos[1] + q_pos[2]);
//	S2p3 = sin(q_pos[1] + q_pos[2]);
//
//	J_e = set_r66_col(J_e, 0	, -S1 * (a2 + a3 * S2 + (d4 + d6 * C5) * S2p3 + d6 * C2p3 * C4 * S5) - C1 * (d2 + d6 * S4 * S5)
//								, C1 * (a2 + a3 * S2 + (d4 + d6 * C5) * S2p3 + d6 * C2p3 * C4 * S5) - S1 * (d2 + d6 * S4 * S5)
//								, 0.0
//								, 0.0
//								, 0.0
//								, 1.0);
//
//	J_e = set_r66_col(J_e, 1	, C1 * (a3 * C2 + C2p3 * (d4 + d6 * C5) - d6 * C4 * S2p3 * S5)
//								, S1 * (a3 * C2 + C2p3 * (d4 + d6 * C5) - d6 * C4 * S2p3 * S5)
//								, -C2 * ((d4 + d6 * C5) * S3 + d6 * C3 * C4 * S5) - S2 * (a3 + C3 * (d4 + d6 * C5) - d6 * C4 * S3 * S5)
//								, -S1
//								, C1
//								, 0.0);
//
//	J_e = set_r66_col(J_e, 2	, C1 * (C2p3 * (d4 + d6 * C5) - d6 * C4 * S2p3 * S5)
//								, S1 * (C2p3 * (d4 + d6 * C5) - d6 * C4 * S2p3 * S5)
//								, -(d4 + d6 * C5) * S2p3 - d6 * C2p3 * C4 * S5
//								, -S1
//								, C1
//								, 0.0);
//
//	J_e = set_r66_col(J_e, 3	, -d6 * (C4 * S1 + C1 * C2p3 * S4) * S5
//								, d6 * (C1 * C4 - C2p3 * S1 * S4) * S5
//								, d6 * S2p3 * S4 * S5
//								, C1 * S2p3
//								, S1 * S2p3
//								, C2p3);
//
//	J_e = set_r66_col(J_e, 4	, -d6 * (C5 * S1 * S4 + C1 * (-C2p3 * C4 * C5 + S2p3 * S5))
//								, d6 * (-C4 * C5 * S1 * S2 * S3 + C1 * C5 * S4 - C3 * S1 * S2 * S5 + C2 * S1 * (C3 * C4 * C5 - S3 * S5))
//								, -d6 * (C4 * C5 * S2p3 + C2p3 * S5)
//								, -C4 * S1 - C1 * C2p3 * S4
//								, C1 * C4 - C2p3 * S1 * S4
//								, S2p3 * S4);
//
//	J_e = set_r66_col(J_e, 5	, 0.0
//								, 0.0
//								, 0.0
//								, -S1 * S4 * S5 + C1 * (C5 * S2p3 + C2p3 * C4 * S5)
//								, C5 * S1 * S2p3 + (C2p3 * C4 * S1 + C1 * S4) * S5
//								, C2p3 * C5 - C4 * S2p3 * S5);
//
//	return J_e;
//}
