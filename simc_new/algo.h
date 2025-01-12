//*********************************************************************
// ~NAME : algo.h
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Implementation of simulation algo
//**********************************************************************
#ifndef _ALGO_H_
#define _ALGO_H_

#include "datatypes.h"

// ----------------------------------------------------------------
// _AlgoData structure
// ----------------------------------------------------------------
typedef struct __AlgoData
{
	int			m_move, no_trj;			// indicate which joint, and in which direction
	double		m_time;					// time of movement (seconds)
	double		m_time_final;			// final time of movement (seconds)

	/*double q_posFbk[6], q_velFbk[6], q_posDes[6], q_velDes[6], q_accDes[6], q_accFbk[6];
	double q_torFbk[6], q_torCmd[6];*/

	rn q_posFbk, q_velFbk, q_posDes, q_velDes, q_accDes, q_accFbk;
	rn q_torFbk, q_torCmd;

	screw x_velFbk;
} _AlgoData;
_AlgoData	_ad;

// ----------------------------------------------------------------
// _WorldData structure
// ----------------------------------------------------------------
typedef struct __WorldData
{
	rn q_pos, q_vel, q_acc;
	se3 x_pos;
	screw x_vel, ff_e;

	rn tau_frc, tau_spr;
} _WorldData;
_WorldData	_wd;

void initAlgo(void);
void computeAlgo(double tcyc, double tstp, double tfin);

#endif //_ALGO_H_
