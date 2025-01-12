//*********************************************************************
// ~NAME : dynamics.h
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Dynamic properties
//**********************************************************************
#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include "datatypes.h"

// ----------------------------------------------------------------
// dynamicsData structure
// ----------------------------------------------------------------
typedef struct __DynData
{
	r1 m_i[N_DOF];
	r3 ri_i_ci[N_DOF], di_i[N_DOF];
	r3x3 Ii_ci[N_DOF], Ii_i[N_DOF];

	rn tau_frc, tau_spr;

	double tool_mass;
	r3 tool_CoM_6F;
} _DynData;
_DynData _dd;

rn getFrcTorque(const rn q_vel);
rn getSprTorque(const rn q_pos);
void initDynPara(void);
//void drcDynNE(double q_acc[6], const double tau[6], const double q_pos[6], const double q_vel[6]);

#endif //_DYNAMICS_H_
