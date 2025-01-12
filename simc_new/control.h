//*********************************************************************
// ~NAME : control.h
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Control algorithms
//**********************************************************************
#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdbool.h>

#include "datatypes.h"

// ----------------------------------------------------------------
// ctrlFrame enum
// ----------------------------------------------------------------
typedef enum ControlFrame
{
	BASE_FRAME_CTRL,
	TOOL_FRAME_CTRL
} controlFrame;

// ----------------------------------------------------------------
// passivityImp enum
// ----------------------------------------------------------------
typedef enum PassivityImp
{
	PASS_NON,
	PASS_A,
	PASS_B,
	PASS_C
} passivityImp;

// ----------------------------------------------------------------
// controlMethod enum
// ----------------------------------------------------------------
typedef enum ControlMethod
{
	MOTION_CTRL,
	FORCE_CTRL,
	HYBRID_PF_CTRL,
	IMPEDANCE_CTRL,
	UNIFIED_CTRL,
	PASSIVITY_CTRL
} controlMethod;

// ----------------------------------------------------------------
// controlSpace enum
// ----------------------------------------------------------------
typedef enum ControlSpace
{
	JNTSPC_CTRL,
	TSKSPC_CTRL,
	MIXSPC_CTRL
} controlSpace;

// ----------------------------------------------------------------
// controlType enum
// ----------------------------------------------------------------
typedef enum ControlType
{
	PID,
	CTC
} controlType;

// ----------------------------------------------------------------
// _ControlData structure
// ----------------------------------------------------------------
typedef struct __ControlData
{
	//double K_pq_diag[6], K_iq_diag[6], K_dq_diag[6];
	rn K_pq_diag, K_iq_diag, K_dq_diag;

	//double e_q[6], e_qsum[6], e_qdot[6];
	rn e_q, e_qsum, e_qdot;

	//double tau_cmd[6], tau_ctrl[6];
	rn tau_cmd, tau_ctrl;

	bool comp_frc, comp_spr, comp_grv, comp_cor;

	controlMethod	ctrlMthd;
	controlSpace	ctrlSpc;
	controlType		ctrlType;
	controlFrame	ctrlFrame;
	passivityImp	passImp;
} _ControlData;
_ControlData _cd;

void initCtrlPara(void);

rn joint_pid(const rn K_pq_diag, const rn K_iq_diag, const rn K_dq_diag, const rn e_q, const rn e_qsum, const rn e_qdot);
rn mnea(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, const r10 p[N_DOF]);
rn anea(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF]);

rn mtrx_func_new(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF]);
rn mtrx_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF]);
rn kwsk_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF]);
rn yuan_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF]);
rn mnea_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF]);

#endif //_CONTROL_H_
