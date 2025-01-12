//*********************************************************************
// ~NAME : test.c
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Main file
//**********************************************************************

#include <stdio.h>

#include "test.h"
#include "algo.h"
#include "functions.h"
#include "movement.h"
#include "datatypes.h"
#include "kinematics.h"
#include "dynamics.h"
#include "control.h"
#include "printdata.h"

// ----------------------------------------------------------------
// initSimPara
// ----------------------------------------------------------------
void initSimPara(void)
{
	_sd.cycle_time = 0.004;
	_sd.sample_time = 0.0001;
	_sd.stop_time = 5;
}

// ----------------------------------------------------------------
// initConfig
// ----------------------------------------------------------------
void initConfig(void)
{
	_md.q_posIni.data[0] = _DEG_TO_RAD * -30;
	_md.q_posIni.data[1] = _DEG_TO_RAD * 60;
	_md.q_posIni.data[2] = _DEG_TO_RAD * 12;
	_md.q_posIni.data[3] = _DEG_TO_RAD * 10;
	_md.q_posIni.data[4] = _DEG_TO_RAD * 20;
	_md.q_posIni.data[5] = _DEG_TO_RAD * -35;
}

// ----------------------------------------------------------------
// _test
// ----------------------------------------------------------------
static void _test(void)
{
	int i;
	//screw deneme1 = { 0.0 }, deneme2 = { 0.0 }, deneme3 = { 0.0 }, temp1 = { 0.0 }, temp2 = { 0.0 }, temp3 = { 0.0 };
	//r3x3 Rh_i[6] = { 0.0 };
	//r3 ph_i[6] = { 0.0 }, ci_i = { 0.0 }, zi_i = { 0.0, 0.0, 1.0 };
	//screw zzi_i[6] = { 0.0 }, vvh_h = { 0.0 };
	//s66 XXh_i[6] = { 0.0 };
	rn q_pos = { 0.0 }, q_vel = { 0.0 }, qR_vel = { 0.0 }, qR_acc = { 0.0 }, s = { 0.0 };
	r3 g = { 0.0, 0.0, -9.81 };
	r10 p1[N_DOF] = { 0.0 }, p2[N_DOF] = { 0.0 }, p3[N_DOF] = { 0.0 }, p4[N_DOF] = { 0.0 }, p5[N_DOF] = { 0.0 }, p_new[N_DOF] = { 0.0 }, w[N_DOF] = { 0.0 }, Pdiag[N_DOF] = { 0.0 };
	rn0 p_bar = { 0.0 }, p_bar_new = { 0.0 }, sigma_bar = { 0.0 }, Pdiag_bar = { 0.0 };
	rnxn0 Y = { 0.0 };
	rn tau = { 0.0 }, tau_f = { 0.0 }, tau_fyuan = { 0.0 }, tau_fkwsk = { 0.0 }, tau_fmtrx = { 0.0 }, Y_j[N_DOF * 10] = { 0.0 };

	_cd.passImp = PASS_A;

	q_pos.data[0] = 26.845443 * _DEG_TO_RAD;
	q_pos.data[1] = 16.520273 * _DEG_TO_RAD;
	q_pos.data[2] = 49.560818 * _DEG_TO_RAD;
	q_pos.data[3] = -12.390205 * _DEG_TO_RAD;
	q_pos.data[4] = -24.780409 * _DEG_TO_RAD;
	q_pos.data[5] = 18.585307 * _DEG_TO_RAD;

	q_vel.data[0] = 6.786452 * _DEG_TO_RAD;
	q_vel.data[1] = 4.176278 * _DEG_TO_RAD;
	q_vel.data[2] = 12.528834 * _DEG_TO_RAD;
	q_vel.data[3] = -3.132209 * _DEG_TO_RAD;
	q_vel.data[4] = -6.264417 * _DEG_TO_RAD;
	q_vel.data[5] = 4.698313 * _DEG_TO_RAD;

	qR_vel.data[0] = 5.786452 * _DEG_TO_RAD;
	qR_vel.data[1] = 5.176278 * _DEG_TO_RAD;
	qR_vel.data[2] = 8.528834 * _DEG_TO_RAD;
	qR_vel.data[3] = -5.132209 * _DEG_TO_RAD;
	qR_vel.data[4] = -8.264417 * _DEG_TO_RAD;
	qR_vel.data[5] = 3.698313 * _DEG_TO_RAD;

	qR_acc.data[0] = 0.289666 * _DEG_TO_RAD;
	qR_acc.data[1] = 0.178256 * _DEG_TO_RAD;
	qR_acc.data[2] = 0.534767 * _DEG_TO_RAD;
	qR_acc.data[3] = -0.133692 * _DEG_TO_RAD;
	qR_acc.data[4] = -0.267384 * _DEG_TO_RAD;
	qR_acc.data[5] = 0.200538 * _DEG_TO_RAD;

	//s = sub_rn(q_vel, qR_vel);

	for (i = 0; i < N_DOF; i++)
	{
		p1[i] = compose_r10(_dd.m_i[i], _dd.di_i[i], uniq_vec(_dd.Ii_i[i]));
		p2[i] = p1[i];
		p3[i] = p1[i];
		p4[i] = p1[i];
		p5[i] = p1[i];
	}

	Pdiag[0] = set_r10(0.008147, 0.009058, 0.001270, 0.009134, 0.006324, 0.000975, 0.002785, 0.005469, 0.009575, 0.009649);
	Pdiag[1] = set_r10(0.001576, 0.009706, 0.009572, 0.004854, 0.008003, 0.001419, 0.004218, 0.009157, 0.007922, 0.009595);
	Pdiag[2] = set_r10(0.006557, 0.000357, 0.008491, 0.009340, 0.006787, 0.007577, 0.007431, 0.003922, 0.006555, 0.001712);
	Pdiag[3] = set_r10(0.007060, 0.000318, 0.002769, 0.000462, 0.000971, 0.008235, 0.006948, 0.003171, 0.009502, 0.000344);
	Pdiag[4] = set_r10(0.004387, 0.003816, 0.007655, 0.007952, 0.001869, 0.004898, 0.004456, 0.006463, 0.007094, 0.007547);
	Pdiag[5] = set_r10(0.002760, 0.006797, 0.006551, 0.001626, 0.001190, 0.004984, 0.009597, 0.003404, 0.005853, 0.002238);

	printf("g : %f, %f, %f\n", g.data[0], g.data[1], g.data[2]);

	printf("\nm1:");
	printf("\n%f", _dd.m_i[0].data);
	printf("\n");

	/*printf("\nd1_1:");
	printf("\n%f", _dd.di_i[0].data[0]);
	printf("\n%f", _dd.di_i[0].data[1]);
	printf("\n%f", _dd.di_i[0].data[2]);
	printf("\n");*/

	printf("\nI1_1:");
	printf("\n%f %f %f", _dd.Ii_i[0].data[0][0], _dd.Ii_i[0].data[0][1], _dd.Ii_i[0].data[0][2]);
	printf("\n%f %f %f", _dd.Ii_i[0].data[1][0], _dd.Ii_i[0].data[1][1], _dd.Ii_i[0].data[1][2]);
	printf("\n%f %f %f", _dd.Ii_i[0].data[2][0], _dd.Ii_i[0].data[2][1], _dd.Ii_i[0].data[2][2]);
	printf("\n");

	printf("\np1:");
	printf("\n%f", p1[0].data[0]);
	printf("\n%f", p1[0].data[1]);
	printf("\n%f", p1[0].data[2]);
	printf("\n%f", p1[0].data[3]);
	printf("\n%f", p1[0].data[4]);
	printf("\n%f", p1[0].data[5]);
	printf("\n%f", p1[0].data[6]);
	printf("\n%f", p1[0].data[7]);
	printf("\n%f", p1[0].data[8]);
	printf("\n%f", p1[0].data[9]);
	printf("\n");

	tau = mnea(q_pos, q_vel, qR_vel, qR_acc, g, p1);

	printf("\ntau (mnea):");
	printf("\n%*.*f", 18, 13, tau.data[0]);
	printf("\n%*.*f", 18, 13, tau.data[1]);
	printf("\n%*.*f", 18, 13, tau.data[2]);
	printf("\n%*.*f", 18, 13, tau.data[3]);
	printf("\n%*.*f", 18, 13, tau.data[4]);
	printf("\n%*.*f", 18, 13, tau.data[5]);

	tau_f = mnea_func(q_pos, q_vel, qR_vel, qR_acc, g, p1, Pdiag);

	printf("\ntau_f (mnea full):");
	printf("\n%18.13f", tau_f.data[0]);
	printf("\n%18.13f", tau_f.data[1]);
	printf("\n%18.13f", tau_f.data[2]);
	printf("\n%18.13f", tau_f.data[3]);
	printf("\n%18.13f", tau_f.data[4]);
	printf("\n%18.13f", tau_f.data[5]);

	tau_f = anea(q_pos, q_vel, qR_vel, qR_acc, g, p2, Pdiag);

	printf("\ntau_f (anea):");
	printf("\n%18.13f", tau_f.data[0]);
	printf("\n%18.13f", tau_f.data[1]);
	printf("\n%18.13f", tau_f.data[2]);
	printf("\n%18.13f", tau_f.data[3]);
	printf("\n%18.13f", tau_f.data[4]);
	printf("\n%18.13f", tau_f.data[5]);

	tau_fyuan = yuan_func(q_pos, q_vel, qR_vel, qR_acc, g, p3, Pdiag);

	printf("\ntau_f (yuan):");
	printf("\n%18.13f", tau_fyuan.data[0]);
	printf("\n%18.13f", tau_fyuan.data[1]);
	printf("\n%18.13f", tau_fyuan.data[2]);
	printf("\n%18.13f", tau_fyuan.data[3]);
	printf("\n%18.13f", tau_fyuan.data[4]);
	printf("\n%18.13f", tau_fyuan.data[5]);

	tau_fkwsk = kwsk_func(q_pos, q_vel, qR_vel, qR_acc, g, p4, Pdiag);

	printf("\ntau_f (kwsk):");
	printf("\n%18.13f", tau_fkwsk.data[0]);
	printf("\n%18.13f", tau_fkwsk.data[1]);
	printf("\n%18.13f", tau_fkwsk.data[2]);
	printf("\n%18.13f", tau_fkwsk.data[3]);
	printf("\n%18.13f", tau_fkwsk.data[4]);
	printf("\n%18.13f", tau_fkwsk.data[5]);

	tau_fmtrx = mtrx_func(q_pos, q_vel, qR_vel, qR_acc, g, p5, Pdiag);

	printf("\ntau_f (mtrx):");
	printf("\n%18.13f", tau_fmtrx.data[0]);
	printf("\n%18.13f", tau_fmtrx.data[1]);
	printf("\n%18.13f", tau_fmtrx.data[2]);
	printf("\n%18.13f", tau_fmtrx.data[3]);
	printf("\n%18.13f", tau_fmtrx.data[4]);
	printf("\n%18.13f", tau_fmtrx.data[5]);

	tau_fmtrx = mtrx_func_new(q_pos, q_vel, qR_vel, qR_acc, g, p5, Pdiag);

	//printf("\np_anea1:");
	//printf("\n%f", p2[1].data[0]);
	//printf("\n%f", p2[1].data[1]);
	//printf("\n%f", p2[1].data[2]);
	//printf("\n%f", p2[1].data[3]);
	//printf("\n%f", p2[1].data[4]);
	//printf("\n%f", p2[1].data[5]);
	//printf("\n%f", p2[1].data[6]);
	//printf("\n%f", p2[1].data[7]);
	//printf("\n%f", p2[1].data[8]);
	//printf("\n%f", p2[1].data[9]);
	//printf("\n");

	//printf("\np_anea2:");
	//printf("\n%f", p2[2].data[0]);
	//printf("\n%f", p2[2].data[1]);
	//printf("\n%f", p2[2].data[2]);
	//printf("\n%f", p2[2].data[3]);
	//printf("\n%f", p2[2].data[4]);
	//printf("\n%f", p2[2].data[5]);
	//printf("\n%f", p2[2].data[6]);
	//printf("\n%f", p2[2].data[7]);
	//printf("\n%f", p2[2].data[8]);
	//printf("\n%f", p2[2].data[9]);
	//printf("\n");

}

// ----------------------------------------------------------------
// _test
// ----------------------------------------------------------------
//static void _test(void)
//{
//	int i;
//	screw deneme1 = { 0.0 }, deneme2 = { 0.0 }, deneme3 = { 0.0 }, temp1 = { 0.0 }, temp2 = { 0.0 }, temp3 = { 0.0 };
//	r3x3 Rh_i[6] = { 0.0 };
//	r3 ph_i[6] = { 0.0 }, ci_i = { 0.0 }, zi_i = { 0.0, 0.0, 1.0 };
//	screw zzi_i[6] = { 0.0 }, vvh_h = { 0.0 };
//	//s66 XXh_i[6] = { 0.0 };
//	rn q_pos = { 0.0 }, q_vel = { 0.0 };
//
//	deneme1 = set_screw(set_r3(1.1, 2.6, -0.4),set_r3(12.3, 11, 0.8));
//	deneme2 = set_screw(set_r3(12.3, 11, 0.8),set_r3(1.1, 2.6, -0.4));
//
//	printf("deneme1 : %f, %f, %f\n\t%f, %f, %f\n",deneme1.lin.data[0], deneme1.lin.data[1], deneme1.lin.data[2], deneme1.ang.data[0], deneme1.ang.data[1], deneme1.ang.data[2]);
//	printf("deneme2 : %f, %f, %f\n\t%f, %f, %f\n", deneme2.lin.data[0], deneme2.lin.data[1], deneme2.lin.data[2], deneme2.ang.data[0], deneme2.ang.data[1], deneme2.ang.data[2]);
//
//	deneme3 = add_screw(deneme1, deneme2);
//
//	printf("deneme3 : %f, %f, %f\n\t%f, %f, %f\n", deneme3.lin.data[0], deneme3.lin.data[1], deneme3.lin.data[2], deneme3.ang.data[0], deneme3.ang.data[1], deneme3.ang.data[2]);
//
//	deneme1 = set_screw_lin(deneme1, 2.4, -3.2, -1.8);
//
//	printf("deneme1 : %f, %f, %f\n\t%f, %f, %f\n", deneme1.lin.data[0], deneme1.lin.data[1], deneme1.lin.data[2], deneme1.ang.data[0], deneme1.ang.data[1], deneme1.ang.data[2]);
//
//	for (i = 0; i < 6; i++)
//	{
//		Rh_i[i] = get_Rh_i(Rh_i[i], i, _md.q_posIni.data[i]);
//		printf("Rh_i : %f, %f, %f\n\t%f, %f, %f\n\t%f, %f, %f\n", Rh_i[i].data[0][0], Rh_i[i].data[0][1], Rh_i[i].data[0][2], Rh_i[i].data[1][0], Rh_i[i].data[1][1], Rh_i[i].data[1][2], Rh_i[i].data[2][0], Rh_i[i].data[2][1], Rh_i[i].data[2][2]);
//
//		ph_i[i] = get_ph_i(ph_i[i], i);
//		printf("ph_i : %f, %f, %f\n", ph_i[i].data[0], ph_i[i].data[1], ph_i[i].data[2]);
//	}
//
//	for (i = 0; i < 6; i++)
//	{
//		ci_i = mul_double_and_r3(_dd.m_i[i], _dd.ri_i_ci[i]);
//		printf("ci_i%d : %f, %f, %f\n", i, ci_i.data[0], ci_i.data[1], ci_i.data[2]);
//	}
//	for (i = 0; i < 6; i++)
//	{
//		zzi_i[i].ang = zi_i;
//		printf("zzi_i %d : %f, %f, %f\n\t%f, %f, %f\n", i, zzi_i[i].lin.data[0], zzi_i[i].lin.data[1], zzi_i[i].lin.data[2], zzi_i[i].ang.data[0], zzi_i[i].ang.data[1], zzi_i[i].ang.data[2]);
//	}
//	/*for (i = 0; i < 6; i++)
//	{
//		XXh_i[i] = set_s66(Rh_i[i], ph_i[i]);
//	}
//	for (i = 0; i < 6; i++)
//	{
//		temp1 = mul_s66inv_and_screw(XXh_i[i], vvh_h);
//		printf("temp1 %d : %f, %f, %f\n\t%f, %f, %f\n", i, temp1.lin.data[0], temp1.lin.data[1], temp1.lin.data[2], temp1.ang.data[0], temp1.ang.data[1], temp1.ang.data[2]);
//		temp2 = mul_double_and_screw(2.2, zzi_i[i]);
//		printf("temp2 %d : %f, %f, %f\n\t%f, %f, %f\n", i, temp2.lin.data[0], temp2.lin.data[1], temp2.lin.data[2], temp2.ang.data[0], temp2.ang.data[1], temp2.ang.data[2]);
//		temp3 = add_screw(temp1, temp2);
//		printf("temp3 %d : %f, %f, %f\n\t%f, %f, %f\n", i, temp3.lin.data[0], temp3.lin.data[1], temp3.lin.data[2], temp3.ang.data[0], temp3.ang.data[1], temp3.ang.data[2]);
//		vvh_h = add_screw(mul_s66inv_and_screw(XXh_i[i], vvh_h), mul_double_and_screw(2.2, zzi_i[i]));
//		printf("vvh_h %d : %f, %f, %f\n\t%f, %f, %f\n", i, vvh_h.lin.data[0], vvh_h.lin.data[1], vvh_h.lin.data[2], vvh_h.ang.data[0], vvh_h.ang.data[1], vvh_h.ang.data[2]);
//	}*/
//
//	getFwdKin(q_pos, q_vel);
//
//	printf("\np0_1:");
//	printf("\n%f", _kd.ph_i[0].data[0]);
//	printf("\n%f", _kd.ph_i[0].data[1]);
//	printf("\n%f", _kd.ph_i[0].data[2]);
//	printf("\n");
//	printf("\np1_2:");
//	printf("\n%f", _kd.ph_i[1].data[0]);
//	printf("\n%f", _kd.ph_i[1].data[1]);
//	printf("\n%f", _kd.ph_i[1].data[2]);
//	printf("\n");
//	printf("\np2_3:");
//	printf("\n%f", _kd.ph_i[2].data[0]);
//	printf("\n%f", _kd.ph_i[2].data[1]);
//	printf("\n%f", _kd.ph_i[2].data[2]);
//	printf("\n");
//	printf("\np3_4:");
//	printf("\n%f", _kd.ph_i[3].data[0]);
//	printf("\n%f", _kd.ph_i[3].data[1]);
//	printf("\n%f", _kd.ph_i[3].data[2]);
//	printf("\n");
//	printf("\np4_5:");
//	printf("\n%f", _kd.ph_i[4].data[0]);
//	printf("\n%f", _kd.ph_i[4].data[1]);
//	printf("\n%f", _kd.ph_i[4].data[2]);
//	printf("\n");
//	printf("\np5_6:");
//	printf("\n%f", _kd.ph_i[5].data[0]);
//	printf("\n%f", _kd.ph_i[5].data[1]);
//	printf("\n%f", _kd.ph_i[5].data[2]);
//	printf("\n");
//	printf("\nR0_1:");
//	printf("\n%f %f %f", _kd.Rh_i[0].data[0][0], _kd.Rh_i[0].data[0][1], _kd.Rh_i[0].data[0][2]);
//	printf("\n%f %f %f", _kd.Rh_i[0].data[1][0], _kd.Rh_i[0].data[1][1], _kd.Rh_i[0].data[1][2]);
//	printf("\n%f %f %f", _kd.Rh_i[0].data[2][0], _kd.Rh_i[0].data[2][1], _kd.Rh_i[0].data[2][2]);
//	printf("\n");
//	printf("\nR1_2:");
//	printf("\n%f %f %f", _kd.Rh_i[1].data[0][0], _kd.Rh_i[1].data[0][1], _kd.Rh_i[1].data[0][2]);
//	printf("\n%f %f %f", _kd.Rh_i[1].data[1][0], _kd.Rh_i[1].data[1][1], _kd.Rh_i[1].data[1][2]);
//	printf("\n%f %f %f", _kd.Rh_i[1].data[2][0], _kd.Rh_i[1].data[2][1], _kd.Rh_i[1].data[2][2]);
//	printf("\n");
//	printf("\nR2_3:");
//	printf("\n%f %f %f", _kd.Rh_i[2].data[0][0], _kd.Rh_i[2].data[0][1], _kd.Rh_i[2].data[0][2]);
//	printf("\n%f %f %f", _kd.Rh_i[2].data[1][0], _kd.Rh_i[2].data[1][1], _kd.Rh_i[2].data[1][2]);
//	printf("\n%f %f %f", _kd.Rh_i[2].data[2][0], _kd.Rh_i[2].data[2][1], _kd.Rh_i[2].data[2][2]);
//	printf("\n");
//	printf("\nR3_4:");
//	printf("\n%f %f %f", _kd.Rh_i[3].data[0][0], _kd.Rh_i[3].data[0][1], _kd.Rh_i[3].data[0][2]);
//	printf("\n%f %f %f", _kd.Rh_i[3].data[1][0], _kd.Rh_i[3].data[1][1], _kd.Rh_i[3].data[1][2]);
//	printf("\n%f %f %f", _kd.Rh_i[3].data[2][0], _kd.Rh_i[3].data[2][1], _kd.Rh_i[3].data[2][2]);
//	printf("\n");
//	printf("\nR4_5:");
//	printf("\n%f %f %f", _kd.Rh_i[4].data[0][0], _kd.Rh_i[4].data[0][1], _kd.Rh_i[4].data[0][2]);
//	printf("\n%f %f %f", _kd.Rh_i[4].data[1][0], _kd.Rh_i[4].data[1][1], _kd.Rh_i[4].data[1][2]);
//	printf("\n%f %f %f", _kd.Rh_i[4].data[2][0], _kd.Rh_i[4].data[2][1], _kd.Rh_i[4].data[2][2]);
//	printf("\n");
//	printf("\nR5_6:");
//	printf("\n%f %f %f", _kd.Rh_i[5].data[0][0], _kd.Rh_i[5].data[0][1], _kd.Rh_i[5].data[0][2]);
//	printf("\n%f %f %f", _kd.Rh_i[5].data[1][0], _kd.Rh_i[5].data[1][1], _kd.Rh_i[5].data[1][2]);
//	printf("\n%f %f %f", _kd.Rh_i[5].data[2][0], _kd.Rh_i[5].data[2][1], _kd.Rh_i[5].data[2][2]);
//	printf("\n");
//	printf("\nT0_n:");
//	printf("\n%f %f %f %f", _kd.T0_n.rot.data[0][0], _kd.T0_n.rot.data[0][1], _kd.T0_n.rot.data[0][2], _kd.T0_n.trn.data[0]);
//	printf("\n%f %f %f %f", _kd.T0_n.rot.data[1][0], _kd.T0_n.rot.data[1][1], _kd.T0_n.rot.data[1][2], _kd.T0_n.trn.data[1]);
//	printf("\n%f %f %f %f", _kd.T0_n.rot.data[2][0], _kd.T0_n.rot.data[2][1], _kd.T0_n.rot.data[2][2], _kd.T0_n.trn.data[2]);
//	printf("\n");
//}

// ----------------------------------------------------------------
// _stop
// ----------------------------------------------------------------
static void _stop(void)
{
	printf("destruction OK\n");
}

// ----------------------------------------------------------------
// _state
// ----------------------------------------------------------------
static void _state(void)
{
	// state and errors title
	printf("..\\STATE AND ERRORS> \n\n");

	// display init state
	printf("\tInit state           = %s\n", "LLI_INITIALIZED");
	// display enable state
	printf("\tEnable state         = %s\n", "LLI_ENABLED");
	// display calibrate state
	printf("\tCalibrate state      = %s\n", "LLI_CALIBRATED");
	// display settle state
	printf("\tSettle state         = %s\n", "LLI_SETTLED");
	// display emergency stop state
	printf("\tEmergency stop state = %s\n", "LLI_NO_ESTOP");
	printf("\n");
}

// ----------------------------------------------------------------
// printRobotInfo
// ----------------------------------------------------------------
static void _robotInfo(void)
{
	char			l_choice = 0;
	unsigned int	l_jnt;

	// Task space trajectory selection
	printf("..\\ROBOT INFO> \n\n");

	printf("\n\tRobot manufacturer\t\t: Staubli");
	printf("\n\tRobot model\t\t\t: ");
	if (_rd.robot_model == RX160)
		printf("RX160");
	else if (_rd.robot_model == RX160L)
		printf("RX160L");
	else if (_rd.robot_model == TX90)
		printf("TX90");
	else
		printf("ERROR");
	printf("\n\tJoint type\t\t\t: Articulated (Rotational joints)");
	printf("\n\tNumber of axis\t\t\t: 6-axis");
	printf("\n\tLoad capacity\t\t\t: ");
	if (_rd.robot_model == RX160)
		printf("34 kg (maximum), 20 kg (nominal)");
	else if (_rd.robot_model == RX160L)
		printf("28 kg (maximum), 14 kg (nominal)");
	else if (_rd.robot_model == TX90)
		printf("20 kg (maximum), 7 kg (nominal)");
	else
		printf("ERROR");
	printf("\n\tAttachment of base\t\t: Floor");
	printf("\n\tAttachment(s) to end-effector\t: ");
	if (_rd.ee_attachments == NO_EE)
		printf("NONE");
	else if (_rd.ee_attachments == SENSOR)
		printf("ATI Delta F/T Transducer");
	else if (_rd.ee_attachments == GRIPPER)
		printf("ATI Delta F/T Transducer + Gripper");
	else if (_rd.ee_attachments == BALL_SPRING)
		printf("ATI Delta F/T Transducer + Ball&Spring");
	else
		printf("ERROR");

	printf("\n");
	printf("\n\t\tDH Table:");
	printf("\n\t\t---------------------------------");
	printf("\n\t\t|   a   | alpha |   d   | theta |");
	printf("\n\t\t---------------------------------");
	for (l_jnt = 0; l_jnt < MAX_NB_JNT; l_jnt++)
	{
		printf("\n\t\t");
		printf("|%6.3f |", _kd.a[l_jnt]);
		if ((_kd.alpha[l_jnt] > 0) && (fabs(_kd.alpha[l_jnt] - PI) < S_SMALL_FLOAT))
			printf("   PI  |");
		else if ((_kd.alpha[l_jnt] < 0) && (fabs(_kd.alpha[l_jnt] + PI) < S_SMALL_FLOAT))
			printf("  -PI  |");
		else if ((_kd.alpha[l_jnt] > 0) && (fabs(_kd.alpha[l_jnt] - PI / 2) < S_SMALL_FLOAT))
			printf("  PI/2 |");
		else if ((_kd.alpha[l_jnt] < 0) && (fabs(_kd.alpha[l_jnt] + PI / 2) < S_SMALL_FLOAT))
			printf(" -PI/2 |");
		else
			printf("%6.3f |", _kd.alpha[l_jnt]);
		printf("%6.3f |", _kd.d[l_jnt]);
		if ((_kd.theta[l_jnt] > 0) && (fabs(_kd.theta[l_jnt] - PI) < S_SMALL_FLOAT))
			printf("   PI  |");
		else if ((_kd.theta[l_jnt] < 0) && (fabs(_kd.theta[l_jnt] + PI) < S_SMALL_FLOAT))
			printf("  -PI  |");
		else if ((_kd.theta[l_jnt] > 0) && (fabs(_kd.theta[l_jnt] - PI / 2) < S_SMALL_FLOAT))
			printf("  PI/2 |");
		else if ((_kd.theta[l_jnt] < 0) && (fabs(_kd.theta[l_jnt] + PI / 2) < S_SMALL_FLOAT))
			printf(" -PI/2 |");
		else
			printf("%6.3f |", _kd.theta[l_jnt]);
	}
	printf("\n\t\t---------------------------------\n\n");

	printf("\t<--\t 0:\tBack to main menu \n");
	printf("\t\t 1:\tChange end-effector attachments\n");
	printf("\t< >\t 2:\tChange robot model\n");

	printf("\n\t Selection ? ");
	l_choice = inputMenu();
	printf("\n\n");

	// ~SWITCH 
	switch (l_choice)
	{
	case '0':	break;
	case '1':
		if (_rd.ee_attachments == NO_EE)
			_rd.ee_attachments = SENSOR;
		else if (_rd.ee_attachments == SENSOR)
			_rd.ee_attachments = GRIPPER;
		else if (_rd.ee_attachments == GRIPPER)
			_rd.ee_attachments = BALL_SPRING;
		else
			_rd.ee_attachments = NO_EE;
		initKinPara();
		getEEparameters();
		initDynPara();

		_robotInfo();
		break;
	case '2':
		if (_rd.robot_model == TX90)
			_rd.robot_model = RX160;
		else if (_rd.robot_model == RX160)
			_rd.robot_model = RX160L;
		else
			_rd.robot_model = TX90;
		initKinPara();
		getDHparameters();
		initDynPara();

		_robotInfo();
		break;
	default:	_robotInfo(); break;
	}
}

// ----------------------------------------------------------------
// _printDataMenu
// ----------------------------------------------------------------
void _printDataMenu(void)
{
	char l_choice = 0;

	printf("Do you want to print data?[y/n] ");
	l_choice = inputMenu();
	printf("\n\n");

	if ((l_choice == 'Y') || (l_choice == 'y'))
	{
		printf("\nMotion is saving.\n");
		++_pd.testNb;
		_pd.unprinted_test = false;
		printAlgoData(_pd.testNb);
	}
	else
	{
		printf("\nMotion saving canceled!");
	}
}

// ----------------------------------------------------------------
// printCurrentState
// ----------------------------------------------------------------
static void printCurrentState(void)
{
	char date_time[21];

	time_t rawtime;
	time(&rawtime);
	localtime_s(&timeinfo, &rawtime);

	dateTimeForm(date_time, "%s-%s-%s %s:%s:%s", timeinfo);

	printf("\tDate/time\t: %s", date_time);

	printf("\n\tRobot model\t: ");
	if (_rd.robot_model == RX160)
		printf("RX160");
	else if (_rd.robot_model == RX160L)
		printf("RX160L");
	else if (_rd.robot_model == TX90)
		printf("TX90");
	else
		printf("ERROR");

	printf("\n\tEnd-effector\t: ");
	if (_rd.ee_attachments == NO_EE)
		printf("NONE");
	else if (_rd.ee_attachments == SENSOR)
		printf("F/T SENSOR");
	else if (_rd.ee_attachments == GRIPPER)
		printf("F/T SENSOR + GRIPPER");
	else if (_rd.ee_attachments == BALL_SPRING)
		printf("F/T SENSOR + BALL&SPRING");
	else
		printf("ERROR");

	if (_rd.ee_attachments != NO_EE)
	{
		printf("\n\ttool mass\t: %f", _dd.tool_mass);
		printf("\n\ttool CoM 6F\t: %f %f %f", _dd.tool_CoM_6F.data[0], _dd.tool_CoM_6F.data[1], _dd.tool_CoM_6F.data[2]);
	}

	printf("\t\n\n");
}

// ----------------------------------------------------------------
// printMainMenu
// ----------------------------------------------------------------
void printMainMenu(void)
{
	printf("   MAIN MENU> \n\n");

	printCurrentState();

	printf("\t[X]\t 0:\tStop LLI\n");
	printf("\n");
	printf("\t-v-\t 1:\tState and errors\n");
	printf("\t\\..\t 2:\tMove\n");
	printf("\t\\..\t 3:\tControl settings\n");
	printf("\n");

	if (_pd.unprinted_test == true)
		printf("\t\t p:\tPrint data (Current test number: %d)\n", _pd.testNb + 1);
	printf("\t\\..\t r:\tRobot info\n");
	printf("\t\\..\t s:\tSettings\n");
	printf("\t\\..\t t:\tTest\n");
}

// ----------------------------------------------------------------
// switchMainMenu
// ----------------------------------------------------------------
char switchMainMenu(char x_choice)
{
	char l_choice = 0;

	// ~SWITCH 
	switch (x_choice)
	{
	case '0':

		printf("Do you want to stop LLI?[y/n] ");
		l_choice = inputMenu();
		printf("\n\n");

		if ((l_choice == 'Y') || (l_choice == 'y'))
		{
			_stop();

			return 'q';
		}

		break;
	case '1': _state();		break;
	case '2': _moveMenu();	break;
	case 'p':
		if (_pd.unprinted_test == true)
			_printDataMenu();
		break;
	case 'q': return ' ';	break;
	case 'r': _robotInfo();	break;
	case 't': _test();		break;
	default: break;
	} // ~ENDSWITCH 

	return x_choice;
}

// ----------------------------------------------------------------
// menu
// ----------------------------------------------------------------
void menu(void)
{
	char l_choice = 0;

	// ~DO 
	while (l_choice != 'q')
	{
		// Print menu
		printf("\t\n\n");
		printMainMenu();

		printf("\n\t Selection ? ");
		l_choice = inputMenu();
		printf("\n\n");

		l_choice = switchMainMenu(l_choice);
	}
	printf("bye!\n");
}

// ----------------------------------------------------------------
// testControl
// ----------------------------------------------------------------
void testControl(void)
{
	printf("Constructing robot\n");
	printf("construction OK\n");

	_rd.robot_model = RX160;
	/*g_jntNb = MAX_NB_JNT;
	g_cycleTime = 0.001;*/

	initSimPara();
	initKinPara();
	initDynPara();
	initCtrlPara();
	initConfig();
	initAlgo();

	printf("initializing robot\n");
	printf("initialization OK\n");

	printf("\n\n\n\n");
	printf("\t\t***************************\n");
	printf("\t\t*   Tests of LLI module   *\n");
	printf("\t\t***************************\n");
	printf("\n\n");

	menu();
}

void test(void)
{
	testControl();
}

int main()
{
	test();
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
