//*********************************************************************
// ~NAME : algo.c
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Implementation of simulation algo
//**********************************************************************

#include <stdio.h>

#include "algo.h"
#include "test.h"
#include "movement.h"
#include "control.h"
#include "kinematics.h"
#include "dynamics.h"
#include "printdata.h"

// ----------------------------------------------------------------
// initAlgo
// ----------------------------------------------------------------
void initAlgo(void)
{
    _ad.q_posFbk = _md.q_posIni;
    _ad.q_posDes = _md.q_posIni;
    _ad.q_velFbk = get_rn_zero();
    _ad.q_velDes = get_rn_zero();

    _wd.q_pos = _md.q_posIni;
    _wd.q_vel = get_rn_zero();
    _wd.q_acc = get_rn_zero();

    _pd.rec = STOP_RECORD;
    _pd.cycleNb = 0;
    /*
    _ad.q_posFbk[0] = _DEG_TO_RAD * 26.845443;
    _ad.q_posFbk[1] = _DEG_TO_RAD * 16.520273;
    _ad.q_posFbk[2] = _DEG_TO_RAD * 49.560818;
    _ad.q_posFbk[3] = _DEG_TO_RAD * -12.390205;
    _ad.q_posFbk[4] = _DEG_TO_RAD * -24.780409;
    _ad.q_posFbk[5] = _DEG_TO_RAD * 18.585307;

    _ad.q_velFbk[0] = _DEG_TO_RAD * 6.786452;
    _ad.q_velFbk[1] = _DEG_TO_RAD * 4.176278;
    _ad.q_velFbk[2] = _DEG_TO_RAD * 12.528834;
    _ad.q_velFbk[3] = _DEG_TO_RAD * -3.132209;
    _ad.q_velFbk[4] = _DEG_TO_RAD * -6.264417;
    _ad.q_velFbk[5] = _DEG_TO_RAD * 4.698313;

    _cd.tau[0] = 0.3816;
    _cd.tau[1] = 0.7655;
    _cd.tau[2] = 0.7952;
    _cd.tau[3] = 0.1869;
    _cd.tau[4] = 0.4898;
    _cd.tau[5] = 0.4456;*/
}

// ----------------------------------------------------------------
// Virtual Environment
// ----------------------------------------------------------------
screw environment(const se3 x_pos)
{
    screw ff_e = { 0.0 };

    return ff_e;
}

// ----------------------------------------------------------------
// Direct Dynamics: Newton Euler Method
// ----------------------------------------------------------------
rn robotAlgo(const rn tau_cmd, const screw ff_e)
{
    int i;
    double H_i[N_DOF] = { 0.0 };
    r3 ph_i = { 0.0 };
    r3x3 Rh_i = { 0.0 };
    rn tau = { 0.0 }, q_acc = { 0.0 };
    screw vvh_h = { 0.0 }, gammai_i[N_DOF] = { 0.0 }, ff_ei_i[N_DOF] = { 0.0 }, betah_h[N_DOF + 1] = { 0.0 }
        , beta2h_h[N_DOF + 1] = { 0.0 }, alphai_i = { 0.0 }, aai_h = { 0.0 }, aah_h = { 0.0 };
    st XXi_h[N_DOF] = { 0.0 }, IIh_h[N_DOF + 1] = { 0.0 }, II2h_h[N_DOF + 1] = { 0.0 }, KKi_i = { 0.0 };

    _wd.tau_spr = getSprTorque(_wd.q_pos);
    _wd.tau_frc = getFrcTorque(_wd.q_vel);

    tau = sub_rn(tau_cmd, add_rn(_wd.tau_spr, _wd.tau_frc));

    // i) First forward recursive computations for i = 1, ..., n
    ff_ei_i[N_DOF - 1] = ff_e; // mul ff_ei_i with -1
    for (i = 0; i <= N_DOF - 1; i++)
    {
        Rh_i = get_Rh_i(Rh_i, i, _wd.q_pos.data[i]);
        ph_i = get_ph_i(ph_i, i);
        XXi_h[i] = set_tr_wrench_st_from_se(Rh_i, ph_i);

        gammai_i[i] = set_screw(mul_r3x3_and_r3(transpose_r3x3(Rh_i), cross_prod(vvh_h.ang, cross_prod(vvh_h.ang, ph_i)))
            , cross_prod(mul_r3x3_and_r3(transpose_r3x3(Rh_i), vvh_h.ang), mul_double_and_r3(_wd.q_vel.data[i], _kd.zi_i)));
        vvh_h = add_screw(mul_st_and_screw(XXi_h[i], vvh_h), mul_double_and_screw(_wd.q_vel.data[i], _kd.zzi_i));
        betah_h[i + 1] = sub_screw(ff_ei_i[i], set_screw(cross_prod(vvh_h.ang, cross_prod(vvh_h.ang, _dd.di_i[i]))
                                                        , cross_prod(vvh_h.ang, mul_r3x3_and_r3(_dd.Ii_i[i], vvh_h.ang))));
        IIh_h[i + 1] = set_st(set_r3x3eye(_dd.m_i[i]), mul_double_and_r3x3(-1.0, skew_sym(_dd.di_i[i])), skew_sym(_dd.di_i[i]), _dd.Ii_i[i]);
    }

    // ii) Backward recursive computations for i = n, ..., 1
    II2h_h[N_DOF] = IIh_h[N_DOF];
    beta2h_h[N_DOF] = betah_h[N_DOF];
    for (i = N_DOF - 1; i >= 0; i--)
    {
        H_i[i] = dot_prod_screw(_kd.zzi_i, mul_st_and_screw(II2h_h[i + 1], _kd.zzi_i));
        KKi_i = sub_st(II2h_h[i + 1], mul_double_and_st((1.0 / H_i[i]), mul_st(II2h_h[i + 1], mul_st(mul_col_and_row_screw(_kd.zzi_i, _kd.zzi_i), II2h_h[i + 1]))));
        alphai_i = sub_screw(add_screw(mul_st_and_screw(KKi_i, gammai_i[i]), mul_double_and_screw((1.0 / H_i[i]) * (tau.data[i] + dot_prod_screw(_kd.zzi_i, beta2h_h[i + 1]))
                                                                                                            , mul_st_and_screw(II2h_h[i + 1], _kd.zzi_i))), beta2h_h[i + 1]);
        beta2h_h[i] = sub_screw(betah_h[i], mul_st_and_screw(transpose_st(XXi_h[i]), alphai_i));
        II2h_h[i] = add_st(IIh_h[i], mul_st(transpose_st(XXi_h[i]), mul_st(KKi_i, XXi_h[i])));
    }

    // iii) Second forward recursive computations for i = 1, ..., n
    aah_h = set_screw(set_r3(0.0, 0.0, 9.81), get_r3_zero());
    for (i = 0; i <= N_DOF - 1; i++)
    {
        aai_h = mul_st_and_screw(XXi_h[i], aah_h);
        q_acc.data[i] = (1.0 / H_i[i]) * (-dot_prod_screw(_kd.zzi_i, mul_st_and_screw(II2h_h[i + 1], add_screw(aai_h, gammai_i[i]))) + tau.data[i] + dot_prod_screw(_kd.zzi_i, beta2h_h[i + 1]));
        aah_h = add_screw(aai_h, add_screw(mul_double_and_screw(q_acc.data[i], _kd.zzi_i), gammai_i[i]));
        //ffi_i = sub_screw(mul_st_and_screw(II2h_h[i + 1], aah_h), beta2h_h[i + 1]);
    }

    return q_acc;
}

// ----------------------------------------------------------------
// computeAlgo
// ----------------------------------------------------------------
void computeAlgo(double tcyc, double tstp, double tfin)
{
	unsigned int k, d;

    d = (unsigned int)(tcyc / tstp);

    _pd.rec = START_RECORD;

    recordData();

	for (k = 1; k <= tfin / tstp; k++)
	{
        getFwdKin(_ad.q_posFbk, _ad.q_velFbk);

        /// control loop ----------------------------- ///
        if (k % d == 0)
        { 
            // error calculation -------------------------- //
            _cd.e_q     = sub_rn(_ad.q_posDes, _ad.q_posFbk);
            _cd.e_qsum  = get_rn_zero();
            _cd.e_qdot  = sub_rn(_ad.q_velDes, _ad.q_velFbk);
            // error calculation -------------------------- //

            _cd.tau_ctrl = joint_pid(_cd.K_pq_diag, _cd.K_iq_diag, _cd.K_dq_diag, _cd.e_q, _cd.e_qsum, _cd.e_qdot);

            /*_dd.tau_spr = getSprTorque(_ad.q_posFbk);
            _dd.tau_frc = getFrcTorque(_ad.q_velFbk);*/
            _dd.tau_spr = get_rn_zero();
            _dd.tau_frc = get_rn_zero();

            _cd.tau_cmd = add_rn(_cd.tau_ctrl, add_rn(_dd.tau_spr, _dd.tau_frc));
            // control algorithm -------------------------- //
        }
        /// control loop ----------------------------- ///

        // virtual robot ------------------------------ //
        _wd.q_acc = robotAlgo(_cd.tau_cmd, _wd.ff_e);
        _wd.q_vel = integrate_rn(_wd.q_vel, _wd.q_acc, tstp);
        _wd.q_pos = integrate_rn(_wd.q_pos, _wd.q_vel, tstp);

        _wd.x_pos = getPose(_wd.q_pos);
        _wd.ff_e = environment(_wd.x_pos);
        // virtual robot ------------------------------ //

        // measurement -------------------------------- //
        if (k % d == 0)
        {
            _ad.q_velFbk = _wd.q_vel;
            _ad.q_posFbk = _wd.q_pos;

            recordData();
        }
        // measurement -------------------------------- //

        //printf("\n");
        //printf("\ni = %d, t = %f", k, k * tstp);
        ///*printf("\nqdd_i :");
        //printf("\n%20.15f %20.15f %20.15f %20.15f %20.15f %20.15f", _ad.q_accFbk[0], _ad.q_accFbk[1], _ad.q_accFbk[2], _ad.q_accFbk[3], _ad.q_accFbk[4], _ad.q_accFbk[5]);*/
        ////printf("\nq_velFbk = %f %f %f %f %f %f", _ad.q_velFbk.data[0], _ad.q_velFbk.data[1], _ad.q_velFbk.data[2], _ad.q_velFbk.data[3], _ad.q_velFbk.data[4], _ad.q_velFbk.data[5]);
        //printf("\nq_posWrl = %f %f %f %f %f %f", _wd.q_pos.data[0], _wd.q_pos.data[1], _wd.q_pos.data[2], _wd.q_pos.data[3], _wd.q_pos.data[4], _wd.q_pos.data[5]);
        //printf("\nq_posFbk = %f %f %f %f %f %f", _ad.q_posFbk.data[0], _ad.q_posFbk.data[1], _ad.q_posFbk.data[2], _ad.q_posFbk.data[3], _ad.q_posFbk.data[4], _ad.q_posFbk.data[5]);

        //printf("\nx_velFbk : %f %f %f %f %f %f", _ad.x_velFbk.lin.data[0], _ad.x_velFbk.lin.data[1], _ad.x_velFbk.lin.data[2], _ad.x_velFbk.ang.data[0], _ad.x_velFbk.ang.data[1], _ad.x_velFbk.ang.data[2]);
        //printf("\nx_velFbk2: %17.15f %17.15f %17.15f %17.15f %17.15f %17.15f\n", _ad.x_velFbk2.data[0], _ad.x_velFbk2.data[1], _ad.x_velFbk2.data[2], _ad.x_velFbk2.data[3]
        //, _ad.x_velFbk2.data[4], _ad.x_velFbk2.data[5]);

        //getchar();
	}

    _pd.rec = STOP_RECORD;
}
