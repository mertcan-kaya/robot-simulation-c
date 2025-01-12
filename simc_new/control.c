//*********************************************************************
// ~NAME : control.c
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Control algorithms
//**********************************************************************

#include "control.h"
#include "algo.h"
#include "test.h"
#include "kinematics.h"
#include "functions.h"

#include <stdio.h>

// ----------------------------------------------------------------
// initCtrlPara
// ----------------------------------------------------------------
void initCtrlPara(void)
{
	_cd.K_pq_diag.data[0] = 100;
	_cd.K_pq_diag.data[1] = 200; // 200
	_cd.K_pq_diag.data[2] = 200; // 200
	//_cd.K_pq_diag.data[1] = 1000; // 200
	//_cd.K_pq_diag.data[2] = 1000; // 200
	_cd.K_pq_diag.data[3] = 50;
	_cd.K_pq_diag.data[4] = 50;
	_cd.K_pq_diag.data[5] = 50;

	_cd.K_dq_diag.data[0] = 30;
	_cd.K_dq_diag.data[1] = 40; // 40
	_cd.K_dq_diag.data[2] = 40; // 40
	//_cd.K_dq_diag.data[1] = 00; // 40
	//_cd.K_dq_diag.data[2] = 00; // 40
	_cd.K_dq_diag.data[3] = 30;
	_cd.K_dq_diag.data[4] = 30;
	_cd.K_dq_diag.data[5] = 10;

	_cd.K_iq_diag.data[0] = 0.0;
	_cd.K_iq_diag.data[1] = 10.0;
	_cd.K_iq_diag.data[2] = 10.0;
	_cd.K_iq_diag.data[3] = 0.0;
	_cd.K_iq_diag.data[4] = 0.0;
	_cd.K_iq_diag.data[5] = 0.0;
}

rn joint_pid(const rn K_pq_diag, const rn K_iq_diag, const rn K_dq_diag, const rn e_q, const rn e_qsum, const rn e_qdot)
{
	rn tau_ctrl = { 0.0 };

	tau_ctrl = add_rn(mul_rn_elem_wise(K_pq_diag, e_q), mul_rn_elem_wise(K_dq_diag, e_qdot));

	return tau_ctrl;
}

rn mnea(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, const r10 p[N_DOF])
{
    // declaration of output
    rn tau_i = { 0.0 };

    // declarations of temporary terms
    int i;
    r3 zeros_3x1 = { 0.0 }, upsi_i = { 0.0 }, wh_h = { 0.0 }, wRh_h = { 0.0 }
        , wdh_h = { 0.0 }, fi_i = { 0.0 }, ni_i = { 0.0 };
    r3x3 Rh_i[N_DOF + 1] = { 0.0 }, Ri_h[N_DOF + 1] = { 0.0 }, Gh_h = { 0.0 }, SSSh_h = { 0.0 };
    r3x6 zeros_3x6 = { 0.0 }, Dh_h = { 0.0 };
    r3x10 Psifi_i[N_DOF] = { 0.0 }, Psini_i[N_DOF] = { 0.0 };

    // forward recursion (link 1 to n)
    r3 muh_h = mul_double_and_r3(-1.0, g);
    for (i = 0; i < N_DOF; i++)
    {
        Rh_i[i] = get_Rh_i(Rh_i[i], i, q.data[i]);
        Ri_h[i] = transpose_r3x3(Rh_i[i]);

        if (_cd.passImp == PASS_A)
            upsi_i = cross_prod(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qd.data[i], _kd.zi_i));
        else if (_cd.passImp == PASS_B)
            upsi_i = cross_prod(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i));
        else // (_cd.passImp == PASS_C)
            upsi_i = avg_r3(cross_prod(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qd.data[i], _kd.zi_i)), cross_prod(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i)));

        wdh_h = add_3_r3(mul_r3x3_and_r3(Ri_h[i], wdh_h), mul_double_and_r3(qRdd.data[i], _kd.zi_i), upsi_i);
        muh_h = mul_r3x3_and_r3(Ri_h[i], add_r3(muh_h, mul_r3x3_and_r3(SSSh_h, _kd.rh_h_i[i])));

        wh_h = add_r3(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qd.data[i], _kd.zi_i));
        wRh_h = add_r3(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i));

        if (_cd.passImp == PASS_A)
        {
            Gh_h = mul_r3x3(skew_sym(wh_h), skew_sym(wRh_h));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wRh_h), uniq_mat(wh_h));
        }
        else if (_cd.passImp == PASS_B)
        {
            Gh_h = mul_r3x3(skew_sym(wRh_h), skew_sym(wh_h));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wh_h), uniq_mat(wRh_h));
        }
        else // (_cd.passImp == PASS_C)
        {
            Gh_h = avg_r3x3(mul_r3x3(skew_sym(wh_h), skew_sym(wRh_h)), mul_r3x3(skew_sym(wRh_h), skew_sym(wh_h)));
            Dh_h = avg_r3x6(mul_r3x3_and_r3x6(skew_sym(wRh_h), uniq_mat(wh_h)), mul_r3x3_and_r3x6(skew_sym(wh_h), uniq_mat(wRh_h)));
        }

        SSSh_h = add_r3x3(skew_sym(wdh_h), Gh_h);

        Psifi_i[i] = compose_r3x10(muh_h, SSSh_h, zeros_3x6);
        Psini_i[i] = compose_r3x10(zeros_3x1, transpose_r3x3(skew_sym(muh_h)), add_r3x6(uniq_mat(wdh_h), Dh_h));
    }

    // backward recursion (link n to 1)
    for (i = N_DOF - 1; i >= 0; i--)
    {
        ni_i = add_r3(mul_r3x10_and_r10(Psini_i[i], p[i]), mul_r3x3_and_r3(Rh_i[i + 1], add_r3(mul_r3x3_and_r3(skew_sym(mul_r3x3_and_r3(Ri_h[i + 1], _kd.rh_h_i[i + 1])), fi_i), ni_i)));
        fi_i = add_r3(mul_r3x10_and_r10(Psifi_i[i], p[i]), mul_r3x3_and_r3(Rh_i[i + 1], fi_i));

        tau_i.data[i] = dot_prod(_kd.zi_i, ni_i);
    }

    return tau_i;
}

rn anea(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF])
{
    // declaration of output
    rn tau_fi = { 0.0 };

    // declarations of temporary terms
    int i;
    r3 zeros_3x1 = { 0.0 }, upsi_i = { 0.0 }, wh_h = { 0.0 }, wRh_h = { 0.0 }, wdh_h = { 0.0 }
        , swh_h = { 0.0 }, svh_h = { 0.0 }, fi_fi = { 0.0 }, ni_fi = { 0.0 };
    r3x3 Rh_i[N_DOF + 1] = { 0.0 }, Ri_h[N_DOF + 1] = { 0.0 }, Gh_h = { 0.0 }, SSSh_h = { 0.0 };
    r3x6 zeros_3x6 = { 0.0 }, Dh_h = { 0.0 };
    r3x10 Psifi_i[N_DOF] = { 0.0 }, Psini_i[N_DOF] = { 0.0 };
    r10 sigma_i = { 0.0 }, phati_i[N_DOF] = { 0.0 };

    // forward recursion(link 1 to n)
    r3 muh_h = mul_double_and_r3(-1.0, g);
    for (i = 0; i < N_DOF; i++)
    {
        Rh_i[i] = get_Rh_i(Rh_i[i], i, q.data[i]);
        Ri_h[i] = transpose_r3x3(Rh_i[i]);

        if (_cd.passImp == PASS_A)
            upsi_i = cross_prod(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qd.data[i], _kd.zi_i));
        else if (_cd.passImp == PASS_B)
            upsi_i = cross_prod(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i));
        else // (_cd.passImp == PASS_C)
            upsi_i = avg_r3(cross_prod(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qd.data[i], _kd.zi_i)), cross_prod(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i)));

        wdh_h = add_3_r3(mul_r3x3_and_r3(Ri_h[i], wdh_h), mul_double_and_r3(qRdd.data[i], _kd.zi_i), upsi_i);
        muh_h = mul_r3x3_and_r3(Ri_h[i], add_r3(muh_h, mul_r3x3_and_r3(SSSh_h, _kd.rh_h_i[i])));

        wh_h = add_r3(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qd.data[i], _kd.zi_i));
        wRh_h = add_r3(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i));

        if (_cd.passImp == PASS_A)
        {
            Gh_h = mul_r3x3(skew_sym(wh_h), skew_sym(wRh_h));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wRh_h), uniq_mat(wh_h));
        }
        else if (_cd.passImp == PASS_B)
        {
            Gh_h = mul_r3x3(skew_sym(wRh_h), skew_sym(wh_h));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wh_h), uniq_mat(wRh_h));
        }
        else // (_cd.passImp == PASS_C)
        {
            Gh_h = avg_r3x3(mul_r3x3(skew_sym(wh_h), skew_sym(wRh_h)), mul_r3x3(skew_sym(wRh_h), skew_sym(wh_h)));
            Dh_h = avg_r3x6(mul_r3x3_and_r3x6(skew_sym(wRh_h), uniq_mat(wh_h)), mul_r3x3_and_r3x6(skew_sym(wh_h), uniq_mat(wRh_h)));
        }

        SSSh_h = add_r3x3(skew_sym(wdh_h), Gh_h);

        Psifi_i[i] = compose_r3x10(muh_h, SSSh_h, zeros_3x6);
        Psini_i[i] = compose_r3x10(zeros_3x1, transpose_r3x3(skew_sym(muh_h)), add_r3x6(uniq_mat(wdh_h), Dh_h));

        svh_h = mul_r3x3_and_r3(Ri_h[i], add_r3(svh_h, cross_prod(swh_h, _kd.rh_h_i[i])));
        swh_h = sub_r3(wh_h, wRh_h);

        sigma_i = add_r10(mul_transpose_r3x10_and_r3(Psifi_i[i], svh_h), mul_transpose_r3x10_and_r3(Psini_i[i], swh_h));
        p[i] = integrate_r10(p[i], mul_r10_elem_wise(Pdiag[i], sigma_i), -_sd.cycle_time);
    }

    // backward recursion recursion(link n to 1)
    for (i = N_DOF - 1; i >= 0; i--)
    {
        ni_fi = add_r3(mul_r3x10_and_r10(Psini_i[i], p[i]), mul_r3x3_and_r3(Rh_i[i + 1], add_r3(mul_r3x3_and_r3(skew_sym(mul_r3x3_and_r3(Ri_h[i + 1], _kd.rh_h_i[i + 1])), fi_fi), ni_fi)));
        fi_fi = add_r3(mul_r3x10_and_r10(Psifi_i[i], p[i]), mul_r3x3_and_r3(Rh_i[i + 1], fi_fi));
        tau_fi.data[i] = dot_prod(_kd.zi_i, ni_fi);
    }

    return tau_fi;
}

rn mtrx_func_new(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF])
{
    int i, j;
    r3 rj_ji[N_DOF][N_DOF] = { 0.0 }, vh_h[N_DOF + 1] = { 0.0 }, wh_h[N_DOF + 1] = { 0.0 }
        , vRh_h[N_DOF + 1] = { 0.0 }, wRh_h[N_DOF + 1] = { 0.0 }, vdh_h = { 0.0 }
        , wdh_h = { 0.0 }, gh = { 0.0 }, muh_h = { 0.0 }, zeros_3x1 = { 0.0 }
        , zeta_lin = { 0.0 }, zeta_ang = { 0.0 };
    r3x3 Rh_i[N_DOF + 1] = { 0.0 }, Ri_h[N_DOF + 1] = { 0.0 }, Ri_j[N_DOF][N_DOF] = { 0.0 }, Gh_h = { 0.0 };
    r3x6 Dh_h = { 0.0 }, zeros_3x6 = { 0.0 };
    r3x10 Psifi_i = { 0.0 }, Psini_i = { 0.0 };
    rn s = { 0.0 }, tau_f = { 0.0 };
    rnxn0 Y = { 0.0 };
    screw Ji_ij[N_DOF][N_DOF] = { 0.0 }, Jdi_ij[N_DOF][N_DOF] = { 0.0 }, JRdi_ij[N_DOF][N_DOF] = { 0.0 };
    screwx10 Psii_i[N_DOF] = { 0.0 };
    rn0 p_bar_new = { 0.0 }, sigma_bar = { 0.0 }, Pdiag_bar = { 0.0 };

    for (i = 0; i < N_DOF; i++)
    {
        Rh_i[i] = get_Rh_i(Rh_i[i], i, q.data[i]);
        Ri_h[i] = transpose_r3x3(Rh_i[i]);
    }

    for (i = N_DOF - 1; i >= 0; i--)
    {
        for (j = i; j >= 0; j--)
        {
            if (j == i)
                Ri_j[i][j] = get_r3x3_idnt();
            else
            {
                Ri_j[i][j] = mul_r3x3(Ri_j[i][j + 1], Ri_h[j + 1]);
                if (j == i - 1)
                    rj_ji[j][i] = _kd.rh_h_i[j + 1];
                else
                    rj_ji[j][i] = add_r3(mul_r3x3_and_r3(Rh_i[j + 1], rj_ji[j + 1][i]), _kd.rh_h_i[j + 1]);
            }
        }
    }

    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j <= i; j++)
        {
            Ji_ij[j][i] = set_screw(mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(transpose_r3x3(skew_sym(rj_ji[j][i])), _kd.zi_i))
                , mul_r3x3_and_r3(Ri_j[i][j], _kd.zi_i));

            vh_h[i + 1] = add_r3(vh_h[i + 1], mul_double_and_r3(qd.data[j], Ji_ij[j][i].lin));
            wh_h[i + 1] = add_r3(wh_h[i + 1], mul_double_and_r3(qd.data[j], Ji_ij[j][i].ang));
            vRh_h[i + 1] = add_r3(vRh_h[i + 1], mul_double_and_r3(qRd.data[j], Ji_ij[j][i].lin));
            wRh_h[i + 1] = add_r3(wRh_h[i + 1], mul_double_and_r3(qRd.data[j], Ji_ij[j][i].ang));
        }
    }

    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j <= i; j++)
        {
            Jdi_ij[j][i] = set_screw(mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(add_r3x3(mul_r3x3(transpose_r3x3(skew_sym(rj_ji[j][i]))
                , skew_sym(wh_h[j + 1])), transpose_r3x3(skew_sym(sub_r3(mul_r3x3_and_r3(transpose_r3x3(Ri_j[i][j])
                    , vh_h[i + 1]), vh_h[j + 1])))), _kd.zi_i))
                , mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(skew_sym(wh_h[j + 1]), _kd.zi_i)));
            JRdi_ij[j][i] = set_screw(mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(add_r3x3(mul_r3x3(transpose_r3x3(skew_sym(rj_ji[j][i]))
                , skew_sym(wRh_h[j + 1])), transpose_r3x3(skew_sym(sub_r3(mul_r3x3_and_r3(transpose_r3x3(Ri_j[i][j])
                    , vRh_h[i + 1]), vRh_h[j + 1])))), _kd.zi_i))
                , mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(skew_sym(wRh_h[j + 1]), _kd.zi_i)));
        }
    }

    gh = g;
    muh_h = mul_double_and_r3(-1.0, gh);
    for (i = 0; i < N_DOF; i++)
    {
        zeta_lin = get_r3_zero();
        zeta_ang = get_r3_zero();
        vdh_h = get_r3_zero();
        wdh_h = get_r3_zero();
        if (_cd.passImp == PASS_A)
        {
            for (j = 0; j <= i; j++)
            {
                zeta_lin = add_r3(zeta_lin, mul_double_and_r3(qd.data[j], JRdi_ij[j][i].lin));
                zeta_ang = add_r3(zeta_ang, mul_double_and_r3(qd.data[j], JRdi_ij[j][i].ang));
            }
            Gh_h = mul_r3x3(skew_sym(wh_h[i + 1]), skew_sym(wRh_h[i + 1]));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wRh_h[i + 1]), uniq_mat(wh_h[i + 1]));
        }
        else if (_cd.passImp == PASS_B)
        {
            for (j = 0; j <= i; j++)
            {
                zeta_lin = add_r3(zeta_lin, mul_double_and_r3(qRd.data[j], Jdi_ij[j][i].lin));
                zeta_ang = add_r3(zeta_ang, mul_double_and_r3(qRd.data[j], Jdi_ij[j][i].ang));
            }
            Gh_h = mul_r3x3(skew_sym(wRh_h[i + 1]), skew_sym(wh_h[i + 1]));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wh_h[i + 1]), uniq_mat(wRh_h[i + 1]));
        }
        else // (_cd.passImp == PASS_C)
        {
            for (j = 0; j <= i; j++)
            {
                zeta_lin = add_r3(zeta_lin, avg_r3(mul_double_and_r3(qd.data[j], JRdi_ij[j][i].lin), mul_double_and_r3(qRd.data[j], Jdi_ij[j][i].lin)));
                zeta_ang = add_r3(zeta_ang, avg_r3(mul_double_and_r3(qd.data[j], JRdi_ij[j][i].ang), mul_double_and_r3(qRd.data[j], Jdi_ij[j][i].ang)));
            }
            Gh_h = avg_r3x3(mul_r3x3(skew_sym(wh_h[i + 1]), skew_sym(wRh_h[i + 1])), mul_r3x3(skew_sym(wRh_h[i + 1]), skew_sym(wh_h[i + 1])));
            Dh_h = avg_r3x6(mul_r3x3_and_r3x6(skew_sym(wRh_h[i + 1]), uniq_mat(wh_h[i + 1])), mul_r3x3_and_r3x6(skew_sym(wh_h[i + 1]), uniq_mat(wRh_h[i + 1])));
        }

        for (j = 0; j <= i; j++)
        {
            vdh_h = add_r3(vdh_h, mul_double_and_r3(qRdd.data[j], Ji_ij[j][i].lin));
            wdh_h = add_r3(wdh_h, mul_double_and_r3(qRdd.data[j], Ji_ij[j][i].ang));
        }
        vdh_h = add_r3(vdh_h, zeta_lin);
        wdh_h = add_r3(wdh_h, zeta_ang);
        gh = mul_r3x3_and_r3(Ri_h[i], gh);
        muh_h = sub_r3(vdh_h, gh);

        printf("\nzeta_lin%d_%d:", i + 1, i + 1);
        print_r3(18, 13, zeta_lin);
        printf("\nzeta_ang%d_%d:", i + 1, i + 1);
        print_r3(18, 13, zeta_ang);
        printf("\nvd%d_%d:", i + 1, i + 1);
        print_r3(18, 13, vdh_h);
        printf("\nwd%d_%d:", i + 1, i + 1);
        print_r3(18, 13, wdh_h);
        printf("\nmu%d_%d:", i + 1, i + 1);
        print_r3(18, 13, muh_h);

        Psifi_i = compose_r3x10(muh_h, add_r3x3(skew_sym(wdh_h), Gh_h), zeros_3x6);
        Psini_i = compose_r3x10(zeros_3x1, transpose_r3x3(skew_sym(muh_h)), add_r3x6(uniq_mat(wdh_h), Dh_h));
        Psii_i[i] = set_screwx10(Psifi_i, Psini_i, SCR_FOR);
    }

    //Y = Ji_bar'*Psii_bar;
    Y = compute_regressor(Ji_ij, Psii_i);

    s = sub_rn(qd, qRd);

    sigma_bar = mul_transpose_rnxn0_and_rn(Y, s);

    Pdiag_bar = set_rn0_from_r10s(Pdiag);
    p_bar_new = integrate_rn0(set_rn0_from_r10s(p), mul_rn0_elem_wise(Pdiag_bar, sigma_bar), -_sd.cycle_time);

    for (i = 0; i < N_DOF; i++)
        p[i] = set_r10_from_rn0_part(p_bar_new, i);

    tau_f = mul_rnxn0_and_rn0(Y, p_bar_new);

    return tau_f;
}

rn mtrx_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF])
{
    int i, j;
    r3 rj_ji[N_DOF][N_DOF] = { 0.0 }, vh_h[N_DOF + 1] = { 0.0 }, wh_h[N_DOF + 1] = { 0.0 }
        , vRh_h[N_DOF + 1] = { 0.0 }, wRh_h[N_DOF + 1] = { 0.0 }, vdh_h = { 0.0 }
        , wdh_h = { 0.0 }, gh = { 0.0 }, muh_h = { 0.0 }, zeros_3x1 = { 0.0 };
    r3x3 Rh_i[N_DOF + 1] = { 0.0 }, Ri_h[N_DOF + 1] = { 0.0 }, Ri_j[N_DOF][N_DOF] = { 0.0 }
        , Gh_h = { 0.0 }, zeros_3x3 = { 0.0 };
    r3x6 Dh_h = { 0.0 }, zeros_3x6 = { 0.0 };
    r3x10 Psifi_i = { 0.0 }, Psini_i = { 0.0 };
    rn s = { 0.0 }, tau_f = { 0.0 };
    rnxn0 Y = { 0.0 };
    screw Ji_ij[N_DOF][N_DOF] = { 0.0 }, Jdi_ij[N_DOF][N_DOF] = { 0.0 }, JRdi_ij[N_DOF][N_DOF] = { 0.0 }
    , vwh_h = { 0.0 }, vwRh_h = { 0.0 }, vwdh_h= { 0.0 }, zeta = { 0.0 };
    screwxn Ji_i[N_DOF] = { 0.0 }, Jdi_i = { 0.0 }, JRdi_i = { 0.0 };
    screwx10 Psii_i[N_DOF] = { 0.0 };
    st Xi_ij = { 0.0 }, Xdi_ij = { 0.0 }, XRdi_ij = { 0.0 };
    rn0 p_bar_new = { 0.0 }, sigma_bar = { 0.0 }, Pdiag_bar = { 0.0 };

    for (i = 0; i < N_DOF; i++)
    {
        Rh_i[i] = get_Rh_i(Rh_i[i], i, q.data[i]);
        Ri_h[i] = transpose_r3x3(Rh_i[i]);
    }

    for (i = N_DOF - 1; i >= 0; i--)
    {
        for (j = i; j >= 0; j--)
        {
            if (j == i)
                Ri_j[i][j] = get_r3x3_idnt();
            else
            {
                Ri_j[i][j] = mul_r3x3(Ri_j[i][j + 1], Ri_h[j + 1]);
                if (j == i - 1)
                    rj_ji[j][i] = _kd.rh_h_i[j + 1];
                else
                    rj_ji[j][i] = add_r3(mul_r3x3_and_r3(Rh_i[j + 1], rj_ji[j + 1][i]), _kd.rh_h_i[j + 1]);
            }
        }
    }

    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j <= i; j++)
        {
            Xi_ij = set_st(Ri_j[i][j], mul_r3x3(Ri_j[i][j], transpose_r3x3(skew_sym(rj_ji[j][i]))), zeros_3x3, Ri_j[i][j]);
            Ji_ij[j][i] = mul_st_and_screw(Xi_ij, _kd.zzi_i);
        }
    }

    for (i = 0; i < N_DOF; i++)
    {
        Ji_i[i] = compose_screwxnxn(Ji_ij, i);
        vwh_h = mul_screwxn_and_rn(Ji_i[i], qd);
        vh_h[i + 1] = vwh_h.lin;
        wh_h[i + 1] = vwh_h.ang;
        vwRh_h = mul_screwxn_and_rn(Ji_i[i], qRd);
        vRh_h[i + 1] = vwRh_h.lin;
        wRh_h[i + 1] = vwRh_h.ang;
        //printf("\nv%d_%d:", i + 1, i + 1);
        //print_r3(18, 13, vh_h[i + 1]);
        //printf("\nw%d_%d:", i + 1, i + 1);
        //print_r3(18, 13, wh_h[i + 1]);
        //printf("\nvR%d_%d:", i + 1, i + 1);
        //print_r3(18, 13, vRh_h[i + 1]);
        //printf("\nwR%d_%d:", i + 1, i + 1);
        //print_r3(18, 13, wRh_h[i + 1]);
    }

    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j <= i; j++)
        {
            Xdi_ij = set_st(mul_r3x3(Ri_j[i][j], skew_sym(wh_h[j + 1]))
                , mul_r3x3(Ri_j[i][j], add_r3x3(mul_r3x3(transpose_r3x3(skew_sym(rj_ji[j][i])), skew_sym(wh_h[j + 1])), transpose_r3x3(skew_sym(sub_r3(mul_r3x3_and_r3(transpose_r3x3(Ri_j[i][j]), vh_h[i + 1]), vh_h[j + 1])))))
                , zeros_3x3, mul_r3x3(Ri_j[i][j], skew_sym(wh_h[j + 1])));
            Jdi_ij[j][i] = mul_st_and_screw(Xdi_ij, _kd.zzi_i);
            XRdi_ij = set_st(mul_r3x3(Ri_j[i][j], skew_sym(wRh_h[j + 1]))
                , mul_r3x3(Ri_j[i][j], add_r3x3(mul_r3x3(transpose_r3x3(skew_sym(rj_ji[j][i])), skew_sym(wRh_h[j + 1])), transpose_r3x3(skew_sym(sub_r3(mul_r3x3_and_r3(transpose_r3x3(Ri_j[i][j]), vRh_h[i + 1]), vRh_h[j + 1])))))
                , zeros_3x3, mul_r3x3(Ri_j[i][j], skew_sym(wRh_h[j + 1])));
            JRdi_ij[j][i] = mul_st_and_screw(XRdi_ij, _kd.zzi_i);
        }
    }

    gh = g;
    muh_h = mul_double_and_r3(-1.0, gh);
    for (i = 0; i < N_DOF; i++)
    {
        Jdi_i = compose_screwxnxn(Jdi_ij, i);
        JRdi_i = compose_screwxnxn(JRdi_ij, i);
        if (_cd.passImp == PASS_A)
        {
            zeta = mul_screwxn_and_rn(JRdi_i, qd);
            Gh_h = mul_r3x3(skew_sym(wh_h[i + 1]), skew_sym(wRh_h[i + 1]));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wRh_h[i + 1]), uniq_mat(wh_h[i + 1]));
        }
        else if (_cd.passImp == PASS_B)
        {
            zeta = mul_screwxn_and_rn(Jdi_i, qRd);
            Gh_h = mul_r3x3(skew_sym(wRh_h[i + 1]), skew_sym(wh_h[i + 1]));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wh_h[i + 1]), uniq_mat(wRh_h[i + 1]));
        }
        else // (_cd.passImp == PASS_C)
        {
            zeta = avg_screw(mul_screwxn_and_rn(JRdi_i, qd), mul_screwxn_and_rn(Jdi_i, qRd));
            Gh_h = avg_r3x3(mul_r3x3(skew_sym(wh_h[i + 1]), skew_sym(wRh_h[i + 1])), mul_r3x3(skew_sym(wRh_h[i + 1]), skew_sym(wh_h[i + 1])));
            Dh_h = avg_r3x6(mul_r3x3_and_r3x6(skew_sym(wRh_h[i + 1]), uniq_mat(wh_h[i + 1])), mul_r3x3_and_r3x6(skew_sym(wh_h[i + 1]), uniq_mat(wRh_h[i + 1])));
        }

        vwdh_h = add_screw(mul_screwxn_and_rn(Ji_i[i], qRdd), zeta);
        vdh_h = vwdh_h.lin;
        wdh_h = vwdh_h.ang;
        gh = mul_r3x3_and_r3(Ri_h[i], gh);
        muh_h = sub_r3(vdh_h, gh);

        printf("\nzeta_lin%d_%d:", i + 1, i + 1);
        print_r3(18, 13, zeta.lin);
        printf("\nzeta_ang%d_%d:", i + 1, i + 1);
        print_r3(18, 13, zeta.ang);
        printf("\nvd%d_%d:", i + 1, i + 1);
        print_r3(18, 13, vdh_h);
        printf("\nwd%d_%d:", i + 1, i + 1);
        print_r3(18, 13, wdh_h);
        printf("\nmu%d_%d:", i + 1, i + 1);
        print_r3(18, 13, muh_h);

        Psifi_i = compose_r3x10(muh_h, add_r3x3(skew_sym(wdh_h), Gh_h), zeros_3x6);
        Psini_i = compose_r3x10(zeros_3x1, transpose_r3x3(skew_sym(muh_h)), add_r3x6(uniq_mat(wdh_h), Dh_h));
        Psii_i[i] = set_screwx10(Psifi_i, Psini_i, SCR_FOR);
    }

    //Y = Ji_bar'*Psii_bar;
    Y = compute_regressor(Ji_ij, Psii_i);

    s = sub_rn(qd, qRd);

    sigma_bar = mul_transpose_rnxn0_and_rn(Y, s);

    Pdiag_bar = set_rn0_from_r10s(Pdiag);
    p_bar_new = integrate_rn0(set_rn0_from_r10s(p), mul_rn0_elem_wise(Pdiag_bar, sigma_bar), -_sd.cycle_time);

    for (i = 0; i < N_DOF; i++)
        p[i] = set_r10_from_rn0_part(p_bar_new, i);

    tau_f = mul_rnxn0_and_rn0(Y, p_bar_new);

    return tau_f;
}

rn kwsk_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF])
{
    int i, j;
    r3 rj_ij = { 0.0 }, Jv = { 0.0 }, Jw = { 0.0 }, zeros_3x1 = { 0.0 }, upsi_i = { 0.0 }
        , wh_h = { 0.0 }, wRh_h = { 0.0 }, muh_h = { 0.0 }, wdh_h = { 0.0 };
    rn s = { 0.0 }, tau_f = { 0.0 };
    r10 yT_ij[N_DOF][N_DOF] = { 0.0 }, sigma_i[N_DOF] = { 0.0 };
    r3x3 Rh_i[N_DOF + 1] = { 0.0 }, Ri_h[N_DOF + 1] = { 0.0 }
        , Gh_h = { 0.0 }, SSSh_h = { 0.0 };
    r3x6 Dh_h = { 0.0 }, zeros_3x6 = { 0.0 };
    r3x10 Psifi_i[N_DOF] = { 0.0 }, Psini_i[N_DOF] = { 0.0 };

    muh_h = mul_double_and_r3(-1.0, g);
    for (i = 0; i < N_DOF; i++)
    {
        Rh_i[i] = get_Rh_i(Rh_i[i], i, q.data[i]);
        Ri_h[i] = transpose_r3x3(Rh_i[i]);

        if (_cd.passImp == PASS_A)
            upsi_i = cross_prod(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qd.data[i], _kd.zi_i));
        else if (_cd.passImp == PASS_B)
            upsi_i = cross_prod(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i));
        else // (_cd.passImp == PASS_C)
            upsi_i = avg_r3(cross_prod(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qd.data[i], _kd.zi_i)), cross_prod(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i)));

        wh_h = add_r3(mul_r3x3_and_r3(Ri_h[i], wh_h), mul_double_and_r3(qd.data[i], _kd.zi_i));
        wRh_h = add_r3(mul_r3x3_and_r3(Ri_h[i], wRh_h), mul_double_and_r3(qRd.data[i], _kd.zi_i));

        if (_cd.passImp == PASS_A)
        {
            Gh_h = mul_r3x3(skew_sym(wh_h), skew_sym(wRh_h));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wRh_h), uniq_mat(wh_h));
        }
        else if (_cd.passImp == PASS_B)
        {
            Gh_h = mul_r3x3(skew_sym(wRh_h), skew_sym(wh_h));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wh_h), uniq_mat(wRh_h));
        }
        else // (_cd.passImp == PASS_C)
        {
            Gh_h = avg_r3x3(mul_r3x3(skew_sym(wh_h), skew_sym(wRh_h)), mul_r3x3(skew_sym(wRh_h), skew_sym(wh_h)));
            Dh_h = avg_r3x6(mul_r3x3_and_r3x6(skew_sym(wRh_h), uniq_mat(wh_h)), mul_r3x3_and_r3x6(skew_sym(wh_h), uniq_mat(wRh_h)));
        }

        wdh_h = add_3_r3(mul_r3x3_and_r3(Ri_h[i], wdh_h), mul_double_and_r3(qRdd.data[i], _kd.zi_i), upsi_i);
        muh_h = mul_r3x3_and_r3(Ri_h[i], add_r3(muh_h, mul_r3x3_and_r3(SSSh_h, _kd.rh_h_i[i])));

        SSSh_h = add_r3x3(skew_sym(wdh_h), Gh_h);

        Psifi_i[i] = compose_r3x10(muh_h, SSSh_h, zeros_3x6);
        Psini_i[i] = compose_r3x10(zeros_3x1, transpose_r3x3(skew_sym(muh_h)), add_r3x6(uniq_mat(wdh_h), Dh_h));
    }

    s = sub_rn(qd, qRd);

    for (i = 0; i < N_DOF; i++)
    {
        for (j = i; j < N_DOF; j++)
        {
            if (j == i)
            {
                rj_ij = get_r3_zero();
                Jw = _kd.zi_i;
            }
            else
            {
                rj_ij = mul_r3x3_and_r3(Ri_h[j], add_r3(rj_ij, _kd.rh_h_i[j]));
                Jw = mul_r3x3_and_r3(Ri_h[j], Jw);
            }
            Jv = cross_prod(Jw, rj_ij);

            yT_ij[i][j] = add_r10(mul_transpose_r3x10_and_r3(Psifi_i[j], Jv)
                , mul_transpose_r3x10_and_r3(Psini_i[j], Jw));
        }
    }

    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j <= i; j++)
        {
            sigma_i[i] = add_r10(sigma_i[i], mul_double_and_r10(s.data[j], yT_ij[j][i]));
        }

        p[i] = integrate_r10(p[i], mul_r10_elem_wise(Pdiag[i], sigma_i[i]), -_sd.cycle_time);
    }

    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j < N_DOF; j++)
            tau_f.data[i] = tau_f.data[i] + mul_transpose_r10_and_r10(yT_ij[i][j], p[j]);
    }
    
    return tau_f;
}

rn yuan_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF])
{
    int i, j;
    r1 kappa = { 0.0 };
    r3 rj_ji[N_DOF][N_DOF] = { 0.0 }, vh_h[N_DOF + 1] = { 0.0 }, wh_h[N_DOF + 1] = { 0.0 }
        , vRh_h[N_DOF + 1] = { 0.0 }, wRh_h[N_DOF + 1] = { 0.0 }, vdh_h = { 0.0 }
        , wdh_h = { 0.0 }, gh = { 0.0 }, muh_h = { 0.0 }, zeros_3x1 = { 0.0 }, beta = { 0.0 };
    r3x3 Rh_i[N_DOF + 1] = { 0.0 }, Ri_h[N_DOF + 1] = { 0.0 }, Ri_j[N_DOF][N_DOF] = { 0.0 }, Gh_h = { 0.0 };
    r3x6 Dh_h = { 0.0 }, zeros_3x6 = { 0.0 };
    r3x10 Psifi_i = { 0.0 }, Psini_i = { 0.0 };
    r6 alpha = { 0.0 };
    rn s = { 0.0 }, tau_f = { 0.0 };
    r10 sigma_i = { 0.0 };
    screw Ji_ij[N_DOF][N_DOF] = { 0.0 }, Jdi_ij[N_DOF][N_DOF] = { 0.0 }, JRdi_ij[N_DOF][N_DOF] = { 0.0 }
        , vwh_h = { 0.0 }, vwRh_h = { 0.0 }, vwdh_h = { 0.0 }, zeta = { 0.0 }, svwi_i = { 0.0 };
    screwxn Ji_i[N_DOF] = { 0.0 }, Jdi_i = { 0.0 }, JRdi_i = { 0.0 };
    screwx10 Psii_i = { 0.0 };

    for (i = 0; i < N_DOF; i++)
    {
        Rh_i[i] = get_Rh_i(Rh_i[i], i, q.data[i]);
        Ri_h[i] = transpose_r3x3(Rh_i[i]);
    }

    for (i = N_DOF - 1; i >= 0; i--)
    {
        for (j = i; j >= 0; j--)
        {
            if (j == i)
                Ri_j[i][j] = get_r3x3_idnt();
            else
            {
                Ri_j[i][j] = mul_r3x3(Ri_j[i][j + 1], Ri_h[j + 1]);
                if (j == i - 1)
                    rj_ji[j][i] = _kd.rh_h_i[j + 1];
                else
                    rj_ji[j][i] = add_r3(mul_r3x3_and_r3(Rh_i[j + 1], rj_ji[j + 1][i]), _kd.rh_h_i[j + 1]);
            }
        }
    }

    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j <= i; j++)
            Ji_ij[j][i] = set_screw( mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(transpose_r3x3(skew_sym(rj_ji[j][i])),_kd.zi_i))
                                   , mul_r3x3_and_r3(Ri_j[i][j], _kd.zi_i));
    }

    for (i = 0; i < N_DOF; i++)
    {
        Ji_i[i] = compose_screwxnxn(Ji_ij, i);
        vwh_h = mul_screwxn_and_rn(Ji_i[i], qd);
        vh_h[i + 1] = vwh_h.lin;
        wh_h[i + 1] = vwh_h.ang;
        vwRh_h = mul_screwxn_and_rn(Ji_i[i], qRd);
        vRh_h[i + 1] = vwRh_h.lin;
        wRh_h[i + 1] = vwRh_h.ang;
    }
    
    for (i = 0; i < N_DOF; i++)
    {
        for (j = 0; j <= i; j++)
        {
            Jdi_ij[j][i] = set_screw(mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(add_r3x3(mul_r3x3(transpose_r3x3(skew_sym(rj_ji[j][i]))
                                        , skew_sym(wh_h[j + 1])), transpose_r3x3(skew_sym(sub_r3(mul_r3x3_and_r3(transpose_r3x3(Ri_j[i][j])
                                            , vh_h[i + 1]), vh_h[j + 1])))), _kd.zi_i))
                                    , mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(skew_sym(wh_h[j + 1]), _kd.zi_i)));
            JRdi_ij[j][i] = set_screw(mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(add_r3x3(mul_r3x3(transpose_r3x3(skew_sym(rj_ji[j][i]))
                                        , skew_sym(wRh_h[j + 1])), transpose_r3x3(skew_sym(sub_r3(mul_r3x3_and_r3(transpose_r3x3(Ri_j[i][j])
                                            , vRh_h[i + 1]), vRh_h[j + 1])))), _kd.zi_i))
                                    , mul_r3x3_and_r3(Ri_j[i][j], mul_r3x3_and_r3(skew_sym(wRh_h[j + 1]), _kd.zi_i)));
        }
    }

    s = sub_rn(qd, qRd);

    gh = g;
    muh_h = mul_double_and_r3(-1.0, gh);
    for (i = 0; i < N_DOF; i++)
    {
        Jdi_i = compose_screwxnxn(Jdi_ij, i);
        JRdi_i = compose_screwxnxn(JRdi_ij, i);
        if (_cd.passImp == PASS_A)
        {
            zeta = mul_screwxn_and_rn(JRdi_i, qd);
            Gh_h = mul_r3x3(skew_sym(wh_h[i + 1]), skew_sym(wRh_h[i + 1]));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wRh_h[i + 1]), uniq_mat(wh_h[i + 1]));
        }
        else if (_cd.passImp == PASS_B)
        {
            zeta = mul_screwxn_and_rn(Jdi_i, qRd);
            Gh_h = mul_r3x3(skew_sym(wRh_h[i + 1]), skew_sym(wh_h[i + 1]));
            Dh_h = mul_r3x3_and_r3x6(skew_sym(wh_h[i + 1]), uniq_mat(wRh_h[i + 1]));
        }
        else // (_cd.passImp == PASS_C)
        {
            zeta = avg_screw(mul_screwxn_and_rn(JRdi_i, qd), mul_screwxn_and_rn(Jdi_i, qRd));
            Gh_h = avg_r3x3(mul_r3x3(skew_sym(wh_h[i + 1]), skew_sym(wRh_h[i + 1])), mul_r3x3(skew_sym(wRh_h[i + 1]), skew_sym(wh_h[i + 1])));
            Dh_h = avg_r3x6(mul_r3x3_and_r3x6(skew_sym(wRh_h[i + 1]), uniq_mat(wh_h[i + 1])), mul_r3x3_and_r3x6(skew_sym(wh_h[i + 1]), uniq_mat(wRh_h[i + 1])));
        }

        vwdh_h = add_screw(mul_screwxn_and_rn(Ji_i[i], qRdd), zeta);
        vdh_h = vwdh_h.lin;
        wdh_h = vwdh_h.ang;
        gh = mul_r3x3_and_r3(Ri_h[i], gh);
        muh_h = sub_r3(vdh_h, gh);

        Psifi_i = compose_r3x10(muh_h, add_r3x3(skew_sym(wdh_h), Gh_h), zeros_3x6);
        Psini_i = compose_r3x10(zeros_3x1, transpose_r3x3(skew_sym(muh_h)), add_r3x6(uniq_mat(wdh_h), Dh_h));
        Psii_i = set_screwx10(Psifi_i, Psini_i, SCR_FOR);

        svwi_i = mul_screwxn_and_rn(Ji_i[i], s);

        // kappa, beta, alpha
        kappa = mul_transpose_r3_and_r3(muh_h, svwi_i.lin);
        beta = add_r3(mul_r3x3_and_r3(transpose_r3x3(add_r3x3(skew_sym(wdh_h), Gh_h)), svwi_i.lin)
                    , mul_r3x3_and_r3(skew_sym(muh_h), svwi_i.ang));
        alpha = mul_transpose_r3x6_r3(add_r3x6(uniq_mat(wdh_h), Dh_h), svwi_i.ang);

        //sigma_i = mul_transpose_screwx10_and_screw(Psii_i, svwi_i);
        sigma_i = compose_r10(kappa, beta, alpha);
        //p[i] = sub_r10(p[i], mul_r10_elem_wise(Pdiag[i], mul_double_and_r10(_sd.cycle_time, sigma_i)));
        p[i] = integrate_r10(p[i], mul_r10_elem_wise(Pdiag[i], sigma_i), -_sd.cycle_time);

        tau_f = add_rn(tau_f, mul_transpose_screwxn_and_screw(Ji_i[i], mul_screwx10_and_r10(Psii_i, p[i])));
    }

    return tau_f;
}

rn mnea_func(const rn q, const rn qd, const rn qRd, const rn qRdd, const r3 g, r10 p[N_DOF], const r10 Pdiag[N_DOF])
{
    int i, j, k;
    rn s = { 0.0 };
    r10 w[N_DOF] = { 0.0 };
    rn0 p_bar_new = { 0.0 }, sigma_bar = { 0.0 }, Pdiag_bar = { 0.0 };
    rnxn0 Y = { 0.0 };
    rn tau_f = { 0.0 }, Y_j[N_DOF * 10] = { 0.0 };

    for (i = 0; i < N_DOF * 10; i++)
    {
        j = i / 10;
        k = i % 10;

        if (i != 0 && k != 0)
            w[j].data[k - 1] = 0.0;
        if (j > 0 && k == 0)
            w[j - 1].data[9] = 0.0;
        w[i / 10].data[i % 10] = 1.0;

        Y_j[i] = mnea(q, qd, qRd, qRdd, g, w);
    }

    Y = set_rnxn0_from_rns(Y_j);

    s = sub_rn(qd, qRd);
    sigma_bar = mul_transpose_rnxn0_and_rn(Y, s);

    Pdiag_bar = set_rn0_from_r10s(Pdiag);
    //p_bar_new = sub_rn0(set_rn0_from_r10s(p), mul_rn0_elem_wise(Pdiag_bar, mul_double_and_rn0(_sd.cycle_time, sigma_bar)));
    p_bar_new = integrate_rn0(set_rn0_from_r10s(p), mul_rn0_elem_wise(Pdiag_bar, sigma_bar), -_sd.cycle_time);

    for (i = 0; i < N_DOF; i++)
        p[i] = set_r10_from_rn0_part(p_bar_new, i);

    //tau_f = mul_rnxn0_and_rn0(Y, p_bar_new);

    tau_f = mnea(q, qd, qRd, qRdd, g, p);

    return tau_f;
}
