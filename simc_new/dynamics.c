//*********************************************************************
// ~NAME : dynamics.c
// ~INSTITUTION : ITU
// ~DESCRIPTION : Dynamic properties
//**********************************************************************

#include <math.h>

#include "dynamics.h"
#include "test.h"
#include "functions.h"
#include "kinematics.h"
#include "datatypes.h"
#include "algo.h"

//#include <stdio.h>

// ----------------------------------------------------------------
// Joint Friction Model Parameters
// ----------------------------------------------------------------

const double vis[7] = { 111.2, 87.28, 36.49, 9.596, 15.42, 0.1652, 6.564 };
const double cou[7] = { 35.98, 55.28, 27.51, 7.647, 4.747, 0.5175, 2.805 };
const double off[7] = { 0.9811, -4.794, -7.222, 0.01566, 0.7782, 0.007565, -0.2884 };
const double neg[7] = { 461.2, 688.0, 383.8, 85.91, 55.11, 5.265, 37.50 };
const double pos[7] = { 480.8, 592.1, 239.4, 86.22, 70.67, 5.416, 31.73 };

// ----------------------------------------------------------------
// Spring Model Parameters
// ----------------------------------------------------------------

// Constant parameters for balance mechanism (spring) at joint 2
const double r = 0.0800;						// eccentric misalignment radius (m)
const double L = 0.7280;						// lifting beam distance (m)

const double k_rx160 = 23950;					// spring stiffness (N/m)
const double Pc_rx160 = 5314;					// spring preload (N)
//const double a0_rx160 = -0.8458 * _DEG_TO_RAD;	// mount angle (rad)
const double a0_rx160 = -0.014762;	// mount angle (rad)

const double k_rx160l = 19690;					// spring stiffness (N/m)
const double Pc_rx160l = 4897;					// spring preload (N)
//const double a0_rx160l = -1.186 * _DEG_TO_RAD;	// mount angle (rad)
const double a0_rx160l = -0.0206996;	// mount angle (rad)

// ----------------------------------------------------------------
// getFrcTorque
// ----------------------------------------------------------------
rn getFrcTorque(const rn q_vel)
{
    unsigned int i;
    double tau[N_DOF + 1], vel[N_DOF + 1];
    rn tau_frc = { 0.0 };

    vel[0] = q_vel.data[0];
    vel[1] = q_vel.data[1];
    vel[2] = q_vel.data[2];
    vel[3] = q_vel.data[3];
    vel[4] = q_vel.data[4];
    vel[5] = q_vel.data[5];
    vel[6] = q_vel.data[4] + q_vel.data[5];

    for (i = 0; i < N_DOF + 1; i++)
    {
        if (vel[i] > -1e-1 && vel[i] <= 0)
            tau[i] = neg[i] * vel[i];
        else if(vel[i] > 0 && vel[i] <= 1e-1)
            tau[i] = pos[i] * vel[i];
        else
            tau[i] = vis[i] * vel[i] + cou[i] * sgn(vel[i]) + off[i];
    }

    tau_frc.data[0] = tau[0];
    tau_frc.data[1] = tau[1];
    tau_frc.data[2] = tau[2];
    tau_frc.data[3] = tau[3];
    tau_frc.data[4] = tau[4] + tau[6];
    tau_frc.data[5] = tau[5] + tau[6];

    return tau_frc;
}

// ----------------------------------------------------------------
// getSprTorque
// ----------------------------------------------------------------
rn getSprTorque(const rn q_pos)
{
    double x;
    rn tau_spr = { 0.0 };

    if (_rd.robot_model == RX160)
    {
        x = sqrt(pow(r, 2) + pow((r + L), 2) - 2 * r * (r + L) * cos(a0_rx160 + q_pos.data[1])) - L;
        tau_spr.data[1] = (k_rx160 * x + Pc_rx160) * ((L + r) * r * sin(a0_rx160 + q_pos.data[1])) / (L + x);
    }
    else
    {
        x = sqrt(pow(r, 2) + pow((r + L), 2) - 2 * r * (r + L) * cos(a0_rx160l + q_pos.data[1])) - L;
        tau_spr.data[1] = (k_rx160l * x + Pc_rx160l) * ((L + r) * r * sin(a0_rx160l + q_pos.data[1])) / (L + x);
    }

    return tau_spr;
}

// ----------------------------------------------------------------
// initDynPara
// ----------------------------------------------------------------
void initDynPara(void)
{
    unsigned int i;
    double sixth_link_mass, outter_sensor_mass, outter_adapter_mass, inner_sensor_mass, gripper_mass, ball_spring_mass;
    r3 sixth_link_CoM, sensor_CoM_6F, tool_CoM_sF;

    if (_rd.ee_attachments == NO_EE) // no sensor or tool attached
    {
        outter_sensor_mass = 0.0;

        sensor_CoM_6F = get_r3_zero();

        _dd.tool_mass = 0.0;
        tool_CoM_sF = get_r3_zero();
        _dd.tool_CoM_6F = get_r3_zero();
    }
    else // sensor attached alone or with tool
    {
        // masses (kg)
        outter_sensor_mass = 0.918 - 0.114; // sensor's full mass minus its inner mass (including inner plate mass)
        outter_adapter_mass = 0.253;
        inner_sensor_mass = 0.114;
        gripper_mass = 1.120;
        ball_spring_mass = 1.203;

        // sensor(outter) center of mass in sixth frame
        sensor_CoM_6F = set_r3(0.0, 0.0, 0.0136);

        if (_rd.ee_attachments == GRIPPER)
        {
            _dd.tool_mass = inner_sensor_mass + outter_adapter_mass + gripper_mass;
            tool_CoM_sF = set_r3(0.0117, 0.0097, 0.0622);
        }
        else if (_rd.ee_attachments == BALL_SPRING)
        {
            _dd.tool_mass = inner_sensor_mass + outter_adapter_mass + ball_spring_mass;
            tool_CoM_sF = set_r3(0.0076, 0.0104, 0.0815);
        }
        else
        {
            _dd.tool_mass = inner_sensor_mass + outter_adapter_mass;
            tool_CoM_sF = set_r3(0.0, 0.0, -0.005);
        }

        _dd.tool_CoM_6F = add_r3(rotateOnAnAxis(tool_CoM_sF, Z_AXIS, -_kd.sensor_mount_ang), set_r3(0.0, 0.0, _kd.sensor_length));
    }

    // These parameters are taken from https://github.com/ros-industrial/staubli

    // Masses
    sixth_link_mass = 0.038484;

    _dd.m_i[0] = set_r1(45.357763);
    _dd.m_i[1] = set_r1(51.266495);
    _dd.m_i[2] = set_r1(19.754401);
    if (_rd.robot_model == RX160)
		_dd.m_i[3] = set_r1(15.287896);
    else
        _dd.m_i[3] = set_r1(27.640019);
    _dd.m_i[4] = set_r1(0.548088);
    _dd.m_i[5] = set_r1(sixth_link_mass + outter_sensor_mass + _dd.tool_mass);

    // CoMs
    sixth_link_CoM = set_r3(-0.000245, 0.000000, -0.007626);

    _dd.ri_i_ci[0] = set_r3(0.085479, -0.002547, -0.040488);
	_dd.ri_i_ci[1] = set_r3(-0.000002, 0.264164, 0.347704);
    _dd.ri_i_ci[2] = set_r3(-0.000255, 0.016478, -0.003819);
    if (_rd.robot_model == RX160)
        _dd.ri_i_ci[3] = set_r3(-0.015515, 0.000164, 0.340348);
    else
        _dd.ri_i_ci[3] = set_r3(-0.009602, 0.000028, 0.472641);
	_dd.ri_i_ci[4] = set_r3(0.000000, -0.000347, 0.023671);
    _dd.ri_i_ci[5] = div_r3_by_double(add_3_r3(mul_double_and_r3(sixth_link_mass, sixth_link_CoM), mul_double_and_r3(outter_sensor_mass, sensor_CoM_6F)
                                            , mul_double_and_r3(_dd.tool_mass, _dd.tool_CoM_6F)), sixth_link_mass + outter_sensor_mass + _dd.tool_mass);
        
    // Inertia tensors on CoM of links in joint frames
    _dd.Ii_ci[0] = set_r3x3sym( 0.839141,
                                0.022637, 1.030146,
                                0.162457, 0.027416, 1.064698);
    
    _dd.Ii_ci[1] = set_r3x3sym( 5.272800,
                                0.000010, 5.536809,
                                0.000017, -0.008261, 0.486191);

    _dd.Ii_ci[2] = set_r3x3sym( 0.249388,
                                -0.004901, 0.211780,
                                0.004825, 0.000574, 0.238488);

    if (_rd.robot_model == RX160)
    {
        _dd.Ii_ci[3] = set_r3x3sym( 0.331593,
                                    0.000055, 0.320895,
                                    -0.010688, 0.000058, 0.086557);
    }
    else
    {
        _dd.Ii_ci[3] = set_r3x3sym( 1.378080,
                                    0.000062, 1.366754,
                                    -0.042670, 0.000069, 0.170331);
    }
    
    _dd.Ii_ci[4] = set_r3x3sym( 0.000876,
                                0.000000, 0.000889,
                                0.000000, 0.000005, 0.000412);

    _dd.Ii_ci[5] = set_r3x3sym( 0.000011,
                                0.000000, 0.000011,
                                0.000000, 0.000000, 0.000021);

    // Standard inertial parameters
    for (i = 0; i < 6; i++)
    {
        _dd.di_i[i] = mul_r1_and_r3(_dd.m_i[i], _dd.ri_i_ci[i]);
        _dd.Ii_i[i] = sub_r3x3(_dd.Ii_ci[i], mul_r1_and_r3x3(_dd.m_i[i], mul_r3x3(skew_sym(_dd.ri_i_ci[i]), skew_sym(_dd.ri_i_ci[i]))));
    }

}

//// ----------------------------------------------------------------
//// Direct Dynamics: Newton Euler Method
//// ----------------------------------------------------------------
//void drcDynNE(double q_acc[6], const double tau_cmd[6], const double q_pos[6], const double q_vel[6])
//{
//    int i;
//    double tau[6] = { 0.0 }, H_i[6] = { 0.0 };
//    screw vvh_h = { 0.0 }, gammai_i[6] = { 0.0 }, betah_h[7] = { 0.0 }, beta2h_h[7] = { 0.0 }, alphai_i = { 0.0 }, aai_h = { 0.0 }, aah_h = { 0.0 };
//    st IIh_h[7] = { 0.0 }, II2h_h[7] = { 0.0 }, KKi_i = { 0.0 };
//
//    getFrcTorque(_dd.tau_frc, q_vel);
//    getSprTorque(_dd.tau_spr, q_pos);
//
//    // i) First forward recursive computations for i = 1, ..., n
//    for (i = 0; i <= 5; i++)
//    {
//        tau[i] = tau_cmd[i] - (_dd.tau_frc[i] + _dd.tau_spr[i]);
//
//        gammai_i[i] = set_screw(mul_r33_and_r3(transpose_r33(_kd.Rh_i[i]), cross_prod(vvh_h.ang, cross_prod(vvh_h.ang, _kd.ph_i[i])))
//            , cross_prod(mul_r33_and_r3(transpose_r33(_kd.Rh_i[i]), vvh_h.ang), mul_double_and_r3(q_vel[i], _kd.zi_i)));
//        vvh_h = add_screw(mul_st_and_screw(_kd.XXi_h[i], vvh_h), mul_double_and_screw(q_vel[i], _kd.zzi_i));
//        //ff_ei_i = set_screw(f_ei_i, m_ei_i);
//        betah_h[i + 1] = set_screw(mul_double_and_r3(-1.0, cross_prod(vvh_h.ang, cross_prod(vvh_h.ang, _dd.ci_i[i])))
//            , mul_double_and_r3(-1.0, cross_prod(vvh_h.ang, mul_r33_and_r3(_dd.Ii_i[i], vvh_h.ang)))); // -ff_ei_i
//        IIh_h[i + 1] = set_st(set_r33eye(_dd.m_i[i]), mul_double_and_r33(-1.0, skew_sym(_dd.ci_i[i])), skew_sym(_dd.ci_i[i]), _dd.Ii_i[i]);
//    }
//    
//    // ii) Backward recursive computations for i = n, ..., 1
//    II2h_h[6] = IIh_h[6];
//    beta2h_h[6] = betah_h[6];
//    for (i = 5; i >= 0; i--)
//    {
//        //H_i[i] = mul_quad_screw(II2h_h[i + 1], zzi_i);
//        H_i[i] = dot_prod_screw(_kd.zzi_i, mul_st_and_screw(II2h_h[i + 1], _kd.zzi_i));
//        //KKi_i = sub_st(II2h_h[i + 1], mul_double_and_st(1 / H_i[i], mul_quad_tr_st_and_screw(II2h_h[i + 1], zzi_i))); // something is a little off
//        KKi_i = sub_st(II2h_h[i + 1], mul_double_and_st((1.0 / H_i[i]), mul_st(II2h_h[i + 1], mul_st(mul_col_and_row_screw(_kd.zzi_i, _kd.zzi_i), II2h_h[i + 1]))));
//        alphai_i = sub_screw(add_screw(mul_st_and_screw(KKi_i, gammai_i[i]), mul_double_and_screw((1.0 / H_i[i]) * (tau[i] + dot_prod_screw(_kd.zzi_i, beta2h_h[i + 1])), mul_st_and_screw(II2h_h[i + 1], _kd.zzi_i))), beta2h_h[i + 1]);
//        beta2h_h[i] = sub_screw(betah_h[i], mul_st_and_screw(transpose_st(_kd.XXi_h[i]), alphai_i));
//        II2h_h[i] = add_st(IIh_h[i], mul_st(transpose_st(_kd.XXi_h[i]), mul_st(KKi_i, _kd.XXi_h[i])));
//    }
//
//    // iii) Second forward recursive computations for i = 1, ..., n
//    aah_h = set_screw(set_r3(0.0, 0.0, 9.81), set_r3(0.0, 0.0, 0.0));
//    for (i = 0; i <= 5; i++)
//    {
//        aai_h = mul_st_and_screw(_kd.XXi_h[i], aah_h);
//        q_acc[i] = (1.0 / H_i[i]) * (-dot_prod_screw(_kd.zzi_i, mul_st_and_screw(II2h_h[i + 1], add_screw(aai_h, gammai_i[i]))) + tau[i] + dot_prod_screw(_kd.zzi_i, beta2h_h[i + 1]));
//        aah_h = add_screw(aai_h, add_screw(mul_double_and_screw(q_acc[i], _kd.zzi_i), gammai_i[i]));
//    }
//
//}
