//*********************************************************************
// ~NAME : printdata.c
// ~INSTITUTION : ITU
// ~DESCRIPTION : Data export function
//**********************************************************************

#include <stdlib.h>
#include <stdio.h>

#include "printdata.h"
#include "test.h"
#include "algo.h"
#include "functions.h"
#include "movement.h"
#include "kinematics.h"
#include "dynamics.h"
#include "control.h"

#include <direct.h>
#include <string.h>

FILE *print_info;

FILE *q_posFbk, *q_velFbk;
FILE *q_torCmd, *q_torCtr, *q_torFrc, *q_torSpr;
FILE *x_posDes, *x_velDes, *x_accDes, *x_posFbk, *x_velFbk;

// ----------------------------------------------------------------
// printAlgoData
// ----------------------------------------------------------------
void printAlgoData(int testNb)
{
	int err;
	char root_dir[WIN_ROOT_CHAR_NB];
	char main_dir[WIN_ROOT_CHAR_NB + 7];
	char test_dir[WIN_ROOT_CHAR_NB + 33];
	char test_file[WIN_ROOT_CHAR_NB + 48];
	char date_time[21];

	time_t rawtime;
	time(&rawtime);
	localtime_s(&timeinfo, &rawtime);

	printf("\n");
	printf("Printing test #%d... \n", testNb);

	dateTimeForm(date_time, "%s_%s_%s__%s_%s_%s", timeinfo);

	char buff[256];

	// ----------------------------------------------------------------
	// Name of the root directory "D:\Documents"
	// ----------------------------------------------------------------
	sprintf_s(root_dir, WIN_ROOT_CHAR_NB, "D:/Documents");

	// ----------------------------------------------------------------
	// Create directory "D:\Documents\myData"
	// ----------------------------------------------------------------
	sprintf_s(main_dir, WIN_ROOT_CHAR_NB + 7, "%s/myData", root_dir);

	err = _mkdir(main_dir);

	if (err == 0)
	{
		printf("Creating directory at %s: success \n", main_dir);
	}
	else
	{
		strerror_s(buff, 100, errno);
		printf("Creating directory at %s: failed (", main_dir);
		fprintf(stderr, buff);
		printf(") \n");
	}

	// ----------------------------------------------------------------
	// Create directory "D:\Documents\myData\test_date__time)"
	// ----------------------------------------------------------------
	sprintf_s(test_dir, WIN_ROOT_CHAR_NB + 33, "%s/test_%s", main_dir, date_time);

	err = _mkdir(test_dir);

	if (err == 0)
	{
		printf("Creating directory at %s: success \n", test_dir);
	}
	else
	{
		strerror_s(buff, 100, errno);
		printf("Creating directory at %s: failed (", test_dir);
		fprintf(stderr, buff);
		printf(") \n");
	}

	// ----------------------------------------------------------------
	// Create test files "D:\Documents\myData\test_date__time\file_name.txt"
	// ----------------------------------------------------------------
	errno_t errs;

	sprintf_s(test_file, WIN_ROOT_CHAR_NB + 48, "%s/print_info.txt", test_dir);	errs = fopen_s(&print_info, test_file, "w");

	sprintf_s(test_file, WIN_ROOT_CHAR_NB + 48, "%s/q_posFbk.txt", test_dir); errs = fopen_s(&q_posFbk, test_file, "w");
	sprintf_s(test_file, WIN_ROOT_CHAR_NB + 48, "%s/q_velFbk.txt", test_dir); errs = fopen_s(&q_velFbk, test_file, "w");
	sprintf_s(test_file, WIN_ROOT_CHAR_NB + 48, "%s/q_torCmd.txt", test_dir); errs = fopen_s(&q_torCmd, test_file, "w");
	sprintf_s(test_file, WIN_ROOT_CHAR_NB + 48, "%s/q_torCtr.txt", test_dir); errs = fopen_s(&q_torCtr, test_file, "w");
	sprintf_s(test_file, WIN_ROOT_CHAR_NB + 48, "%s/q_torFrc.txt", test_dir); errs = fopen_s(&q_torFrc, test_file, "w");
	sprintf_s(test_file, WIN_ROOT_CHAR_NB + 48, "%s/q_torSpr.txt", test_dir); errs = fopen_s(&q_torSpr, test_file, "w");

	//--------------------------------------------------------------------//
	//--------------------------- RECORDING DATA -------------------------//
	//--------------------------------------------------------------------//

	printf("\nPrinting motion data...\n");
	printMotionInfo();
	printf("done!\n\t\t ");

	printf("\nPrinting motion data (%u cycle)...\n", _pd.cycleNb);
	printMotionData();
	printf("done!\n\n");

	printf("All data has been written!\t\t ");
}

// ----------------------------------------------------------------
// printMotionInfo
// ----------------------------------------------------------------
void printMotionInfo(void)
{
	fprintf(print_info, "data_name\tdata_value\n");
	fprintf(print_info, "cycle_time\t%f\n", (float)g_cycleTime);
	fprintf(print_info, "robot_model\t%d\n", _rd.robot_model);

	fprintf(print_info, "geo_path\t%d\n", _md.trjSpace);
	if (_md.trjSpace > 0)
	{
		if (_md.trjSpace == TASK_SPC)
			fprintf(print_info, "mot_frame\t%d\n", _md.motFrame);
		fprintf(print_info, "trj_type\t%d\n", _md.trjType);
		if (_md.trjType == PNT2PNT)
			fprintf(print_info, "p2p_type\t%d\n", _md.p2pType);
		if (_md.trjType == PARAMETRIC)
			fprintf(print_info, "para_type\t%d\n", _md.paraType);
	}

	fprintf(print_info, "rot_rprsnt\t%d\n", _kd.rotRepresent);
	if (_kd.rotRepresent == EULER_ANG)
		fprintf(print_info, "euler_set\t%d\n", _kd.eulerSet);
	fprintf(print_info, "ee_attachments\t%d\n", _rd.ee_attachments);
	if (_rd.ee_attachments != NO_EE)
	{
		fprintf(print_info, "tool_mass\t%f\n", (float)_dd.tool_mass);
		fprintf(print_info, "tool_com_sf1\t%f\n", (float)_dd.tool_CoM_6F.data[0]);
		fprintf(print_info, "tool_com_sf2\t%f\n", (float)_dd.tool_CoM_6F.data[1]);
		fprintf(print_info, "tool_com_sf3\t%f\n", (float)_dd.tool_CoM_6F.data[2]);
	}
	fprintf(print_info, "ctrl_mthd\t%d\n", _cd.ctrlMthd);
	fprintf(print_info, "ctrl_spc\t%d\n", _cd.ctrlSpc);
	fprintf(print_info, "ctrl_type\t%d\n", _cd.ctrlType);

	fprintf(print_info, "K_p_diag1\t%f\n", (float)_cd.K_pq_diag.data[0]);
	fprintf(print_info, "K_p_diag2\t%f\n", (float)_cd.K_pq_diag.data[1]);
	fprintf(print_info, "K_p_diag3\t%f\n", (float)_cd.K_pq_diag.data[2]);
	fprintf(print_info, "K_p_diag4\t%f\n", (float)_cd.K_pq_diag.data[3]);
	fprintf(print_info, "K_p_diag5\t%f\n", (float)_cd.K_pq_diag.data[4]);
	fprintf(print_info, "K_p_diag6\t%f\n", (float)_cd.K_pq_diag.data[5]);

	fprintf(print_info, "K_i_diag1\t%f\n", (float)_cd.K_iq_diag.data[0]);
	fprintf(print_info, "K_i_diag2\t%f\n", (float)_cd.K_iq_diag.data[1]);
	fprintf(print_info, "K_i_diag3\t%f\n", (float)_cd.K_iq_diag.data[2]);
	fprintf(print_info, "K_i_diag4\t%f\n", (float)_cd.K_iq_diag.data[3]);
	fprintf(print_info, "K_i_diag5\t%f\n", (float)_cd.K_iq_diag.data[4]);
	fprintf(print_info, "K_i_diag6\t%f\n", (float)_cd.K_iq_diag.data[5]);

	fprintf(print_info, "K_d_diag1\t%f\n", (float)_cd.K_dq_diag.data[0]);
	fprintf(print_info, "K_d_diag2\t%f\n", (float)_cd.K_dq_diag.data[1]);
	fprintf(print_info, "K_d_diag3\t%f\n", (float)_cd.K_dq_diag.data[2]);
	fprintf(print_info, "K_d_diag4\t%f\n", (float)_cd.K_dq_diag.data[3]);
	fprintf(print_info, "K_d_diag5\t%f\n", (float)_cd.K_dq_diag.data[4]);
	fprintf(print_info, "K_d_diag6\t%f\n", (float)_cd.K_dq_diag.data[5]);
			
	fprintf(print_info, "comp_cor\t%d\n", _cd.comp_cor);
	fprintf(print_info, "comp_grv\t%d\n", _cd.comp_grv);
	fprintf(print_info, "comp_frc\t%d\n", _cd.comp_frc);
	fprintf(print_info, "comp_spr\t%d\n", _cd.comp_spr);

	fclose(print_info);
}

// ----------------------------------------------------------------
// printMotionData
// ----------------------------------------------------------------
void printMotionData(void)
{
	unsigned int i, l_jnt;

	for (i = 0; i < _pd.cycleNb; i++)
	{
		for (l_jnt = 0; l_jnt < 6; l_jnt++)
		{
			fprintf(q_posFbk, "%f\t", _pd.q_posFbk[i][l_jnt]);
			fprintf(q_velFbk, "%f\t", _pd.q_velFbk[i][l_jnt]);
			fprintf(q_torCmd, "%f\t", _pd.q_torCmd[i][l_jnt]);
			fprintf(q_torCtr, "%f\t", _pd.q_torCtr[i][l_jnt]);
			fprintf(q_torFrc, "%f\t", _pd.q_torFrc[i][l_jnt]);
			fprintf(q_torSpr, "%f\t", _pd.q_torSpr[i][l_jnt]);
		}

		fprintf(q_posFbk, "\n");
		fprintf(q_velFbk, "\n");
		fprintf(q_torCmd, "\n");
		fprintf(q_torCtr, "\n");
		fprintf(q_torFrc, "\n");
		fprintf(q_torSpr, "\n");
	}
	fclose(q_posFbk);
	fclose(q_velFbk);
	fclose(q_torCmd);
	fclose(q_torCtr);
	fclose(q_torFrc);
	fclose(q_torSpr);
}

// ----------------------------------------------------------------
// recordData
// ----------------------------------------------------------------
void recordData(void)
{
	unsigned int l_jnt;

	if (_pd.rec == START_RECORD)
	{
		if (_pd.cycleNb < MAX_ARRAY_SAMPLE)
		{
			for (l_jnt = 0; l_jnt < 6; l_jnt++)
			{
				_pd.q_posFbk[_pd.cycleNb][l_jnt] = (float)_ad.q_posFbk.data[l_jnt];
				_pd.q_velFbk[_pd.cycleNb][l_jnt] = (float)_ad.q_velFbk.data[l_jnt];
				_pd.q_torCmd[_pd.cycleNb][l_jnt] = (float)_cd.tau_cmd.data[l_jnt];
				_pd.q_torCtr[_pd.cycleNb][l_jnt] = (float)_cd.tau_ctrl.data[l_jnt];
				_pd.q_torFrc[_pd.cycleNb][l_jnt] = (float)_dd.tau_frc.data[l_jnt];
				_pd.q_torSpr[_pd.cycleNb][l_jnt] = (float)_dd.tau_spr.data[l_jnt];
			}
			++_pd.cycleNb;
		}
	}
}
