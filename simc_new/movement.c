//*********************************************************************
// ~NAME : movement.c
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Available movement types for the robot
//**********************************************************************

#include <stdio.h>

#include "movement.h"
#include "functions.h"
#include "algo.h"
#include "printdata.h"
#include "test.h"

int moveConfirmation(void)
{
	char l_choice = 0;

	printf("\nStart?[y/n] ");
	l_choice = inputMenu();
	printf("\n\n");

	if ((l_choice == 'Y') || (l_choice == 'y'))
	{
		printf("\nMotion has started.\n");

		_pd.unprinted_test = true;
		/*if (_md.geoPath > 0)
			_md.no_trajectory = 0;
		else
			_md.no_trajectory = 1;

		_ad.no_trj = _md.no_trajectory;
		_ad.m_time_final = _md.tf;
		_ad.m_move = 1;*/
		return 1;
	}
	else
	{
		printf("\nMotion canceled!");
		return -1;
	}
}

// ----------------------------------------------------------------
// move
// ----------------------------------------------------------------
void move(void)
{
	int	l_confirm = 0;

	l_confirm = moveConfirmation();

	if (l_confirm == 1)
	{
		initAlgo();
		computeAlgo(_sd.cycle_time, _sd.sample_time, _sd.stop_time);
	}
	else
	{
		return;
	}
}

// ----------------------------------------------------------------
// _noTrjMenu
// ----------------------------------------------------------------
void _noTrjMenu(void)
{
	char l_choice;

	// Joint space trajectory selection
	printf("..\\MOVEMENT MENU\\NO TRAJECTORY MENU> \n");


	printf("\n");
	printf("\n\tCyclele time: %6.4f sec\n", _sd.cycle_time);
	printf("\n\tSample time: %6.4f sec\n", _sd.sample_time);
	printf("\n\tStop time: %6.4f sec\n", _sd.stop_time);
	printf("\n");

	printf("\n\tWhich selection do you want to choose?\n\n");
	printf("\t<--\t 0:\tBack to previous menu \n");
	printf("\t\t 1:\tSet sample time \n");
	printf("\t\t 2:\tSet stop time \n");
	printf("\n");
	printf("\t |>\t s:\tStart \n");

	printf("\n\t Selection ? ");
	l_choice = inputMenu();
	printf("\n\n");

	if (l_choice == '1')
	{
		printf("\n\n\tEnter sample time (in sec): ");
		_sd.sample_time = inputDouble(NOT_SPEC);
		_noTrjMenu();
	}
	else if (l_choice == '2')
	{
		printf("\n\n\tEnter stop time (in sec): ");
		_sd.stop_time = inputDouble(NOT_SPEC);
		_noTrjMenu();
	}
	else if ((l_choice >= 'S') && (l_choice <= 's'))
	{
		_md.trjType = NO_TRJ;
		_md.tf = _sd.stop_time;
		move();
	}
	else if (l_choice == '0')
	{
		_moveMenu();
	}
	else
	{
		// Invalid selection
		printf("\nInvalid choice! Enter a number given in the menu.\n\n");
		_noTrjMenu();
	}
}

// ----------------------------------------------------------------
// _moveJntMenu
// ----------------------------------------------------------------
void _moveJntMenu(void)
{
	char l_choice = 0;

	// Joint space trajectory selection
	printf("..\\MOVEMENT MENU\\JOINT SPACE> \n");

	printf("\n\tEntering type: ");
	if (_md.jntEnterType == ABSOLUTE_JNT)
		printf("ABSOLUTE JOINT");
	else
		printf("INCREMENTAL JOINT");
	printf("\n");

	printf("\n\tSelect a trajectory: \n\n");
	printf("\t<--\t 0:\tBack to previous menu \n");
	printf("\t\t --------- PERIODIC --------- \n");
	printf("\t\\..\t 1:\tSinusoidal \n");
	printf("\t\t ------ POINT-TO-POINT ------ \n");
	printf("\t\t 2:\tCubic \n");
	printf("\t\t 3:\tQuintic \n");
	printf("\t\t 4:\tBang-Bang \n");
	printf("\t\t 5:\tTrapezoidal \n");
	printf("\t\t 6:\tSmooth Trapezoidal \n");
	printf("\t\t ------ MULTIPLE POINTS ----- \n");
	printf("\t\t 7:\tVia Points \n");
	printf("\n");
	printf("\t< >\t j:\tJoint entering type \n");


	printf("\n\t Selection ? ");
	l_choice = inputMenu();
	printf("\n\n");

	if ((l_choice >= '1') && (l_choice <= '7'))
	{
		if ((l_choice >= '2') && (l_choice <= '6'))
			_md.trjType = PNT2PNT;
		else
			_md.p2pType = NO_POLY;

		switch (l_choice)
		{
		case '1': _md.trjType = SINUSOIDAL;		break;
		case '2': _md.p2pType = CUBIC;			break;
		case '3': _md.p2pType = QUINTIC;		break;
		case '4': _md.p2pType = BANGBANG;		break;
		case '5': _md.p2pType = TRAPEZOIDAL;	break;
		case '6': _md.p2pType = STRAPEZOIDAL;	break;
		case '7': _md.trjType = VIAPNTS;		break;
		default: break;
		}

		//if (_md.trjType == SINUSOIDAL)
		//	//_moveJntSinusoidalMenu();
		//else
		//	move();
	}
	else if ((l_choice == 'C') || (l_choice == 'c'))
	{
		_md.trjType = PNT2PNT;
		_md.p2pType = QUINTIC;

		//_customJointMenu();
	}
	else if ((l_choice == 'J') || (l_choice == 'j'))
	{
		if (_md.jntEnterType == ABSOLUTE_JNT)
		{
			_md.jntEnterType = INCRMNTL_JNT;
		}
		else
		{
			_md.jntEnterType = ABSOLUTE_JNT;
		}

		_moveJntMenu();
	}
	else if (l_choice == '0')
	{
		_md.trjType = NO_TRJ;
		_moveMenu();
	}
	else
	{
		// Invalid selection
		printf("\nInvalid choice! Enter a number given in the menu.\n\n");
		_moveJntMenu();
	}
}

// ----------------------------------------------------------------
// _moveTskMenu
// ----------------------------------------------------------------
void _moveTskMenu(void)
{
	char l_choice;

	// Task space trajectory selection
	printf("..\\MOVEMENT MENU\\TASK SPACE> \n");

	printf("\n\tMotion frame: ");
	if (_md.motFrame == BASE_FRAME)
		printf("BASE FRAME");
	else
		printf("TASK FRAME");
	printf("\n");

	printf("\n\tSelect a trajectory: \n\n");
	printf("\t<--\t 0:\tBack to previous menu \n");
	printf("\t\t ----------- PERIODIC ---------- \n");
	printf("\t\\..\t 1:\tSinusoidal translation \n");
	printf("\t\t ---------- PARAMETRIC --------- \n");
	printf("\t\\..\t 2:\tCircle in a plane \n");
	printf("\t\\..\t 3:\tEight curve in a plane \n");
	printf("\t\t ------- MULTIPLE POINTS ------- \n");
	printf("\t\t 4:\tVia Points \n");
	printf("\n");
	printf("\t< >\t f:\tFrame selection \n");

	printf("\n\t Selection ? ");
	l_choice = inputMenu();
	printf("\n\n");

	if ((l_choice >= '1') && (l_choice <= '4'))
	{
		if ((l_choice >= '2') && (l_choice <= '3'))
			_md.trjType = PARAMETRIC;
		else
			_md.paraType = NO_PARA;

		switch (l_choice)
		{
		case '1':
			_md.trjType = SINUSOIDAL;
			//_moveTskSinusoidalMenu();
			break;
		case '2':
			_md.paraType = CIRCULAR;
			break;
		case '3':
			_md.paraType = EIGHT;
			break;
		case '4':
			_md.trjType = VIAPNTS;
			//move();
			break;
		default: break;
		}

		//if ((l_choice >= '2') && (l_choice <= '3'))
			//_moveParametricMenu();
	}
	//else if ((l_choice == 'C') || (l_choice == 'c'))
	//{
	//	if ((_md.motType == SINGLE_MOTION) && (_kd.rotRepresent == EULER_ANG))
	//		_customTaskMenu();
	//	else
	//		_moveTskMenu();
	//}
	else if ((l_choice == 'F') || (l_choice == 'f'))
	{
		if (_md.motFrame == BASE_FRAME)
			_md.motFrame = TASK_FRAME;
		else
			_md.motFrame = BASE_FRAME;

		_moveTskMenu();
	}
	else if (l_choice == '0')
	{
		_md.trjType = NO_TRJ;
		_moveMenu();
	}
	else
	{
		// Invalid selection
		printf("\nInvalid choice! Enter a number given in the menu.\n\n");
		_moveTskMenu();
	}
}

// ----------------------------------------------------------------
// _moveMenu
// ----------------------------------------------------------------
void _moveMenu(void)
{
	char l_choice = 0;

	printf("..\\MOVEMENT MENU> \n");

	printf("\n\tWhich selection do you want to choose?\n\n");
	printf("\t<--\t 0:\tBack to main menu\n");
	printf("\t\\..\t 1:\tNo trajectory\n");
	printf("\t\\..\t 2:\tMove in joint space\n");
	printf("\t\\..\t 3:\tMove in task space\n");

	printf("\n\t Selection ? ");
	l_choice = inputMenu();
	printf("\n\n");

	if (l_choice >= '1' && l_choice <= '3')
	{
		if (l_choice == '1')
		{
			_md.trjSpace = NO_SPC;
			_noTrjMenu();
		}
		else if (l_choice == '2')
		{
			_md.trjSpace = JOINT_SPC;
			_moveJntMenu();
		}
		else
		{
			_md.trjSpace = TASK_SPC;
			_moveTskMenu();
		}
	}
	else if (l_choice != '0')
	{
		printf("\nInvalid choice! Enter a number given in the menu.\n\n");

		_moveMenu();
	}
	else
	{
		_md.trjSpace = NO_SPC;
	}
}
