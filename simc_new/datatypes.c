//*********************************************************************
// ~NAME : datatypes.c
// ~AUTHOR : Mertcan Kaya
// ~DESCRIPTION : Functions for datatypes
//**********************************************************************

#include "functions.h"
#include "datatypes.h"
#include <stdio.h>

 const int nl[3][3] =
 {
	 {0,1,2},
	 {1,2,0},
	 {2,0,1}
 };

 double dot_prod(const r3 var1, const r3 var2)
 {
	 double out = { 0.0 };
	 int i;

	 for (i = 0; i < 3; i++)
		 out += var1.data[i] * var2.data[i];

	 return out;
 }

 double dot_prod_screw(const screw var1, const screw var2)
 {
	 double out = { 0.0 };

	 out = dot_prod(var1.lin, var2.lin) + dot_prod(var1.ang, var2.ang);

	 return out;
 }

 double mul_quad_screw(const st stvar, const screw scr)
 {
	 double out = { 0.0 };

	 out = dot_prod(scr.lin, add_r3(mul_r3x3_and_r3(stvar.b11, scr.lin), mul_r3x3_and_r3(stvar.b12, scr.ang))) 
		 + dot_prod(scr.ang, add_r3(mul_r3x3_and_r3(stvar.b21, scr.lin), mul_r3x3_and_r3(stvar.b22, scr.ang)));

	 return out;
 }

 double mul_transpose_r10_and_r10(const r10 var1, const r10 var2)
 {
	 double out = { 0.0 };
	 int i;

	 for (i = 0; i < 10; i++)
		 out += var1.data[i] * var2.data[i];

	 return out;
 }

 double mul_transpose_screw_and_screw(const screw var1, const screw var2)
 {
	 double out = { 0.0 };
	 int i;

	 if (var1.scr_typ == var2.scr_typ)
	 {
		 for (i = 0; i < 3; i++)
		 {
			 out += var1.lin.data[i] * var2.lin.data[i] + var1.ang.data[i] * var2.ang.data[i];
		 }
	 }
	 else
	 {
		 printf("Error!");
	 }

	 return out;
 }

 r1 set_r1(const double data)
 {
	 r1 out = { 0.0 };

	 out.data = data;

	 return out;
 }

 r1 mul_transpose_r3_and_r3(const r3 var1, const r3 var2)
 {
	 r1 out = { 0.0 };
	 int i;

	 for (i = 0; i < 3; i++)
		 out.data += var1.data[i] * var2.data[i];

	 return out;
 }

 r3 get_r3_zero(void)
 {
	 r3 out = { 0.0 };

	 return out;
 }

r3 set_r3(const double x, const double y, const double z)
{
	r3 out = { 0.0 };

	out.data[X_AXIS] = x;
	out.data[Y_AXIS] = y;
	out.data[Z_AXIS] = z;

	return out;
}

r3 add_r3(const r3 var1, const r3 var2)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = var1.data[i] + var2.data[i];

	return out;
}

r3 add_3_r3(const r3 var1, const r3 var2, const r3 var3)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = var1.data[i] + var2.data[i] + var3.data[i];

	return out;
}

r3 sub_r3(const r3 var1, const r3 var2)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = var1.data[i] - var2.data[i];

	return out;
}

r3 mul_double_and_r3(const double dblvar, const r3 r3var)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = dblvar * r3var.data[i];

	return out;
}

r3 mul_r1_and_r3(const r1 r1var, const r3 r3var)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = r1var.data * r3var.data[i];

	return out;
}

r3 div_r3_by_double(const r3 r3var, const double dblvar)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = r3var.data[i] / dblvar;

	return out;
}

r3 cross_prod(const r3 var1, const r3 var2)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = var1.data[nl[i][1]] * var2.data[nl[i][2]] - var1.data[nl[i][2]] * var2.data[nl[i][1]];

	return out;
}

r3 mul_r3x3_and_r3(const r3x3 r33var, const r3 r3var)
{
	r3 out = { 0.0 };
	int row, col;

	for (row = 0; row < 3; row++)
	{
		double val = 0.0;
		for (col = 0; col < 3; col++)
			val += r33var.data[row][col] * r3var.data[col];
		out.data[row] = val;
	}

	return out;
}

r3 mul_r3x10_and_r10(const r3x10 r3x10var, const r10 r10var)
{
	r3 out = { 0.0 };
	int row, col;

	for (row = 0; row < 3; row++)
	{
		double val = 0.0;
		for (col = 0; col < 10; col++)
			val += r3x10var.data[row][col] * r10var.data[col];
		out.data[row] = val;
	}

	return out;
}

r3 mul_r3_10_and_r10(const r3 r3var[10], const r10 r10var)
{
	r3 out = { 0.0 };
	int row, col;

	for (row = 0; row < 3; row++)
	{
		double val = 0.0;
		for (col = 0; col < 10; col++)
			val += r3var[col].data[row] * r10var.data[col];
		out.data[row] = val;
	}

	return out;
}

r3 mul_r3xn_and_rn(const r3 r3var[N_DOF], const rn rnvar)
{
	r3 out = { 0.0 };
	int row, col;

	for (row = 0; row < 3; row++)
	{
		double val = 0.0;
		for (col = 0; col < N_DOF; col++)
			val += r3var[col].data[row] * rnvar.data[col];
		out.data[row] = val;
	}

	return out;
}

r3 avg_r3(const r3 var1, const r3 var2)
{
	r3 out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
		out.data[i] = (var1.data[i] + var2.data[i])/2.0;

	return out;
}

r3x3 get_r3x3_zero(void)
{
	r3x3 out = { 0.0 };

	return out;
}

r3x3 get_r3x3_idnt(void)
{
	r3x3 out = { 0.0 };

	out.data[X_AXIS][X_AXIS] = 1.0;
	out.data[Y_AXIS][Y_AXIS] = 1.0;
	out.data[Z_AXIS][Z_AXIS] = 1.0;

	return out;
}

r3x3 set_r3x3(	const double e11, const double e12, const double e13,
				const double e21, const double e22, const double e23,
				const double e31, const double e32, const double e33)
{
	r3x3 out = { 0.0 };

	out.data[X_AXIS][X_AXIS] = e11; out.data[X_AXIS][Y_AXIS] = e12; out.data[X_AXIS][Z_AXIS] = e13;
	out.data[Y_AXIS][X_AXIS] = e21; out.data[Y_AXIS][Y_AXIS] = e22; out.data[Y_AXIS][Z_AXIS] = e23;
	out.data[Z_AXIS][X_AXIS] = e31; out.data[Z_AXIS][Y_AXIS] = e32; out.data[Z_AXIS][Z_AXIS] = e33;

	return out;
}

r3x3 set_r3x3sym(	const double xx,
					const double yx, const double yy,
					const double zx, const double zy, const double zz)
{
	r3x3 r33sym = { 0.0 };

	r33sym.data[X_AXIS][X_AXIS] = xx; r33sym.data[X_AXIS][Y_AXIS] = yx; r33sym.data[X_AXIS][Z_AXIS] = zx;
	r33sym.data[Y_AXIS][X_AXIS] = yx; r33sym.data[Y_AXIS][Y_AXIS] = yy; r33sym.data[Y_AXIS][Z_AXIS] = zy;
	r33sym.data[Z_AXIS][X_AXIS] = zx; r33sym.data[Z_AXIS][Y_AXIS] = zy; r33sym.data[Z_AXIS][Z_AXIS] = zz;

	return r33sym;
}

r3x3 set_r3x3eye(const r1 var)
{
	r3x3 r33diag = { 0.0 };

	r33diag.data[X_AXIS][X_AXIS] = var.data;
	r33diag.data[Y_AXIS][Y_AXIS] = var.data;
	r33diag.data[Z_AXIS][Z_AXIS] = var.data;

	return r33diag;
}

r3x3 transpose_r3x3(const r3x3 var)
{
	r3x3 out = { 0.0 };
	int row, col;

	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 3; col++)
			out.data[col][row] = var.data[row][col];
	}

	return out;
}

r3x3 skew_sym(const r3 var)
{
	r3x3 out = { 0.0 };

	out.data[0][1] = -var.data[2]; out.data[0][2] =  var.data[1];
	out.data[1][0] =  var.data[2]; out.data[1][2] = -var.data[0];
	out.data[2][0] = -var.data[1]; out.data[2][1] =  var.data[0];

	return out;
}

r3x3 add_r3x3(const r3x3 var1, const r3x3 var2)
{
	r3x3 out = { 0.0 };
	int row, col;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
			out.data[row][col] = var1.data[row][col] + var2.data[row][col];
	}

	return out;
}

r3x3 sub_r3x3(const r3x3 var1, const r3x3 var2)
{
	r3x3 out = { 0.0 };
	int row, col;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
			out.data[row][col] = var1.data[row][col] - var2.data[row][col];
	}

	return out;
}

r3x3 mul_r3x3(const r3x3 var1, const r3x3 var2)
{
	r3x3 r33new = { 0.0 };
	int row, col, k;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
		{
			double val = 0.0;
			for (k = 0; k < 3; k++)
				val += var1.data[row][k] * var2.data[k][col];
			r33new.data[row][col] = val;
		}
	}

	return r33new;
}

r3x3 mul_double_and_r3x3(const double dblvar, const r3x3 r33var)
{
	r3x3 r33new = { 0.0 };
	int row, col;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
			r33new.data[row][col] = dblvar * r33var.data[row][col];
	}

	return r33new;
}

r3x3 mul_r1_and_r3x3(const r1 r1var, const r3x3 r33var)
{
	r3x3 out = { 0.0 };
	int row, col;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
			out.data[row][col] = r1var.data * r33var.data[row][col];
	}

	return out;
}

r3x3 mul_col_and_row_r3(const r3 var1, const r3 var2)
{
	r3x3 out = { 0.0 };
	int row, col;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
			out.data[row][col] = var1.data[row] * var2.data[col];
	}

	return out;
}

r3x3 avg_r3x3(const r3x3 var1, const r3x3 var2)
{
	r3x3 out = { 0.0 };
	int row, col;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
			out.data[row][col] = (var1.data[row][col] + var2.data[row][col]) / 2.0;
	}

	return out;
}

r3x6 add_r3x6(const r3x6 var1, const r3x6 var2)
{
	r3x6 out = { 0.0 };
	int row, col;

	for (col = 0; col < 6; col++)
	{
		for (row = 0; row < 3; row++)
			out.data[row][col] = var1.data[row][col] + var2.data[row][col];
	}

	return out;
}

r3x6 mul_r3x3_and_r3x6(const r3x3 r3x3var, const r3x6 r3x6var)
{
	r3x6 out = { 0.0 };
	int row, col, k;

	for (col = 0; col < 6; col++)
	{
		for (row = 0; row < 3; row++)
		{
			double val = 0.0;
			for (k = 0; k < 3; k++)
				val += r3x3var.data[row][k] * r3x6var.data[k][col];
			out.data[row][col] = val;
		}
	}

	return out;
}

r3x6 uniq_mat(const r3 var)
{
	r3x6 out = { 0.0 };

	out.data[0][0] = var.data[0]; out.data[1][3] = var.data[0]; out.data[2][4] = var.data[0];
	out.data[1][1] = var.data[1]; out.data[0][3] = var.data[1]; out.data[2][5] = var.data[1];
	out.data[2][2] = var.data[2]; out.data[0][4] = var.data[2]; out.data[1][5] = var.data[2];

	return out;
}

r3x6 avg_r3x6(const r3x6 var1, const r3x6 var2)
{
	r3x6 out = { 0.0 };
	int row, col;

	for (col = 0; col < 6; col++)
	{
		for (row = 0; row < 3; row++)
			out.data[row][col] = (var1.data[row][col] + var2.data[row][col]) / 2.0;
	}

	return out;
}

r3x10 compose_r3x10(const r3 r3var, const r3x3 r3x3var, const r3x6 r3x6var)
{
	r3x10 out = { 0.0 };
	int row, col;

	for (row = 0; row < 3; row++)
	{
		for (col = 0; col < 10; col++)
		{
			if (col < 1)
				out.data[row][col] = r3var.data[row];
			else if (col < 4)
				out.data[row][col] = r3x3var.data[row][col - 1];
			else
				out.data[row][col] = r3x6var.data[row][col - 4];
		}
	}

	return out;
}

rn get_rn_zero(void)
{
	rn out = { 0.0 };

	return out;
}

rn add_rn(const rn var1, const rn var2)
{
	rn out = { 0.0 };
	int col;

	for (col = 0; col < N_DOF; col++)
		out.data[col] = var1.data[col] + var2.data[col];

	return out;
}

rn sub_rn(const rn var1, const rn var2)
{
	rn out = { 0.0 };
	int col;

	for (col = 0; col < N_DOF; col++)
		out.data[col] = var1.data[col] - var2.data[col];
		
	return out;
}

rn mul_rn_elem_wise(const rn var1, const rn var2)
{
	rn out = { 0.0 };
	int col;

	for (col = 0; col < N_DOF; col++)
		out.data[col] = var1.data[col] * var2.data[col];

	return out;
}

rn mul_r6xn_and_doublen(const r6xn J, const double q_vel[N_DOF])
{
	rn out = { 0.0 };
	int row, col;

	for (row = 0; row < 6; row++)
	{
		double val = 0.0;
		for (col = 0; col < N_DOF; col++)
			val += J.data[row][col] * q_vel[col];
		out.data[row] = val;
	}

	return out;
}

rn integrate_rn(const rn prev, const rn new, const double tstp)
{
	rn out = { 0.0 };
	int col;

	for (col = 0; col < N_DOF; col++)
		out.data[col] = prev.data[col] + new.data[col] * tstp;

	return out;
}

rn mul_rnxn0_and_rn0(const rnxn0 rnxn0var, const rn0 rn0var)
{
	rn out = { 0.0 };
	int row, col;

	for (row = 0; row < N_DOF; row++)
	{
		double val = 0.0;
		for (col = 0; col < N_DOF * 10; col++)
			val += rnxn0var.data[row][col] * rn0var.data[col];
		out.data[row] = val;
	}

	return out;
}

rn mul_transpose_screwxn_and_screw(const screwxn screwxnvar, const screw screwvar)
{
	rn out = { 0.0 };
	int i;

	for (i = 0; i < N_DOF; i++)
		out.data[i] = dot_prod(screwxnvar.lin[i], screwvar.lin) + dot_prod(screwxnvar.ang[i], screwvar.ang);

	return out;
}

rn0 sub_rn0(const rn0 var1, const rn0 var2)
{
	rn0 out = { 0.0 };
	int i;

	for (i = 0; i < N_DOF * 10; i++)
		out.data[i] = var1.data[i] - var2.data[i];

	return out;
}

rn0 set_rn0_from_r10s(const r10 var[N_DOF])
{
	rn0 out = { 0.0 };
	int i, j;

	for (i = 0; i < N_DOF; i++)
	{
		for (j = 0; j < 10; j++)
			out.data[i * 10 + j] = var[i].data[j];
	}

	return out;
}

rn0 mul_double_and_rn0(const double dblvar, const rn0 rn0var)
{
	rn0 out = { 0.0 };
	int i;

	for (i = 0; i < N_DOF * 10; i++)
		out.data[i] = dblvar * rn0var.data[i];

	return out;
}

rn0 mul_rn0_elem_wise(const rn0 var1, const rn0 var2)
{
	rn0 out = { 0.0 };
	int col;

	for (col = 0; col < N_DOF * 10; col++)
		out.data[col] = var1.data[col] * var2.data[col];

	return out;
}

rn0 mul_transpose_rnxn0_and_rn(const rnxn0 rnxn0var, const rn rnvar)
{
	rn0 out = { 0.0 };
	int row, col;

	for (col = 0; col < N_DOF * 10; col++)
	{
		double val = 0.0;
		for (row = 0; row < N_DOF; row++)
			val += rnxn0var.data[row][col] * rnvar.data[row];
		out.data[col] = val;
	}

	return out;
}

rn0 integrate_rn0(const rn0 prev, const rn0 new, const double tstp)
{
	rn0 out = { 0.0 };
	int col;

	for (col = 0; col < N_DOF * 10; col++)
		out.data[col] = prev.data[col] + new.data[col] * tstp;

	return out;
}

rnxn0 set_rnxn0_from_rns(const rn var[N_DOF * 10])
{
	rnxn0 out = { 0.0 };
	int row, col;

	for (row = 0; row < N_DOF; row++)
	{
		for (col = 0; col < N_DOF * 10; col++)
			out.data[row][col] = var[col].data[row];
	}

	return out;
}

rnxn0 set_rnxn0_from_r10s(const r10 var[N_DOF][N_DOF])
{
	rnxn0 out = { 0.0 };
	int row, col, elm;

	for (row = 0; row < N_DOF; row++)
	{
		for (col = 0; col < N_DOF; col++)
		{
			for (elm = 0; elm < 10; elm++)
				out.data[row][col*10 + elm] = var[col][row].data[elm];
		}
	}

	return out;
}

r6 uniq_vec(const r3x3 var)
{
	r6 out = { 0.0 };

	out.data[0] = var.data[0][0];
	out.data[1] = var.data[1][1];
	out.data[2] = var.data[2][2];
	out.data[3] = var.data[0][1];
	out.data[4] = var.data[0][2];
	out.data[5] = var.data[1][2];

	return out;
}

r6 mul_transpose_r3x6_r3(const r3x6 r3x6var, const r3 r3var)
{
	r6 out = { 0.0 };
	int row, col;

	for (col = 0; col < 6; col++)
	{
		double val = 0.0;
		for (row = 0; row < 3; row++)
			val += r3x6var.data[row][col] * r3var.data[row];
		out.data[col] = val;
	}

	return out;
}

r6xn set_r6xn_elem(r6xn var, const unsigned int row, const unsigned int col, const double c)
{
	var.data[row][col] = c;

	return var;
}

r6xn set_r6xn_col(r6xn J_e, const unsigned int col	, const double v1col
													, const double v2col
													, const double v3col
													, const double v4col
													, const double v5col
													, const double v6col)
{
	J_e.data[0][col] = v1col;
	J_e.data[1][col] = v2col;
	J_e.data[2][col] = v3col;
	J_e.data[3][col] = v4col;
	J_e.data[4][col] = v5col;
	J_e.data[5][col] = v6col;

	return J_e;
}

r6xn add_r66(const r6xn var1, const r6xn var2)
{
	r6xn r66new = { 0.0 };
	int row, col;

	for (col = 0; col < 6; col++)
	{
		for (row = 0; row < 6; row++)
			r66new.data[row][col] = var1.data[row][col] + var2.data[row][col];
	}

	return r66new;
}

r6xn sub_r6xn(const r6xn var1, const r6xn var2)
{
	r6xn r66new = { 0.0 };
	int row, col;

	for (col = 0; col < 6; col++)
	{
		for (row = 0; row < 6; row++)
			r66new.data[row][col] = var1.data[row][col] - var2.data[row][col];
	}

	return r66new;
}

r6xn mul_r6xn(const r6xn var1, const r6xn var2)
{
	r6xn r66new = { 0.0 };
	int row, col, k;

	for (col = 0; col < 6; col++)
	{
		for (row = 0; row < 6; row++)
		{
			double val = 0.0;
			for (k = 0; k < 6; k++)
				val += var1.data[row][k] * var2.data[k][col];
			r66new.data[row][col] = val;
		}
	}

	return r66new;
}

r10 set_r10(const double d1, const double d2, const double d3, const double d4, const double d5
	, const double d6, const double d7, const double d8, const double d9, const double d10)
{
	r10 out = { 0.0 };

	out.data[0] = d1;
	out.data[1] = d2;
	out.data[2] = d3;
	out.data[3] = d4;
	out.data[4] = d5;
	out.data[5] = d6;
	out.data[6] = d7;
	out.data[7] = d8;
	out.data[8] = d9;
	out.data[9] = d10;

	return out;
}

r10 add_r10(const r10 var1, const r10 var2)
{
	r10 out = { 0.0 };
	int i;

	for (i = 0; i < 10; i++)
		out.data[i] = var1.data[i] + var2.data[i];

	return out;
}

r10 sub_r10(const r10 var1, const r10 var2)
{
	r10 out = { 0.0 };
	int i;

	for (i = 0; i < 10; i++)
		out.data[i] = var1.data[i] - var2.data[i];

	return out;
}

r10 mul_double_and_r10(const double dblvar, const r10 r10var)
{
	r10 out = { 0.0 };
	int i;

	for (i = 0; i < 10; i++)
		out.data[i] = dblvar * r10var.data[i];

	return out;
}

r10 mul_transpose_r3x10_and_r3(const r3x10 r3x10var, const r3 r3var)
{
	r10 out = { 0.0 };
	int row, col;

	for (col = 0; col < 10; col++)
	{
		double val = 0.0;
		for (row = 0; row < 3; row++)
			val += r3x10var.data[row][col] * r3var.data[row];
		out.data[col] = val;
	}

	return out;
}

r10 mul_transpose_screwx10_and_screw(const screwx10 screwx10var, const screw screwvar)
{
	r10 out = { 0.0 };
	int i;

	for (i = 0; i < 10; i++)
	{
		out.data[i] = dot_prod(screwx10var.lin[i], screwvar.lin) + dot_prod(screwx10var.ang[i], screwvar.ang);

		//printf("\nscrewx10var.lin %d:", i);
		//printf("\n%f", screwx10var.lin[i].data[0]);
		//printf("\n%f", screwx10var.lin[i].data[1]);
		//printf("\n%f", screwx10var.lin[i].data[2]);
		//printf("\n");
	}

	//printf("\nscrewx10var.lin:");
	//printf("\n%f", screwvar.lin.data[0]);
	//printf("\n%f", screwvar.lin.data[1]);
	//printf("\n%f", screwvar.lin.data[2]);
	//printf("\n");

	return out;
}

r10 mul_r10_elem_wise(const r10 var1, const r10 var2)
{
	r10 out = { 0.0 };

	int i;

	for (i = 0; i < 10; i++)
		out.data[i] = var1.data[i] * var2.data[i];

	return out;
}

r10 compose_r10(const r1 r1var, const r3 r3var, const r6 r6var)
{
	r10 out = { 0.0 };
	int i;

	for (i = 0; i < 10; i++)
	{
		if (i < 1)
			out.data[i] = r1var.data;
		else if (i < 4)
			out.data[i] = r3var.data[i - 1];
		else
			out.data[i] = r6var.data[i - 4];
	}

	return out;
}

r10 set_r10_from_rn0_part(const rn0 var, const int i)
{
	r10 out = { 0.0 };
	int j;

	for (j = 0; j < 10; j++)
		out.data[j] = var.data[(i * 10) + j];

	return out;
}

r10 integrate_r10(const r10 prvvar, const r10 newvar, const double tstp)
{
	r10 out = { 0.0 };
	int col;

	for (col = 0; col < 10; col++)
		out.data[col] = prvvar.data[col] + newvar.data[col] * tstp;

	return out;
}

screw set_screw(const r3 linvec, const r3 angvec)
{
	screw scr = { 0.0 };

	scr.lin.data[0] = linvec.data[0]; scr.lin.data[1] = linvec.data[1]; scr.lin.data[2] = linvec.data[2];
	scr.ang.data[0] = angvec.data[0]; scr.ang.data[1] = angvec.data[1]; scr.ang.data[2] = angvec.data[2];

	return scr;
}

screw set_screw_lin(screw scr, const double l1, const double l2, const double l3)
{
	scr.lin.data[0] = l1; scr.lin.data[1] = l2; scr.lin.data[2] = l3;

	return scr;
}

screw set_screw_ang(screw scr, const double a1, const double a2, const double a3)
{
	scr.ang.data[0] = a1; scr.ang.data[1] = a2; scr.ang.data[2] = a3;

	return scr;
}

screw add_screw(const screw scr1, const screw scr2)
{
	screw out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
	{
		out.lin.data[i] = scr1.lin.data[i] + scr2.lin.data[i];
		out.ang.data[i] = scr1.ang.data[i] + scr2.ang.data[i];
	}

	return out;
}

screw sub_screw(const screw scr1, const screw scr2)
{
	screw out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
	{
		out.lin.data[i] = scr1.lin.data[i] - scr2.lin.data[i];
		out.ang.data[i] = scr1.ang.data[i] - scr2.ang.data[i];
	}

	return out;
}

screw mul_double_and_screw(const double dbl, const screw scr)
{
	screw out = { 0.0 };
	int i;

	for (i = 0; i < 3; i++)
	{
		out.lin.data[i] = dbl * scr.lin.data[i];
		out.ang.data[i] = dbl * scr.ang.data[i];
	}

	return out;
}

screw mul_st_and_screw(const st stvar, const screw scr)
{
	screw out = { 0.0 };

	out.lin = add_r3(mul_r3x3_and_r3(stvar.b11, scr.lin), mul_r3x3_and_r3(stvar.b12, scr.ang));
	out.ang = add_r3(mul_r3x3_and_r3(stvar.b21, scr.lin), mul_r3x3_and_r3(stvar.b22, scr.ang));;

	return out;
}

screw mul_screwxn_and_rn(const screwxn screwxnvar, const rn rnvar)
{
	screw out = { 0.0 };

	out.lin = mul_r3xn_and_rn(screwxnvar.lin, rnvar);
	out.ang = mul_r3xn_and_rn(screwxnvar.ang, rnvar);

	return out;
}

screw mul_screwx10_and_r10(const screwx10 screwx10var, const r10 r10var)
{
	screw out = { 0.0 };

	out.lin = mul_r3_10_and_r10(screwx10var.lin, r10var);
	out.ang = mul_r3_10_and_r10(screwx10var.ang, r10var);

	return out;
}

screw avg_screw(const screw var1, const screw var2)
{
	screw out = { 0.0 };
	int i;

	if (var1.scr_typ != var2.scr_typ)
		printf("Error: screws are not in the same types!");
	else
	{
		for (i = 0; i < 3; i++)
		{
			out.lin.data[i] = (var1.lin.data[i] + var2.lin.data[i]) / 2.0;
			out.ang.data[i] = (var1.ang.data[i] + var2.ang.data[i]) / 2.0;
		}
	}

	return out;
}

screwxn compose_screwxnxn(const screw scr[N_DOF][N_DOF], const int i)
{
	screwxn out = { 0.0 };
	int j, k;

	for (j = 0; j < N_DOF; j++)
	{
		for (k = 0; k < 3; k++)
		{
			out.lin[j].data[k] = scr[j][i].lin.data[k];
			out.ang[j].data[k] = scr[j][i].ang.data[k];
		}
	}

	out.scr_typ = scr[j][i].scr_typ;

	return out;
}

screwx10 set_screwx10(const r3x10 linvec, const r3x10 angvec, const screwType scr_typ)
{
	screwx10 scr = { 0.0 };
	int j, k;

	for (j = 0; j < 10; j++)
	{
		for (k = 0; k < 3; k++)
		{
			scr.lin[j].data[k] = linvec.data[k][j];
			scr.ang[j].data[k] = angvec.data[k][j];
		}
	}

	scr.scr_typ = scr_typ;

	return scr;
}

//screw mul_r66_and_screw(const r66 r66var, const screw scr)
//{
//	screw out = { 0.0 };
//	unsigned int row, k;
//
//	for (row = 0; row < 6; row++)
//	{
//		double val = 0.0;
//		for (k = 0; k < 3; k++)
//			val += r66var.data[row][k] * scr.lin.data[k] + r66var.data[row][k + 3] * scr.ang.data[k];
//
//		if (row < 3)
//			out.lin.data[row] = val;
//		else
//			out.ang.data[row - 3] = val;
//	}
//
//	return out;
//}

st set_st(const r3x3 v11, const r3x3 v12, const r3x3 v21, const r3x3 v22)
{
	st out = { 0.0 };

	out.b11 = v11;
	out.b12 = v12;
	out.b21 = v21;
	out.b22 = v22;

	return out;
}

st set_twist_st_from_se(const r3x3 rot, const r3 trn)
{
	st out = { 0.0 };

	out.b11 = rot;
	out.b12 = mul_r3x3(skew_sym(trn), rot);
	out.b22 = rot;

	return out;
}

st set_wrench_st_from_se(const r3x3 rot, const r3 trn)
{
	st out = { 0.0 };

	out.b11 = rot;
	out.b21 = mul_r3x3(skew_sym(trn),rot);
	out.b22 = rot;

	return out;
}

st set_tr_twist_st_from_se(const r3x3 rot, const r3 trn)
{
	st out = { 0.0 };
	r3x3 rottr = { 0.0 };

	rottr = transpose_r3x3(rot);

	out.b11 = rottr;
	out.b21 = mul_double_and_r3x3(-1.0, mul_r3x3(rottr, skew_sym(trn)));
	out.b22 = rottr;

	return out;
}

st set_tr_wrench_st_from_se(const r3x3 rot, const r3 trn)
{
	st out = { 0.0 };
	r3x3 rottr = { 0.0 };

	rottr = transpose_r3x3(rot);

	out.b11 = rottr;
	out.b12 = mul_double_and_r3x3(-1.0, mul_r3x3(rottr, skew_sym(trn)));
	out.b22 = rottr;

	return out;
}

st add_st(const st var1, const st var2)
{
	st out = { 0.0 };

	out.b11 = add_r3x3(var1.b11, var2.b11);
	out.b12 = add_r3x3(var1.b12, var2.b12);
	out.b21 = add_r3x3(var1.b21, var2.b21);
	out.b22 = add_r3x3(var1.b22, var2.b22);

	return out;
}

st sub_st(const st var1, const st var2)
{
	st out = { 0.0 };

	out.b11 = sub_r3x3(var1.b11, var2.b11);
	out.b12 = sub_r3x3(var1.b12, var2.b12);
	out.b21 = sub_r3x3(var1.b21, var2.b21);
	out.b22 = sub_r3x3(var1.b22, var2.b22);

	return out;
}

st mul_st(const st var1, const st var2)
{
	st out = { 0.0 };

	out.b11 = add_r3x3(mul_r3x3(var1.b11, var2.b11), mul_r3x3(var1.b12, var2.b21));
	out.b12 = add_r3x3(mul_r3x3(var1.b11, var2.b12), mul_r3x3(var1.b12, var2.b22));
	out.b21 = add_r3x3(mul_r3x3(var1.b21, var2.b11), mul_r3x3(var1.b22, var2.b21));
	out.b22 = add_r3x3(mul_r3x3(var1.b21, var2.b12), mul_r3x3(var1.b22, var2.b22));

	return out;
}

st mul_double_and_st(const double c, const st stvar)
{
	st out = { 0.0 };

	out.b11 = mul_double_and_r3x3(c, stvar.b11);
	out.b12 = mul_double_and_r3x3(c, stvar.b12);
	out.b21 = mul_double_and_r3x3(c, stvar.b21);
	out.b22 = mul_double_and_r3x3(c, stvar.b22);

	return out;
}

st mul_quad_tr_st_and_screw(const st stvar, const screw scr)
{
	st out = { 0.0 };
	unsigned int row, col, k;
	double temp1 = 0.0, temp2 = 0.0;

	for (col = 0; col < 3; col++)
	{
		for (row = 0; row < 3; row++)
		{
			temp1 = 0.0;
			temp2 = 0.0;
			for (k = 0; k < 3; k++)
			{
				temp1 += stvar.b11.data[k][col] * scr.lin.data[k] + stvar.b21.data[k][col] * scr.ang.data[k];
				temp2 += stvar.b11.data[row][k] * scr.lin.data[k] + stvar.b12.data[row][k] * scr.ang.data[k];
			}
			out.b11.data[row][col] = temp1 * temp2;

			temp1 = 0.0;
			temp2 = 0.0;
			for (k = 0; k < 3; k++)
			{
				temp1 += stvar.b12.data[k][col] * scr.lin.data[k] + stvar.b22.data[k][col] * scr.ang.data[k];
				temp2 += stvar.b21.data[row][k] * scr.lin.data[k] + stvar.b22.data[row][k] * scr.ang.data[k];
			}
			out.b12.data[row][col] = temp1 * temp2;

			temp1 = 0.0;
			temp2 = 0.0;
			for (k = 0; k < 3; k++)
			{
				temp1 += stvar.b11.data[k][col] * scr.lin.data[k] + stvar.b21.data[k][col] * scr.ang.data[k];
				temp2 += stvar.b21.data[row][k] * scr.lin.data[k] + stvar.b22.data[row][k] * scr.ang.data[k];
			}
			out.b21.data[row][col] = temp1 * temp2;

			temp1 = 0.0;
			temp2 = 0.0;
			for (k = 0; k < 3; k++)
			{
				temp1 += stvar.b12.data[k][col] * scr.lin.data[k] + stvar.b22.data[k][col] * scr.ang.data[k];
				temp2 += stvar.b21.data[row][k] * scr.lin.data[k] + stvar.b22.data[row][k] * scr.ang.data[k];
			}
			out.b21.data[row][col] = temp1 * temp2;
		}
	}

	return out;
}

st mul_col_and_row_screw(const screw scr1, const screw scr2)
{
	st out = { 0.0 };

	out.b11 = mul_col_and_row_r3(scr1.lin, scr2.lin);
	out.b12 = mul_col_and_row_r3(scr1.lin, scr2.ang);
	out.b21 = mul_col_and_row_r3(scr1.ang, scr2.lin);
	out.b22 = mul_col_and_row_r3(scr1.ang, scr2.ang);

	return out;
}

st transpose_st(const st var)
{
	st out = { 0.0 };

	out.b11 = transpose_r3x3(var.b11);
	out.b12 = transpose_r3x3(var.b21);
	out.b21 = transpose_r3x3(var.b12);
	out.b22 = transpose_r3x3(var.b22);

	return out;
}

//st mul_quad_st_and_st(const st var1, const st var2)
//{
//	st out = { 0.0 };
//	unsigned int row, col, j, k;
//	double temp1 = 0.0, temp2 = 0.0;
//
//	for (j = 0; j < 0; j++)
//	{
//		for (k = 0; k < 0; k++)
//			sum += var2.b11.data[row][k] * var1.b11.data[k][j] + var2.b21.data[row][k] * var1.b21.data[k][j]
//	}
//	out.b11.data[row][col] = var2.b11.data[0][col] * temp1 + var2.b11.data[1][col] * temp2 + var2.b11.data[2][col] * () + var2.b21.data[0][col] * () + var2.b21.data[1][col] * () + var2.b21.data[2][col] * ();
//
//
//	for (k = 0; k < 0; k++)
//		temp1 +=  var2.b11.data[k][col] * (var2.b21.data[row][0] * var1.b11.data[0][k] + var2.b22.data[row][1] * var1.b21.data[1][k]) 
//				+ var2.b21.data[k][col] * ();
//	out.b11.data[row][col] = var2.b11 + var2.b21;
//
//	out.b11.data[row][col] = var2.b12 + var2.b22;
//	out.b21.data[row][col] = var2.b11 + var2.b21;
//	out.b22.data[row][col] = var2.b12 + var2.b22;
//
//	return out;
//}
