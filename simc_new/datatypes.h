#ifndef _DATATYPES_H_
#define _DATATYPES_H_

#define N_DOF	6

typedef struct
{
	double data;
} r1;

typedef struct
{
	double data[3];
} r3;

typedef struct
{
	double data[6];
} r6;

typedef struct
{
	double data[N_DOF];
} rn;

typedef struct
{
	double data[N_DOF][N_DOF * 10];
} rnxn0;

typedef struct
{
	double data[10];
} r10;

typedef struct
{
	double data[N_DOF * 10];
} rn0;

typedef struct
{
	double data[3][3];
} r3x3;

typedef struct
{
	double data[3][6];
} r3x6;

typedef struct
{
	double data[3][10];
} r3x10;

typedef struct
{
	double data[6][N_DOF];
} r6xn;

typedef struct
{
	r3x3 rot;
	r3 trn;
} se3;

typedef enum ScrewType
{
	SCR_MOT,
	SCR_FOR
} screwType;

typedef struct
{
	r3 lin;
	r3 ang;

	screwType scr_typ;
} screw;

typedef struct
{
	r3 lin[N_DOF];
	r3 ang[N_DOF];

	screwType scr_typ;
} screwxn;

typedef struct
{
	r3 lin[10];
	r3 ang[10];

	screwType scr_typ;
} screwx10;

typedef struct
{
	r3x3 b11;
	r3x3 b12;
	r3x3 b21;
	r3x3 b22;

	screwType scr_typ;
} st;

//double mul_quad_r66_and_screw(const r66 r66var, const screw scr);
double dot_prod(const r3 var1, const r3 var2);
double dot_prod_screw(const screw var1, const screw var2);
double mul_quad_screw(const st stvar, const screw scr);
double mul_transpose_r10_and_r10(const r10 var1, const r10 var2);
double mul_transpose_screw_and_screw(const screw var1, const screw var2);
r1 set_r1(const double data);
r1 mul_transpose_r3_and_r3(const r3 var1, const r3 var2);
r3 get_r3_zero(void);
r3 set_r3(const double x, const double y, const double z);
r3 add_r3(const r3 var1, const r3 var2);
r3 add_3_r3(const r3 var1, const r3 var2, const r3 var3);
r3 sub_r3(const r3 var1, const r3 var2);
r3 mul_double_and_r3(const double dblvar, const r3 r3var);
r3 mul_r1_and_r3(const r1 r1var, const r3 r3var);
r3 div_r3_by_double(const r3 r3var, const double dblvar);
r3 cross_prod(const r3 var1, const r3 var2);
r3 mul_r3x3_and_r3(const r3x3 r33var, const r3 r3var);
r3 mul_r3x10_and_r10(const r3x10 r3x10var, const r10 r10var);
r3 mul_r3_10_and_r10(const r3 r3var[10], const r10 r10var);
r3 mul_r3xn_and_rn(const r3 r3var[N_DOF], const rn rnvar);
r3 avg_r3(const r3 var1, const r3 var2);
r3x3 get_r3x3_zero(void);
r3x3 get_r3x3_idnt(void);
r3x3 set_r3x3(	const double e11, const double e12, const double e13,
				const double e21, const double e22, const double e23,
				const double e31, const double e32, const double e33);
r3x3 set_r3x3sym(	const double xx,
					const double yx, const double yy,
					const double zx, const double zy, const double zz);
r3x3 set_r3x3eye(const r1 var);
r3x3 transpose_r3x3(const r3x3 r33var);
r3x3 skew_sym(const r3 var);
r3x3 add_r3x3(const r3x3 var1, const r3x3 var2);
r3x3 sub_r3x3(const r3x3 var1, const r3x3 var2);
r3x3 mul_r3x3(const r3x3 var1, const r3x3 var2);
r3x3 mul_double_and_r3x3(const double dblvar, const r3x3 r33var);
r3x3 mul_r1_and_r3x3(const r1 r1var, const r3x3 r33var);
r3x3 mul_col_and_row_r3(const r3 var1, const r3 var2);
r3x3 avg_r3x3(const r3x3 var1, const r3x3 var2);
r3x6 add_r3x6(const r3x6 var1, const r3x6 var2);
r3x6 mul_r3x3_and_r3x6(const r3x3 r3x3var, const r3x6 r3x6var);
r3x6 uniq_mat(const r3 var);
r3x6 avg_r3x6(const r3x6 var1, const r3x6 var2);
r3x10 compose_r3x10(const r3 r3var, const r3x3 r3x3var, const r3x6 r3x6var);
rn get_rn_zero(void);
rn add_rn(const rn var1, const rn var2);
rn sub_rn(const rn var1, const rn var2);
rn mul_rn_elem_wise(const rn var1, const rn var2);
rn mul_r6xn_and_doublen(const r6xn J, const double q_vel[N_DOF]);
rn integrate_rn(const rn prev, const rn new, const double tstp);
rn mul_rnxn0_and_rn0(const rnxn0 rnxn0var, const rn0 rn0var);
rn mul_transpose_screwxn_and_screw(const screwxn screwxnvar, const screw screwvar);
rn0 sub_rn0(const rn0 var1, const rn0 var2);
rn0 set_rn0_from_r10s(const r10 var[N_DOF]);
rn0 mul_double_and_rn0(const double dblvar, const rn0 rn0var);
rn0 mul_rn0_elem_wise(const rn0 var1, const rn0 var2);
rn0 mul_transpose_rnxn0_and_rn(const rnxn0 rnxn0var, const rn rnvar);
rn0 integrate_rn0(const rn0 prev, const rn0 new, const double tstp);
rnxn0 set_rnxn0_from_rns(const rn var[N_DOF * 10]);
rnxn0 set_rnxn0_from_r10s(const r10 var[N_DOF][N_DOF]);
r6 uniq_vec(const r3x3 var);
r6 mul_transpose_r3x6_r3(const r3x6 r3x6var, const r3 r3var);
r6xn set_r6xn_elem(r6xn var, const unsigned int row, const unsigned int col, const double c);
r6xn set_r6xn_col(r6xn J_e, const unsigned int col	, const double v1col
													, const double v2col
													, const double v3col
													, const double v4col
													, const double v5col
													, const double v6col);
r6xn add_r6xn(const r6xn var1, const r6xn var2);
//r66 sub_r66(const r6x6 var1, const r6x6 var2);
r6xn mul_r6xn(const r6xn var1, const r6xn var2);
r10 set_r10(const double d1, const double d2, const double d3, const double d4, const double d5
		, const double d6, const double d7, const double d8, const double d9, const double d10);
r10 add_r10(const r10 var1, const r10 var2);
r10 sub_r10(const r10 var1, const r10 var2);
r10 mul_double_and_r10(const double dblvar, const r10 r10var);
r10 mul_transpose_r3x10_and_r3(const r3x10 r3x10var, const r3 r3var);
r10 mul_transpose_screwx10_and_screw(const screwx10 screwx10var, const screw screwvar);
r10 mul_r10_elem_wise(const r10 var1, const r10 var2);
r10 compose_r10(const r1 r1var, const r3 r3var, const r6 r6var);
r10 set_r10_from_rn0_part(const rn0 var, const int i);
r10 integrate_r10(const r10 prvvar, const r10 newvar, const double tstp);
screw set_screw(const r3 linvec, const r3 angvec);
screw set_screw_lin(screw scr, const double l1, const double l2, const double l3);
screw set_screw_ang(screw scr, const double a1, const double a2, const double a3);
screw add_screw(const screw scr1, const screw scr2);
screw sub_screw(const screw scr1, const screw scr2);
screw mul_double_and_screw(const double dbl, const screw scr);
screw mul_st_and_screw(const st stvar, const screw scr);
screw mul_screwxn_and_rn(const screwxn screwxnvar, const rn rnvar);
screw mul_screwx10_and_r10(const screwx10 screwx10var, const r10 r10var);
screw avg_screw(const screw var1, const screw var2);
screwxn compose_screwxnxn(const screw scr[N_DOF][N_DOF], const int i);
screwx10 set_screwx10(const r3x10 linvec, const r3x10 angvec, const screwType scr_typ);
//screw mul_r66_and_screw(const r66 r66var, const screw scr);
st set_st(const r3x3 v11, const r3x3 v12, const r3x3 v21, const r3x3 v22);
st set_twist_st_from_se(const r3x3 rot, const r3 trn);
st set_wrench_st_from_se(const r3x3 rot, const r3 trn);
st set_tr_twist_st_from_se(const r3x3 rot, const r3 trn);
st set_tr_wrench_st_from_se(const r3x3 rot, const r3 trn);
st add_st(const st var1, const st var2);
st sub_st(const st var1, const st var2);
st mul_st(const st var1, const st var2);
st mul_double_and_st(const double c, const st stvar);
st mul_quad_tr_st_and_screw(const st stvar, const screw scr);
st mul_col_and_row_screw(const screw scr1, const screw scr2);
st transpose_st(const st var);
//st mul_quad_st_and_st(const st var1, const st var2);

#endif // _DATATYPES_H_
