#ifndef _MDA_H_
#define _MDA_H_

typedef struct
{
	size_t rows;
	size_t cols;
	size_t pags;
	double *data;
} mda;

mda *mda_new(const size_t rows, const size_t cols);

#endif
