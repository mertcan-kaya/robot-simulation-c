#include <assert.h>
#include "mda.h"

#define ELEM1(mda, row) mda->data[(row-1)]
#define ELEM2(mda, row, col) mda->data[(col-1) * mda->rows + (row-1)]
#define ELEM3(mda, row, col, pag) mda->data[(pag-1) * (mda->cols * mda-> rows) + (col-1) * mda->rows + (row-1)]

mda *mda_new(const size_t rows, const size_t cols)
{
	unsigned int i;
	// allocate a matrix structure
	mda *mdaNew;

	mdaNew = (mda *)malloc(sizeof(mda));

	// set dimensions
	mdaNew->rows = rows;
	mdaNew->cols = cols;
	mdaNew->pags = 1;
	if (rows > 0 && cols > 0) {
		// allocate a double array of length rows * cols
		mdaNew->data = (double *)malloc(rows * cols * sizeof(double));
		// set all data to 0
		for (i = 0; i < rows * cols; i++)
			mdaNew->data[i] = 0.0;
	}
	else
		mdaNew->data = NULL;

	return mdaNew;
}
