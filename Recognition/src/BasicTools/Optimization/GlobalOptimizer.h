#ifndef GLOBALOPTIMIZER_H_
#define GLOBALOPTIMIZER_H_

#include "../Functions/ScalarField.h"


class GlobalOptimizer
{
public:
	GlobalOptimizer(){}
	virtual ~GlobalOptimizer(){}

	/** The method minimizes the scalar field 'f' over the 'searchSpace' and saves the
	 * minimizer in 'minimizer'. The return value of the method is the function value at 'minimizer'.
	 * 'f' is a scalar field of 'n' variables ('n' = f->getNumberOfVariables()),
	 * 'searchSpace' is an 'n'-dimensional interval of the form [a_1, b_1] x ... x [a_n, b_n],
	 * 'minimizer' is a double array of size 'n'.
	 * 'minimizer_eps[i]' is the desired precision for the i-th component (coordinate) of the minimizer.
	 * 'minimum_eps' is the desired precision for the minimum found. */
	virtual double minimize(ScalarField* f, const double** searchSpace,
			const double* minimizer_eps, double minimum_eps, double* minimizer) = 0;
};

#endif /*GLOBALOPTIMIZER_H_*/
