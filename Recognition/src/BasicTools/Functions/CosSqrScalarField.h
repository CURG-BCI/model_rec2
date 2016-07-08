/*
 * CosSqrScalarField.h
 *
 *  Created on: Apr 10, 2010
 *      Author: papazov
 */

#ifndef COSSQRSCALARFIELD_H_
#define COSSQRSCALARFIELD_H_

#include "ScalarField.h"
#include <cmath>

/** This is a two-dimensional scalar field which implements the interface 'ScalarField'.
  * The function is a linear combination of two cosine waves and a square distance field. */
class CosSqrScalarField: public ScalarField
{
public:
	CosSqrScalarField():ScalarField(2){}
	virtual ~CosSqrScalarField(){}

	/** Returns the function value at 'x'. */
	inline double f(const double* x);
};

//=== inline stuff ===============================================================================================================

inline double CosSqrScalarField::f(const double* x)
{
	return -1.2*cos(x[0]) -1.3*cos(x[1]) + 0.02*(x[0]*x[0] + x[1]*x[1]);
}

//================================================================================================================================

#endif /* COSSQRSCALARFIELD_H_ */
