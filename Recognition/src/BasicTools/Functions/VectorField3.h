/*
 * VectorField3.h
 *
 *  Created on: Sep 9, 2010
 *      Author: papazov
 */

#ifndef VECTORFIELD3_H_
#define VECTORFIELD3_H_

class VectorField3
{
public:
	VectorField3(){}
	virtual ~VectorField3(){}

	/** y = F(x). */
	virtual void F(const double* x, double* y) = 0;
};

#endif /* VECTORFIELD3_H_ */
