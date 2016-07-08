/*
 * Function.h
 *
 *  Created on: Mar 29, 2010
 *      Author: papazov
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_

/** A function of the form f: R -> R. */
class Function
{
public:
	Function(){}
	virtual ~Function(){}

	/** A function of the form f: R -> R. */
	virtual double f(double x) = 0;
};

#endif /* FUNCTION_H_ */
