#ifndef SCALARFIELD_H_
#define SCALARFIELD_H_


class ScalarField
{
public:
	ScalarField(int numOfVariables){ mNumOfVariables = numOfVariables;}
	virtual ~ScalarField(){}

	/** Returns the function value at 'x'. */
	virtual double f(const double* x) = 0;

	int getNumberOfVariables(){ return mNumOfVariables;}

protected:
	int mNumOfVariables;
};

#endif /*SCALARFIELD_H_*/
