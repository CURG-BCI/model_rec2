/*
 * TPSVectorField3.h
 *
 *  Created on: Feb 24, 2011
 *      Author: papazov
 */

#ifndef TPSVECTORFIELD3_H_
#define TPSVECTORFIELD3_H_

#include "../LinearAlgebra/Vector.h"
#include <cmath>
#include <list>

using namespace std;
using namespace tum;

class TPSVectorField3
{
public:
	TPSVectorField3();
	virtual ~TPSVectorField3();

	/** Computes the Thin Plate Spline (TPS) vector-valued interpolant which will attain the vector
	  * (fx[k], fy[k], fz[k]) at the point p[k]. %fx, %fy and %fz have to be of size %p.size(). */
	void build(list<const double*>& p, const double* fx, const double* fy, const double* fz);

	inline void F(const double* x, double* y);

protected:
	inline double rbf(const double* x);

protected:
	/** These are the weights for each dimension. */
	double *m_wx, *m_wy, *m_wz;
	list<const double*> mPoints;
	int mNumOfPoints;
};

//=== inline methods =====================================================================================================================

inline void TPSVectorField3::F(const double* x, double* y)
{
	int i;
	double rbf, xp[3], fx = 0.0, fy = 0.0, fz = 0.0;
	list<const double*>::iterator p;

	for ( i = 0, p = mPoints.begin() ; p != mPoints.end() ; ++i, ++p )
	{
		// Evaluate the radial basis function
		Vector::diff(x, *p, xp);
		rbf = this->rbf(xp);
		// Accumulate the result
		fx += m_wx[i]*rbf;
		fy += m_wy[i]*rbf;
		fz += m_wz[i]*rbf;
	}

	// Accumulate further (for the polynomial)
	y[0] = fx + m_wx[i] + x[0]*m_wx[i+1] + x[1]*m_wx[i+2] + x[2]*m_wx[i+3];
	y[1] = fy + m_wy[i] + x[0]*m_wy[i+1] + x[1]*m_wy[i+2] + x[2]*m_wy[i+3];
	y[2] = fz + m_wz[i] + x[0]*m_wz[i+1] + x[1]*m_wz[i+2] + x[2]*m_wz[i+3];
}

//========================================================================================================================================

inline double TPSVectorField3::rbf(const double* x)
{
	double sqrlen = Vector::sqrLength3(x);

	if ( sqrlen == 0.0 )
		return 0.0;

	return sqrlen*log(sqrt(sqrlen));
}

//========================================================================================================================================

#endif /* TPSVECTORFIELD3_H_ */
