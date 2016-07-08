/*
 * Rotations.h
 *
 *  Created on: Mar 29, 2010
 *      Author: papazov
 */

#ifndef ROTATIONS_H_
#define ROTATIONS_H_

#include <cmath>

class Rotations
{
public:
	Rotations(){}
	virtual ~Rotations(){}

	/** This method computes a rotation matrix for a rotation of amount 'theta' about a rotation axis n.
	  * n is defined by its spherical coordinates 'phi' and 'psi', where 'phi' is the angle between the x-axis and
	  * the projection of n onto the xy-plane and 'psi' is the angle between the z-axis and n. More precisely
	  * we have n = (cos(phi)sin(psi), sin(phi)sin(psi), cos(psi)). All angles have to be in radians. The rotation
	  * matrix is saved in 'm'. Furthermore, the angles should lie in the following intervals:
	  * 'phi' in [0, 2*PI),
	  * 'psi' in [0, PI] and
	  * 'theta' in [0, PI]. */
	inline void getRotationMatrixFromAxisAngle(double phi, double psi, double theta, double** m);
};

//=== inline methods ==========================================================================================================================

void Rotations::getRotationMatrixFromAxisAngle(double phi, double psi, double theta, double** m)
{
	double n1 = cos(phi)*sin(psi), n2 = sin(phi)*sin(psi), n3 = cos(psi), c = cos(theta), s = sin(theta);
	double omc = 1.0 - c;
	double n1n2omc = n1*n2*omc, n1n3omc = n1*n3*omc, n2n3omc = n2*n3*omc, n1sin = n1*s, n2sin = n2*s, n3sin = n3*s;

	m[0][0] = c + n1*n1*omc;    m[0][1] = n1n2omc - n3sin;  m[0][2] = n1n3omc + n2sin;
	m[1][0] = n1n2omc + n3sin;  m[1][1] = c + n2*n2*omc;    m[1][2] = n2n3omc - n1sin;
	m[2][0] = n1n3omc - n2sin;  m[2][1] = n2n3omc + n1sin;  m[2][2] = c + n3*n3*omc;
}

//=============================================================================================================================================

#endif /* ROTATIONS_H_ */
