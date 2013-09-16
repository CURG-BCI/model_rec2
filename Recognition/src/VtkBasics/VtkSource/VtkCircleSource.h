/*
 * VtkCircleSource.h
 *
 *  Created on: Jan 12, 2010
 *      Author: papazov
 */

#ifndef VTKCIRCLESOURCE_H_
#define VTKCIRCLESOURCE_H_

#include <vtkPoints.h>

class VtkCircleSource
{
public:
	VtkCircleSource();
	virtual ~VtkCircleSource();

	/** Samples 'numOfPoints' 3d points from a circle with 'center' and 'radius' and saves them in 'out'.
	  * The circle lies in the plane parallel to the xy-plane which contains the point 'center'. */
	void getPointsOnCircle(double* center, double radius, int numOfPoints, vtkPoints* out);
};

#endif /* VTKCIRCLESOURCE_H_ */
