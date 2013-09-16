/*
 * VtkCircleSource.cpp
 *
 *  Created on: Jan 12, 2010
 *      Author: papazov
 */

#include "VtkCircleSource.h"
#include <cmath>

VtkCircleSource::VtkCircleSource()
{
}

VtkCircleSource::~VtkCircleSource()
{
}

//============================================================================================================================

void VtkCircleSource::getPointsOnCircle(double* center, double radius, int numOfPoints, vtkPoints* out)
{
	double angle = 0.0, p[3], step = (2.0*M_PI)/(double)(numOfPoints);

	for ( int i = 0 ; i < numOfPoints ; ++i, angle += step )
	{
		p[0] = center[0] + radius*cos(angle);
		p[1] = center[1] + radius*sin(angle);
		p[2] = center[2];
		out->InsertNextPoint(p);
	}
}

//============================================================================================================================
