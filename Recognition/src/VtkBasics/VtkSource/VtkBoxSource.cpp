#include "VtkBoxSource.h"
#include <BasicTools/Stochastics/RandomGenerator.h>


VtkBoxSource::VtkBoxSource()
{
	mx[0] = -1.0; mx[1] = 1.0;
	my[0] = -1.0; my[1] = 1.0;
	mz[0] = -1.0; mz[1] = 1.0;
}

VtkBoxSource::~VtkBoxSource()
{
}

//============================================================================================================================

void VtkBoxSource::setBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
	mx[0] = xmin; mx[1] = xmax;
	my[0] = ymin; my[1] = ymax;
	mz[0] = zmin; mz[1] = zmax;
}

//============================================================================================================================

void VtkBoxSource::getSurfacePointsAtRandom(int numOfPoints, vtkPoints* out)
{
	RandomGenerator random;
	double min[3], max[3], p[3];

	min[0] = mx[0]; max[0] = mx[1];
	min[1] = my[0]; max[1] = my[1];
	min[2] = mz[0]; max[2] = mz[1];

	for ( int i = 0 ; i < numOfPoints ; ++i )
	{
		random.getRandomPointWithinBox(min, max, p, 3/*dimension*/);
		// Which id to change
		int id = random.getRandomInteger(0, 2);
		// Set to which side of the box?
		if ( random.getRandomInteger(0, 1) ) p[id] = min[id];
		else p[id] = max[id];
		// Save the point
		out->InsertNextPoint(p);
	}
}

//============================================================================================================================
