#include "VtkLineSource.h"
#include <BasicTools/LinearAlgebra/Vector.h>

using namespace tum;

VtkLineSource::VtkLineSource()
{
}

VtkLineSource::~VtkLineSource()
{
}

//==================================================================================================================================

void VtkLineSource::getPointsOnLine(const double* p1, const double* p2, int numOfPoints, vtkPoints* out)
{
	double line_part = 0.0, step = 1.0/(double)(numOfPoints-1);
	double line[3], p[3];

	Vector::diff(p2, p1, line);

	for ( int i = 0 ; i < numOfPoints ; ++i, line_part += step )
	{
		p[0] = p1[0] + line_part*line[0];
		p[1] = p1[1] + line_part*line[1];
		p[2] = p1[2] + line_part*line[2];
		out->InsertNextPoint(p);
	}
}

//==================================================================================================================================
