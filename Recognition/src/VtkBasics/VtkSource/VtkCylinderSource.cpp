#include "VtkCylinderSource.h"
#include <BasicTools/Stochastics/RandomGenerator.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <cmath>


VtkCylinderSource::VtkCylinderSource()
{
	this->cappingOff();
	mRadius = 0.5;
	mZMin = -1.0;
	mZMax =  1.0;
	mt[0] = mt[1] = mt[2] = 0.0;
}

VtkCylinderSource::~VtkCylinderSource()
{
}

//============================================================================================================================

void VtkCylinderSource::getSurfacePointsAtRandom(int numOfPoints, vtkPoints* out)
{
	RandomGenerator random;

	double thresh, angle, r, p[3], PIx2 = 2.0*M_PI;
	double height = mZMax - mZMin;

	if ( mCapping )
	{
		double capsurf = M_PI*mRadius*mRadius, sidesurf = height*2.0*M_PI*mRadius;
		thresh = sidesurf/(capsurf+sidesurf);
	}
	else
		thresh = 1.0;

	for ( int i = 0 ; i < numOfPoints ; ++i )
	{
		if ( random.getRandomNumberInUnitInterval() <= thresh )
		{
			// Get a random point on the side of the cylinder
			angle = random.getRandomNumberInInterval(0.0, PIx2);
			p[0] = mRadius*cos(angle) + mt[0];
			p[1] = mRadius*sin(angle) + mt[1];
			p[2] = random.getRandomNumberInInterval(mZMin, mZMax) + mt[2];
			out->InsertNextPoint(p);
		}
		else
		{
			// Get two random points on each cap of the cylinder
			r = random.getRandomNumberInInterval(0.0, mRadius);
			angle = random.getRandomNumberInInterval(0.0, PIx2);
			p[0] = r*cos(angle) + mt[0];
			p[1] = r*sin(angle) + mt[1];
			p[2] = mZMin +  + mt[2];
			out->InsertNextPoint(p);
			// Get next random point
			r = random.getRandomNumberInInterval(0.0, mRadius);
			angle = random.getRandomNumberInInterval(0.0, PIx2);
			p[0] = r*cos(angle) + mt[0];
			p[1] = r*sin(angle) + mt[1];
			p[2] = mZMax + mt[2];
			out->InsertNextPoint(p);
		}
	}
}

//============================================================================================================================

void VtkCylinderSource::getSurfacePointsWithNormals(int heightRes, int widthRes, vtkPolyData* out)
{
	double angle_step = 2.0*M_PI/(double)widthRes, height_step = (mZMax-mZMin)/(double)(heightRes-1);
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkDoubleArray* normals = vtkDoubleArray::New();
	  normals->SetNumberOfComponents(3);
	int i, j;
	double p[3], angle;

	for ( i = 0, p[2] = mZMin ; i < heightRes ; ++i, p[2] += height_step )
	{
		for ( j = 0, angle = 0.0 ; j < widthRes ; ++j, angle += angle_step )
		{
			p[0] = cos(angle);
			p[1] = sin(angle);
			normals->InsertNextTuple3(p[0], p[1], 0.0);
			p[0] *= mRadius;
			p[1] *= mRadius;
			points->InsertNextPoint(p);
		}
	}
	// Save the result
	out->SetPoints(points);
	out->GetPointData()->SetNormals(normals);
	// Clean up
	points->Delete();
	normals->Delete();
}

//============================================================================================================================
