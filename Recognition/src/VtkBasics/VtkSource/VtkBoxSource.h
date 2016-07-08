#ifndef VTKBOXSOURCE_H_
#define VTKBOXSOURCE_H_

#include <vtkPoints.h>


class VtkBoxSource
{
public:
	VtkBoxSource();
	virtual ~VtkBoxSource();

	void setX(double min, double max){ mx[0] = min; mx[1] = max;}
	void setY(double min, double max){ my[0] = min; my[1] = max;}
	void setZ(double min, double max){ mz[0] = min; mz[1] = max;}
	void setBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);

	void getSurfacePointsAtRandom(int numOfPoints, vtkPoints* out);

protected:
	double mx[2], my[2], mz[2];
};

#endif /*VTKBOXSOURCE_H_*/
