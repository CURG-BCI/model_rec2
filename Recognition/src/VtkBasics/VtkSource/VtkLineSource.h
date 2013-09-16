#ifndef VTKLINESOURCE_H_
#define VTKLINESOURCE_H_

#include <vtkPoints.h>


class VtkLineSource
{
public:
	VtkLineSource();
	virtual ~VtkLineSource();

	void getPointsOnLine(const double* p1, const double* p2, int numOfPoints, vtkPoints* out);
};

#endif /*VTKLINESOURCE_H_*/
