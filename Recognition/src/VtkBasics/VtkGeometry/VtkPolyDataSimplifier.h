#ifndef VTKPOLYDATASIMPLIFIER_H_
#define VTKPOLYDATASIMPLIFIER_H_

#include <vtkPolyData.h>

class VtkPolyDataSimplifier
{
public:
	VtkPolyDataSimplifier();
	virtual ~VtkPolyDataSimplifier();

	vtkPolyData* simplify(vtkPolyData *obj, double reductionLevel);
};

#endif /*VTKPOLYDATASIMPLIFIER_H_*/
