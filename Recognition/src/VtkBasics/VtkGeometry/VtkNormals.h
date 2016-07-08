#ifndef VTKNORMALS_H_
#define VTKNORMALS_H_

#include <vtkPolyData.h>

class VtkNormals
{
public:
	VtkNormals();
	virtual ~VtkNormals();

	void getPointNormals(vtkPolyData* input, vtkPolyData* out, double scaleFactor = 1.0);
};

#endif /*VTKNORMALS_H_*/
