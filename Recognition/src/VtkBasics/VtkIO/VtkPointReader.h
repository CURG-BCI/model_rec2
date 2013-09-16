#ifndef VTKPOINTREADER_H_
#define VTKPOINTREADER_H_

#include <vtkPoints.h>


class VtkPointReader
{
public:
	VtkPointReader();
	virtual ~VtkPointReader();

	virtual bool read(char* fileName, vtkPoints* out) = 0;
};

#endif /*VTKPOINTREADER_H_*/
