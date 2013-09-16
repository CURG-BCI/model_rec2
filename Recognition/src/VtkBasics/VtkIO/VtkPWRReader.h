#ifndef VTKPWRREADER_H_
#define VTKPWRREADER_H_

#include "VtkPointReader.h"
#include <vtkPoints.h>


class VtkPWRReader: public VtkPointReader
{
public:
	VtkPWRReader();
	virtual ~VtkPWRReader();

	bool read(char* fileName, vtkPoints* out);
};

#endif /*VTKPWRREADER_H_*/
