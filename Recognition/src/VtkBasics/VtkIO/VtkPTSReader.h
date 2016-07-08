#ifndef VTKPTSREADER_H_
#define VTKPTSREADER_H_

#include <vtkPoints.h>


class VtkPTSReader
{
public:
	VtkPTSReader();
	virtual ~VtkPTSReader();

	bool read(char* fileName, vtkPoints* out);
};

#endif /*VTKPTSREADER_H_*/
