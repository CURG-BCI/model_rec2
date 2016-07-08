/*
 * VtkOFFReader.h
 *
 *  Created on: Apr 21, 2011
 *      Author: papazov
 */

#ifndef VTKOFFREADER_H_
#define VTKOFFREADER_H_

#include <vtkPolyData.h>

class VtkOFFReader
{
public:
	VtkOFFReader();
	virtual ~VtkOFFReader();

	bool read(const char* filename, vtkPolyData* out);
};

#endif /* VTKOFFREADER_H_ */
