/*
 * VtkPNTReader.h
 *
 *  Created on: Oct 30, 2010
 *      Author: papazov
 */

#ifndef VTKPNTREADER_H_
#define VTKPNTREADER_H_

#include <vtkPoints.h>
#include <vtkPolyData.h>

struct PNTPoint
{
	float px,py,pz;//position in 3d
	float nx,ny,nz;//normal
	unsigned char r,g,b;//color (raw scanner data typically contains very dark gray-scale values from scanner only)
	unsigned int x,y; //camera pixel position
	short pcol; //projector column number
};

class PNT
{
public:
	PNT(){ point = NULL; id = -1;}
	virtual ~PNT(){}
public:
	PNTPoint* point;
	int id;
};

class VtkPNTReader
{
public:
	VtkPNTReader();
	virtual ~VtkPNTReader();

	bool read(char* filename, vtkPoints* out);
	/** Creates a mesh and saves it in 'out'. */
	bool read(char* filename, vtkPolyData* out);
};

#endif /* VTKPNTREADER_H_ */
