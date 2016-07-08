/*
 * VtkPNTReader.cpp
 *
 *  Created on: Oct 30, 2010
 *      Author: papazov
 */

#include "VtkPNTReader.h"
#include <BasicTools/DataStructures/PtrArray2d.h>
#include <vtkCellArray.h>
#include <fstream>
#include <cstdio>

using namespace std;

VtkPNTReader::VtkPNTReader()
{
}

VtkPNTReader::~VtkPNTReader()
{
}

//=====================================================================================================================

bool VtkPNTReader::read(char* filename, vtkPoints* out)
{
	int numOfPoints;
	fstream file;

	// Try to open the file
	file.open(filename, std::ios_base::in|std::ios_base::binary);
	if ( file.fail() )
	{
		fprintf(stderr, "ERROR in 'VtkPNTReader::%s()': can not open file '%s'\n",
				__func__, filename); fflush(stderr);
		return false;
	}

	// How many points?
	file.read((char*)&numOfPoints, sizeof(int));
	// Alloc memory
	PNTPoint *data = new PNTPoint[numOfPoints];
	// Load the points
	file.read((char*)data, numOfPoints*sizeof(PNTPoint));
	file.close();

	// Convert to vtkPoints
	out->SetNumberOfPoints(numOfPoints);
	for ( int i = 0 ; i < numOfPoints ; ++i )
		out->SetPoint(i, data[i].px, data[i].py, data[i].pz);

	// Cleanup
	delete[] data;

	return true;
}

//=====================================================================================================================

bool VtkPNTReader::read(char* filename, vtkPolyData* out)
{
	int numOfPoints;
	fstream file;

	// Try to open the file
	file.open(filename, std::ios_base::in|std::ios_base::binary);
	if ( file.fail() )
	{
		fprintf(stderr, "ERROR in 'VtkPNTReader::%s()': can not open file '%s'\n",
				__func__, filename); fflush(stderr);
		return false;
	}

	// How many points?
	file.read((char*)&numOfPoints, sizeof(int));
	// Alloc memory
	PNTPoint *data = new PNTPoint[numOfPoints];
	// Load the points
	file.read((char*)data, numOfPoints*sizeof(PNTPoint));
	file.close();

	unsigned int width = 0, height = 0;

	vtkCellArray *edges = vtkCellArray::New(), *trias = vtkCellArray::New();
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	vtkIdType edge_ids[2], tria_ids[3];

	// Compute the width and height of the range image
	for ( int i = 0 ; i < numOfPoints ; ++i )
	{
		// Check the width and the height
		if ( data[i].x > width  ) width  = data[i].x;
		if ( data[i].y > height ) height = data[i].y;
	}
	++width; ++height;
	printf("VtkPNTReader::%s(): image size = [%u, %u]\n", __func__, width, height);

	// Create the image
	PtrArray2d<PNT> img(width, height);
	PNT*** pixels = img.mData;

	// Initialize the image
	for ( int i = 0 ; i < numOfPoints ; ++i )
	{
		PNT* pnt = new PNT();
		pnt->point = &data[i];
		// Save the pixel
		if ( pixels[data[i].x][data[i].y] )
		{
			printf("WARNING in 'VtkPNTReader::%s()': range image coordinates of the points are not unique!\n", __func__);
			fflush(stdout);
		}
		pixels[data[i].x][data[i].y] = pnt;
	}

	// Insert the points
	for ( unsigned int j = 0, k = 0 ; j < height ; ++j )
	{
		for ( unsigned int i = 0 ; i < width ; ++i )
		{
			if ( !pixels[i][j] )
				continue;

			points->InsertNextPoint(pixels[i][j]->point->px, pixels[i][j]->point->py, pixels[i][j]->point->pz);
			pixels[i][j]->id = k++;
		}
	}

	// Now create the edges and triangles
	for ( unsigned int j = 0 ; j < height-1 ; ++j )
	{
		for ( unsigned int i = 0 ; i < width-1 ; ++i )
		{
			if ( !pixels[i][j] )
				continue;

			// First triangle and first edges
			if ( pixels[i][j+1] && pixels[i+1][j+1] )
			{
				tria_ids[0] = pixels[i][j]->id;
				tria_ids[1] = pixels[i][j+1]->id;
				tria_ids[2] = pixels[i+1][j+1]->id;
				trias->InsertNextCell(3, tria_ids);
			}
			else if ( pixels[i][j+1] )
			{
				edge_ids[0] = pixels[i][j]->id;
				edge_ids[1] = pixels[i][j+1]->id;
				edges->InsertNextCell(2, edge_ids);
			}
			else if ( pixels[i+1][j+1] )
			{
				edge_ids[0] = pixels[i][j]->id;
				edge_ids[1] = pixels[i+1][j+1]->id;
				edges->InsertNextCell(2, edge_ids);
			}
			// Second triangle and second group of edges
			if ( pixels[i+1][j] && pixels[i+1][j+1] )
			{
				tria_ids[0] = pixels[i][j]->id;
				tria_ids[1] = pixels[i+1][j]->id;
				tria_ids[2] = pixels[i+1][j+1]->id;
				trias->InsertNextCell(3, tria_ids);
			}
			else if ( pixels[i+1][j] )
			{
				edge_ids[0] = pixels[i][j]->id;
				edge_ids[1] = pixels[i+1][j]->id;
				edges->InsertNextCell(2, edge_ids);
			}
		}
	}

	// Save the points, edges and triangles
	out->SetPoints(points);
	out->SetLines(edges);
	out->SetPolys(trias);

	// Cleanup
	delete[] data;
	points->Delete();
	edges->Delete();
	trias->Delete();

	return true;
}

//=====================================================================================================================
