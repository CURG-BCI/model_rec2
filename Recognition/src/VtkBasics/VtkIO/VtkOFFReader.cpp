/*
 * VtkOFFReader.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: papazov
 */

#include "VtkOFFReader.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <cstdio>

VtkOFFReader::VtkOFFReader()
{
}

VtkOFFReader::~VtkOFFReader()
{
}

//===================================================================================================================

#define MAX_LINE_LEN  1024

bool VtkOFFReader::read(const char* filename, vtkPolyData* out)
{
	FILE* fp = fopen(filename, "r");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'VtkOFFReader::%s()': can not open file '%s'\n", __func__, filename);
		return false;
	}

	printf("VtkOFFReader::%s(): Loading file '%s' ", __func__, filename); fflush(stdout);

	int numOfPoints = 0, numOfTrias = 0, numOfLines = 0;
	char line[MAX_LINE_LEN];

	// Initialization cycle
	while ( fgets(line, MAX_LINE_LEN, fp) )
	{
		if ( '0' <= line[0] && line[0] <= '9'  )
		{
			sscanf(line, "%i %i %i\n", &numOfPoints, &numOfTrias, &numOfLines);
			break;
		}
	}
	printf("[%i points, %i triangles] ... ", numOfPoints, numOfTrias); fflush(stdout);

	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray* trias = vtkCellArray::New();
	int i, tria_ids[3];
	vtkIdType vtk_ids[3];
	double p[3];

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		fscanf(fp, "%lf %lf %lf\n", &p[0], &p[1], &p[2]);
		points->InsertNextPoint(p);
	}

	if ( numOfLines > 0 )
	{
		while ( fgets(line, MAX_LINE_LEN, fp) )
		{
			if ( strlen(line) <= 1 )
				continue;

			sscanf(line, "%i %i %i", &tria_ids[0], &tria_ids[1], &tria_ids[2]);
			vtk_ids[0] = tria_ids[0] - 1;
			vtk_ids[1] = tria_ids[1] - 1;
			vtk_ids[2] = tria_ids[2] - 1;
			trias->InsertNextCell(3, vtk_ids);
		}
	}
	else
	{
		int type;
		while ( fgets(line, MAX_LINE_LEN, fp) )
		{
			if ( strlen(line) <= 1 )
				continue;

			sscanf(line, "%i %i %i %i", &type, &tria_ids[0], &tria_ids[1], &tria_ids[2]);
			vtk_ids[0] = tria_ids[0];
			vtk_ids[1] = tria_ids[1];
			vtk_ids[2] = tria_ids[2];
			trias->InsertNextCell(3, vtk_ids);
		}
	}

	if ( (int)trias->GetNumberOfCells() != numOfTrias )
		printf("\n\tWARNING: could read %i triangles (should be %i)!\n",
				(int)trias->GetNumberOfCells(), numOfTrias);

	// Save the result
	out->SetPoints(points);
	out->SetPolys(trias);
	// Cleanup
	points->Delete();
	trias->Delete();
	fclose(fp);

	printf("done.\n");

	return true;
}

//===================================================================================================================
