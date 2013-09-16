#include "VtkPWRReader.h"
#include <vtkPoints.h>
#include <cstdio>


VtkPWRReader::VtkPWRReader()
{
}

VtkPWRReader::~VtkPWRReader()
{
}

//=======================================================================================================================================

bool VtkPWRReader::read(char* fileName, vtkPoints* out)
{
	FILE *fp = fopen(fileName, "r");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'VtkPWRReader::%s()': can not open file '%s'\n", __func__, fileName);
		return false;
	}

// PRINT
	printf("'VtkPWRReader::%s()': Loading point file '%s' ... ", __func__, fileName); fflush(stdout);
// PRINT END

	char line[512];	double p[3];

	while ( fgets(line, 511, fp)  )
	{
		if ( line[0] == '#' )
			continue;
		sscanf(line, "%lf %lf %lf\n", &p[0], &p[1], &p[2]);
// PRINT
//		printf("(%f, %f, %f)\n", p[0], p[1], p[2]);
// PRINT END
		out->InsertNextPoint(p);
	}
	fclose(fp);

// PRINT
	printf("%i points.\n", (int)out->GetNumberOfPoints());
// PRINT END

	return true;
}

//=======================================================================================================================================
