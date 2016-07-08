#include "VtkPTSReader.h"
#include <cstdio>


VtkPTSReader::VtkPTSReader()
{
}

VtkPTSReader::~VtkPTSReader()
{
}

//===============================================================================================================================

#define MAX_LINE_LEN 2047

bool VtkPTSReader::read(char* fileName, vtkPoints* out)
{
	FILE *fp = fopen(fileName, "r");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'VtkPTSReader::%s()': can not open file '%s'\n", __func__, fileName);
		return false;
	}

// PRINT
	printf("'VtkPTSReader::%s()': reading point file '%s' ... ", __func__, fileName); fflush(stdout);
// PRINT END

	char line[MAX_LINE_LEN+1]; double p[3];

	while ( fgets(line, MAX_LINE_LEN, fp)  )
	{
		sscanf(line, "%lf %lf %lf\n", &p[0], &p[1], &p[2]);
		out->InsertNextPoint(p);
	}
	fclose(fp);

// PRINT
	printf("OK [%i points].\n", (int)out->GetNumberOfPoints());
// PRINT END

	return true;
}

//===============================================================================================================================
