#include "VtkSVSReader.h"
#include <BasicTools/LinearAlgebra/Matrix.h>

using namespace tum;

VtkSVSReader::VtkSVSReader()
{
}

VtkSVSReader::~VtkSVSReader()
{
}

//=======================================================================================================================================

bool VtkSVSReader::read(char* fileName, vtkPoints* out, double** bbox, double** range)
{
	FILE *fp = fopen(fileName, "r");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'VtkSVSReader::%s()': can not open file '%s'\n", __func__, fileName);
		return false;
	}

	printf("VtkSVSReader::%s(): Loading point file '%s' ... \n", __func__, fileName); fflush(stdout);

	char line[VTK_SVS_READER_MAX_LINE_LENGTH], word[VTK_SVS_READER_MAX_WORD_LENGTH];
	double p[3];
	Matrix bb(3, 2);

	// Initialization cycle
	while ( fgets(line, VTK_SVS_READER_MAX_LINE_LENGTH, fp) )
	{
		if ( !this->isBeginOfNumber(line[0]) )
			continue;

		// Read the word at the end of the current line
		this->getLastWord(line, word, VTK_SVS_READER_MAX_WORD_LENGTH);
		// Check if the word/line is interesting for us
		if ( strcmp(word, VTK_SVS_READER_INVALID_LINE_STRING_1) == 0 || strcmp(word, VTK_SVS_READER_INVALID_LINE_STRING_2) == 0 )
			continue;

		// We have a valid line -> scan it
		sscanf(line, "%lf %lf %lf\n", &p[0], &p[1], &p[2]);

		// Check if 'p' is in the range
		if ( range ) if ( !this->isInside(p, range) )
			continue;

		bb.m[0][0] = bb.m[0][1] = p[0];
		bb.m[1][0] = bb.m[1][1] = p[1];
		bb.m[2][0] = bb.m[2][1] = p[2];

		// Insert the first point
		out->InsertNextPoint(p);

		break; // break because this is only the initialization cycle
	}

	while ( fgets(line, VTK_SVS_READER_MAX_LINE_LENGTH, fp) )
	{
		if ( !this->isBeginOfNumber(line[0]) )
			continue;

		// Read the word at the end of the current line
		this->getLastWord(line, word, VTK_SVS_READER_MAX_WORD_LENGTH);
		// Check if the word/line is interesting for us
		if ( strcmp(word, VTK_SVS_READER_INVALID_LINE_STRING_1) == 0 || strcmp(word, VTK_SVS_READER_INVALID_LINE_STRING_2) == 0 )
			continue;

		// We have a valid line -> scan it
		sscanf(line, "%lf %lf %lf\n", &p[0], &p[1], &p[2]);

		// Check if 'p' is in the range
		if ( range ) if ( !this->isInside(p, range) )
			continue;

		// X bounds
		if      ( p[0] < bb.m[0][0] ) bb.m[0][0] = p[0];
		else if ( p[0] > bb.m[0][1] ) bb.m[0][1] = p[0];
		// Y bounds
		if      ( p[1] < bb.m[1][0] ) bb.m[1][0] = p[1];
		else if ( p[1] > bb.m[1][1] ) bb.m[1][1] = p[1];
		// Z bounds
		if      ( p[2] < bb.m[2][0] ) bb.m[2][0] = p[2];
		else if ( p[2] > bb.m[2][1] ) bb.m[2][1] = p[2];

		// Save the point
		out->InsertNextPoint(p);
	}
	fclose(fp);

	char label[] = "VtkSVSReader: SVS bounding box";
	bb.print(stdout, label);
	if ( bbox ) bb.copyto(bbox);

	printf("VtkSVSReader::%s(): '%s' has %i point(s).\n", __func__, fileName, (int)out->GetNumberOfPoints());
	fflush(stdout);

	return true;
}

//=======================================================================================================================================
