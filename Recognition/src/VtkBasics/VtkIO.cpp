#include "VtkIO.h"

//==============================================================================================================================================

void VtkIO::print(FILE *fp, vtkMatrix4x4 *m, char *label)
{
	if ( label )
		fprintf(fp, "--- %s ---\n", label);

	fprintf(fp, "%.15f %.15f %.15f %.15f\n%.15f %.15f %.15f %.15f\n%.15f %.15f %.15f %.15f\n%.15f %.15f %.15f %.15f\n",
			m->GetElement(0,0), m->GetElement(0,1), m->GetElement(0,2),  m->GetElement(0,3),
			m->GetElement(1,0), m->GetElement(1,1), m->GetElement(1,2),  m->GetElement(1,3),
			m->GetElement(2,0), m->GetElement(2,1), m->GetElement(2,2),  m->GetElement(2,3),
			m->GetElement(3,0), m->GetElement(3,1), m->GetElement(3,2),  m->GetElement(3,3));
	if ( label )
		fprintf(fp, "--- --- ---\n");
}

//==============================================================================================================================================

#define MAX_NUM_LEN		512

bool VtkIO::loadPointsFromMFile(char *fileName, vtkPoints *points)
{
	FILE *fp = fopen(fileName, "rb");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR on opening file!\n");
		return false;
	}
	if ( points->GetNumberOfPoints() )
	{
		fprintf(stderr, "Input argument 'points' should be empty!\n");
		return false;
	}

	// Get size of file in bytes
	fseek(fp, 0, SEEK_END);
	int numOfBytes = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	char *bytes = new char[numOfBytes];
	if ( bytes == NULL )
	{
		fprintf(stderr, "ERROR: can not allocate %i bytes!\n", numOfBytes);
		return false;
	}

	// Load all bytes in memory
	int readBytes = fread(bytes, 1, numOfBytes, fp);

	if ( readBytes != numOfBytes )
	{
		fprintf(stderr, "ERROR: has read only %i bytes instead of %i!\n", readBytes, numOfBytes);
		delete[] bytes;
		return false;
	}

	int i = 0, is = 0, n = 0, nPts = 0;
	char str[MAX_NUM_LEN];
	double p[3];

	// Check for the begining of a decimal number
	while ( !VtkIO::isBeginningOfDecimal(bytes[i]) )
		++i;
	// Assume that x-coordinates begin
	do
	{
		for ( is = 0 ; is < MAX_NUM_LEN-1 && VtkIO::isPartOfDecimal(bytes[i]) ; ++i, ++is )
			str[is] = bytes[i];
		str[is] = '\0';

		if ( is != 0 ) // Convert the x-coordinate to a double
		{
			p[0] = atof(str);
			points->InsertPoint(nPts++, p);
//			printf("x: %s\n", str); fflush(stdout);
//			printf("x: %f\n", p[0]); fflush(stdout);
		}

		while ( isspace(bytes[i]) )
			++i;
	}
	while (VtkIO::isBeginningOfDecimal(bytes[i]));

	n = 0;
	// Check for the begining of a decimal number
	while ( !VtkIO::isBeginningOfDecimal(bytes[i]) )
		++i;
	// Assume that y-coordinates begin
	do
	{
		for ( is = 0 ; is < MAX_NUM_LEN-1 && VtkIO::isPartOfDecimal(bytes[i]) ; ++i, ++is )
			str[is] = bytes[i];
		str[is] = '\0';

		if ( is != 0 ) // Convert the y-coordinate to a double
		{
			points->GetPoint(n, p);
			p[1] = atof(str);
			points->SetPoint(n++, p);
//			printf("y: %s\n", str); fflush(stdout);
//			printf("y: %f\n", p[1]); fflush(stdout);
		}

		while ( isspace(bytes[i]) )
			++i;
	}
	while (VtkIO::isBeginningOfDecimal(bytes[i]));

	if ( n != nPts )
	{
		fprintf(stderr, "ERROR invalid input file!\n");
		delete[] bytes;
		return false;
	}

	n = 0;
	// Check for the begining of a decimal number
	while ( !VtkIO::isBeginningOfDecimal(bytes[i]) )
		++i;
	// Assume that z-coordinates begin
	do
	{
		for ( is = 0 ; is < MAX_NUM_LEN-1 && VtkIO::isPartOfDecimal(bytes[i]) ; ++i, ++is )
			str[is] = bytes[i];
		str[is] = '\0';

		if ( is != 0 ) // Convert the z-coordinate to a double
		{
			points->GetPoint(n, p);
			p[2] = atof(str);
			points->SetPoint(n++, p);
//			printf("z: %s\n", str); fflush(stdout);
//			printf("z: %f\n", p[2]); fflush(stdout);
		}

		while ( isspace(bytes[i]) )
			++i;
	}
	while (VtkIO::isBeginningOfDecimal(bytes[i]));

	if ( n != nPts )
	{
		fprintf(stderr, "ERROR invalid input file!\n"); fflush(stdout);
		delete[] bytes;
		return false;
	}

	// Clean up
	delete[] bytes;

	return true;
}

//==============================================================================================================================================
