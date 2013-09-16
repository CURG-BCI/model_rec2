#ifndef VTKSVSREADER_H_
#define VTKSVSREADER_H_

//#include "VtkPointReader.h"
#include <vtkPoints.h>
#include <cstring>
#include <list>

using namespace std;


#define VTK_SVS_READER_MAX_LINE_LENGTH			512
#define VTK_SVS_READER_MAX_WORD_LENGTH			64
#define VTK_SVS_READER_INVALID_LINE_STRING_1		"d-1"
#define VTK_SVS_READER_INVALID_LINE_STRING_2		"d0"


class VtkSVSReader//: public VtkPointReader
{
public:
	VtkSVSReader();
	virtual ~VtkSVSReader();

	/** Inherited from 'VtkPointReader'. 'bbox' should be NULL or a 3x2 matrix. In the latter case
	  * the bounding box of the points will be saved in 'bbox'.
	  * 'range' should be NULL or a 3x2 matrix, which defines the maximum extensions the points in
	  * the SVS file can have. Points with coordinates outside 'range' will be ignored, i.e.,
	  * they will not be saved in 'out'. If 'range' is NULL all points will be saved in 'out'. */
	bool read(char* fileName, vtkPoints* out, double** bbox = NULL, double** range = NULL);

	inline bool isBeginOfNumber(char c);
	inline bool getLastWord(char *str, char *out, int outlen);

	/** 'p' has to be a 3dim point and 'box' a 3x2 matrix. Returns 'true' iff 'p' is inside or on 'box'. */
	inline bool isInside(double* p, double** box);
};

//=== inline stuff ===============================================================================================================

bool VtkSVSReader::isBeginOfNumber(char c)
{
	return ( c == '-' || c == '+' || ('0' <= c && c <= '9') || c == '.' );
}

//================================================================================================================================

bool VtkSVSReader::getLastWord(char *str, char *out, int outlen)
{
	int i, j, len = strlen(str);

	// Just ignore the newline-character(s) and the blank(s) at the end of 'str'
	for ( i = len-1 ; (str[i] == '\n' || str[i] == ' ') && i >= 0 ; --i )
		;// do nothing

	if ( i < 0 ) // We have nothing but newline characters and/or blanks
	{
		out[0] = '\0';
		return false;
	}

	list<char> outstr;
	// Insert the string in the right order to the list 'outstr'
	for ( j = 0 ; str[i] != '\n' && str[i] != ' ' && j < outlen-1 && i >= 0 ; --i, ++j )
		outstr.push_front(str[i]);

	list<char>::iterator it;
	// Copy the string from the list to the cstring
	for ( j = 0, it = outstr.begin() ; it != outstr.end() ; ++j, it++ )
		out[j] = *it;
	// Terminate the string
	out[j] = '\0';

	return true;
}

//================================================================================================================================

inline bool VtkSVSReader::isInside(double* p, double** box)
{
	if ( p[0] < box[0][0] || p[0] > box[0][1] ||
         p[1] < box[1][0] || p[1] > box[1][1] ||
         p[2] < box[2][0] || p[2] > box[2][1] )
		return false;

	return true;
}

//================================================================================================================================

#endif /*VTKSVSREADER_H_*/
