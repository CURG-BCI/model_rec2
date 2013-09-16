#ifndef VTKIO_H_
#define VTKIO_H_

#include <stdio.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>

class VtkIO
{
public:
	static void print(FILE *fp, vtkMatrix4x4 *m, char *label = NULL);
	static bool loadPointsFromMFile(char *fileName, vtkPoints *points);
	
	inline static bool isPartOfDecimal(char c);
	inline static bool isBeginningOfDecimal(char c);
};

//=== inline stuff ======================================================================================================

inline bool VtkIO::isPartOfDecimal(char c)
{
	return ( ('0' <= c && c <= '9') || c == '.' || c == '+' || c == '-' || c == 'e' || c == 'E' );  
}

inline bool VtkIO::isBeginningOfDecimal(char c)
{
	return ( ('0' <= c && c <= '9') || c == '.' || c == '+' || c == '-' );  
}

#endif /*VTKIO_H_*/
