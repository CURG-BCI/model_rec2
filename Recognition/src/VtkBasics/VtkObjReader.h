#ifndef VTKOBJREADER_H_
#define VTKOBJREADER_H_

#include "VtkPolyData.h"
#include <vector>

class VtkObjReader
{
public:
	VtkObjReader();
	virtual ~VtkObjReader();

	void read(char **fileNames, char *folderName, int numOfFiles, std::vector<VtkPolyData*> &objects);

	/**
	 * @param src should be '\0'-terminated and @param dst should have the length(@param src)+1 long.
	 * The additional 1 is for the '\0'-character, that is appended to @param dst. Characters with
	 * a decimal value <= 32 are ignored (take a look in an ASCII-table).
	 */
	void copy(const char *src, char *dst);

protected:
};

#endif /*VTKOBJREADER_H_*/
