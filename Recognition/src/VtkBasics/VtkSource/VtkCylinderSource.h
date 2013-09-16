#ifndef VTKCYLINDERSOURCE_H_
#define VTKCYLINDERSOURCE_H_

#include <vtkPolyData.h>


class VtkCylinderSource
{
public:
	VtkCylinderSource();
	virtual ~VtkCylinderSource();

	void setRadius(double radius){ mRadius = radius;}
	void setZ(double zmin, double zmax){ mZMin = zmin; mZMax = zmax;}
	void cappingOn(){ mCapping = true;}
	void cappingOff(){ mCapping = false;}

	void getSurfacePointsAtRandom(int numOfPoints, vtkPoints* out);
	void setTranslation(double t[3]){ mt[0] = t[0]; mt[1] = t[1]; mt[2] = t[2];}
	void setTranslation(double x, double y, double z){ mt[0] = x; mt[1] = y; mt[2] = z;}

	void getSurfacePointsWithNormals(int heightRes, int widthRes, vtkPolyData* out);

protected:
	double mRadius, mZMin, mZMax, mt[3];
	bool mCapping;
};

#endif /*VTKCYLINDERSOURCE_H_*/
