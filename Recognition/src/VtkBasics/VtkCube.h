#ifndef VTKCUBE_H_
#define VTKCUBE_H_

#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

class VtkCube
{
public:
	VtkCube();
	virtual ~VtkCube();

	void setXLength(double len){ mCubeSrc->SetXLength(len);}
	void setYLength(double len){ mCubeSrc->SetYLength(len);}
	void setZLength(double len){ mCubeSrc->SetZLength(len);}
	void setCenter(double x, double y, double z){ mCubeSrc->SetCenter(x, y, z);}

	vtkActor* getActor(){ return mActor;}
	vtkPolyData* getPolyData(){ return mCubeSrc->GetOutput();}

protected:
	vtkCubeSource *mCubeSrc;
	vtkPolyDataMapper *mMapper;
	vtkActor *mActor;
};

#endif /*VTKCUBE_H_*/
