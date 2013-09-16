#ifndef VTKSPHERE_H_
#define VTKSPHERE_H_

#include <vtkSphereSource.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>

class VtkSphere: public vtkCommand
{
public:
	VtkSphere(double radius = 1.0, const double *center = NULL, double r = 0.7, double g = 0.7, double b = 0.7, int phiRes = 8, int thetaRes = 8);
	virtual ~VtkSphere();

	void hide(){ mActor->VisibilityOff();}
	void show(){ mActor->VisibilityOn();}

	void setCenter(const double center[3]){ mSrc->SetCenter(center[0], center[1], center[2]);}
	void setCenter(double x, double y, double z){ mSrc->SetCenter(x, y, z);}
	void getCenter(double *center){ mSrc->GetCenter(center);}
	void setColor(double r, double g, double b){ mActor->GetProperty()->SetColor(r, g, b);}
	vtkActor *getActor(){ return mActor;}

	void setOpacity(double value){ mActor->GetProperty()->SetOpacity(value);}
	void getColor(double rgb[3]){ mActor->GetProperty()->GetColor(rgb);}

	void registerForPickEvents(vtkRenderWindowInteractor *observable);
	void moveToPicked3DPointOn();
	void moveToPicked3DPointOff();
	/**
	 * Inherited from the class vtkCommand.
	 */
	void Execute(vtkObject *caller, unsigned long eventId, void *callData);

protected:
	vtkActor *mActor;
	vtkPolyDataMapper *mMapper;
	vtkSphereSource *mSrc;

	bool mMoveToPicked3DPoint;
};

#endif /*VTKSPHERE_H_*/
