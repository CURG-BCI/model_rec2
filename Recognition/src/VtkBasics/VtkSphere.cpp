#include "VtkSphere.h"
#include <vtkAbstractPicker.h>

VtkSphere::VtkSphere(double radius, const double *center, double r, double g, double b,
		int phiRes, int thetaRes)
{
	mSrc = vtkSphereSource::New();
	mMapper = vtkPolyDataMapper::New();
	mActor = vtkActor::New();

	if ( center )
		mSrc->SetCenter(center[0], center[1], center[2]);
	else
		mSrc->SetCenter(0.0, 0.0, 0.0);

	mSrc->SetRadius(radius);
	mSrc->SetThetaResolution(thetaRes);
	mSrc->SetPhiResolution(phiRes);

	mMapper->SetInput(mSrc->GetOutput());
	mActor->SetMapper(mMapper);

	mActor->GetProperty()->SetColor(r, g, b);
	mActor->VisibilityOn();

	mMoveToPicked3DPoint = false;
}

VtkSphere::~VtkSphere()
{
	mSrc->Delete();
	mMapper->Delete();
	mActor->Delete();
}

//=========================================================================================================================

void VtkSphere::Execute(vtkObject *caller, unsigned long eventId, void *callData)
{
	vtkRenderWindowInteractor *renwi;
	renwi = dynamic_cast<vtkRenderWindowInteractor*>(caller);
	
	if ( !renwi )
		return;

	switch ( eventId )
	{
		case vtkCommand::EndPickEvent:
			double pickpos[3];
			renwi->GetPicker()->GetPickPosition(pickpos);
			printf("pickpos = (%f, %f, %f)\n", pickpos[0], pickpos[1], pickpos[2]); fflush(stdout);

			if ( mMoveToPicked3DPoint )
			{
				mSrc->SetCenter(pickpos);
				renwi->Render();
			}
			break;

		default:
			break;
	}
}

//=========================================================================================================================

void VtkSphere::registerForPickEvents(vtkRenderWindowInteractor *observable)
{
	observable->AddObserver(vtkCommand::EndPickEvent, this, 0.0/*priority*/);
}

//=========================================================================================================================

void VtkSphere::moveToPicked3DPointOn()
{
	this->mMoveToPicked3DPoint = true;
}

//=========================================================================================================================

void VtkSphere::moveToPicked3DPointOff()
{
	this->mMoveToPicked3DPoint = false;
}

//=========================================================================================================================
