#include "VtkCube.h"

VtkCube::VtkCube()
{
	mCubeSrc = vtkCubeSource::New();
	mMapper = vtkPolyDataMapper::New();
	  mMapper->SetInputConnection(mCubeSrc->GetOutputPort());
	mActor = vtkActor::New();
	  mActor->SetMapper(mMapper);
}

VtkCube::~VtkCube()
{
	mCubeSrc->Delete();
	mMapper->Delete();
	mActor->Delete();
}
