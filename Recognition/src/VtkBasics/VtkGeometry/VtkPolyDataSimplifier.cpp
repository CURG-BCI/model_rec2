#include "VtkPolyDataSimplifier.h"
#include <vtkDecimatePro.h>
#include <vtkTriangleFilter.h>

VtkPolyDataSimplifier::VtkPolyDataSimplifier()
{
}

VtkPolyDataSimplifier::~VtkPolyDataSimplifier()
{
}

//==============================================================================================================================

vtkPolyData* VtkPolyDataSimplifier::simplify(vtkPolyData *obj, double reductionLevel)
{
	vtkTriangleFilter *tria = vtkTriangleFilter::New();
	  tria->SetInput(obj);

	vtkDecimatePro *decPro = vtkDecimatePro::New();
	  decPro->PreserveTopologyOn();
	  decPro->SetMaximumError(VTK_DOUBLE_MAX);
	  decPro->SetTargetReduction(reductionLevel);
	  decPro->SetInputConnection(tria->GetOutputPort());
	  decPro->Update();

	vtkPolyData *mesh = decPro->GetOutput();
	  mesh->SetReferenceCount(mesh->GetReferenceCount() + 1);

	// Clean up
	decPro->Delete();
	tria->Delete();

	return mesh;
}

//==============================================================================================================================
