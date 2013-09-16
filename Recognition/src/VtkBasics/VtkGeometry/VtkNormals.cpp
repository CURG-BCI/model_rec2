#include "VtkNormals.h"
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>

VtkNormals::VtkNormals()
{
}

VtkNormals::~VtkNormals()
{
}

//==================================================================================================================================

void VtkNormals::getPointNormals(vtkPolyData* input, vtkPolyData* out, double scaleFactor)
{
	vtkDataArray* normals = input->GetPointData()->GetNormals();

	if ( !normals )
		return;

	vtkCellArray *normalLines = vtkCellArray::New();
	vtkPoints *normalPoints = vtkPoints::New(VTK_DOUBLE);
	vtkIdType ids[2] = {0, 1};
	double p[3], n[3];

	for ( int i = 0 ; i < input->GetNumberOfPoints() ; ++i, ids[0] += 2, ids[1] += 2 )
	{
		// Get the normal
		normals->GetTuple(i, n);
		// Scale it
		n[0] *= scaleFactor;
		n[1] *= scaleFactor;
		n[2] *= scaleFactor;

		// Insert the first point (where the normal starts)
		input->GetPoint(i, p);
		normalPoints->InsertNextPoint(p);
		// Compute the second point (where the normal ends)
		p[0] += n[0];
		p[1] += n[1];
		p[2] += n[2];
		// Insert the second point (where the normal ends)
		normalPoints->InsertNextPoint(p);
		// Save the line
		normalLines->InsertNextCell(2, ids);
	}

	// Save the info
	out->SetPoints(normalPoints);
	out->SetLines(normalLines);
	// Cleanup
	normalPoints->Delete();
	normalLines->Delete();
}

//==================================================================================================================================
