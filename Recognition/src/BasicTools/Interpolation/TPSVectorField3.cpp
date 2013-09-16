/*
 * TPSVectorField3.cpp
 *
 *  Created on: Feb 24, 2011
 *      Author: papazov
 */

#include "TPSVectorField3.h"
#include "../LinearAlgebra/Matrix.h"

using namespace tum;

TPSVectorField3::TPSVectorField3()
{
	m_wx = m_wy = m_wz = NULL;
	mNumOfPoints = 0;
}

TPSVectorField3::~TPSVectorField3()
{
	if ( m_wx ) delete[] m_wx;
	if ( m_wy ) delete[] m_wy;
	if ( m_wz ) delete[] m_wz;
}

//=============================================================================================================================================

void TPSVectorField3::build(list<const double*>& p, const double* fx, const double* fy, const double* fz)
{
	mPoints.clear();
	mPoints = p;
	mNumOfPoints = (int)p.size();
	// Some variables
	int i, j, msize = mNumOfPoints + 4;
	Matrix A(msize, msize);
	double p_ij[3];
	list<const double*>::iterator p_i, p_j;
	// Cleanup
	if ( m_wx ) delete[] m_wx;
	if ( m_wy ) delete[] m_wy;
	if ( m_wz ) delete[] m_wz;
	// Alloc
	m_wx = new double[msize];
	m_wy = new double[msize];
	m_wz = new double[msize];

	// The ids of the last 4 columns (or last 4 rows)
	int id_0 = mNumOfPoints, id_1 = mNumOfPoints + 1, id_2 = mNumOfPoints + 2, id_3 = mNumOfPoints + 3;

	// Compose the diagonal of the matrix
	for ( i = 0 ; i < msize ; ++i )
		A.m[i][i] = 0.0;

	// Compose the upper left block of the matrix
	for ( i = 0, p_i = p.begin() ; p_i != p.end() ; ++i, ++p_i )
	{
		p_j = p_i;
		for ( j = i+1, ++p_j ; p_j != p.end() ; ++j, ++p_j )
		{
			Vector::diff(*p_i, *p_j, p_ij);
			A.m[i][j] = A.m[j][i] = this->rbf(p_ij);
		}
	}

	// Compose the last 4 columns and rows
	for ( i = 0, p_i = p.begin() ; p_i != p.end() ; ++i, ++p_i )
	{
		A.m[i][id_0] = A.m[id_0][i] = 1.0;
		A.m[i][id_1] = A.m[id_1][i] = (*p_i)[0];
		A.m[i][id_2] = A.m[id_2][i] = (*p_i)[1];
		A.m[i][id_3] = A.m[id_3][i] = (*p_i)[2];
	}

	// The lower right 4x4 block
	for ( i = mNumOfPoints ; i < msize ; ++i )
		for ( j = i+1 ; j < msize ; ++j ) // Note that the diagonal is already initialized
			A.m[i][j] = A.m[j][i] = 0.0;

	// Buffers needed by vtk for the matrix inversion
	double* buffer_d = new double[msize];
	int* buffer_i = new int[msize];
	Matrix Ai(msize, msize);
	// Invert A
	vtkMath::InvertMatrix(A.m, Ai.m, msize, buffer_i, buffer_d);

	// Compute the three arrays with the weighting coefficients
	Matrix::mult(Ai.getm(), msize, mNumOfPoints, fx, m_wx);
	Matrix::mult(Ai.getm(), msize, mNumOfPoints, fy, m_wy);
	Matrix::mult(Ai.getm(), msize, mNumOfPoints, fz, m_wz);

	// Cleanup
	delete[] buffer_d;
	delete[] buffer_i;
}

//=============================================================================================================================================
