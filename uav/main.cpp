#include "Matrix.h"




int main(int argc, char** argv)
{	

	Matrix m1;
	m1.init(2,2);
	m1.Put(0,0, 4.0f);
	m1.Put(0,1, 7.0f);
	m1.Put(1,0, 2.0f);
	m1.Put(1,1, 6.0f);

	Matrix m2;
	m2.init(2,2);
	m2.Put(0,0, 3.0f);
	m2.Put(0,1, 2.0f);
	m2.Put(1,0, 6.0f);
	m2.Put(1,1, 2.0f);

	//Matrix transp = m1.Transpose();

	//Matrix ident = Matrix::Identity(2,2);
	//ident.Inverse();

	//transp.printMatrix();

	Matrix m3 = m1.Subtract(m2);
	// Matrix m3 = m1.Add(m2).Add(m1);

	m3.printMatrix();


	return 0;
}