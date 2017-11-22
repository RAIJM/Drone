#include <iostream>
using namespace std;
#include <stdlib.h>

class Matrix
{
	private:
		int n_rows;
		int n_cols;
		float * matrix;
	public:
		void init(int n_rows, int n_cols);
        ~Matrix();
		Matrix Multiply(Matrix m2);
		Matrix Add(Matrix m2);
		Matrix Subtract(Matrix m2);
		Matrix Transpose();
		void Put(int row,int col,float val);
		float Get(int row, int col);
		void Inverse();
		void printMatrix();
		static Matrix Identity(int n_rows,int n_cols);

};
