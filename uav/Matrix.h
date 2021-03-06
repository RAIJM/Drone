#include <Arduino.h>

class Matrix
{
	private:
		int n_rows;
		int n_cols;
		float matrix[10];
	public:
		void init(int n_rows, int n_cols);

		Matrix Multiply(Matrix m2);
		Matrix Add(Matrix m2);
		Matrix Subtract(Matrix m2);
		Matrix Transpose();
		void setMatrix(Matrix m2);
		void Put(int row,int col,float val);
		float Get(int row, int col);
		void Inverse();
		void printMatrix();
		static Matrix Identity(int n_rows,int n_cols);

};
