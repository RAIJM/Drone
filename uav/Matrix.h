#include <cstdlib>

class Matrix
{
	private:
		int n_rows;
		int n_cols;
		float * matrix;
	public:
		Matrix(int n_rows,int n_cols);
		Matrix* multiply(Matrix* m2);
		Matrix* add(Matrix * m2);
		Matrix* subtract(Matrix * m2);
		Matrix* transpose();
		void put(int row,int col,float val);
		float get(int row, int col);
		void inverse();
		static void Identity();

};