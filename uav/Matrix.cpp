#include "Matrix.h"



Matrix::Matrix(int n_rows, int n_cols)
{
	this->n_rows = n_rows;
	this->n_cols = n_cols;
	this->matrix = (float *) calloc(this->n_rows * this->n_cols,sizeof(float));

}

void Matrix::put(int row,int col, float value)
{
	this->matrix[this->n_cols*row + col] = value;
}

float Matrix::get(int row, int col)
{
	return this->matrix[this->n_cols*row + col];
}


Matrix* Matrix::add(Matrix m2)
{
	int i,j;
	Matrix* m3 = new Matrix(this->n_rows,this->n_cols);
	for(i=0; i <this->n_rows; i++)
		for(j=0; j < this->n_cols; j++)
			m3->put(i,j,this->get(i,j) + m2.get(i,j));
	return m3;

}

Matrix* Matrix::substract(Matrix m2)
{
	int i,j;
	Matrix * m3 = new Matrix(this->n_rows,this->n_cols);
	for(i=0;i<this->n_rows; i++)
		for(j=0; j < this->n_cols; j++)
			m3->put(i,j,this->get(i,j) - m2.get(i,j));
	return m3;
}

Matrix* Matrix::multiply(Matrix m2)
{
	int i,j,k;
	Matrix * m3 = new Matrix(this->n_rows,m2.n_cols)
	for (i=0; i < this->n_rows; i++)
	{
		for(j=0; j < this->n_cols; j++)
		{
			int sum = 0;
			for(k=0; k < this->n_cols; k++)
			{
				sum += this->get(i,k) * m2.get(k,j)
				
			}
			m3->put(i,j,sum);
		}
	}
	return m3;
}

Matrix* Matrix::transpose()
{
	int i,j;
	Matrix * m3 = new Matrix(this->n_cols,this->n_rows);
	for(i=0; i < this->n_cols; i++)
		for(j=0; j < this->n_rows; j++)
			m3->put(i,j,this->get(j,i));
	return m3;
}

void Matrix::inverse()
{
	Matrix * m3 = new Matrix(this->n_rows,this->n_cols);
	int pivrow;
	int k,i,j;
	int pivrows[this->n_rows];
	float tmp;

	for(k =0; k < this->n_rows; k++)
	{
		tmp = 0;
		if(abs(this->get(i,k) >= tmp))
		{
			tmp = abs(this->get(i,k));
			pivrow = i;
		}


		if(this->get(pivrow,k) == 0.0f)
		{
			return 0;	
		}

		if(pivrow!=k)
		{
			for(j=0; j < n; j++)
			{
				tmp = this->get(pivrow,j);
				this->put(pivrow,j,temp);
			}
		}

		pivrows[k] = pivrow;

		tmp = 1.0f/ this->get(k,k);
		this->put(k,k,1.0f);

		for(j = 0; j < this->n_rows; j++)
		{
			this->put(k,j,this->get(k,j) * tmp);
		}

		for(i = 0; i <this->n_rows; i++)
		{
			if(i !=k )
			{
				tmp = this->get(i,k);
				this->get(i,k) = 0.0f;
				for(j=0; j < n; j++)
				{
					this->put(i, j) = this->get(i, j) - this->get(k,j) * tmp;
				}
			}
		}
	}

	for(k = n -1; k >=0; k--)
	{
		if(pivrows[k] != k)
		{
			for(i=0; i < n; i++)
			{
				tmp = this->get(i, k);
				this->put(i,k,this->get(i,pivrows[k]));
				this->put(i,pivrows[k],tmp);
			}
		}
	}

	




}



