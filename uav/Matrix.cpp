#include "Matrix.h"
//#include <stdio.h>
//#include <Arduino.h>


void Matrix::init(int nrows, int ncols)
{
	this->n_rows = nrows;
	this->n_cols = ncols;
	this->matrix = (float *) calloc(this->n_rows * this->n_cols,sizeof(float));

}

Matrix::~Matrix() //destructor
{
    free(this->matrix);
}

//Puts element at row and column
void Matrix::Put(int row,int col, float value)
{
	this->matrix[this->n_cols*row + col] = value;
}


//Returns element at row and column
float Matrix::Get(int row, int col)
{
	return this->matrix[this->n_cols*row + col];
}


//Returns identity matrix
// 1's on the diagonal and 0 elsewhere
// [ 1 0]
// [ 0 1]
Matrix Matrix::Identity(int n_rows,int n_cols)
{
	Matrix ident;
	ident.init(n_rows, n_cols);
	int i,j;
	for(i=0; i< n_rows; i++)
		for(j=0; j < n_cols; j++)
			if(i == j)
				ident.Put(i,j,1.0f);
			else
				ident.Put(i,j,0.0f);
	return ident;
}


//Adds two matrices m1 and m2 and returns a new matrix m3
Matrix Matrix::Add(Matrix m2)
{
	int i,j;
	Matrix m3;
	m3.init(this->n_rows,this->n_cols);
	for(i=0; i < this->n_rows; i++)
		for(j=0; j < this->n_cols; j++)
			m3.Put(i,j,this->Get(i,j) + m2.Get(i,j));
	return m3;

}

//Subtract two matrices m1 and m2 and returns a new matrix m3
Matrix Matrix::Subtract(Matrix  m2)
{
	int i,j;
	Matrix m3;
	m3.init(this->n_rows,this->n_cols);
	for(i=0;i<this->n_rows; i++)
		for(j=0; j < this->n_cols; j++)
			m3.Put(i,j,this->Get(i,j) - m2.Get(i,j));
	return m3;
}


//Multiply two matrices m1 and m2 and returns a new matrix m3
Matrix Matrix::Multiply(Matrix m2)
{
	int i,j,k;
	Matrix m3;
	m3.init(this->n_rows,m2.n_cols);
	for (i=0; i < this->n_rows; i++)
	{
		for(j=0; j < this->n_cols; j++)
		{
			int sum = 0;
			for(k=0; k < this->n_cols; k++)
			{
				sum += this->Get(i,k) * m2.Get(k,j);
				
			}
			m3.Put(i,j,sum);
		}
	}
	return m3;
}

//Returns the reanspose of a matrix
// swaps rows and columns
Matrix Matrix::Transpose()
{
	int i,j;
	Matrix m3;
	m3.init(this->n_cols,this->n_rows);
	for(i=0; i < this->n_rows; i++)
		for(j=0; j < this->n_cols; j++)
			m3.Put(j,i,this->Get(i,j));
	return m3;
}

//Prints a matrix
void Matrix::printMatrix()
{
	int i,j;
	for(i = 0; i < this->n_rows; i++)
		for(j=0; j < this->n_cols; j++)
			cout << this->Get(i,j) << " ";
		cout << "\n";
}

//Finds inverse of matrix using Gauss Jordan method
void Matrix::Inverse()
{
	
	
	int pivrow;
	int k,i,j;
	int pivrows[this->n_rows];
	float tmp;

	for(k =0; k < this->n_rows; k++)
	{
		tmp = 0;
		for(i = k; i < this->n_rows; i++)
		{
			if(abs(this->Get(i,k) >= tmp))
			{
				tmp = abs(this->Get(i,k));
				pivrow = i;
			}
		}


		if(this->Get(pivrow,k) == 0.0f)
		{
			
		}

		if(pivrow!=k)
		{
			for(j=0; j < this->n_rows; j++)
			{
				tmp = this->Get(pivrow,j);
				this->Put(k, j, this->Get(pivrow,j));
				this->Put(pivrow,j,tmp);
			}
		}

		pivrows[k] = pivrow;

		tmp = 1.0f/ this->Get(k,k);
		this->Put(k,k,1.0f);

		for(j = 0; j < this->n_rows; j++)
		{
			this->Put(k,j,this->Get(k,j) * tmp);
		}

		for(i = 0; i <this->n_rows; i++)
		{
			if(i != k)
			{
				tmp = this->Get(i,k);
				this->Put(i,k, 0.0f);
				for(j=0; j < this->n_rows; j++)
				{
					this->Put(i, j,this->Get(i, j) - this->Get(k,j) * tmp);
				}
			}
		}
	}

	for(k = this->n_rows -1; k >=0; k--)
	{
		if(pivrows[k] != k)
		{
			for(i=0; i < this->n_rows; i++)
			{
				tmp = this->Get(i, k);
				this->Put(i,k,this->Get(i,pivrows[k]));
				this->Put(i,pivrows[k],tmp);
			}
		}
	}

	
}



