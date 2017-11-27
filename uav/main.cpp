
//#include "KalmanFilter.h"
//#include <stdio.h>
//
//
//
//
//
//
//int main(int argc, char** argv)
//{	
//	// int n=0;
//    Matrix m1;
//	m1.init(2,2);
//	// // Matrix m2;
//	// // m2.init(2,2);
//	// // Matrix m3;
//	// // m3.init(2,2);
//	 m1.Put(0,0,1.0);
//	 m1.Put(0,1,4.01);
//	 m1.Put(1,0,2.0);
//	 m1.Put(1,1,1.0);
//
//	 Matrix m2;
//	 m2.init(2,2);
//	 m2.Put(0,0,4.0);
//	 m2.Put(0,1,5);
//	 m2.Put(1,0,4.5);
//	 m2.Put(1,1,5);
//
//	Matrix m3 = m1.Multiply(m2);
//	m3.printMatrix();
//
//	 // float a = 2.5;
//	 // float b = 1.0;
//	 // printf("%.2f",m3.Get(0,0));
//
//		
//	// m2.Put(0,0,1.0f);
//	// m2.Put(0,1,0.0f);
//	// m2.Put(1,0,0.0f);
//	// m2.Put(1,1,1.0f);
//
//	    
//	// m3.Put(0,0,0.25f);
//	// m3.Put(0,1,0.0f);
//	// m3.Put(1,0,0.0f);
//	// m3.Put(1,1,0.25f);
//	// int n=0;
//	// KalmanFilter * state_x_kalman_filter;
//	// state_x_kalman_filter = new KalmanFilter(0.0f, 0.0f, 0.2456, 2.0);
//	// while(n < 1000){
//	// 	state_x_kalman_filter->Predict((float)1.0f,0.01);
//	// 	state_x_kalman_filter->Update(2.0f,1.0f,0.0f,0.0f);
//	// 	//printf("%.2f\n",state_x_kalman_filter->getVelocity());
//		
//		
//
//	//     // m2 = m1.Multiply(m2).Multiply(m1.Transpose()).Add(m3);
//
//	//     // m2.printMatrix();
//
//
//	// 	// m2.Put(0,1, 2.0f);
//	// 	// m2.Put(1,0, 6.0f);
//	// 	// m2.Put(1,1, 2.0f);
//
//	// 	//Matrix transp = m1.Transpose();
//
//	// 	//Matrix ident = Matrix::Identity(2,2);
//	// 	//ident.Inverse();
//
//	// 	//transp.printMatrix();
//
//	// 	// Matrix m3 = m1.Multiply(m2);
//	// 	// // Matrix m3 = m1.Add(m2).Add(m1);
//
//	// 	// m3.printMatrix();
//	// 	n++;
//	// }
//
//
//	return 0;
//}
