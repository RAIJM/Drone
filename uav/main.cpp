#include "Matrix.h"
#include <iostream>
using namespace std;




int main(int argc, char ** argv)
{
	//Matrix m1;
	Matrix m1(2,2);
	m1.put(0,0,5.0f);
	m1.put(0,1,7.0f);
	m1.put(1,0,1.0f);
	m1.put(1,1,3.0f);

	Matrix m2(2,2);
	m2.put(0,0,4.0f);
	m2.put(0,1,11.0f);
	m2.put(1,0,9.0f);
	m2.put(1,1,4.0f);

	Matrix* m3 = m1.add(m2);
	cout << m3->get(0,0);
	return 0;

}