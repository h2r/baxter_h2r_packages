/*
 * Sample math operations
*/
#define EIGEN_DEFAULT_TO_ROW_MAJOR
#define DEBUG
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <omp.h>
#include "benchmark-utils.hpp"

using namespace Eigen;
using namespace std;
int Reshape( MatrixXf A[2], MatrixXf& ret )
{
	int origele = A[0].rows() * A[0].cols() * 2;
	int newele = ret.rows() * ret.cols();

	if ( origele != newele ) return -1;

	// FIXME: should really recode this into matrix3d object and get rid of this code...

	MatrixXf current;
	int mat_offset = 0;
	for ( int k = 0 ; k < 2 ; k++ )
	{
		current = A[k];
		// increases performance by a lot
		#pragma omp parallel for
 		for ( int acol = 0 ; acol < current.cols() ; acol++ ) 
		{
 			for ( int arow = 0 ; arow < current.rows() ; arow++ ) 
                	{
				int linearoffset = (k * current.rows() * current.cols()) + (acol * current.rows()) + arow;
                                // map linear index to 2d indexes
                                int newrow = linearoffset % ret.rows();
                                int newcol = linearoffset / ret.rows();
                        	ret( newrow, newcol ) = current( arow, acol );
                	}
        	}
	}
	return 0;
}


int main(int argc, char** args)
{
	MatrixXf* test = new MatrixXf(2048,2048);
	#pragma omp parallel for
	for (int i = 0; i < 2048; i++)
		for (int j = 0; j < 2048; j++)
			(*test)(i,j) = (float) (rand()%1000);
	//for (int i = 0; i < 2048*2048;i++)
	//	*test << rand();
	MatrixXf* test2 = new MatrixXf(2048,2048);
	#pragma omp parallel for
	for (int i = 0; i < 2048; i++)
		for (int j = 0; j < 2048; j++)
			(*test2)(i,j) = (float) (rand()%1000);
//	cout << *test << endl;
//	cout << *test2 << endl;
	cout << "generated stuff" << endl;
	//for (int i = 0; i < 2048*2048;i++)
	//	*test2 << rand();
	//cout << "test one: " << endl << *test << endl<<
	//	"test two: " << endl << *test2 << endl;
	Timer timer;
	double exectime;
	double sum = 0;
	for (int i = 0 ; i < 100; i++) {
		timer.start();
		MatrixXf meh[2] = {*test, *test2};
		MatrixXf meh2(2048*2048, 2);
		Reshape(meh,meh2);
		exectime = timer.get();
		sum += exectime;
//		cout << "Meh: " << endl << "exec time: " << exectime << endl;
	}
	cout << "Average: " << endl << sum/100.0 << endl << "done!" << endl;;
	return 0;
}
