//
//  Xiaofeng Ren, 02/2012
//

//#define EIGEN_DEFAULT_TO_ROW_MAJOR

//#define MODEL_TYPE 0                 // gradient
//#define MODEL_TYPE 2                 // color
//#define MODEL_TYPE 3                 // gradient (depth)
//#define MODEL_TYPE 4                 // spin     (depth)
static unsigned int MODEL_TYPE=0;

#define PI 3.14159265 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <fstream>
#include <vector>
#include <iomanip>
#include <set>
#include <boost/version.hpp>
#include <boost/filesystem.hpp>
#include "benchmark-utils.hpp"
#include "mattest.h"
using namespace std;
using namespace MatIO;
using namespace Eigen;


#ifndef LIBKERNELDESC_H
#define LIBKERNELDESC_H

const int MAX_IMAGE_SIZE = 100;  // 300
//#define KDES_RESIZE_IMAGE  1

#define OBJECTS_TO_PRINT 4

#if BOOST_VERSION >= 104600 && BOOST_FILESYSTEM_VERSION >= 3
  #define NEW_BOOST 1
#endif

static const char* model_names[] = \
{"Gradient Kernel Descriptor",\
"Shape Kernel Descriptor", \
"Color Kernel Descriptor", \
"Gradient Kernel Descriptor (Depth)", \
"Spin Kernel Descriptor (Depth)"};

static const char* model_files[] = \
{"modelgkdes",\
"modellbpkdes",\
"modelrgbkdes",\
"modelgkdes_dep",\
"modelspinkdes"};

static const char* model_vars[] = \
{"modelgkdes",\
"modellbpkdes",\
"modelrgbkdes",\
"modelgkdes",\
"modelspinkdes"};

static const char* param_files[] = \
{"kpcaGDes", "kpcaLBPDes", "kpcaRGBDes","kpcaGDes_dep","kpcaSpinDes"};

// xren: sort with index
typedef std::pair<int,int> myindex;
typedef std::pair<float,myindex> mypair;
static bool comparator ( const mypair& l, const mypair& r)
{ return l.first > r.first; }

static inline bool isnan(float x) { return !(x==x); }

static void debugImg(const char* name, IplImage* img) {
	// view image
	/* create a window */
	cvNamedWindow( name, CV_WINDOW_AUTOSIZE );

	/* display the image */
	cvShowImage( name, img );

	/* wait until user press a key */
	cvWaitKey(0);

	/* free memory */
	cvDestroyWindow( name );
}

static matvarplus_t* load_mat(const char* program_name,const char* mat_file) {
	mat_t* mat = NULL;
	matvar_t* matvar = NULL;
	Mat_LogInit(program_name);
	mat = Mat_Open(mat_file, MAT_ACC_RDONLY);
	if (!mat)
		Mat_Error("Error opening %s\n", mat_file);
	while ((matvar = Mat_VarReadNext(mat)) != NULL) {
		matvarplus_t* temp =  new matvarplus_t(matvar,NULL); // doesn't destruct in memory, so TODO
		print_default(temp);
		Mat_Close(mat);
		return temp;
	}
}

static void isotropic_gaussian(MatrixXf&  result, float sigma, int  patch_size) {
	float sigma_sqr = sigma*sigma;
	float constant = (1.0/(2.0*PI*sigma_sqr));
	//	#pragma omp parallel for
	for (int x = -patch_size/2; x < -patch_size/2 + patch_size; x++) {
		for (int y = -patch_size/2; y < -patch_size/2+patch_size; y++) { 
			result(x+patch_size/2,y+patch_size/2) = constant*exp(-(double)(x*x +y*y)*constant*PI);
		}
	}
	// need normalization 
	result /= result.sum();
}


// mimic matlab rgb2gray function (inefficient)

static IplImage* rgb2gray(const IplImage* im)
{
	const float w[3]={0.1140, 0.5870, 0.2989};

	IplImage* im_g  = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_32F, 1);

	if (im->nChannels==1) {
		im_g = cvCloneImage( im );
	} else {
		for(int i=0; i<im->height; i++)
			for(int j=0; j<im->width; j++) {
				CvScalar s;
				s = cvGet2D(im,i,j);
				double s_g=w[0]*s.val[0]+w[1]*s.val[1]+w[2]*s.val[2];
				s_g = max( min(s_g,1.0),0.0 );
				cvSet2D(im_g,i,j,cvScalar(s_g));
			}
	}
	return im_g;
}


static MatrixXf scaletest_linear(const MatrixXf& imfea, const MatrixXf& minvalue, const MatrixXf& maxvalue)
{
	MatrixXf imfea_s = (imfea.array() - minvalue.array()) / ( maxvalue.array()-minvalue.array() );
	return imfea_s;
}


inline float checkNan(float x)
{
  if ( isnan(x) )
    return 0.0f;
  else
    return x;
}

inline float maxZero(float x)
{
  return max(x,0.0f);
}


static MatrixXf EvalKernelExp( const MatrixXf& data1, const MatrixXf& data2, float kparam )
{
	int asize = data1.cols();
	int bsize = data2.cols();
	int gsize = data1.rows();
	assert( gsize==data2.rows() );

	MatrixXf result( asize,bsize );

	MatrixXf tmp1 = data1.colwise().squaredNorm().transpose();
	//tmp1 = tmp1.unaryExpr(ptr_fun(checkNan));
	MatrixXf tmp2 = data2.colwise().squaredNorm();
	//tmp2 = tmp2.unaryExpr(ptr_fun(checkNan));
	MatrixXf tmp3( asize, bsize );
	tmp3.colwise() = tmp1.col(0);
	tmp3.rowwise() += tmp2.row(0);

	result = tmp3 - data1.transpose() * data2 * 2.0f;
	//result = result.unaryExpr(ptr_fun(maxZero));

	result.array() *= (-kparam);
	result.array() = result.array().exp();

	return result;
}



static MatrixXf EvalKernelExp_d( const MatrixXf& data1_f, const MatrixXf& data2_f, float kparam )
{
	MatrixXd data1=data1_f.cast<double>();
	MatrixXd data2=data2_f.cast<double>();

        int asize = data1.cols();
        int bsize = data2.cols();
        int gsize = data1.rows();
        assert( gsize==data2.rows() );

        MatrixXd result( asize,bsize );

        MatrixXd tmp1 = data1.colwise().squaredNorm().transpose();
        //tmp1 = tmp1.unaryExpr(ptr_fun(checkNan));
        MatrixXd tmp2 = data2.colwise().squaredNorm();
        //tmp2 = tmp2.unaryExpr(ptr_fun(checkNan));
        MatrixXd tmp3( asize, bsize );
        tmp3.colwise() = tmp1.col(0);
        tmp3.rowwise() += tmp2.row(0);

        result = tmp3 - data1.transpose() * data2 * 2.0f;
        //result = result.unaryExpr(ptr_fun(maxZero));

        result.array() *= (-kparam);
        result.array() = result.array().exp();

        return result.cast<float>();
}


/**

This version takes the first sample as parts of x and y, and then takes
in the second sample as a full matrix
Assumes gradient size is 2
Passed by reference to avoid copying
**/
static MatrixXf EvalKernelExp_Img2(const MatrixXf& I_ox, const MatrixXf& I_oy, const MatrixXf& sample2, MatrixXf& params)
{
	int asize = I_ox.rows();
	int bsize = I_ox.cols(); // just need to know the size of just one of tbe matrices
	int gsize = sample2.cols();

	MatrixXf result( asize*bsize,gsize );

	assert( params(0,0)==params(0,1) );   // for simplicity, assume isotropic

	unsigned int index = 0;

	// use column major order here
	//	#pragma omp parallel for
	for (int j = 0; j < bsize; j++)
	{
		for (int i = 0; i < asize; i++) 
		{
			// vectors for each pocket in the matrix
			VectorXf src(2);
			src[0] = I_ox(i,j);
			src[1] = I_oy(i,j);

			(result).block( index, 0, 1, gsize ) = -( sample2.colwise() - src ).colwise().squaredNorm();

			index++;
		}
	}		

	(result) *= params(0,0);
	(result).array() = result.array().exp();

	return result;
}

static MatrixXf EvalKernelExp_Img3(const MatrixXf& im1, const MatrixXf& im2, const MatrixXf& im3, const MatrixXf& sample2, MatrixXf& params)
{
	int asize = im1.rows();
	int bsize = im1.cols(); // just need to know the size of just one of tbe matrices
	int gsize = sample2.cols();

	MatrixXf result( asize*bsize,gsize );

	assert( params(0,0)==params(0,1) );   // for simplicity, assume isotropic

	unsigned int index = 0;

	// use column major order here
	//	#pragma omp parallel for
	for (int j = 0; j < bsize; j++)
	{
		for (int i = 0; i < asize; i++) 
		{
			// vectors for each pocket in the matrix
			VectorXf src(3);
			src[0] = im1(i,j);
			src[1] = im2(i,j);
			src[2] = im3(i,j);

			(result).block( index, 0, 1, gsize ) = -( sample2.colwise() - src ).colwise().squaredNorm();

			index++;
		}
	}		

	(result) *= params(0,0);
	(result).array() = result.array().exp();

	//cout << result.block(100*asize+100,0,10,10) << endl;

	return result;
}





// GKDESDense function
static void GKDESDense(MatrixXf& feaArr, MatrixXf& feaMag, MatrixXf& fgrid_y, MatrixXf& fgrid_x, IplImage* im, matvarplus_t* kdes_params, int grid_space, int patch_size, float low_contrast) {

	MatrixXf gpoints;
	get_matrix(gpoints, kdes_params, "kpcaGDes->gpoints");
	MatrixXf spoints;
	get_matrix(spoints, kdes_params, "kpcaGDes->spoints");
	MatrixXf kparam;
	get_matrix(kparam,kdes_params, "kpcaGDes->kparam");
	float sigma_edge = 0.8; // this wasn't set initially so I'm setting my own
	// make sure image is greyscale
	// note the RGB2GRAY is different OpenCV vs matlab
	//IplImage* im_temp  = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_32F, 1);
	//cvCvtColor(im, im_temp, CV_RGB2GRAY);
	IplImage* im_temp = rgb2gray(im);
	//  	cvReleaseImage( &im );

	im = im_temp;
	//debugImg("GKDESDense Image", im);

	// float max patch_size = max(patch_size) // i haven't seen an instance where there are multiple patch sizes yet TODO
	int img_h = im->height;
	int img_w = im->width;
	float rem_x = ( (img_w-patch_size-1) % grid_space )+1;
	float rem_y = ( (img_h-patch_size-1) % grid_space )+1;

	float offset_x = floor(rem_x/2) + 1;
	float offset_y = floor(rem_y/2) + 1;

	// meshgrid
	int multiplier_x = floor((img_w-patch_size+1 - offset_x)/grid_space) + 1;
	int multiplier_y = floor((img_h - patch_size + 1 - offset_y)/grid_space) + 1;
	RowVectorXf grid_x_v = RowVectorXf(multiplier_x);
	for (int i = 0; i < multiplier_x; i++) {
		grid_x_v(i) = offset_x+grid_space*i;
	}
	MatrixXf grid_x = grid_x_v.replicate(multiplier_y,1);
	VectorXf grid_y_v = VectorXf(multiplier_y);
	for (int i = 0; i < multiplier_y; i++) {
		grid_y_v(i) = offset_y+grid_space*i;
	}
	MatrixXf grid_y = grid_y_v.replicate(1,multiplier_x);

	int patches_amt = grid_x.rows()*grid_y.cols();
	// isotropic gaussian	
	int size = 4 * ceil(sigma_edge) + 1;
	MatrixXf G(size,size); // G gets the isotropic gaussian
	isotropic_gaussian(G, sigma_edge, size);
	// calculate the gradient
	// for matlab behavior, central difference, forward difference on edges
	MatrixXf dAx( size,size );
	dAx.col(0)=G.col(1)-G.col(0);
	dAx.col(size-1)=G.col(size-1)-G.col(size-2);
	for(int c=1; c<size-1; c++)
		dAx.col(c)=(G.col(c+1)-G.col(c-1))/2;
	MatrixXf dAy = dAx.transpose();
	// due to row/column major order mismatch, need to transpose
	dAx.transposeInPlace();
	dAy.transposeInPlace();
	dAx = dAx*2 / dAx.array().abs().sum();
	dAy = dAy*2 / dAy.array().abs().sum();
	//=================================
	IplImage* imX  = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_32F, 1);
	IplImage* imY  = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_32F, 1);
	CvMat dAx_cvfilter = cvMat(dAx.rows(), dAx.cols(), CV_32FC1,dAx.data());
	CvMat dAy_cvfilter = cvMat(dAy.rows(), dAy.cols(), CV_32FC1,dAy.data());
	cvFilter2D(im, imX, &dAx_cvfilter);
	cvFilter2D(im, imY, &dAy_cvfilter);

	MatrixXf imX_mat = MatrixXf::MapAligned((float*)imX->imageData, imX->width,imX->height);
	MatrixXf imY_mat = MatrixXf::MapAligned((float*)imY->imageData, imY->width,imY->height);
	// due to row/column major order mismatch, need to transpose
	imX_mat.transposeInPlace();
	imY_mat.transposeInPlace();
	MatrixXf img_mag = (imX_mat.array().pow(2) + imY_mat.array().pow(2)).sqrt();	

	MatrixXf gvalue = MatrixXf::Constant(img_mag.rows(), img_mag.cols(), 1e-5f);		
	img_mag = img_mag.array().max(gvalue.array());
	MatrixXf im_ox, im_oy;
	im_ox = imX_mat.array() / img_mag.array(); 
	im_oy = imY_mat.array() / img_mag.array();
	//cout << "im_ox:" << endl << im_ox.block(10,10,10,10) << endl;
	//cout << "im_oy:" << endl << im_oy.block(10,10,10,10) << endl;

	MatrixXf kparam_g = kparam.block(0,0,1,kparam.cols()-2);
	MatrixXf im_k = EvalKernelExp_Img2(im_ox,im_oy,gpoints,kparam_g);
	VectorXf sub_vector(patch_size);
	for (float i = 0; i < patch_size; i++)
	{
		sub_vector(i) = i/patch_size;
	}
	MatrixXf yy = sub_vector.replicate(1, patch_size);
	MatrixXf xx = yy.transpose();

	MatrixXf kparam_s = kparam.block(0,kparam.cols()-2,1,2);

	MatrixXf skv = EvalKernelExp_Img2(yy,xx,spoints,kparam_s);
	skv.transposeInPlace();

	MatrixXf kparam_eigen;
	get_matrix(kparam_eigen, kdes_params, "kpcaGDes->eigvectors");

	MatrixXf mwkvs( kparam_eigen.cols(), patches_amt );  // transpose from matlab version
	MatrixXf gkdes_mag( patch_size*patch_size, patches_amt );

	const int gsize = im_k.cols();

	MatrixXf skv_w( skv.rows(), patch_size*patch_size );
	MatrixXf im_p( patch_size*patch_size, gsize );
	MatrixXf mwkv;
	for(int i=0; i<patches_amt; i++) {
		float x_lo = grid_x(i)-1;
		float x_hi = x_lo + patch_size - 1;
		float y_lo = grid_y(i)-1;
		float y_hi = y_lo + patch_size - 1;

		MatrixXf weight = img_mag.block( y_lo, x_lo, patch_size, patch_size );
		weight.resize( 1,weight.rows()*weight.cols() );
		float norm_weight = sqrt( weight.array().pow(2).sum() );
		if (norm_weight>low_contrast) {
			weight.array() /= norm_weight;
		} else {
			weight.array() /= low_contrast;
		}
		gkdes_mag.col(i) = weight.transpose();

		for(int c=0; c<skv_w.rows(); c++)
			skv_w.row(c)=weight;

		skv_w.array() *= skv.array();

		for(int x=0; x<patch_size; x++) {
			im_p.block(x*patch_size,0,patch_size,gsize ) = im_k.block( y_lo+(x_lo+x)*img_h,0,patch_size,gsize );
		}

		mwkv = skv_w*im_p;
		mwkv.transposeInPlace();

		mwkv.resize( mwkv.rows()*mwkv.cols(),1 );
		mwkvs.col(i) = mwkv.col(0);

	}

	feaArr = (kparam_eigen)*mwkvs;
	feaMag = gkdes_mag;

	fgrid_y = grid_y;
	fgrid_y.resize( fgrid_y.rows()*fgrid_y.cols(),1 );
	fgrid_y.array() = fgrid_y.array() + (patch_size*0.5f-0.5f);
	fgrid_x = grid_x;
	fgrid_x.resize( fgrid_x.rows()*fgrid_x.cols(),1 );
	fgrid_x.array() = fgrid_x.array() + (patch_size*0.5f-0.5f);
	//	delete gpoints;
	//	delete spoints;
	//	delete kparam;
	//	delete kparam_eigen;
	//	gpoints = NULL;
	//	spoints = NULL;
	//	kparam = NULL;
	//	kparam_eigen = NULL;
	cvReleaseImage(&im_temp);
	cvReleaseImage(&imX);
	cvReleaseImage(&imY);
	im_temp = NULL;
	imX = NULL;
	imY = NULL;

};




// RGBKDESDense function
static void RGBKDESDense(MatrixXf& feaArr, MatrixXf& feaMag, MatrixXf& fgrid_y, MatrixXf& fgrid_x, IplImage* im, matvarplus_t* kdes_params, int grid_space, int patch_size) {
	const float low_contrast = 0;

	MatrixXf gpoints;
	get_matrix(gpoints, kdes_params, "kpcaRGBDes->rgbpoints");
	MatrixXf spoints;
	get_matrix(spoints, kdes_params, "kpcaRGBDes->spoints");
	MatrixXf kparam; 
	get_matrix(kparam, kdes_params, "kpcaRGBDes->kparam");

	int img_h = im->height;
	int img_w = im->width;

	IplImage* imR = cvCreateImage( cvGetSize(im), IPL_DEPTH_32F, 1 );
	IplImage* imG = cvCreateImage( cvGetSize(im), IPL_DEPTH_32F, 1 );
	IplImage* imB = cvCreateImage( cvGetSize(im), IPL_DEPTH_32F, 1 );

	cvSplit( im, imB, imG, imR, NULL );   // note the BGR order

	//debugImg("GKDESDense Image", im);

	// float max patch_size = max(patch_size) // i haven't seen an instance where there are multiple patch sizes yet TODO

	float rem_x = ( (img_w-patch_size-1) % grid_space )+1;
	float rem_y = ( (img_h-patch_size-1) % grid_space )+1;

	float offset_x = floor(rem_x/2) + 1;
	float offset_y = floor(rem_y/2) + 1;

	// meshgrid
	int multiplier_x = floor((img_w-patch_size+1 - offset_x)/grid_space) + 1;
	int multiplier_y = floor((img_h - patch_size + 1 - offset_y)/grid_space) + 1;
	RowVectorXf grid_x_v = RowVectorXf(multiplier_x);
	for (int i = 0; i < multiplier_x; i++) {
		grid_x_v(i) = offset_x+grid_space*i;
	}
	MatrixXf grid_x = grid_x_v.replicate(multiplier_y,1);
	VectorXf grid_y_v = VectorXf(multiplier_y);
	for (int i = 0; i < multiplier_y; i++) {
		grid_y_v(i) = offset_y+grid_space*i;
	}
	MatrixXf grid_y = grid_y_v.replicate(1,multiplier_x);

	int patches_amt = grid_x.rows()*grid_y.cols();

	MatrixXf imR_mat = MatrixXf::MapAligned((float*)imR->imageData, imR->width,imR->height);
	MatrixXf imG_mat = MatrixXf::MapAligned((float*)imG->imageData, imG->width,imG->height);
	MatrixXf imB_mat = MatrixXf::MapAligned((float*)imB->imageData, imB->width,imB->height);
	// due to row/column major order mismatch, need to transpose
	imR_mat.transposeInPlace();
	imG_mat.transposeInPlace();
	imB_mat.transposeInPlace();

	//cout << "call kernel img3" << endl;

	MatrixXf kparam_g = kparam.block(0,0,1,kparam.cols()-2);
	MatrixXf im_k = EvalKernelExp_Img3(imR_mat,imG_mat,imB_mat,gpoints,kparam_g);

	VectorXf sub_vector(patch_size);
	for (float i = 0; i < patch_size; i++)
	{
		sub_vector(i) = i/patch_size;
	}
	MatrixXf yy = sub_vector.replicate(1, patch_size);
	MatrixXf xx = yy.transpose();

	MatrixXf kparam_s = kparam.block(0,kparam.cols()-2,1,2);

	MatrixXf skv = EvalKernelExp_Img2(yy,xx,spoints,kparam_s);
	skv.transposeInPlace();

	MatrixXf kparam_eigen; 
	get_matrix(kparam_eigen, kdes_params, "kpcaRGBDes->eigvectors");

	MatrixXf mwkvs( kparam_eigen.cols(), patches_amt );  // transpose from matlab version
	MatrixXf gkdes_mag( patch_size*patch_size, patches_amt );

	const int gsize = im_k.cols();

	MatrixXf skv_w( skv.rows(), patch_size*patch_size );
	MatrixXf im_p( patch_size*patch_size, gsize );
	MatrixXf mwkv;

	for(int i=0; i<patches_amt; i++) {

		float x_lo = grid_x(i)-1;
		float x_hi = x_lo + patch_size - 1;
		float y_lo = grid_y(i)-1;
		float y_hi = y_lo + patch_size - 1;

		skv_w.array() = skv.array();

		for(int x=0; x<patch_size; x++) {
			im_p.block(x*patch_size,0,patch_size,gsize ) = im_k.block( y_lo+(x_lo+x)*img_h,0,patch_size,gsize );
		}

		mwkv = skv_w*im_p;
		mwkv.transposeInPlace();
		mwkv /= skv.cols();

		mwkv.resize( mwkv.rows()*mwkv.cols(),1 );
		mwkvs.col(i) = mwkv.col(0);

	}

	feaArr = kparam_eigen*mwkvs;

	//cout << "feaArr" << endl <<  feaArr.block(0,feaArr.cols()-250,10,10) <<endl;

	fgrid_y = grid_y;
	fgrid_y.resize( fgrid_y.rows()*fgrid_y.cols(),1 );
	fgrid_y.array() = fgrid_y.array() + (patch_size*0.5f-0.5f);
	fgrid_x = grid_x;
	fgrid_x.resize( fgrid_x.rows()*fgrid_x.cols(),1 );
	fgrid_x.array() = fgrid_x.array() + (patch_size*0.5f-0.5f);
	/*	delete gpoints;
	delete spoints;
	delete kparam;
	gpoints = NULL;
	spoints = NULL;
	kparam = NULL;
	*/
	cvReleaseImage(&imR);
	cvReleaseImage(&imG);
	cvReleaseImage(&imB);
	imR = NULL;
	imG = NULL;
	imB = NULL;
	// adding 
};


static void depth2cloud(const MatrixXf& depth, MatrixXf& pcloud_x, MatrixXf& pcloud_y, MatrixXf& pcloud_z, const MatrixXf& top_left, double focal=570.3, double ycenter=240.0, double xcenter=320.0)
{
	const int height=depth.rows();
	const int width=depth.cols();

	pcloud_z=depth;
	pcloud_y.resize( height, width );
	pcloud_x.resize( height, width );

	// do it the slow way
	for(int i=0; i<height; i++)
	  for(int j=0; j<width; j++) {
	    pcloud_y(i,j)=(i+1);
	    pcloud_x(i,j)=(j+1);
	  }
	pcloud_y.array() += (top_left(0)-1)-ycenter;
	pcloud_x.array() += (top_left(1)-1)-xcenter;
	pcloud_y.array() *= pcloud_z.array() * (1/focal);
	pcloud_x.array() *= pcloud_z.array() * (1/focal);

	return;
}

// slow but exact copy of the matlab pcnormal function
static void pcnormal(const MatrixXf& pcloud_x, const MatrixXf& pcloud_y, const MatrixXf& pcloud_z,  MatrixXf& normal_x, MatrixXf& normal_y, MatrixXf& normal_z, double normal_thresh, double normal_window, bool pos_z=true )
{
	const int height=pcloud_x.rows();
	const int width=pcloud_x.cols();
	assert( height==pcloud_y.rows() ); assert( height==pcloud_z.rows() );

	normal_x.resize( height, width );
	normal_y.resize( height, width );
	normal_z.resize( height, width );

	MatrixXf wpc( (2*normal_window+1)*(2*normal_window+1), 3 );

	SelfAdjointEigenSolver<Matrix3f> es;

	for(int y=0; y<height; y++)
	  for(int x=0; x<width; x++) {
	    if (pcloud_z(y,x)==0) continue;

	    int count=0;
	    for(int i=y-normal_window; i<=y+normal_window; i++)
	      for(int j=x-normal_window; j<=x+normal_window; j++)
	        if (i>=0 && i<height && j>=0 && j<width && abs(pcloud_z(i,j)-pcloud_z(y,x))<pcloud_z(y,x)*normal_thresh) {
	          wpc(count,0)=pcloud_x(i,j);
	          wpc(count,1)=pcloud_y(i,j);
	          wpc(count,2)=pcloud_z(i,j);
	          count++;
	        }
	    if (count<=3) continue;

	    MatrixXf subwpc=wpc.block(0,0,count,3);
	    MatrixXf sumwpc=subwpc.colwise().sum();
	    subwpc.array() -= (sumwpc.array()/count).replicate(count,1);

	    es.compute( (subwpc.transpose()*subwpc) );

	    Vector3f v=es.eigenvectors().col(0);
	    if (pos_z && v(2)<0) {
	      v=-v;
	    }
	    normal_x(y,x)=v(0);
	    normal_y(y,x)=v(1);
	    normal_z(y,x)=v(2);
	  }

	return;
}

// SpinKDESDense function
static void SpinKDESDense(MatrixXf& feaArr, MatrixXf& fgrid_y, MatrixXf& fgrid_x, IplImage* im, const MatrixXf& top_left, matvarplus_t* kdes_params, int grid_space, int patch_size, double normal_thresh=0.01, double normal_window=5) {

	MatrixXf npoints;
	get_matrix(npoints, kdes_params, "kpcaSpinDes->npoints");
	MatrixXf spoints;
	get_matrix(spoints, kdes_params, "kpcaSpinDes->spoints");
	MatrixXf kparam;
	get_matrix(kparam,kdes_params, "kpcaSpinDes->kparam");

	float radius = get_value<float>(kdes_params,"kpcaSpinDes->radius");
	int maxsample = (int) get_value<float>(kdes_params,"kpcaSpinDes->maxsample");
	int minsample = (int) get_value<float>(kdes_params,"kpcaSpinDes->minsample");

        MatrixXf kparam_eigen;
        get_matrix(kparam_eigen, kdes_params, "kpcaSpinDes->eigvectors");

	// convert depth image to (explicit) point cloud
	MatrixXf im_mat = MatrixXf::MapAligned((float*)im->imageData, im->width,im->height);
	im_mat.transposeInPlace();

	MatrixXf pcloud_x, pcloud_y, pcloud_z;
	depth2cloud(im_mat,pcloud_x,pcloud_y,pcloud_z,top_left);

	MatrixXf normal_x, normal_y, normal_z;
	pcnormal( pcloud_x, pcloud_y, pcloud_z, normal_x, normal_y, normal_z, normal_thresh, normal_window );

	int img_h = im->height;
	int img_w = im->width;

	if (img_h<=patch_size+2 || img_w<=patch_size+2) {
	  cout << "warning: image size (" << img_h << "," << img_w << ") too small for patch size " << patch_size << endl;
	}

	// use half patch_size in grid sampling
	int patch_size_2 = (patch_size/2);

	float rem_x = ( (img_w-patch_size_2-1) % grid_space )+1;
	float rem_y = ( (img_h-patch_size_2-1) % grid_space )+1;

	float offset_x = floor(rem_x/2) + 1;
	float offset_y = floor(rem_y/2) + 1;

	// meshgrid
	int multiplier_x = floor((img_w-patch_size_2+1 - offset_x)/grid_space) + 1;
	int multiplier_y = floor((img_h - patch_size_2 + 1 - offset_y)/grid_space) + 1;
	RowVectorXf grid_x_v = RowVectorXf(multiplier_x);
	for (int i = 0; i < multiplier_x; i++) {
		grid_x_v(i) = offset_x+grid_space*i;
	}
	MatrixXf grid_x = grid_x_v.replicate(multiplier_y,1);
	VectorXf grid_y_v = VectorXf(multiplier_y);
	for (int i = 0; i < multiplier_y; i++) {
		grid_y_v(i) = offset_y+grid_space*i;
	}
	MatrixXf grid_y = grid_y_v.replicate(1,multiplier_x);

	int patches_amt = grid_x.rows()*grid_y.cols();
	//cout << "num of patches = " << patches_amt << endl;

	grid_y.array() += (patch_size_2/2-1);    // c index, starts with 0
	grid_x.array() += (patch_size_2/2-1);

		// random locations: works about the same as regular-grid subsampling
		//for(int i=0; i<grid_y.rows()*grid_y.cols(); i++) {
		//  grid_y(i)= ( rand() % (img_h-patch_size_2) + patch_size_2/2 -1 );
		//  grid_x(i)= ( rand() % (img_w-patch_size_2) + patch_size_2/2 -1 );
		//}

	Vector3f rpoint;
	Vector3f cnormal;
	int it=0;    // count for valid patches
	fgrid_y.resize(patches_amt,1);
	fgrid_x.resize(patches_amt,1);

        MatrixXf mwkvs( kparam_eigen.cols(), patches_amt );  // transpose from matlab version

	for(int j=0; j<grid_x.cols(); j++)
	for(int i=0; i<grid_x.rows(); i++) {
	  rpoint(0)=pcloud_x( grid_y(i,j),grid_x(i,j) );
	  rpoint(1)=pcloud_y( grid_y(i,j),grid_x(i,j) );
	  rpoint(2)=pcloud_z( grid_y(i,j),grid_x(i,j) );
	  cnormal(0)=normal_x( grid_y(i,j),grid_x(i,j) );
	  cnormal(1)=normal_y( grid_y(i,j),grid_x(i,j) );
	  cnormal(2)=normal_z( grid_y(i,j),grid_x(i,j) );

	  if (rpoint(2)>0) {      // valid depth
	    int minh = max(grid_y(i,j)-patch_size/2,0.0f);
	    int maxh = min(grid_y(i,j)+patch_size/2,img_h-1.0f);
	    int minw = max(grid_x(i,j)-patch_size/2,0.0f);
	    int maxw = min(grid_x(i,j)+patch_size/2,img_w-1.0f);
	    int winh=maxh-minh+1;
	    int winw=maxw-minw+1;

	    MatrixXf dx = ( pcloud_x.block(minh,minw,winh,winw).array() - rpoint(0) );
	    MatrixXf dy = ( pcloud_y.block(minh,minw,winh,winw).array() - rpoint(1) );
	    MatrixXf dz = ( pcloud_z.block(minh,minw,winh,winw).array() - rpoint(2) );
	    MatrixXf dist2( winh, winw );
	    dist2 = (dx.array().square()+dy.array().square()+dz.array().square());

	    vector<int> index_y;
	    vector<int> index_x;
	    for(int x=0; x<winw; x++)
	      for(int y=0; y<winh; y++)
	        if ( pcloud_z(minh+y,minw+x)>0 && dist2(y,x)<radius*radius ) {
	          index_y.push_back(y);
	          index_x.push_back(x);
	        }

	    int nindex=index_y.size();
	    if ( nindex>minsample ) {
	      // temp: use fixed subsampling
	      int nele=min( nindex,maxsample );

	      MatrixXf subpcloud( nele,3 );
	      MatrixXf subnormal( nele,3 );
	      MatrixXf subdist2( nele,1 );

	      int c=0;
	      for(int k=0; k<nindex; k++) {
	        int c2= (int)ceil( (double)(k+1)/nindex*nele - 1e-6 )-1;     // 1e-6 needed for numerical error -> +1
	        if (c2>=c) {
	          subpcloud(c2,0)=pcloud_x( minh+index_y[k],minw+index_x[k] );
	          subpcloud(c2,1)=pcloud_y( minh+index_y[k],minw+index_x[k] );
	          subpcloud(c2,2)=pcloud_z( minh+index_y[k],minw+index_x[k] );
	          subnormal(c2,0)=normal_x( minh+index_y[k],minw+index_x[k] );
	          subnormal(c2,1)=normal_y( minh+index_y[k],minw+index_x[k] );
	          subnormal(c2,2)=normal_z( minh+index_y[k],minw+index_x[k] );
	          subdist2(c2,0)=dist2( index_y[k],index_x[k] ); 
                  c++;
	        }
	      }
		// check

	      // subtract center point
	      subpcloud.col(0).array() -= rpoint(0);
	      subpcloud.col(1).array() -= rpoint(1);
	      subpcloud.col(2).array() -= rpoint(2);

	      // projection along cnormal direction
	      MatrixXf spin(nele,2);    // (x,y) as in matlab code
	      spin.col(0) = subpcloud.col(0).array() *cnormal(0) + subpcloud.col(1).array() *cnormal(1) + subpcloud.col(2).array() *cnormal(2); 
	      // the other component
	      spin.col(1) = ( subdist2.array() - spin.col(0).array().square() ).max( ArrayXf::Zero(nele,1) ) .sqrt();

	      MatrixXf nn(nele,2);     // (sin,cos)
	      nn.col(1) = ( subnormal.col(0).array() *cnormal(0) + subnormal.col(1).array() *cnormal(1) + subnormal.col(2).array() *cnormal(2) ) .min( ArrayXf::Ones(nele,1) ) .max( -ArrayXf::Ones(nele,1) );
	      nn.col(0) = ( 1-nn.col(1).array().square() ).sqrt();

	      assert( kparam(0)==kparam(1) );
	      MatrixXf nkv = EvalKernelExp( npoints, nn.transpose(), kparam(0) );
	      assert( kparam(2)==kparam(3) );
	      MatrixXf spinkv = EvalKernelExp( spoints, spin.transpose(), kparam(2) );

	      MatrixXf mwkv = (nkv*spinkv.transpose());
	      mwkv.array() /= nkv.cols();

              mwkv.resize( mwkv.rows()*mwkv.cols(),1 );
              mwkvs.col(it) = mwkv.col(0);

	      fgrid_x(it) = grid_x(i,j);
	      fgrid_y(it) = grid_y(i,j);
	      it++;
	    }
	  }
	}

	feaArr = (kparam_eigen)*mwkvs.block(0,0,mwkvs.rows(),it);
	//cout << "feaArr" << endl << feaArr.block(0,0,10,10) << endl;

	fgrid_y.conservativeResize(it,1);
	fgrid_x.conservativeResize(it,1);
	fgrid_y.array() += -0.5; // (patch_size/2.0-0.5);
	fgrid_x.array() += -0.5; // (patch_size/2.0-0.5);

	return;
}




static void CKSVDEMK(MatrixXf& imfea, const MatrixXf& feaArr, const MatrixXf& feaMag, const MatrixXf& fgrid_y, const MatrixXf& fgrid_x, const int img_h, const int img_w, MatrixXf& words, MatrixXf& G, MatrixXf& pyramid, const float kparam)
{
	MatrixXd Gd = G.cast<double>();

	int wordnum = G.rows();
	assert( wordnum==G.cols() );

	MatrixXf pgrid = pyramid.array().pow(2);
	int sgrid = (int)(pgrid.sum());
	MatrixXf weights = MatrixXf::Constant(pgrid.rows(),pgrid.cols(),1.0f).array() / pgrid.array();
	weights.array() /= weights.sum();

	imfea.resize( sgrid*wordnum, 1 );

	//MatrixXf kz = EvalKernelExp( (feaArr), words, kparam );
	MatrixXf kz = EvalKernelExp_d( (feaArr), words, kparam );

	for(int s=0; s<pyramid.size(); s++)
	{
		float wleng = (float)img_w/((pyramid)(s));
		float hleng = (float)img_h/((pyramid)(s));
		MatrixXf xgrid = fgrid_x;
		for(int k=0; k<xgrid.size(); k++)
			xgrid(k)=ceil( (xgrid(k)+1)/wleng);    //   xgrid start with 0
		MatrixXf ygrid = fgrid_y;
		for(int k=0; k<ygrid.size(); k++)
			ygrid(k)=ceil( (ygrid(k)+1)/hleng);
		MatrixXf allgrid = ( ygrid.array()-1 )*(pyramid)(s) + xgrid.array();
		int gridsize=allgrid.size();

		MatrixXf pimimfea = MatrixXf::Zero( pgrid(s)*wordnum,1 );

		for(int t=1; t<=pgrid(s); t++)
		{
			MatrixXf kzind( kz.rows(), kz.cols() );

			int nkzind=0;
			for(int i=0; i<gridsize; i++)
			  if ( allgrid(i)==t ) {
			  	kzind.row(nkzind)=kz.row(i);
			  	nkzind++;
			  }
			kzind.conservativeResize(nkzind,kzind.cols());

			MatrixXd mkzind = MatrixXd::Zero(1,wordnum);

			vector<mypair> vec;
			MatrixXf::Index minRow=0, minCol=0;
			for(int i=0; i<nkzind; i++) {
				float min = kzind.row(i).maxCoeff(&minRow, &minCol);
				vec.push_back( mypair(min,myindex(i,minCol)) );
			}

			if ( vec.empty() ) continue;
			sort( vec.begin(), vec.end(), comparator );

			std::set<int> picked_words;
			std::set<int> indgrid;
			for(vector<mypair>::iterator it=vec.begin(); it!=vec.end(); it++) {
				int w = it->second.second;
				if ( picked_words.find(w)==picked_words.end() ) {
					picked_words.insert( w );
					indgrid.insert( it->second.first );
				}
			}

			for(set<int>::iterator it=indgrid.begin(); it!=indgrid.end(); it++) {
				mkzind.array() += (kzind.row( *it )).cast<double>().array();
			}
			mkzind.array() /= indgrid.size();

			//pimimfea.block( (t-1)*wordnum, 0, wordnum, 1 ) = (G)*( (mkzind.cast<float>()).transpose());
			pimimfea.block( (t-1)*wordnum, 0, wordnum, 1 ) = ( Gd*(mkzind.transpose()) ).cast<float>();
		}   
		imfea.block( pgrid.block(0,0,1,s).sum() * wordnum, 0, pgrid(s)*wordnum, 1 ) = pimimfea.array() * weights(s);

	}
	//cout << imfea.block(000,0,10,1) << endl;

	return;
}

class KernelDescManager
{

public:
	KernelDescManager(string kdes_dir,string model_name,string model_file,string model_var,string param_file, unsigned int MODEL_TYPE=0, int MAX_IMAGE_SIZE=0);
	~KernelDescManager();
	//		bool Process(MatrixXf& imfea, string image_path);
	bool Process(MatrixXf&imfea, IplImage* image);
	bool Process(MatrixXf&imfea, IplImage* image, const VectorXf& top_left);
	string GetObjectName(VectorXf& scores);
	string GetObjectName(VectorXf& scores, vector<string>& top_objects, vector<double>& top_scores);
    int Classify(VectorXf& scores, const MatrixXf& imfea);
	void PrintModelList();
private:
	string model_name;
	string model_file;
	string model_var;
	string param_file;
	string model_dir;
	string param_dir;
	string model_kdes_treeroot;
	matvarplus_t* model_kdes, *kdes_params;
	vector<string>* model_list;
	unsigned int MODEL_TYPE;
	int MAX_IMAGE_SIZE;
	VectorXf top_left;
    MatrixXf svmW;
};
#endif
