//
//  Xiaofeng Ren, 03/2012
//
#include "libkerneldesc.h"
//#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

// for getting directory information
// needs a link to -lboost_filesystem and -lboost_system
#include <boost/filesystem/operations.hpp> // includes boost/filesystem/path.hpp
#include <boost/filesystem/fstream.hpp>
using namespace  boost::filesystem;
static	IplImage *frame = NULL;
static boost::mutex m;
const char* model_name = model_names[MODEL_TYPE];
const char* model_file = model_files[MODEL_TYPE];
const char* model_var = model_vars[MODEL_TYPE];
const char* param_file = param_files[MODEL_TYPE];


void GetFileListFromDirectory(vector<string>& src, const char* directory)
{
	if (is_directory(directory))
	{
		for (directory_iterator itr(directory); itr != directory_iterator(); ++itr)
		{
			if (!is_directory(itr->status()))
			{
#ifdef NEW_BOOST
				string fn = itr->path().filename().string();  // new boost version
#else
				string fn = itr->path().filename();
#endif
				src.push_back(fn);

			}

		}

	}
	// sort the files by name
	std::sort( src.begin(), src.end() );
}


MatrixXf read_topleft_loc( string filename )
{
	std::ifstream instream;
	string ss;
	MatrixXf top_left(2,1);

	if (!exists(filename)) {
	  cout << "warning: " << filename << " not found; using top_left=(1,1)" << endl;
	  top_left.fill(1.0f);
	  return top_left;
	}

	instream.open( filename.c_str() );
	getline( instream, ss );
	int pos=ss.find(",");
	top_left(0)=atoi( ss.substr(0,pos).c_str() );
	top_left(1)=atoi( ss.substr(pos+1).c_str() );

	instream.close();

	return top_left;
}

bool processImage( KernelDescManager* kdm, const string& filename, std::ofstream& outstream )
{
	MatrixXf imfea2;
	bool result;
	IplImage* img_init;
	if (MODEL_TYPE==2) {    // for color descriptor, read image as color
	  img_init = cvLoadImage( filename.c_str(),CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
	} else {
	  img_init = cvLoadImage( filename.c_str(),CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
	}
	// for depth images, make sure it's 16-bit
	if (MODEL_TYPE>=3)
	  assert( IPL_DEPTH_16U == img_init->depth);
	// for spinkdes, need top_left of the cropped image
	// here assume the input file name is *_depthcrop.png and the loc file is _loc.txt
	if (MODEL_TYPE==4) {
	  MatrixXf top_left = read_topleft_loc( filename.substr(0,filename.length()-14)+"_loc.txt" );
	  result = kdm->Process(imfea2,img_init,top_left);
	} else { 
	  result = kdm->Process(imfea2,img_init);
	}
	cvReleaseImage(&img_init);
	for(int i=0; i<imfea2.size(); i++)
	  outstream << imfea2(i) << " ";
	outstream << endl;

	return result;
}


void ComputeKDES(const char* imagefile, const char* outfile, unsigned int MODEL_TYPE)
{
        // check if imagefile is a directory
	vector<string> filelist;
	std::ofstream outstream;
        bool isDir=false;
        if ( is_directory(imagefile) ) {
	   GetFileListFromDirectory(filelist,imagefile);
           isDir=true;
        } else if ( exists(imagefile) ) {
           filelist.push_back( string(imagefile) );
           isDir=false;
        } else {
           cout << "input image file not found." << endl;
           return;
        }
        outstream.open( outfile, ios_base::app );

	if (outstream.fail()) {
	   cout << "failed to open output file." << endl;
	   return;
	}

	// don't use MAX_IMAGE_SIZE 
	//KernelDescManager* kdm = new KernelDescManager(string(model_names[MODEL_TYPE]), string(model_files[MODEL_TYPE]), string(model_vars[MODEL_TYPE]),string(param_files[MODEL_TYPE]),MODEL_TYPE);
	// use MAX_IMAGE_SIZE for rea-time demo
	KernelDescManager* kdm = new KernelDescManager(".",string(model_names[MODEL_TYPE]), string(model_files[MODEL_TYPE]), string(model_vars[MODEL_TYPE]),string(param_files[MODEL_TYPE]),MODEL_TYPE,MAX_IMAGE_SIZE);

        if (isDir) {

	vector<string>::iterator itr;
	for (itr = filelist.begin(); itr < filelist.end();++itr)
	{
		string filename=string(imagefile + *itr);

		if ( (MODEL_TYPE==3 || MODEL_TYPE==4) && filename.find("depthcrop")==string::npos ) continue;        // for depth descriptors, currently only take _depthcrop.png and _loc.txt pair

		cout << "Processing image: " << filename << endl;
		bool result = processImage( kdm, filename, outstream );

/*
		IplImage* img_init = cvLoadImage(string(imagefile + *itr).c_str(),CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
		// for depth images, make sure it's 16-bit
		if (MODEL_TYPE>=3)
    		  assert( IPL_DEPTH_16U == img_init->depth); 
		// for spinkdes, need top_left of the cropped image
		// here assume the input file name is *_depthcrop.png and the loc file is _loc.txt
		if (MODEL_TYPE==4) {
		  MatrixXf top_left = read_topleft_loc( string(imagefile + *itr) );
		  bool result = kdm->Process(imfea2,img_init,top_left);
		} else { 
		  bool result = kdm->Process(imfea2,img_init);
		}
		cvReleaseImage(&img_init);	
		for(int i=0; i<imfea2.size(); i++)
		  outstream << imfea2(i) << " ";
		outstream << endl;
*/
	}	

	} else {
		string filename = string( imagefile );
		bool result = processImage( kdm, filename, outstream );

/*
                MatrixXf imfea2;
                IplImage* img_init = cvLoadImage(imagefile,CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH);
		// for depth images, make sure it's 16-bit
		if (MODEL_TYPE>=3)
    		  assert( IPL_DEPTH_16U == img_init->depth); 
                bool result = kdm->Process(imfea2,img_init);

                for(int i=0; i<imfea2.size(); i++)
                  outstream << imfea2(i) << " ";
                outstream << endl;
*/
	}
	return;
}


int main(int argc, char* argv[]) {

        if (argc<3 || argc>4) {
           cout << "usage: computekdes image outfile (KDES_TYPE: 0 gradient; 2 color; 3 gradient (depth); 4 spin/normal (depth)" << endl;
           exit(1);
        }

	if (argc==4) {
	   MODEL_TYPE=atoi(argv[3]);
	}

	ComputeKDES(argv[1],argv[2],MODEL_TYPE);

	return 0;
}
