//
//  Xiaofeng Ren, 03/2012
//
#include "libkerneldesc.h"
//#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <stdlib.h>

// for getting directory information
// needs a link to -lboost_filesystem and -lboost_system
#include <boost/filesystem/operations.hpp> // includes boost/filesystem/path.hpp
#include <boost/filesystem/fstream.hpp>
//using namespace  boost::filesystem;
static	IplImage *frame = NULL;
static boost::mutex m;
const char* model_name = model_names[MODEL_TYPE];
const char* model_file = model_files[MODEL_TYPE];
const char* model_var = model_vars[MODEL_TYPE];
const char* param_file = param_files[MODEL_TYPE];

#define	USE_GRABCUT	1

#ifdef USE_GRABCUT
  #include <opencv2/imgproc/imgproc.hpp>
#endif

#define FRAME_WIDTH_LIVE  160*2
#define FRAME_HEIGHT_LIVE  120*2

bool run_grab_cut=false;
bool use_classification_momemtum=true;
const double CLASSIFICATION_MOMEMTUM=3;

string object_name;
	CvFont font;
	CvFont font_s;

CvCapture *capture = NULL;

void ClearCinBufferFlags()
{

	cin.clear();	//opt = -1;
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
}
void SetupFont (CvFont& font)
{
	double hScale=0.8;
	double vScale=0.8;
	int    lineWidth=2;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
	hScale=0.6;
	vScale=0.6;
	lineWidth=1;
	cvInitFont(&font_s,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
}


void ThreadCaptureFrame()
{
	
	cout << "Cam capture started..." << endl;

	int key = 0;


	//capture = cvCaptureFromCAM(0);

	//cvNamedWindow("Camera View", CV_WINDOW_AUTOSIZE);
	while ((char)key != 'q')
	{
		{
			boost::mutex::scoped_lock lock(m);
			frame = cvQueryFrame(capture);

		}
		if (!frame) break;
		cvPutText(frame, object_name.c_str(), cvPoint(10,20), &font,cvScalar(0,256,0));
		//cvShowImage("Camera View", frame);
		key = cvWaitKey(10);

        // toggle single frame classification vs interval
        if (char(key)=='m') {
            use_classification_momemtum = !use_classification_momemtum;
            if (!use_classification_momemtum) {
                cout << "Enter single-frame classification mode..." << endl;
            } else {
                cout << "Enter momemtum-based classification mode..." << endl;
            }
        }
	}
	cvReleaseImage(&frame);
	cvReleaseCapture(&capture);
	cout << "Cam capture ended" << endl;

}
void ThreadRunDescriptors()
{
        MatrixXf imfea;
        VectorXf scores;
        VectorXf scores_mom;

	// rescale window size
	const double frame_ratio=FRAME_WIDTH_LIVE/160;

	// font stuff
	CvFont font;
	SetupFont(font);
	//


	cout << "Descriptor thread started..." << endl;
	int key = 0;
	KernelDescManager* kdm = new KernelDescManager(".",string(model_name), string(model_file), string(model_var), string(param_file), MODEL_TYPE, MAX_IMAGE_SIZE);
	IplImage* img_src = NULL;
	IplImage* img_crop = NULL;
	//cvNamedWindow ("Cropped Frame", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Processed Frame", CV_WINDOW_AUTOSIZE);
	while ((char)key != 'q')
	{
		//cout<< "Looping" << endl;
		MatrixXf imfea2;

		{
			boost::mutex::scoped_lock lock(m);
			if (frame != NULL) { 
				cvReleaseImage(&img_src);
				img_src = cvCloneImage(frame);
			}
		}
			if (img_src != NULL)
			{
				  cv::Rect bounding_rect( (int)(30*frame_ratio), (int)(10*frame_ratio), (int)(100*frame_ratio), (int)(100*frame_ratio) );
						  cvSetImageROI( img_src, bounding_rect );
						  img_crop=cvCreateImage( cvSize(bounding_rect.width,bounding_rect.height), IPL_DEPTH_8U, 3 );
						  cvCopy( img_src, img_crop, NULL );
						  cvResetImageROI( img_src );
				  //img_crop=img_src;

				if (img_crop != NULL)
				{
                    kdm->Process(imfea2,img_crop);
                    kdm->Classify(scores,imfea2);

                    if (!use_classification_momemtum || scores_mom.size()==0) {
                        scores_mom=scores;
                    } else {
                        scores_mom=(scores_mom*CLASSIFICATION_MOMEMTUM+scores)/(1+CLASSIFICATION_MOMEMTUM);
                    }

                    vector<string> top_objects;
                    vector<double> top_scores;

                        object_name = kdm->GetObjectName(scores_mom,top_objects,top_scores);

					// create an expanded image to accommodate a list of objects
					IplImage* img_src_e=cvCreateImage( cvSize(img_src->width,img_src->height+OBJECTS_TO_PRINT*21), IPL_DEPTH_8U, 3 );
					cvZero(img_src_e);
					cvSetImageROI( img_src_e, cv::Rect( 0, 0, img_src->width,img_src->height ) );
					cvCopy( img_src, img_src_e, NULL );
					cvResetImageROI( img_src_e );

					cvPutText(img_src_e, object_name.c_str(), cvPoint(10,20), &font,cvScalar(0,256,0));
					for(int k=0; k<OBJECTS_TO_PRINT; k++) {
						cvPutText(img_src_e, top_objects[k].c_str(), cvPoint(10, img_src->height+(k)*21+14), &font_s,cvScalar(0,256-k*250/OBJECTS_TO_PRINT,0+k*250/OBJECTS_TO_PRINT));
						cvSetImageROI( img_src_e, cv::Rect( 140, img_src->height+k*21+5, min( max( (double)top_scores[k]+1.5,0.0 ), 2.0 )*150/2, 10.0 ) );
						cvSet( img_src_e, cvScalar(0,256-k*250/OBJECTS_TO_PRINT,0+k*250/OBJECTS_TO_PRINT) );
						cvResetImageROI( img_src_e );
					}
					//cvShowImage("Cropped Frame", img_crop);
					    IplImage* img_show=cvCreateImage( cvSize(img_src_e->width*2,img_src_e->height*2), IPL_DEPTH_8U, 3 );
						cvResize( img_src_e, img_show );
					cvShowImage("Processed Frame", img_show);
					//			cv::DisplayOverlay("Camera View", object_name.c_str(), 1000);
					cvReleaseImage(&img_show);
					cvReleaseImage(&img_src_e);
					cvReleaseImage(&img_crop);
				}
			}
		
		key = cvWaitKey(1);
	}
	delete kdm;
	cout << "Descriptor thread ended..." << endl;

}
void GetFileListFromDirectory(vector<string>& src, const char* directory)
{
	if (boost::filesystem::is_directory(directory))
	{
		for (boost::filesystem::directory_iterator itr(directory); itr != boost::filesystem::directory_iterator(); ++itr)
		{
			if (!is_directory(itr->status()))
			{
#ifdef NEW_BOOST
				string fn = itr->path().filename().string(); // new boost version
#else
				string fn = itr->path().filename(); // old boost version?
#endif
				src.push_back(fn);

			}

		}

	} else {
		cout << "Image directory not found in path!" << endl;
	}
}
void DataSetDemo()
{
	// font stuff

	SetupFont(font);

	vector<string> filelist;
	GetFileListFromDirectory(filelist,"testim");
	cout << "first name: " << filelist[0] << endl;
	cvNamedWindow("Processed Image", CV_WINDOW_AUTOSIZE);
	KernelDescManager* kdm = new KernelDescManager(".", string(model_name), string(model_file), string(model_var), string(param_file));

	vector<string>::iterator itr;
	for (itr = filelist.begin(); itr < filelist.end();++itr)
	{
		MatrixXf imfea2;
        VectorXf scores;

		IplImage* img_init = cvLoadImage(string("./testim/" + *itr).c_str(),CV_LOAD_IMAGE_ANYCOLOR);
		kdm->Process(imfea2,img_init);
		kdm->Classify(scores,imfea2);

		//cvShowImage("ProcessedImage", frame);
		string object_name = kdm->GetObjectName(scores);
		cvPutText(img_init, object_name.c_str(), cvPoint(30,30), &font ,cvScalar(0,256,0));
		cvShowImage("Processed Image", img_init);
		char c = cvWaitKey(0);
		cvReleaseImage(&img_init);
	}	
}
int main(int argc, char* argv[]) {

	int opt = -1;
	while (opt != 1 || opt != 2 || opt != 3)
	{
		cout << "Enter '1' for camera demo (full image) or '2' for demo with image dataset: ";
		cin >> opt;
		cout << "Opt: " << opt << endl;
		if (opt==1)
		{
	
			// On Windows: change camera resolution only works in the main thread
			capture = cvCaptureFromCAM(0);
			if (!capture)
			{
				fprintf(stderr, "Cannot open the webcam.\n");
				return 1;
			}
			cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH_LIVE );
			cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT_LIVE );

			int key = 0;
			boost::thread workerThread(&ThreadCaptureFrame);
			boost::thread descriptorThread(&ThreadRunDescriptors);
			cout << "Starting worker thread" << endl;
			workerThread.join();
			cout << "Starting descriptor thread" << endl;
			descriptorThread.join();
		} else if (opt == 2)
		{
			DataSetDemo();
		} else 
		{
			ClearCinBufferFlags();
		}
	}
	fflush(stdout);

	return 0;
}
