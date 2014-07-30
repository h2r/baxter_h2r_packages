//#define DRAW_WHITE
#define DRAW_ALL_GREEN
#define DRAW_BLUE
#define DRAW_GRAY
//#define DRAW_RED
#define DRAW_LABEL
#define DRAW_ORIENTOR

#define RUN_INFERENCE
const int k = 1;

//#define PUBLISH_OBJECTS

int numRedBoxes = 5;

// if you add more examples, you must reload the data for them to 
//   be taken into account.
// if you add a class or many new examples, it would be wise to
//   recalculate the vocab.
// if you add an immense number of examples or some new classes and
//   you begin having discriminative problems (confusion), you can
//   increase the number of words.
//#define RELOAD_DATA
//#define RELEARN_VOCAB
const int vocabNumWords = 1000;
const double grayBlur = 1.0;

// you can mine hard negatives by modifying the box saving routine 
//   to save examples not belonging to class C while only showint it
//   class C objects
//#define SAVE_BOXES
//#define SAVE_ANNOTATED_BOXES
// always increment the prefix so that the next person doesn't overwrite
//   your examples.  
char run_prefix[] = "autolearn_16";
int fcRange = 10;


#include <dirent.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
//#include <object_recognition_msgs/RecognizedObjectArray.h>
//#include <object_recognition_msgs/RecognizedObject.h>
#include <sstream>
//#include "libkerneldesc.cc"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <meldon_detection/RecognizedObjectArray.h>
//#include <cv.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/opencv.hpp>

#include "/home/oberlin/catkin_ws_baxter/src/BING-Objectness-master/Objectness/stdafx.h"
#include "/home/oberlin/catkin_ws_baxter/src/BING-Objectness-master/Objectness/Objectness.h"
#include "/home/oberlin/catkin_ws_baxter/src/BING-Objectness-master/Objectness/ValStructVec.h"
#include "/home/oberlin/catkin_ws_baxter/src/BING-Objectness-master/Objectness/CmShow.h"

//KernelDescManager* kdm;
static unsigned int LOC_MODEL_TYPE=0; //0 or 3

cv::Mat cam_img;
cv::Mat depth_img;
ros::Publisher rec_objs;
bool real_img = false;
bool vis = true;

// Top variables are top left corners of bounding boxes (smallest coordinates)
// Bot variables are bottom right corners of bounding boxes (largenst coordinates)
// Cen cariables are centers of bounding boxes
cv::vector<cv::Point> wTop;
cv::vector<cv::Point> wBot;
cv::vector<cv::Point> nTop;
cv::vector<cv::Point> nBot;

Objectness *glObjectness;
int fc;
int cropCounter;

double *temporalDensity = NULL;
double *temporalDepth = NULL;
double densityDecay = 0.7;
double depthDecay = 0.7;

DescriptorMatcher *matcher;
FeatureDetector *detector;
DescriptorExtractor *extractor;
BOWKMeansTrainer *bowtrainer; 
BOWImgDescriptorExtractor *bowExtractor;
CvKNearest *kNN;

#define MY_FONT FONT_HERSHEY_PLAIN

#define ORIENTATIONS 36 
#define O_FILTER_WIDTH 25
#define O_FILTER_SPOON_HEAD_WIDTH 8 
#define O_FILTER_SPOON_SHAFT_WIDTH 1
Mat *orientedFilters;
int biggestL1 = 0;

typedef struct {
  int classLabel;
  cv::Point top;
  cv::Point bot;
  int rootBlueBox;
  int numGreenBoxes;
  cv::Point anchor;
  double persistence;
} redBox;

redBox *redBoxes;
double redDecay = 0.7;

// forward declare
void bowGetFeatures(const char *classDir, const char *className, double sigma);
void kNNGetFeatures(const char *classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels);

void depthCallback(const sensor_msgs::ImageConstPtr& msg){
/*

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Size sz = depth_img.size();
  int imW = sz.width;
  int imH = sz.height;

  if (temporalDepth == NULL) {
    temporalDepth= new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	temporalDepth[y*imW + x] = 0;
      }
    }
  }

// /camera/depth_registered/points sensor_msgs:PointCloud2

  uint maxDepth = 0.0; 
  uint minDepth = 256*256-1;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uint tDepth = depth_img.at<unsigned short>(y,x);
      maxDepth = max(maxDepth, tDepth);
      if (tDepth > 0)
	minDepth = min(minDepth, tDepth);
      //cout << " " << float(depth_img.at<uint>(y,x));
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uint tDepth = depth_img.at<unsigned short>(y,x);
      temporalDepth[y*imW+x] = depthDecay*temporalDepth[y*imW+x] + (1.0-depthDecay)*float(tDepth);
      //cout << " " << float(depth_img.at<uint>(y,x));
      if (tDepth <= minDepth)
	depth_img.at<short>(y,x) = 0;
      //else
	//depth_img.at<uint>(y,x) = tDepth - minDepth;
    }
  }

  maxDepth = max(maxDepth,uint(1));

  cv::imshow("Depth Viewer", depth_img*20);
  cout << depth_img.size() << " " << depth_img.type() << " " << maxDepth << " " << minDepth << endl;
*/
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

  cv::Rect bound;
  cv::Mat boxed;

  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_img = cv_ptr->image;
    real_img = true;
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  meldon_detection::RecognizedObjectArray to_send;

  int boxesPerSize = 800;
  
  // XXX find best method and stop all this cloning, it might be relatively slow
  Mat original_cam_img = cam_img.clone();

  Mat img_cvt = cam_img.clone();
  Mat img_cvtH = cam_img.clone();
  Mat img_cvtG = cam_img.clone();
  Mat img_cvtGtmp = cam_img.clone();

  Mat img_cvt_blur = cam_img.clone();
  //img_cvt.copyTo(cv_ptr->image);

  cvtColor(cam_img, img_cvtGtmp, CV_RGB2GRAY);
  cvtColor(cam_img, img_cvtH, CV_RGB2HSV);

  vector<Mat> channels;
  channels.push_back(img_cvtGtmp);
  channels.push_back(img_cvtGtmp);
  channels.push_back(img_cvtGtmp);
  merge(channels, img_cvtG);

  // input image is noisy so blurring is a good idea
  //GaussianBlur(img_cvt, img_cvt, cv::Size(0,0), 1.0);

  ValStructVec<float, Vec4i> boxes;
  //glObjectness->getObjBndBoxes(cam_img, boxes, boxesPerSize);
  glObjectness->getObjBndBoxes(img_cvt, boxes, boxesPerSize);


  int numBoxes = boxes.size();
  // box[0] minx box[1] miny box[2] maxx box[3] maxy
  cout << numBoxes << "    " << fc <<  endl;

  nTop.resize(numBoxes);
  nBot.resize(numBoxes);

  int boxesToConsider= 50000;

  fc = (fc + 1) % fcRange;

  Size sz = img_cvt.size();
  int imW = sz.width;
  int imH = sz.height;
  double *integralDensity = new double[imW*imH];
  double *density = new double[imW*imH];
  double *differentialDensity = new double[imW*imH];
  if (temporalDensity == NULL) {
    temporalDensity = new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	temporalDensity[y*imW + x] = 0;
      }
    }
  }


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      density[y*imW + x] = 0;
      differentialDensity[y*imW + x] = 0;
    }
  }

  if(vis){
    // draw the ork bounding boxes
    for(int i =0; i<wTop.size(); i++){
#ifdef DRAW_WHITE
      rectangle(cv_ptr->image, wTop[i], wBot[i], cv::Scalar(255,255,255));
#endif
    }
  }

  // XXX make this prettier
  for(int i =0; i < min(numBoxes, boxesToConsider); i++){
    int boxInd = numBoxes-1-i;
    boxInd = max(0, boxInd);
    nTop[i].x = boxes[boxInd][0] - 1;
    nTop[i].y = boxes[boxInd][1] - 1;
    nBot[i].x = boxes[boxInd][2] - 1;
    nBot[i].y = boxes[boxInd][3] - 1;
    double width = nBot[i].x - nTop[i].x;
    double height= nBot[i].y - nTop[i].y;
    double ratio = width / height;
    double area = width*height;
    double aspectThresh = 20.0;
    if (ratio < aspectThresh && ratio > 1.0/aspectThresh) {
      //rectangle(cv_ptr->image, nTop[i], nBot[i], cv::Scalar(0,255,0));
      
      double toAdd = 1.0 / area;
      //double toAdd = 1.0;
      int x = nTop[i].x;
      int y = nTop[i].y;
      differentialDensity[y*imW + x] += toAdd;
      x = nBot[i].x;
      y = nBot[i].y;
      differentialDensity[y*imW + x] += toAdd;
      x = nTop[i].x;
      y = nBot[i].y;
      differentialDensity[y*imW + x] -= toAdd;
      x = nBot[i].x;
      y = nTop[i].y;
      differentialDensity[y*imW + x] -= toAdd;
    }
  }


  // integrate the differential density into the density
  density[0] = differentialDensity[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    density[y*imW+x] = density[y*imW+(x-1)] + differentialDensity[y*imW + x];
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    density[y*imW+x] = density[(y-1)*imW+x] + differentialDensity[y*imW + x];
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      density[y*imW+x] = 
	density[(y-1)*imW+x]+density[y*imW+(x-1)]-density[(y-1)*imW+(x-1)]+differentialDensity[y*imW + x];
    }
  }

  // now update the exponential average of the density
  // and set the density to be a thresholded version of this
  double threshFraction = 0.35;
  double maxDensity = 0;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      temporalDensity[y*imW+x] = densityDecay*temporalDensity[y*imW+x] + (1.0-densityDecay)*density[y*imW+x];
      density[y*imW+x] = temporalDensity[y*imW+x];
      maxDensity = max(maxDensity, density[y*imW+x]);
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (density[y*imW+x] < maxDensity * threshFraction)
	density[y*imW+x] = 0;
    }
  }


  // integrate density into the integral density
  integralDensity[0] = density[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    integralDensity[y*imW+x] = integralDensity[y*imW+(x-1)] + density[y*imW + x];
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    integralDensity[y*imW+x] = integralDensity[(y-1)*imW+x] + density[y*imW + x];
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      integralDensity[y*imW+x] = 
	integralDensity[(y-1)*imW+x]+integralDensity[y*imW+(x-1)]-integralDensity[(y-1)*imW+(x-1)]+density[y*imW + x];
    }
  }

  // determine table edges, i.e. the gray boxes
  int top_gray_bottom = 40;
  int bottom_gray_top = imH-40;

#ifdef DRAW_GRAY
  rectangle(cv_ptr->image, cv::Point(0,0), cv::Point(imW-1, top_gray_bottom), cv::Scalar(128,128,128));
  rectangle(cv_ptr->image, cv::Point(0,bottom_gray_top), cv::Point(imW-1, imH-1), cv::Scalar(128,128,128));
#endif

  // truncate the density above the gray boxes
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < top_gray_bottom; y++) {
      density[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = bottom_gray_top; y < imH; y++) {
      density[y*imW+x] = 0;
    }
  }

  // reintegrate the density
  //double maxIntegralDensity = 0;
  integralDensity[0] = density[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    integralDensity[y*imW+x] = integralDensity[y*imW+(x-1)] + density[y*imW + x];
    //maxIntegralDensity = max(maxIntegralDensity, integralDensity[y*imW+x]);
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    integralDensity[y*imW+x] = integralDensity[(y-1)*imW+x] + density[y*imW + x];
    //maxIntegralDensity = max(maxIntegralDensity, integralDensity[y*imW+x]);
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      integralDensity[y*imW+x] = 
	integralDensity[(y-1)*imW+x]+integralDensity[y*imW+(x-1)]-integralDensity[(y-1)*imW+(x-1)]+density[y*imW + x];
      //maxIntegralDensity = max(maxIntegralDensity, integralDensity[y*imW+x]);
    }
  }

  int gBoxW = 10;
  int gBoxH = 10;
  int gBoxStrideX = ceil(float(gBoxW / 2.0));
  int gBoxStrideY = ceil(float(gBoxH / 2.0));
  double intThresh = 3;

  double *gBoxIndicator = new double[imW*imH];
  double *gBoxGrayNodes = new double[imW*imH];
  double *gBoxComponentLabels = new double[imW*imH];

  vector<cv::Point> cTops; 
  vector<cv::Point> cBots;

  vector<int> parentX;
  vector<int> parentY;
  vector<int> parentD;

  const int directionX[] = {1, 0, -1,  0};
  const int directionY[] = {0, 1,  0, -1};

  int total_components = 0;

  for (int x = 0; x < imW-gBoxW; x+=gBoxStrideX) {
    for (int y = 0; y < imH-gBoxH; y+=gBoxStrideY) {

      int xt = x;
      int yt = y;
      int xb = x+gBoxW;
      int yb = y+gBoxH;
      cv::Point thisTop(xt,yt);
      cv::Point thisBot(xb,yb);

      gBoxComponentLabels[y*imW+x] = -1;
      gBoxGrayNodes[y*imW+x] = 0;
      gBoxIndicator[y*imW+x] = 0;

      double thisIntegral = integralDensity[yb*imW+xb]-integralDensity[yb*imW+xt]-integralDensity[yt*imW+xb]+integralDensity[yt*imW+xt];
      if (thisIntegral > intThresh) {
	gBoxIndicator[y*imW+x] = 1;
#ifdef DRAW_ALL_GREEN
	rectangle(cv_ptr->image, thisTop, thisBot, cv::Scalar(0,255,0));
#endif
      }

    }
  }


  for (int x = 0; x < imW-gBoxW; x+=gBoxStrideX) {
    for (int y = 0; y < imH-gBoxH; y+=gBoxStrideY) {
      if (gBoxIndicator[y*imW+x] == 1 && gBoxGrayNodes[y*imW+x] == 0) {

	gBoxGrayNodes[y*imW+x] = 1;
	parentX.push_back(x);
	parentY.push_back(y);
	parentD.push_back(0);

	gBoxComponentLabels[y*imW+x] = total_components;
	total_components++;

	int xt = x;
	int yt = y;
	int xb = x+gBoxW;
	int yb = y+gBoxH;
	cv::Point thisTop(xt,yt);
	cv::Point thisBot(xb,yb);
	cTops.push_back(thisTop);
	cBots.push_back(thisBot);


	while( parentX.size() > 0 ) {
	  int index = parentX.size()-1;
	  int direction = parentD[index];
	  parentD[index]++;
	  int nextX = parentX[index] + gBoxStrideX*directionX[direction];
	  int nextY = parentY[index] + gBoxStrideY*directionY[direction];

	  // if we have no more directions, then pop this parent 
	  if (direction > 3) {
	    parentX.pop_back();
	    parentY.pop_back();
	    parentD.pop_back();
	  } 
	  // if the next direction is valid, push it on to the stack and increment direction counter
	  else if(gBoxIndicator[nextY*imW+nextX] == 1 && gBoxGrayNodes[nextY*imW+nextX] == 0
		  && nextX > -1 && nextX < imW && nextY > -1 && nextY < imH) {

	    gBoxGrayNodes[nextY*imW+nextX] = 1;
	    gBoxComponentLabels[nextY*imW+nextX] = gBoxComponentLabels[parentY[index]*imW+parentX[index]];

	    int nxt = nextX;
	    int nyt = nextY;
	    int nxb = nextX+gBoxW;
	    int nyb = nextY+gBoxH;
	    cTops[gBoxComponentLabels[nextY*imW+nextX]].x = min(cTops[gBoxComponentLabels[nextY*imW+nextX]].x, nxt);
	    cTops[gBoxComponentLabels[nextY*imW+nextX]].y = min(cTops[gBoxComponentLabels[nextY*imW+nextX]].y, nyt);
	    cBots[gBoxComponentLabels[nextY*imW+nextX]].x = max(cBots[gBoxComponentLabels[nextY*imW+nextX]].x, nxb);
	    cBots[gBoxComponentLabels[nextY*imW+nextX]].y = max(cBots[gBoxComponentLabels[nextY*imW+nextX]].y, nyb);

	    parentX.push_back(nextX);
	    parentY.push_back(nextY);
	    parentD.push_back(0);
	  } 
	}


      }
    }
  }


  // create the blue boxes from the parental green boxes
  vector<cv::Point> bTops; 
  vector<cv::Point> bBots;
  vector<cv::Point> bCens;
  // track the half widths and half heights of the blue boxes for the merge step later
  vector<int> bHWs;
  vector<int> bHHs;

  double rejectScale = 2.0;
  double rejectArea = 6*6*gBoxW*gBoxH;
  double rejectHigh = 70;
  double rejectLow = imH - 70;
  for (int c = 0; c < total_components; c++) {
    int allow = 1;
    if (cBots[c].x - cTops[c].x < rejectScale*gBoxW || cBots[c].y - cTops[c].y < rejectScale*gBoxH)
      allow = 0;
    if ((cBots[c].x - cTops[c].x)*(cBots[c].y - cTops[c].y) < rejectArea)
      allow = 0;
    //if (cTops[c].y > rejectLow || cBots[c].y < rejectHigh)
      //allow = 0;
    if (allow == 1) {
      bTops.push_back(cTops[c]);
      bBots.push_back(cBots[c]);
      bCens.push_back(cv::Point((cTops[c].x+cBots[c].x)/2, (cTops[c].y+cBots[c].y)/2));
      int t = bTops.size()-1;
      bHWs.push_back(bCens[t].x-bTops[t].x);
      bHHs.push_back(bCens[t].y-bTops[t].y);
    }
  }


  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uchar val = uchar(min( 255.0 * density[y*imW+x] / maxDensity, 255.0));
      img_cvt.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);
      //img_cvt.at<uchar>(y, x, 0) = 0;
    }
  }

#ifdef RUN_INFERENCE
  vector<int> bLabels;
  // classify the crops
  for (int c = 0; c < bTops.size(); c++) {
    vector<KeyPoint> keypoints;
    Mat descriptors;

    Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
    Mat gray_image;
    cvtColor(crop, gray_image, CV_BGR2GRAY);
    GaussianBlur(gray_image, gray_image, cv::Size(0,0), grayBlur);

    detector->detect(gray_image, keypoints);
    bowExtractor->compute(gray_image, keypoints, descriptors);

    double label = 0.0;
    if (!descriptors.empty()) {
      label = kNN->find_nearest(descriptors,k);
    }

    char labelName[256]; 
    if (label == 0)
      sprintf(labelName, "VOID");
    if (label == 1)
      sprintf(labelName, "gyroBowl");
    if (label == 2)
      sprintf(labelName, "mixBowl");
    if (label == 3)
      sprintf(labelName, "woodSpoon");
    if (label == 4)
      sprintf(labelName, "plasticSpoon");
    if (label == 5)
      sprintf(labelName, "background");
    if (label == 6)
      sprintf(labelName, "human");
    if (label == 7)
      sprintf(labelName, "sippyCup");

    bLabels.push_back(label);
  
  #ifdef SAVE_ANNOTATED_BOXES
    // save the crops
    if (fc == 0) {
      Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
      char buf[1000];
      sprintf(buf, "/home/oberlin/catkin_ws_baxter/src/baxter_h2r_packages/meldon_detection/src/classCrops/%s/%s_%d.ppm", 
	labelName, run_prefix, cropCounter);
      imwrite(buf, crop);
      cropCounter++;
    }
  #endif
    int winningO = -1;
  #ifdef DRAW_ORIENTOR
    if (label == 3 || label == 4) {

      Mat gCrop = img_cvt(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
      cv::resize(gCrop, gCrop, orientedFilters[0].size());
      gCrop.convertTo(gCrop, orientedFilters[0].type());

      Mat gcChannels[3];
      split(gCrop, gcChannels);

      double winningScore = -1;
      for (int o = 0; o < ORIENTATIONS; o++) {
	double thisScore = gcChannels[1].dot(orientedFilters[o]);
	if (thisScore > winningScore) {
	  winningScore = thisScore;
	  winningO = o;
	}
      }

      Mat vCrop = cv_ptr->image(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
      vCrop = vCrop.mul(0.5);

      Mat scaledFilter;
      cv::resize(orientedFilters[winningO], scaledFilter, vCrop.size());
      scaledFilter = biggestL1*scaledFilter;
       
      vector<Mat> channels;
      channels.push_back(Mat::zeros(scaledFilter.size(), scaledFilter.type()));
      channels.push_back(Mat::zeros(scaledFilter.size(), scaledFilter.type()));
      channels.push_back(scaledFilter);
      merge(channels, scaledFilter);

      scaledFilter.convertTo(scaledFilter, vCrop.type());
      vCrop = vCrop + 128*scaledFilter;
    }
  #endif
  #ifdef DRAW_LABEL
    cv::Point text_anchor(bTops[c].x, bBots[c].y);
    putText(cv_ptr->image, labelName, text_anchor, MY_FONT, 1.5, Scalar(255,0,0), 2.0);
  #endif

  #ifdef PUBLISH_OBJECTS
    if (label >= 1 && label <= 4) {

      geometry_msgs::Pose object_pose;

      // XXX obtain the quaternion pose for the table
      Eigen::Quaternionf tableQuaternion;

      cv::Matx33f R;
      R(0,0) = 1;R(0,1) = 0;R(0,2) = 0;
      R(1,0) = 0;R(1,1) = 1;R(1,2) = 0;
      R(2,0) = 0;R(2,1) = 0;R(2,2) = 1;

      // handle the rotation differently depending on the class
      // if we have a spoon
      if (label == 3 || label == 4) {
	double theta = (pi / 2.0) + (winningO*2*pi/ORIENTATIONS);
	R(0,0) = cos(theta);R(0,1) = -sin(theta);R(0,2) = 0;
	R(1,0) = sin(theta);R(1,1) =  cos(theta);R(1,2) = 0;
	R(2,0) = 0;R(2,1) = 0;R(2,2) = 1;
      }

      // XXX determine the x,y,z coordinates of the object from bounding box and point cloud
      object_pose.position.x = ;
      object_pose.position.y = ;
      object_pose.position.z = ;

      Eigen::Matrix3f rotation;
      rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
      Eigen::Quaternionf objectQuaternion(rotation);

      objectQuaternion = tableQuaternion * objectQuaternion;

      object_pose.orientation.w = objectQuaternion.w;
      object_pose.orientation.x = objectQuaternion.x;
      object_pose.orientation.y = objectQuaternion.y;
      object_pose.orientation.z = objectQuaternion.z;

      // XXX form the final message to send
      //to_send.objects.resize(msg.markers.size());
      // XXX fill out the point cloud
      //geometry_msgs/Point[] bounding_contours

    }
  rec_objs.publish(to_send);
  #endif

  }

  #ifdef DRAW_RED
  #pragma omp parallel for
  for (int r = 0; r < numRedBoxes; r++) {

    redBox *thisRedBox = &(redBoxes[r]);
    int thisClass = thisRedBox->classLabel;

    int accepted = 0;
    // check to see if there is a class who wants this red box
    //   if so, scan it, if not, scan all
    for (int c = 0; c < bTops.size(); c++) {
      if (bLabels[c] == thisClass) {
	accepted = 1;
	thisRedBox->rootBlueBox = c;
      }
    }


    int redStride = 30;
    
    if (accepted) {
      
      cv::Point hTop = bTops[thisRedBox->rootBlueBox];
      cv::Point hBot = bBots[thisRedBox->rootBlueBox];

      int winJ = -1;
      float winD = 1e6;
      cv::Point winTop(0,0);
      cv::Point winBot(0,0);
      
      int ix = min(hTop.x + thisRedBox->bot.x, hBot.x);
      int iy = min(hTop.y + thisRedBox->bot.y, hBot.y);
      /*1*/cv::Point itTop(hTop.x,0);
      /*1*/cv::Point itBot(ix , 0);
      for (/*1*/; itBot.x <= hBot.x; /*2*/) {
	
	/*3*/itTop.y = hTop.y;
	/*3*/itBot.y = iy;
	for (/*3*/; itBot.y <= hBot.y; /*4*/) {


	  // score this crop
	  vector<KeyPoint> keypoints;
	  Mat descriptors;

	  Mat crop = original_cam_img(cv::Rect(itTop.x, itTop.y, itBot.x-itTop.x, itBot.y-itTop.y));
	  Mat gray_image;
	  cvtColor(crop, gray_image, CV_BGR2GRAY);
	  GaussianBlur(gray_image, gray_image, cv::Size(0,0), grayBlur);

	  detector->detect(gray_image, keypoints);
	  bowExtractor->compute(gray_image, keypoints, descriptors);

	  int thisK = 16;
	  Mat neighbors(1, thisK, CV_32F);
	  Mat dist;
	  if (!descriptors.empty()) {
	    kNN->find_nearest(descriptors, thisK, 0, 0, &neighbors, &dist);
	  }

	  int thisJ = 0;
	  float thisD = 1e6;
	  for (int n = 0; n < thisK; n++) {
	    if (neighbors.at<float>(n,1) == float(thisClass)) { 
	      thisJ++;
	      thisD = min(dist.at<float>(n,1), winD);
	    }
	  }
	  cout << thisJ << endl;
	  if (thisJ > 0 && thisD < winD) {
	    winJ = thisJ;
	    winD = thisD;
	    winTop = itTop;
	    winBot = itBot;
	  }


	  cout << "class: " << thisClass << " descriptors: " << keypoints.size() << " " << itBot << itTop << "  " << hTop << hBot << endl;


	  /*4*/itTop.y += redStride;
	  /*4*/itBot.y += redStride;
	}

	/*2*/itTop.x += redStride;
	/*2*/itBot.x += redStride;
      }

      // always decay persistence but only add it if we got a hit
      thisRedBox->persistence = redDecay*thisRedBox->persistence;
      if (winD < 1e6) {
	thisRedBox->anchor.x = redDecay*thisRedBox->anchor.x + (1.0-redDecay)*winTop.x;
	thisRedBox->anchor.y = redDecay*thisRedBox->anchor.y + (1.0-redDecay)*winTop.y;
	thisRedBox->persistence = thisRedBox->persistence + (1.0-redDecay)*1.0;
      }

      //cv::Point dTop(winTop.x+10, winTop.y+10);
      //cv::Point dBot(winBot.x-10, winBot.y-10);
      cv::Point dTop(thisRedBox->anchor.x, thisRedBox->anchor.y);
      cv::Point dBot(thisRedBox->anchor.x + winBot.x - winTop.x, thisRedBox->anchor.y + winBot.y - winTop.y);


      char labelName[256]; 
      if (thisClass == 0)
	sprintf(labelName, "VOID");
      if (thisClass == 1)
	sprintf(labelName, "gyroBowl");
      if (thisClass == 2)
	sprintf(labelName, "mixBowl");
      if (thisClass == 3)
	sprintf(labelName, "woodSpoon");
      if (thisClass == 4)
	sprintf(labelName, "plasticSpoon");
      if (thisClass == 5)
	sprintf(labelName, "background");
      if (thisClass == 6)
	sprintf(labelName, "human");
      if (thisClass == 7)
	sprintf(labelName, "sippyCup");

      if (thisRedBox->persistence > 0.5) {
	rectangle(cv_ptr->image, dTop, dBot, cv::Scalar(0,0,255));
	cv::Point text_anchor(dTop.x, dTop.y+20);
	putText(cv_ptr->image, labelName, text_anchor, MY_FONT, 1.5, Scalar(0,0,255), 2.0);
      }
    } else {
      int winJ = -1;
      float winD = 1e6;
      cv::Point winTop(0,0);
      cv::Point winBot(0,0);

      for (int c = 0; c < bTops.size(); c++) {
	cv::Point hTop = bTops[c];
	cv::Point hBot = bBots[c];
	
	int ix = min(hTop.x + thisRedBox->bot.x, hBot.x);
	int iy = min(hTop.y + thisRedBox->bot.y, hBot.y);
	/*1*/cv::Point itTop(hTop.x,0);
	/*1*/cv::Point itBot(ix , 0);
	for (/*1*/; itBot.x <= hBot.x; /*2*/) {
	  
	  /*3*/itTop.y = hTop.y;
	  /*3*/itBot.y = iy;
	  for (/*3*/; itBot.y <= hBot.y; /*4*/) {


	    // score this crop
	    vector<KeyPoint> keypoints;
	    Mat descriptors;

	    Mat crop = original_cam_img(cv::Rect(itTop.x, itTop.y, itBot.x-itTop.x, itBot.y-itTop.y));
	    Mat gray_image;
	    cvtColor(crop, gray_image, CV_BGR2GRAY);
	    GaussianBlur(gray_image, gray_image, cv::Size(0,0), grayBlur);

	    detector->detect(gray_image, keypoints);
	    bowExtractor->compute(gray_image, keypoints, descriptors);

	    int thisK = 16;
	    Mat neighbors(1, thisK, CV_32F);
	    Mat dist;
	    if (!descriptors.empty()) {
	      kNN->find_nearest(descriptors, thisK, 0, 0, &neighbors, &dist);
	    }

	    int thisJ = 0;
	    float thisD = 1e6;
	    for (int n = 0; n < thisK; n++) {
	      if (neighbors.at<float>(n,1) == float(thisClass)) {
		thisJ++;
		thisD = min(dist.at<float>(n,1), winD);
	      }
	    }
	    cout << thisJ << endl;
	    if (thisJ > 0 && thisD < winD) {
	      winJ = thisJ;
	      winD = thisD;
	      winTop = itTop;
	      winBot = itBot;
	    }


	    cout << "class: " << thisClass << " descriptors: " << keypoints.size() << " " << itBot << itTop << "  " << hTop << hBot << endl;


	    /*4*/itTop.y += redStride;
	    /*4*/itBot.y += redStride;
	  }

	  /*2*/itTop.x += redStride;
	  /*2*/itBot.x += redStride;
	}


      }

      // always decay persistence but only add it if we got a hit
      thisRedBox->persistence = redDecay*thisRedBox->persistence;
      if (winD < 1e6) {
	thisRedBox->anchor.x = redDecay*thisRedBox->anchor.x + (1.0-redDecay)*winTop.x;
	thisRedBox->anchor.y = redDecay*thisRedBox->anchor.y + (1.0-redDecay)*winTop.y;
	thisRedBox->persistence = thisRedBox->persistence + (1.0-redDecay)*1.0;
      }

      //cv::Point dTop(winTop.x+10, winTop.y+10);
      //cv::Point dBot(winBot.x-10, winBot.y-10);
      cv::Point dTop(thisRedBox->anchor.x, thisRedBox->anchor.y);
      cv::Point dBot(thisRedBox->anchor.x + winBot.x - winTop.x, thisRedBox->anchor.y + winBot.y - winTop.y);


      char labelName[256]; 
      if (thisClass == 0)
	sprintf(labelName, "VOID");
      if (thisClass == 1)
	sprintf(labelName, "gyroBowl");
      if (thisClass == 2)
	sprintf(labelName, "mixBowl");
      if (thisClass == 3)
	sprintf(labelName, "woodSpoon");
      if (thisClass == 4)
	sprintf(labelName, "plasticSpoon");
      if (thisClass == 5)
	sprintf(labelName, "background");
      if (thisClass == 6)
	sprintf(labelName, "human");
      if (thisClass == 7)
	sprintf(labelName, "sippyCup");

      if (thisRedBox->persistence > 0.5) {
	rectangle(cv_ptr->image, dTop, dBot, cv::Scalar(0,0,255));
	cv::Point text_anchor(dTop.x, dTop.y+20);
	putText(cv_ptr->image, labelName, text_anchor, MY_FONT, 1.5, Scalar(0,0,255), 2.0);
      }
    }

  }
  #endif

#endif

#ifdef SAVE_BOXES
  // save the crops
  if (fc == 0) {
    for (int c = bTops.size()-1; c >= 0; c--) {
      Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
      char buf[1000];
      sprintf(buf, "/home/oberlin/catkin_ws_baxter/src/baxter_h2r_packages/meldon_detection/src/savedCrops/%s_%d.ppm", 
	run_prefix, cropCounter);
      // uncomment if statement for hard negative mining
      //if(bLabels[c] != 7) {
	imwrite(buf, crop);
	cropCounter++;
      //}
    }
  }
#endif


/*
  // form red boxes by merging close parents
  vector<cv::Point> rTops; 
  vector<cv::Point> rBots;
  vector<cv::Point> rCens;
  
  int mergePadding = 50;
  int areaThresh = 200*200;

  for (int c = bTops.size()-1; c >= 0; c--) {
    for (int d = c-1; d >= 0; d--) {

      if ( fabs(bCens[c].x-bCens[d].x) <= bHWs[c]+bHWs[d]+mergePadding && fabs(bCens[c].y-bCens[d].y) <= bHHs[c]+bHHs[d]+mergePadding ) {
	cv::Point thisTop(min(bTops[c].x,bTops[d].x), min(bTops[c].y,bTops[d].y));
	cv::Point thisBot(max(bBots[c].x,bBots[d].x), max(bBots[c].y,bBots[d].y));
	int area = (thisBot.x - thisTop.x) * (thisBot.y - thisTop.y);
	if (area < areaThresh) {
	  rTops.push_back(thisTop);
	  rBots.push_back(thisBot);
	  int t = rTops.size()-1;
	  cv::Point thisCen = cv::Point((rTops[t].x+rBots[t].x)/2, (rTops[t].y+rBots[t].y)/2);
	  rCens.push_back(thisCen);
	  rectangle(cv_ptr->image, rTops[t], rBots[t], cv::Scalar(0,0,255));
	}
      }
    }
  }
*/

#ifdef DRAW_BLUE
  for (int c = bTops.size()-1; c >= 0; c--) {
    rectangle(cv_ptr->image, bTops[c], bBots[c], cv::Scalar(255,0,0));
  }
#endif

  cv::imshow("Object Viewer", cv_ptr->image);
  cv::imshow("Density Viewer", img_cvt);

  delete gBoxIndicator;
  delete gBoxGrayNodes;
  delete gBoxComponentLabels;
  delete integralDensity;
  delete density;
  delete differentialDensity;

  img_cvt.release();
  img_cvt_blur.release();
  img_cvtG.release();
  img_cvtGtmp.release();
  img_cvtH.release();

  cv::waitKey(1);
}

void clusterCallback(const visualization_msgs::MarkerArray& msg){
	if(real_img){
		meldon_detection::RecognizedObjectArray to_send;
		to_send.objects.resize(msg.markers.size());
		//MatrixXf imfea2;
		//VectorXf scores;
		IplImage temp = cam_img;
		int height = cam_img.size().height;
		int width = cam_img.size().width;
		ROS_INFO("(%d,%d)", height, width);
		float minx, miny, maxx, maxy, px, py;
		cv::Rect bound;
		cv::Mat boxed;
		geometry_msgs::Point p;
		wTop.resize(msg.markers.size());
		wBot.resize(msg.markers.size());
		ROS_INFO("Objects found: %d", int(msg.markers.size()));
		float f;
		std::string res;
		if(LOC_MODEL_TYPE == 0) f= 580;
		else f=525;
		float cx = (width/2) - 0.5;
		float cy = (height/2) - 0.5;
		for(int i=0; i<msg.markers.size(); i++){
			minx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
			maxx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
			miny = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
			maxy = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
			for(int j = 0; j < msg.markers[i].points.size(); j++){
				p = msg.markers[i].points[j];
				px = f*(p.x/p.z) + cx;
				py = f*(p.y/p.z) + cy;
				if(px < minx){minx = px;}
				if(px > maxx){maxx = px;}
				if(py < miny){miny = py;}
				if(py > maxy){maxy = py;}
			}
			wTop[i] = cv::Point(minx - 5, miny - 5);
			wBot[i] = cv::Point(maxx + 5, maxy + 5);
			if(wTop[i].x < 0){wTop[i].x = 0;}
			else if(wTop[i].x > width - 1){wTop[i].x = width-1;}
			if(wTop[i].y < 0) wTop[i].y = 0;
			else if(wTop[i].y > height-1){wTop[i].y = height-1;}
			if(wBot[i].x < 0){wBot[i].x = 0;}
			else if(wBot[i].x > width - 1){wBot[i].x = width-1;}
			if(wBot[i].y < 0) wBot[i].y = 0;
			else if(wBot[i].y > height-1){wBot[i].y = height-1;}
		}/*
		for(int i=0; i<wTop.size(); i++){
			ROS_INFO("BOXED: (%d, %d) to (%d, %d)", wTop[i].x, wTop[i].y, wBot[i].x, wBot[i].y);
			bound = cv::Rect(wTop[i].x, wTop[i].y, wBot[i].x - wTop[i].x, wBot[i].y - wTop[i].y);
			boxed = cam_img(bound);
			temp = boxed;
			kdm->Process(imfea2, &temp);
			kdm->Classify(scores, imfea2);
			res = kdm->GetObjectName(scores);
			ROS_INFO("Tabletop %d identified as %s", i, res.c_str());
			meldon_detection::RecognizedObject to_add;
			to_add.points = msg.markers[i].points;
			to_add.name = res.c_str();
			to_send.objects[i] = to_add;
		}*/
		temp = cam_img;
		//kdm->Process(imfea2, &temp);
		//kdm->Classify(scores, imfea2);
		//rec_objs.publish(to_send);
	}
	ROS_INFO("Identification complete");
}

int main(int argc, char **argv) {

  // initialize the redBoxes
  //   it is ok to add more redBoxes and ok to duplicate classes,
  //   i.e. you can add a second gyroBowl
  // number class
  // 1 gyro
  // 2 mix
  // 3 wood
  // 4 plastic
  // 7 sippy

#ifdef DRAW_RED

  redBoxes = new redBox[numRedBoxes];
  redBoxes[0].classLabel = 1;
  redBoxes[0].top = cv::Point(0,0);
  redBoxes[0].bot = cv::Point(150, 150);
  redBoxes[1].classLabel = 2;
  redBoxes[1].top = cv::Point(0,0);
  redBoxes[1].bot = cv::Point(200, 200);
  redBoxes[2].classLabel = 3;
  redBoxes[2].top = cv::Point(0,0);
  redBoxes[2].bot = cv::Point(250,250);
  redBoxes[3].classLabel = 4;
  redBoxes[3].top = cv::Point(0,0);
  redBoxes[3].bot = cv::Point(250,250);
  redBoxes[4].classLabel = 7;
  redBoxes[4].top = cv::Point(0,0);
  redBoxes[4].bot = cv::Point(100,100);

  // initialize the rest
  for (int r = 0; r < numRedBoxes; r++) {
    redBoxes[r].rootBlueBox = 0;
    redBoxes[r].numGreenBoxes;
    redBoxes[r].anchor = cv::Point(0,0);
    redBoxes[r].persistence = 0;
  }

#endif

  ros::init(argc, argv, "meldon_detection");
  ros::NodeHandle n;
  std::string s;

  image_transport::Subscriber image_sub;
  ros::Subscriber cloud_sub;

  image_transport::ImageTransport it(n);
  ros::Subscriber clusters = n.subscribe("/tabletop/clusters", 1, clusterCallback);

  //rec_objs = n.advertise<object_recognition_msgs::RecognizedObjectArray>("labeled_objects", 10);
  rec_objs = n.advertise<meldon_detection::RecognizedObjectArray>("labeled_objects", 10);

  image_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  cloud_sub = n.subscribe("/camera/depth_registered/points", 1, pointCloudCallback);

  cv::namedWindow("Object Viewer");
  cv::namedWindow("Density Viewer");
  //cv::namedWindow("Depth Viewer");
  //cv::moveWindow("Depth Viewer", 2000+700, 0);
  cv::moveWindow("Density Viewer", 2000+0, 700);
  cv::moveWindow("Object Viewer", 2000+0, 0);


  DataSetVOC voc("../VOC2007/");
  Objectness objNess(voc, 2, 8, 2);
  glObjectness = &(objNess);

  CvMat* my_matrix;
  my_matrix = (CvMat*)cvLoad("/home/oberlin/catkin_ws_baxter/src/baxter_h2r_packages/meldon_detection/src/ObjectnessTrainedModel/ObjNessB2W8I.idx.yml");
  int *data = my_matrix->data.i;

  cout << "test" << endl << "test" << data[0] << data[1] << data[2] << data[6] << endl << endl;

  //int result = objNess.loadTrainedModel("/home/oberlin/catkin_ws_baxter/src/baxter_h2r_packages/meldon_detection/src/ObjectnessTrainedModel/ObjNessB2W8I");
  //int result = objNess.loadTrainedModel("/home/oberlin/catkin_ws_baxter/src/baxter_h2r_packages/meldon_detection/src/ObjectnessTrainedModel/ObjNessB2W8HSV");
  int result = objNess.loadTrainedModel("/home/oberlin/catkin_ws_baxter/src/baxter_h2r_packages/meldon_detection/src/ObjectnessTrainedModel/ObjNessB2W8MAXBGR");
  cout << "result: " << result << endl << endl;

  fc = 0;
  cropCounter = 0;


#ifdef RUN_INFERENCE

  // SIFT 
  //detector = new SiftFeatureDetector(0, 3, 0.04, 10, 1.6);
  detector = new FastFeatureDetector(4);
  extractor = new SiftDescriptorExtractor();

  // BOW time
  bowtrainer = new BOWKMeansTrainer(vocabNumWords);

  // read the class image data
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  const char classDir[] = "/home/oberlin/catkin_ws_baxter/src/baxter_h2r_packages/meldon_detection/src/classCrops/";
  char buf[1024];

  char vocabularyPath[1024];
  char featuresPath[1024];
  sprintf(vocabularyPath, "%s/vocab.yml", classDir);
  sprintf(featuresPath, "%s/features.yml", classDir);

  Mat vocabulary;

#ifdef RELEARN_VOCAB
  bowGetFeatures(classDir, "gyroBowl", grayBlur);
  bowGetFeatures(classDir, "mixBowl", grayBlur);
  bowGetFeatures(classDir, "woodSpoon", grayBlur);
  bowGetFeatures(classDir, "plasticSpoon", grayBlur);
  bowGetFeatures(classDir, "background", grayBlur);
  bowGetFeatures(classDir, "human", grayBlur);
  bowGetFeatures(classDir, "sippyCup", grayBlur);

  cout << "Clustering features...";
  vocabulary = bowtrainer->cluster();
  cout << "done." << endl;

  FileStorage fsvO;
  cout<<"Writing vocab..."<< endl << vocabularyPath << endl << "...";
  fsvO.open(vocabularyPath, FileStorage::WRITE);
  fsvO << "vocab" << vocabulary;
  fsvO.release();
  cout << "done." << endl;
#else
  FileStorage fsvI;
  cout<<"Reading vocab..."<< endl << vocabularyPath << endl << "...";
  fsvI.open(vocabularyPath, FileStorage::READ);
  fsvI["vocab"] >> vocabulary;
  cout << "done." << vocabulary.size() << endl;
#endif

  matcher = new BFMatcher(NORM_L2);
  bowExtractor = new BOWImgDescriptorExtractor(extractor,matcher);
  bowExtractor->setVocabulary(vocabulary);

  Mat kNNfeatures;
  Mat kNNlabels;

#ifdef RELOAD_DATA
  kNNGetFeatures(classDir, "gyroBowl", 1, grayBlur, kNNfeatures, kNNlabels);
  kNNGetFeatures(classDir, "mixBowl", 2, grayBlur, kNNfeatures, kNNlabels);
  kNNGetFeatures(classDir, "woodSpoon", 3, grayBlur, kNNfeatures, kNNlabels);
  kNNGetFeatures(classDir, "plasticSpoon", 4, grayBlur, kNNfeatures, kNNlabels);
  kNNGetFeatures(classDir, "background", 5, grayBlur, kNNfeatures, kNNlabels);
  kNNGetFeatures(classDir, "human", 6, grayBlur, kNNfeatures, kNNlabels);
  kNNGetFeatures(classDir, "sippyCup", 7, grayBlur, kNNfeatures, kNNlabels);

  FileStorage fsfO;
  cout<<"Writing features and labels..."<< endl << featuresPath << endl << "...";
  fsfO.open(featuresPath, FileStorage::WRITE);
  fsfO << "features" << kNNfeatures;
  fsfO << "labels" << kNNlabels;
  fsfO.release();
  cout << "done." << endl;
#else
  FileStorage fsfI;
  cout<<"Reading features and labels..."<< endl << featuresPath << endl << "...";
  fsfI.open(featuresPath, FileStorage::READ);
  fsfI["features"] >> kNNfeatures;
  fsfI["labels"] >> kNNlabels;
  cout << "done." << kNNfeatures.size() << " " << kNNlabels.size() << endl;
#endif

  cout << kNNlabels.size().height << " " << kNNlabels.size().width << endl;
  cout << kNNfeatures.size().height << " " << kNNfeatures.size().width << endl;

  kNN = new CvKNearest(kNNfeatures, kNNlabels);

#endif

  // manually definining spoon filters
  orientedFilters = new Mat[ORIENTATIONS];
  orientedFilters[0].create(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F);
  for (int x = 0; x < O_FILTER_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 0.0;
    }
  }

/*
  // Diagonal Spoon Filter
  for (int x = 0; x < O_FILTER_SPOON_HEAD_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_SPOON_HEAD_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
  for (int x = 0; x < O_FILTER_WIDTH; x++) {
    for (int y = max(0,x-O_FILTER_SPOON_SHAFT_WIDTH); y < min(O_FILTER_WIDTH, x+O_FILTER_SPOON_SHAFT_WIDTH); y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
*/
  // Vertical Spoon Filter
  int center = (O_FILTER_WIDTH-1)/2;
  for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH; x <= center+O_FILTER_SPOON_SHAFT_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
  for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH-1; x <= center+O_FILTER_SPOON_SHAFT_WIDTH+1; x++) {
    for (int y = 1; y < 3+O_FILTER_SPOON_HEAD_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
  for (int x = center-O_FILTER_SPOON_HEAD_WIDTH; x <= center+O_FILTER_SPOON_HEAD_WIDTH; x++) {
    for (int y = 2; y < 2+O_FILTER_SPOON_HEAD_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }

  // it is important to L1 normalize the filters so that comparing dot products makes sense.
  // that is, they should all respond equally to a constant image.
  double l1norm = orientedFilters[0].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
  biggestL1 = l1norm;

  for (int o = 1; o < ORIENTATIONS; o++) {
    // Compute a rotation matrix with respect to the center of the image
    Point center = Point(O_FILTER_WIDTH/2, O_FILTER_WIDTH/2);
    double angle = o*360.0/ORIENTATIONS;
    double scale = 1.0;

    // Get the rotation matrix with the specifications above
    Mat rot_mat = getRotationMatrix2D( center, angle, scale );

    // Rotate the warped image
    warpAffine(orientedFilters[0], orientedFilters[o], rot_mat, orientedFilters[o].size());

    double l1norm = orientedFilters[o].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
    orientedFilters[o] = orientedFilters[o] / l1norm;
    
    if (l1norm > biggestL1)
      biggestL1 = l1norm;
  }

  l1norm = orientedFilters[0].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
  orientedFilters[0] = orientedFilters[0] / l1norm;



  ros::spin();
  return 0;
}

void bowGetFeatures(const char *classDir, const char *className, double sigma) {

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s", classDir, className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
     while (epdf = readdir(dpdf)){
	if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	  vector<KeyPoint> keypoints;
	  Mat descriptors;

	  char filename[1024];
	  sprintf(filename, "%s%s/%s", classDir, className, epdf->d_name);
	  Mat image;
	  image = imread(filename);
	  Mat gray_image;
	  cvtColor(image, gray_image, CV_BGR2GRAY);
	  GaussianBlur(gray_image, gray_image, cv::Size(0,0), sigma);

	  detector->detect(gray_image, keypoints);
	  extractor->compute(gray_image, keypoints, descriptors);

	  cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << endl;

	  if (!descriptors.empty())
	    bowtrainer->add(descriptors);
	}
     }
  }

}

void kNNGetFeatures(const char *classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels) {

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s", classDir, className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
     while (epdf = readdir(dpdf)){
	if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	  vector<KeyPoint> keypoints;
	  Mat descriptors;

	  char filename[1024];
	  sprintf(filename, "%s%s/%s", classDir, className, epdf->d_name);
	  Mat image;
	  image = imread(filename);
	  Mat gray_image;
	  cvtColor(image, gray_image, CV_BGR2GRAY);
	  GaussianBlur(gray_image, gray_image, cv::Size(0,0), sigma);

	  detector->detect(gray_image, keypoints);
	  bowExtractor->compute(gray_image, keypoints, descriptors);

	  cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << endl;

	  if (!descriptors.empty()) {
	    kNNfeatures.push_back(descriptors);
	    kNNlabels.push_back(label);
	  }
	}
     }
  }

}

/* Notes

Eventually features should be calculated on at most the green boxes.


*/
