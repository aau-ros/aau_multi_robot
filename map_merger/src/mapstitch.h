#ifndef MAPSTITCH_H
#define MAPSTITCH_H

#include <stdio.h>
#include <math.h>
#include <opencv/cv.h>
#include "ros/ros.h"
#include "ros/console.h"
using namespace cv;
using namespace std;

class StitchedMap
{
public:
  StitchedMap(Mat &img1, Mat &img2,int max_trans,
              int max_rotation,float max_pairwise_distance,
              cv::Mat oldTransform);
  ~StitchedMap();
  bool works;
  float mid_distance;
  Mat get_debug();
  Mat get_stitch();
  Mat H; // transformation matrix
  Mat image1, image2,
      dscv1, dscv2;
  vector<KeyPoint> kpv1,kpv2;
  vector<KeyPoint> fil1,fil2;
  vector<Point2f>  coord1,coord2;
  vector<DMatch>   matches;
  vector<Rect> rects1,rects2,rects3 ;
  double rotation,transx,transy,scalex,scaley;

  cv::Mat cur_trans;

};

#endif // MAPSTITCH_H
