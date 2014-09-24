/** Copyright (c) 2013, TU Darmstadt, Philipp M. Scholl
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.  Redistributions in binary
form must reproduce the above copyright notice, this list of conditions and the
following disclaimer in the documentation and/or other materials provided with
the distribution.  Neither the name of the <ORGANIZATION> nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.  THIS SOFTWARE IS PROVIDED
BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "mapstitch.h"
#include "math.h"
//#include "highgui.h"




StitchedMap::StitchedMap(Mat &img1, Mat &img2, int max_trans, int max_rotation, float max_pairwise_distance, cv::Mat oldTransform)
{
  // load images, TODO: check that they're grayscale
  image1 = img1.clone();
  image2 = img2.clone();
  if(image1.size != image2.size)
      cv::resize(image2,image2,image1.size());
  works = true;
  // create feature detector set.
  OrbFeatureDetector detector;
  OrbDescriptorExtractor dexc;
  BFMatcher dematc(NORM_HAMMING, false);

  // 1. extract keypoint          s
  detector.detect(image1, kpv1);
  detector.detect(image2, kpv2);

  // 2. extract descriptors
  dexc.compute(image1, kpv1, dscv1);
  dexc.compute(image2, kpv2, dscv2);

  // 3. match keypoints
  if(kpv1.size() == 0|| kpv2.size() == 0)
  {
      ROS_WARN("No KPV");
      works = false;
      return;
  }
//  ROS_INFO("Kpv1:%i entries\t Kpv2:%i entries",kpv1.size(),kpv2.size());
  dematc.match(dscv1, dscv2, matches);

  // 4. find matching point pairs with same distance in both images
  for (size_t i=0; i<matches.size(); i++) {
    KeyPoint a1 = kpv1[matches[i].queryIdx],
             b1 = kpv2[matches[i].trainIdx];

    if (matches[i].distance > 30)
      continue;

    for (size_t j=0; j<matches.size(); j++) {
      KeyPoint a2 = kpv1[matches[j].queryIdx],
               b2 = kpv2[matches[j].trainIdx];

      if (matches[j].distance > 30)
        continue;

      if ( fabs(norm(a1.pt-a2.pt) - norm(b1.pt-b2.pt)) > max_pairwise_distance ||
           fabs(norm(a1.pt-a2.pt) - norm(b1.pt-b2.pt)) == 0)
        continue;


      coord1.push_back(a1.pt);
      coord1.push_back(a2.pt);
      coord2.push_back(b1.pt);
      coord2.push_back(b2.pt);


      fil1.push_back(a1);
      fil1.push_back(a2);
      fil2.push_back(b1);
      fil2.push_back(b2);
    }
  }

   // cv::imwrite("img1.pgm",image1);
   // cv::imwrite("img2.pgm",image2);
  // 5. find homography
 // ROS_INFO("Found %i matches",matches.size());
  if(coord1.size() < 1)
  {
      ROS_WARN("Problem by transforming map,this migth just an start up problem \n Coord1:%lu",coord1.size());
      works = false;
      return;
  }

  ROS_DEBUG("Compute estimateRigid");
  H = estimateRigidTransform(coord2, coord1,false);
  if(H.empty())
  {
      ROS_WARN("H contain no data, cannot find valid transformation");
      works = false;
      return;
  }
  //ROS_DEBUG("H: size:%lu|empty:%i",H.size,H.empty());

  rotation = 180./M_PI*atan2(H.at<double>(0,1),H.at<double>(1,1));
  transx   = H.at<double>(0,2);
  transy   = H.at<double>(1,2);
  scalex   = sqrt(pow(H.at<double>(0,0),2)+pow(H.at<double>(0,1),2));
  scaley   = sqrt(pow(H.at<double>(1,0),2)+pow(H.at<double>(1,1),2));
  ROS_DEBUG("H: transx:%f|transy%f|scalex:%f,scaley:%f|rotation:%f",transx,transy,scalex,scaley,rotation);
  //first_x_trans = transx;
  //first_y_trans = transy;
  float scale_change = 0.05;

  if(scalex > 1 + scale_change || scaley > 1 + scale_change)
  {
      ROS_WARN("Map should not scale change is to lagre");
      works = false;
      return;
  }
  if(scalex < 1 - scale_change|| scaley < 1 - scale_change)
  {
      ROS_WARN("Map should not scale change is to small");
      works = false;
      return;
  }
  if(max_trans != -1)
  {
      if(transx > max_trans || transy > max_trans)
      {
          ROS_WARN("Map should not trans so strong");
          works = false;
          return;
      }
  }
  if(max_rotation != -1)
  {
      if(rotation > max_rotation || rotation < -1 * max_rotation)
      {
          ROS_WARN("Map should not rotate so strong");
          works = false;
          return;
      }
  }
  cur_trans = H;
  ROS_DEBUG("Finished estimateRigid");
  //evaluade transformation
  //evaluate
  if(works)
  {
      Mat tmp (image2.size(),image2.type());
      Mat thr;

      Mat image(image2.size(), image2.type());
      warpAffine(image2,image,H,image.size());
      addWeighted(image,.5,image1,.5,0.0,image);

      warpAffine(image2,tmp,H,image2.size());
      addWeighted(tmp,.5,image1,.5,0.0,image);
      //cvtColor(image1,tmp,CV_BGR2GRAY);
      threshold(tmp,thr,0,255,THRESH_BINARY);
      Mat K=(Mat_<uchar>(5,5)<<   0,  0,  1,  0,  0,\
                                     0,  0,  1,  0,  0,\
                                     1,  1,  1,  1,  1,\
                                     0,  0,  1,  0,  0,\
                                     0,  0,  1,  0,  0);

      erode(thr,thr,K,Point(-1,-1),1,BORDER_CONSTANT);
         vector< vector <Point> > contours; // Vector for storing contour
         vector< Vec4i > hierarchy;
         findContours( thr, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
         for( int i = 0; i< contours.size(); i++ )
         {
            Rect r= boundingRect(contours[i]);
            rects2.push_back(r);
            rectangle(tmp,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,0,255),2,8,0);
          }//Opened contour

         Mat thr1;
         //cvtColor(image1,tmp,CV_BGR2GRAY);
         threshold(image1,thr1,0,255,THRESH_BINARY);
         erode(thr1,thr1,K,Point(-1,-1),1,BORDER_CONSTANT);
            vector< vector <Point> > contours1; // Vector for storing contour
            vector< Vec4i > hierarchy1;
            findContours( thr1, contours1, hierarchy1,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

            for( int i = 0; i< contours1.size(); i++ ){
             Rect r= boundingRect(contours1[i]);
             rects1.push_back(r);
             //rectangle(image1,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,0,255),2,8,0);
             }//Opened contour
         vector<Rect> near1,near2;
         int offset = 5;
         for(int i = 0; i < rects1.size(); i++)
         {
             Rect ri = rects1.at(i);
             if(ri.x == 1 && ri.y == 1)
                 continue;
             for(int j = 0; j < rects2.size();j++)
             {
                 Rect rj = rects2.at(j);
                 if(ri.x < rj.x + offset && ri.x > rj.x -offset)
                 {
                     if(ri.y < rj.y + offset && ri.y > rj.y -offset)
                     {
                         near1.push_back(ri);
                         near2.push_back(rj);
                     }
                 }
             }
         }
         double eudis = 0;
         double disX,disY;
         for(int i = 0; i < near1.size(); i++)
         {
             Rect ri = near1.at(i);
             Rect rj = near2.at(i);
             disX = ri.x - rj.x;
             disY = ri.y - rj.y;
             if(disX < 0)
                 disX = disX * (-1);
             if(disY < 0)
                 disY = disY * (-1);
             eudis += sqrt((disX*disX)+(disY*disY));
         }
         if(near1.size() < 2)
             eudis = -1;
         else
             eudis = eudis / near1.size();
         ROS_DEBUG("EudisNew:%f\t near1Size:%lu:\toldTran:%i",eudis,near1.size(),oldTransform.empty());
         //calc EudisOld

         Mat thr3,tmp1;
         //cvtColor(image1,tmp,CV_BGR2GRAY);
         if(oldTransform.empty())
             return;
         warpAffine(image2,tmp1,oldTransform,image2.size());
         threshold(tmp1,thr3,0,255,THRESH_BINARY);

         erode(thr3,thr3,K,Point(-1,-1),1,BORDER_CONSTANT);
            vector< vector <Point> > contours3; // Vector for storing contour
            vector< Vec4i > hierarchy3;
            findContours( thr3, contours3, hierarchy3,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

            for( int i = 0; i< contours3.size(); i++ ){
             Rect r= boundingRect(contours3[i]);
             rects3.push_back(r);
            }//Opened contour
            near1.clear();
            near2.clear();
            for(int i = 0; i < rects1.size(); i++)
            {
                Rect ri = rects1.at(i);
                if(ri.x == 1 && ri.y == 1)
                    continue;
                for(int j = 0; j < rects3.size();j++)
                {
                    Rect rj = rects3.at(j);
                    if(ri.x < rj.x + offset && ri.x > rj.x -offset)
                    {
                        if(ri.y < rj.y + offset && ri.y > rj.y -offset)
                        {
                            near1.push_back(ri);
                            near2.push_back(rj);
                        }
                    }
                }
            }
            double eudisOLD = 0;
            for(int i = 0; i < near1.size(); i++)
            {
                Rect ri = near1.at(i);
                Rect rj = near2.at(i);
                disX = ri.x - rj.x;
                disY = ri.y - rj.y;
                if(disX < 0)
                    disX = disX * (-1);
                if(disY < 0)
                    disY = disY * (-1);
                eudisOLD += sqrt((disX*disX)+(disY*disY));
            }
            if(near1.size() < 2)
                eudis = -1;
            else
                eudisOLD = eudisOLD / near1.size();
            //if(eudisOLD < eudis)
               // works = false;
            ROS_WARN("EudisOLD:%f\t near1Size:%lu:|works:%i",eudis,near1.size(),works);
            //calc EudisOld
         /*  for(int i = 0; i < rects1.size(); i++)
           {
               Rect r = rects1.at(i);
               rectangle(image1,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,0,255),2,8,0);
            }*/

  }
  return;
}

Mat
StitchedMap::get_debug()
{
  Mat out;
  drawKeypoints(image1, kpv1, image1, Scalar(255,0,0));
  drawKeypoints(image2, kpv2, image2, Scalar(255,0,0));
  drawMatches(image1,fil1, image2,fil2, matches,out,Scalar::all(-1),Scalar::all(-1));
  return out;
}

Mat // return the stitched maps
StitchedMap::get_stitch()
{
  // create storage for new image and get transformations
  Mat image(image2.size(), image2.type());
  warpAffine(image2,image,H,image.size());

  // blend image1 onto the transformed image2
  //addWeighted(image,.5,image1,.5,0.0,image);

  return image;
}


StitchedMap::~StitchedMap() { }
