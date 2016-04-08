#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv/cv.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */

/** @function main */
int main( int argc, char** argv ) {
  Mat src, src_gray, dst;
  int img_width, img_height;
  int dif;

  /// Load an image
  src = imread( argv[1] );
  if( !src.data )
    { return -1; }

  // Get the width and height of the image
  img_width = src.cols;
  img_height = src.rows;

  GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
  cvtColor(src, src_gray, COLOR_RGB2GRAY);
  Canny(src_gray, dst, 40, 100, 3);

  vector<Vec2f> lines;

  // Configured for image 500 pixels wide
  HoughLines(dst, lines, 1, CV_PI/180, 60, 0, 0 );

  for( size_t i = 0; i < lines.size(); i++ )
    {
       float rho = lines[i][0], theta = lines[i][1];
       Point pt1, pt2;
       double a = cos(theta), b = sin(theta);
       double x0 = a*rho, y0 = b*rho;
       pt1.x = cvRound(x0 + 1000*(-b));           
       pt1.y = cvRound(y0 + 1000*(a));            
       pt2.x = cvRound(x0 - 1000*(-b));
       pt2.y = cvRound(y0 - 1000*(a));
       line( src, pt1, pt2, Scalar(255,0,0), 1, 8);
    }

  // Scan the image at different heights
  for(int y = 1; y < img_height; y += 20) {
      // Scan from middle to right
      // start at the middle pixel and compare the colour until a blue pixel is found i.e. a hough line
      Point right;
      right.x = -1;
      right.y = y;
      for(int x = img_width/2; x < img_width; x++) {
          // Check to see if a blue pixel has been found
          if(src.at<cv::Vec3b>(y, x)[0] >= 200) {
              right.x = x;                    // set the x coordinate to the value where a blue pixel was detected
              break;
          }
      }

      // Scan from middle to left
      Point left;
      left.x = -1;
      left.y = y;
      for(int x = img_width/2; x > 0; x--) {
        // Check if colour of pixel is blue
        if(src.at<cv::Vec3b>(y, x)[0] >= 200) {
          left.x = x;
          break;
        }
      }

    // If a the lane lines are detected, draw from the middle to that line
    if(left.x > 0) {
      line(src, Point(img_width/2, y), left, Scalar(0, 255, 0), 1, 8);     // draw a green line from the middle to left point
    }
    if(right.x > 0) {
      line(src, Point(img_width/2, y), right, Scalar(0, 0, 255), 1, 8);
    }
    
    // Calculate the difference in length of the red and green lines
    // i.e. how off centre the car is
      /*
       *  Move the car based on this difference
       *  Could calculate the average dif for each "line sample"
       *  positive --> further left, negative --> further right
       *
       */
    dif = (right.x - img_width/2) - (img_width/2 - left.x);
    cout <<"The difference is: "<<dif<<endl;
  }

 imshow("source", src);
 //imshow("detected lines", src);

 waitKey();

 return 0;
}