/*
*   EC444 webcam demo: detect brightness changes of BU desk lamp
*
*   Ellen Lo
*
*/

#include "opencv2/opencv.hpp"

using namespace cv;

int w, h;
Size small;

int main(int, char**)
{
    VideoCapture cap(1); // open video capture of external camera
    if(!cap.isOpened())
        return -1;

    Mat frame, smallFrame, bwFrame, brightFrame;
    cap >> frame;
    w = frame.cols * 0.1;
    h = frame.rows * 0.1;
    small = Size(w, h);

    namedWindow("video cap",1);
    while(1)
    {
        cap >> frame;
        resize(frame, smallFrame, small, 0, 0, INTER_LINEAR); // reduce size of image processing to speed up
        // cvtColor(smallFrame, bwFrame, COLOR_BGR2GRAY);

        // 1st try
        // int sum = 0;
        // for(int y = 0; y < h; y ++) {
        //   for(int x = 0; x < w; x++) {
        //     sum += (int)smallFrame.at<uchar>(y, x);
        //   }
        // }
        // float avg = (float)sum / (w * h) / 255;
        // printf("Brightness: %f\n", avg);

        // 2nd try: find brightest point
        // Point light(0, 0);
        // unsigned char maxBrightness = 0;
        // for(int y = 0; y < h; y ++) {
        //   for(int x = 0; x < w; x++) {
        //     unsigned char c = bwFrame.at<uchar>(y, x);
        //     if(c > maxBrightness) {
        //       maxBrightness = c;
        //       light.x = x;
        //       light.y = y;
        //     }
        //   }
        // }
        // circle(bwFrame, light, 4, Scalar(0, 0, 255), 1, 8, 0);

        // 4th try: using HSV color space
        // cvtColor(smallFrame, brightFrame, COLOR_BGR2HSV);
        // int sum = 0;
        // for(int y = 0; y < h; y ++) {
        //   for(int x = 0; x < w; x ++) {
        //     Vec3b hsv = brightFrame.at<Vec3b>(y, x);
        //     unsigned char v = hsv[0];
        //     sum += (int)v;
        //   }
        // }
        // float avg = (float)sum / (float)(w * h) / 255.0;
        // printf("Brightness: %f\n", avg);

        // 3rd try: using Lab color space
        cvtColor(smallFrame, brightFrame, COLOR_BGR2Lab);
        int sum = 0;
        for(int y = 0; y < h; y ++) {
          for(int x = 0; x < w; x ++) {
            Vec3b lab = brightFrame.at<Vec3b>(y, x);
            unsigned char l = lab[0];
            sum += (int)l;
          }
        }
        int avg = sum / (w * h);

        // Custom threshold for brightness changes
        if( avg < 100 ) {
          printf("Dark\n");
        } else if ( avg >= 100 && avg < 135 ) {
          printf("Mildly bright\n");
        } else {
          printf("Bright\n");
        }
        printf("Brightness: %d\n", avg);

        imshow("video cap", brightFrame);
        if(waitKey(30) >= 0) break;
    }
    return 0;
}
