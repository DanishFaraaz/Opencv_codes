#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**){
    VideoCapture video_capture(0);

    if(!video_capture.isOpened()){
        return -1;
    }

    Mat gray_image;
    namedWindow("Edges", 1);
     
    while (true)
    {

        Mat frame;

        video_capture >> frame;
        cvtColor(frame, gray_image, COLOR_BGR2GRAY);


        imshow("Gray Image", gray_image);
        if(waitKey(30)>=0) break;
    }

    return 0;

}