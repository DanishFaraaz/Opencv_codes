#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main() {

    Mat image;

    image = imread("/home/catkin_ws/src/beginner_tutorials/opecv_codes/Images/blackwhite.jpg", CV_LOAD_IMAGE_COLOR);

    if(!image.data)
    {
        cout<<"Could not open or find the image"<<endl;
        return -1;
    }
    
    namedWindow("Window", CV_WINDOW_AUTOSIZE);
    imshow("Window", image);

    imwrite("/home/catkin_ws/src/beginner_tutorials/opecv_codes/Images/Copy/copy.jpg", image);

    waitKey(0);
    return 0;
}