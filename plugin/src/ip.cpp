#include "ip.h"


cv::Mat ip::segmentateHSV(cv::Mat image){

    int hueMin = 0, saturationMin = 0, valueMin = 0;
    int hueMax = 255, saturationMax = 255, valueMax = 255;

    //convert to HSV
    cv::Mat HSV(image.rows, image.cols, CV_8UC3);
    cv::cvtColor(image, HSV, CV_BGR2HSV);

    cv::Mat dst;

    inRange(HSV, cv::Scalar(hueMin,saturationMin,valueMin), cv::Scalar(hueMax,saturationMax,valueMax), dst);

    return dst;
}

cv::Mat ip::opening(cv::Mat image, int elementType, int kernelSize){
    //elementType 0 = MORPTH_RECT, 1 = MORPH_CROSS, 2 = MORPH_ELLIPSE
    cv::Mat dst;
    cv::Mat element = cv::getStructuringElement(elementType,
                      cv::Size(2*kernelSize+1, 2*kernelSize+1),
                      cv::Point(kernelSize,kernelSize));

    cv::erode(image,dst,element);
    cv::dilate(dst, dst, element);


    return dst;
}

cv::Mat ip::closing(cv::Mat image, int elementType, int kernelSize){
    //elementType 0 = MORPTH_RECT, 1 = MORPH_CROSS, 2 = MORPH_ELLIPSE
    cv::Mat dst;
    cv::Mat element = cv::getStructuringElement(elementType,
                      cv::Size(2*kernelSize+1, 2*kernelSize+1),
                      cv::Point(kernelSize,kernelSize));

    cv::dilate(image,dst,element);
    cv::erode(dst,dst,element);


    return dst;
}

std::vector<std::vector<cv::Point> > ip::findContours(cv::Mat image){

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(image,contours,cv::RETR_TREE, cv::CHAIN_APPROX_NONE );

    return contours;
}


cv::Mat ip::drawContours(std::vector<std::vector<cv::Point> > contours, cv::Mat image){

    cv::Mat contourImg(image.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255,0,0);
    colors[1] = cv::Scalar(0,255,0);
    colors[2] = cv::Scalar(0,0,255);

    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(contourImg, contours, i, colors[i % 3]);
    }

    return contourImg;
}

std::vector<cv::Moments> ip::getMu(std::vector<std::vector<cv::Point> > contours ){
    std::vector<cv::Moments> mu(contours.size() );
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = cv::moments(contours[i],false);
    }
    return mu;
}

std::vector<cv::Point2i> ip::getCenterPoints(std::vector<cv::Moments> moments, std::vector<std::vector<cv::Point> > contours){
    std::vector<cv::Point2i> centerPoints(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        centerPoints[i] = cv::Point2f(moments[i].m10/moments[i].m00, moments[i].m01/moments[i].m00);
    }
    return centerPoints;
}


cv::Mat ip::drawPoints(std::vector<cv::Point2i> points, cv::Mat image){

    cv::Mat dst = image.clone();

    for (int i = 0; i < points.size(); i++) {
        dst.at<cv::Vec3b>(points[i]) = cv::Vec3b(254,254,254);
    }

    return dst;
}

std::vector<cv::Point2i> ip::toRobotPoints(std::vector<cv::Point2i> points, cv::Mat image){

    cv::Point2i UV(image.cols/2,image.rows/2);
    std::vector<cv::Point2i> newPoints = points;
    //std::cout << "x: " << UV.x << " y: " << UV.y << std::endl;
    for (int i = 0; i < points.size() ; ++i) {
       // std::cout << "x: " << (UV-points[i]).x << " y: " << (UV-points[i]).y << std::endl;
        newPoints[i] = UV-points[i];
    }
    return newPoints;
}
