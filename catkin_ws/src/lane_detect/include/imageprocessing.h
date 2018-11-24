#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include "header.h"

class ImageProcessor
{
private:
    ImageProcessor() = default;

public:
    void Binarialize(const Mat &hsvImg, char flag, Mat &binImg);
    Mat PreProcess(const Mat &colorImg);

    void fillLane(Mat &src);
    Mat morphological(const Mat &imgHSV);
    void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst);
    Mat birdViewTranform(const Mat &source);
};

#endif