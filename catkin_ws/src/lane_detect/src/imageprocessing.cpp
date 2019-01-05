#include "imageprocessing.h"

void ImageProcessor::Binarialize(const Mat &hsvImg, char flag, Mat &binImg)
{
    Mat tmpBin(binImg.size(), CV_8UC1);

    if (flag & EColor::BLACK)
    {
        inRange(hsvImg, MIN_HSV_BLACK, MAX_HSV_BLACK, tmpBin);
        bitwise_or(tmpBin, binImg, binImg);
    }
    if (flag & EColor::BLUE)
    {
        inRange(hsvImg, MIN_HSV_BLUE, MAX_HSV_BLUE, tmpBin);
        bitwise_or(tmpBin, binImg, binImg);
    }
}

// Mat ImageProcessor::morphological(const Mat &img)
// {
//     Mat dst;

//     // erode(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
//     // dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );

//     dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)));
//     erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)));

//     // blur(dst, dst, Size(3, 3));

//     return dst;
// }

// void ImageProcessor::transform(Point2f *src_vertices, Point2f *dst_vertices, Mat &src, Mat &dst)
// {
//     Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
//     warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
// }

Mat ImageProcessor::birdViewTranform(const Mat &src)
{
    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, SKYLINE);
    src_vertices[1] = Point(width, SKYLINE);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(105, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}