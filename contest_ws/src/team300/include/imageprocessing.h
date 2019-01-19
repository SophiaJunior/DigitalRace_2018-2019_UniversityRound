#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "header.h"

class ImageProcessor
{
  private:
    ImageProcessor() = default;

  public:
    void Binarialize(const Mat &hsvImg, char flag, Mat &binImg);

    Mat birdViewTranform(const Mat &source);
};

#endif