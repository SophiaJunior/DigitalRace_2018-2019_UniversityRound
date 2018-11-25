#ifndef SIGN_DETECT_H
#define SIGN_DETECT_H

#include "header.h"

class SignDetector
{
private:
	SignDetector();

	HOGDescriptor m_hog;
	Ptr<SVM> m_svm;

	// For detecting
	vector< vector<Point> > m_contours;
	vector<bool> m_detectedContours;
	vector<Rect> m_rects;

	// For recognizing
	ESignType m_type = ESignType::NONE;
	Rect m_rect = RECT_ZERO;

public:
	SignDetector(const char *modelPath);

private:
	bool Detect(const Mat &binImg);

	ESignType IpRecognize(const Mat &binImg);
	ESignType MlRecognize(const Mat &grayImg);
	ESignType Recognize(const Mat &binImg, const Mat &grayImg);

public:
	ESignType Update(const Mat& colorImg);

	Rect GetRect() { return m_rect; };
	ESignType GetType() { return m_type; };
};

#endif