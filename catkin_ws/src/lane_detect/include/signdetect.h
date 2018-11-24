#ifndef SIGN_H
#define SIGN_H

#include "header.h"

class SignDetector
{
private:
	SignDetector();

	HOGDescriptor m_hog = {};
	Ptr<SVM> m_svm;

	Rect m_rect = RECT_ZERO;
	vector<Rect> m_rects;
	
	ESignType m_type = ESignType::NONE;

public:
	SignDetector(const char *modelPath);

private:
	bool Detect(const Mat &binImg);
	ESignType Recognize(const Mat &grayImg);

public:
	ESignType Update(const Mat& hsvImg, const Mat& grayImg);

	Rect GetRect() { return m_rect; };
	ESignType GetType() { return m_type; };
};

#endif