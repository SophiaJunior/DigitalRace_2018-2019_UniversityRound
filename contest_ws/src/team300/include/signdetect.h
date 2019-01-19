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
	vector<vector<Point>> m_contours;
	vector<bool> m_detectedContours;
	vector<Rect> m_rects;

	// For recognizing
	SignType m_type = SignType::NONE;
	Rect m_rect = RECT_ZERO;

public:
	SignDetector(const char *modelPath);

private:
	bool Detect(const Mat &binImg);

	SignType IpRecognize(const Mat &binImg);
	SignType MlRecognize(const Mat &grayImg);
	SignType Recognize(const Mat &binImg, const Mat &grayImg);

public:
	void Update(const Mat &colorImg);

	Rect GetRect() { return m_rect; };
	SignType GetType() { return m_type; };
};

#endif