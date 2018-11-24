#include "signdetect.h"
#include "imageprocessing.h"

SignDetector::SignDetector(const char *modelPath)
{
	m_hog = HOGDescriptor(SIGN_SIZE,
		Size(SIGN_SIZE.width / 2, SIGN_SIZE.height / 2),
		Size(SIGN_SIZE.width / 4, SIGN_SIZE.height / 4),
		Size(SIGN_SIZE.width / 4, SIGN_SIZE.height / 4),
		9);

	// m_svm = cv::ml::SVM::create();
	// m_svm = cv::ml::SVM::create();
    // m_svm->setType(ml::SVM::Types::C_SVC);
    // m_svm->setKernel(ml::SVM::KernelTypes::RBF);
	// m_svm->setC(12.5);
	// m_svm->setGamma(0.50625);
	//m_svm = Algorithm::load<SVM>(modelPath);
	m_svm = SVM::load(modelPath);
	//m_svm->load(modelPath);
	
	m_rect = RECT_ZERO;
	m_type = ESignType::NONE;
}

bool SignDetector::Detect(const Mat &binImg)
{
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(binImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Reset
	//cout << m_rects.
	//m_rects.push_back(Rect(0, 0, 0, 0));
	//cout << m_rects.size() << endl;
	//m_rects.clear();
	//m_rects.reserve(contours.size());
	
	return false;

	for (int iContour = 0; iContour < contours.size(); iContour++)
	{
		// Areas
		double area = contourArea(contours[iContour]);
		Rect rect = boundingRect(contours[iContour]);
		double ellipseArea = PI * (rect.width / 2) * (rect.height / 2);
	
		// Ratios
		double boundWidthPerHeight = static_cast<double>(rect.width) / rect.height;
		double areaPerEllipse = static_cast<double>(area) / ellipseArea;
		double rectPerFrame = static_cast<double>(rect.area()) / (binImg.size().width * binImg.size().height);

		// Check constraints			
		if (rectPerFrame > MIN_SIGN_SIZE_PER_FRAME
			&& (1 - LIMIT_DIF_SIGN_SIZE < boundWidthPerHeight && boundWidthPerHeight < 1 + LIMIT_DIF_SIGN_SIZE))
		{
			if (1 - LIMIT_DIF_SIGN_AREA < areaPerEllipse && areaPerEllipse < 1 + LIMIT_DIF_SIGN_AREA)
			{
				m_rects.push_back(rect);
			}
		}
	}

	sort(m_rects.begin(), m_rects.end(), [](const Rect &r1, const Rect &r2) { return r1.area() > r2.area(); });

	return m_rects.size() > 0;
}

ESignType SignDetector::Recognize(const Mat &grayImg)
{
	// Reset
	m_type = ESignType::NONE;
	
	for (size_t iRect = 0; iRect < m_rects.size(); iRect++)
	{
		Rect rect = m_rects[iRect];

		// Crop
		Mat graySign;
		resize(grayImg(rect), graySign, SIGN_SIZE);

		// Compute HOG descriptor
		vector<float> descriptors;
		m_hog.compute(graySign, descriptors);

		Mat fm(descriptors, CV_32F);
		// predict matrix transposition
		int classID = static_cast<int>(m_svm->predict(fm.t()));

		if (classID == 1)
			m_type = ESignType::TURN_LEFT;
		if (classID == 2)
			m_type = ESignType::TURN_RIGHT;

		if (m_type != ESignType::NONE)
		{
			m_rect = rect;
			break;
		}
	}

	return m_type;
}

ESignType SignDetector::Update(const Mat& hsvImg, const Mat& grayImg)
{
	Mat binImg(hsvImg.size(), CV_8UC1, Scalar(0));
	ip->Binarialize(hsvImg, EColor::BLUE, binImg);

	imshow("Sign Binary", binImg);

	if (this->Detect(binImg))
	  	;//return this->Recognize(grayImg);
	//else
		return ESignType::NONE;
}