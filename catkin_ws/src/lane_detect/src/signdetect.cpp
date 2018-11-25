#include "signdetect.h"
#include "imageprocessing.h"

Mat debugImg;

SignDetector::SignDetector(const char *modelPath)
{
	if (USE_ML)
	{
		m_hog = HOGDescriptor(SIGN_SIZE,
			Size(SIGN_SIZE.width / 2, SIGN_SIZE.height / 2),
			Size(SIGN_SIZE.width / 4, SIGN_SIZE.height / 4),
			Size(SIGN_SIZE.width / 4, SIGN_SIZE.height / 4),
			9);

		m_svm = SVM::load(modelPath);

		// m_svm = cv::ml::SVM::create();
		// m_svm->setType(ml::SVM::Types::C_SVC);
		// m_svm->setKernel(ml::SVM::KernelTypes::RBF);
		// m_svm->setC(12.5);
		// m_svm->setGamma(0.50625);
		//m_svm = Algorithm::load<SVM>(modelPath);
		//m_svm->load(modelPath);
	}
	
}

bool SignDetector::Detect(const Mat &binImg)
{
	// Reset
	m_contours.clear();
	m_detectedContours.clear();
	m_rects.clear();

	// find contours
	vector<Vec4i> hierarchy;
	findContours(binImg, m_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	if (m_contours.size())
	{
		// sort contours by decreasing of bounding box area
		sort(m_contours.begin(), m_contours.end(), [](const vector<Point> &c1, const vector<Point> &c2) 
		{
			Rect r1 = boundingRect(c1);
			Rect r2 = boundingRect(c2);
			
			return r1.area() > r2.area();
		});
	}
	
	// init detected vector
	m_detectedContours.resize(m_contours.size());
	fill(m_detectedContours.begin(), m_detectedContours.end(), false);

	// rect vector
	m_rects.resize(m_contours.size());

	// filter contours
	bool detected = false;
	for (size_t iContour = 0; iContour < m_contours.size(); iContour++)
	{
		// areas
		Rect rect = boundingRect(m_contours[iContour]);
		double ellipseArea = PI * (rect.width / 2) * (rect.height / 2);
		double area = contourArea(m_contours[iContour]);
	
		m_rects[iContour] = rect;

		// ratios
		double boundWidthPerHeight = static_cast<double>(rect.width) / rect.height;
		double areaPerEllipse = static_cast<double>(area) / ellipseArea;
		double rectPerFrame = static_cast<double>(rect.area()) / (binImg.size().width * binImg.size().height);

		// check constraints			
		if (rectPerFrame > MIN_SIGN_SIZE_PER_FRAME)
			if (1 - LIMIT_DIF_SIGN_SIZE < boundWidthPerHeight && boundWidthPerHeight < 1 + LIMIT_DIF_SIGN_SIZE)
				if (1 - LIMIT_DIF_SIGN_AREA < areaPerEllipse && areaPerEllipse < 1 + LIMIT_DIF_SIGN_AREA)
				{
					m_detectedContours[iContour] = true;					
					detected = true;
				}
	}

	// if (detected)
	// {
	// 	cout << "Detected " << count(m_detectedContours.begin(), m_detectedContours.end(), true) << endl;
	// }
	return detected;
}

ESignType SignDetector::IpRecognize(const Mat &binImg)
{
	cout << "IP" << endl;

	// reset
	m_type = ESignType::NONE;
	m_rect = RECT_ZERO;

	Mat mask(binImg.size(), CV_8UC1, Scalar(255));

	// browse each detected contours
	for (size_t iContour = 0; iContour < m_contours.size(); iContour++)
		if (m_detectedContours[iContour])
		{
			// fill mask with zero value
			mask.setTo(Scalar(255));
			drawContours(mask, m_contours, iContour, Scalar(0), CV_FILLED);

			bitwise_or(binImg, mask, mask);
			bitwise_not(mask, mask);

			Mat crop = mask(m_rects[iContour]);
			int width = crop.size().width;
			int height = crop.size().height;
			
			int countTopLeft = countNonZero(crop(Rect(0, 0, width / 2, height / 2)));
			int countTopRight = countNonZero(crop(Rect(width / 2, 0, width / 2, height / 2)));
			int countBottomLeft = countNonZero(crop(Rect(0, height / 2, width / 2, height / 2)));
			int countBottomRight = countNonZero(crop(Rect(width / 2, height / 2, width / 2, height / 2)));

			int minCount = min({countTopLeft, countTopRight, countBottomLeft, countBottomRight});
			int count = countTopLeft + countTopRight + countBottomLeft + countBottomRight;

			if (countBottomLeft < countTopLeft 
			&& countBottomLeft < countTopRight
			&& countBottomLeft < countBottomRight) 
			{
				//cout << "Turn left" << endl;
				//cout << countTopLeft << ' ' << countTopRight << ' ' << countBottomLeft << ' ' << countBottomRight << endl;
				//imshow("crop", crop);
				m_rect = m_rects[iContour];
				m_type = ESignType::TURN_LEFT;
				break;
			}

			if (countBottomRight < countTopLeft
			&& countBottomRight < countTopRight
			&& countBottomRight < countBottomLeft)
			{
				//cout << "Turn right" << endl;
				//cout << countTopLeft << ' ' << countTopRight << ' ' << countBottomLeft << ' ' << countBottomRight << endl;
				//imshow("crop", crop);
				m_rect = m_rects[iContour];
				m_type = ESignType::TURN_RIGHT;
				break;
			}
		}

	return m_type;
}

ESignType SignDetector::MlRecognize(const Mat &grayImg)
{
	cout << "ML" << endl;
	
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

ESignType SignDetector::Recognize(const Mat &binImg, const Mat &grayImg)
{
	return (USE_ML ? MlRecognize(grayImg) : IpRecognize(binImg));
}

ESignType SignDetector::Update(const Mat& colorImg)
{
	Mat hsvImg, grayImg;
	cvtColor(colorImg, hsvImg, CV_BGR2HSV);
    cvtColor(colorImg, grayImg, CV_BGR2GRAY);
        
	Mat binImg(hsvImg.size(), CV_8UC1, Scalar(0));
	ip->Binarialize(hsvImg, EColor::BLUE, binImg);

	imshow("Sign Binary", binImg);

	if (this->Detect(binImg))
		return this->Recognize(binImg, grayImg);
	else
		return ESignType::NONE;
}