#include "stdafx.h"

using namespace cv;
using namespace std;

class Video {
private:
	VideoCapture camera;
	Mat origin, hand, diff, backUp, imgROI, drawROI, state, meas;
	Size sz;
	//float fps, curTime;
	string str;
	int pre_fingercount, index = 0;
	int width, height;
	Rect ROI[2];
	Point centerOfHand, rect_backup;
	bool isClicked, found = false;
	vector<vector<Point>> contours, circle_contour;
	double radius = 0, pre_radius = 0;
	KalmanFilter kf;
public:
	Video() {
		isClicked = false;
		sz.width = 1280;
		sz.height = 720;
		ROI[0] = Rect(700, 120, 550, 500);
		//ROI[1] = Rect(700, 120, 550, 500);
		width = 550, height = 500;
		camera = VideoCapture(0);
		if (!camera.isOpened())
			printf("Camera not connected!");
	}

	void readCam() {
		char ch = 0;
		//char init = 0;
		// >>>> Kalman Filter
		int stateSize = 6;
		int measSize = 4;
		int contrSize = 0;

		unsigned int type = CV_32F;
		kf = KalmanFilter(stateSize, measSize, contrSize, type);

		state = Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
		meas = Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
											//cv::Mat procNoise(stateSize, 1, type)
											// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

											// Transition State Matrix A
											// Note: set dT at each processing step!
											// [ 1 0 dT 0  0 0 ]
											// [ 0 1 0  dT 0 0 ]
											// [ 0 0 1  0  0 0 ]
											// [ 0 0 0  1  0 0 ]
											// [ 0 0 0  0  1 0 ]
											// [ 0 0 0  0  0 1 ]
		setIdentity(kf.transitionMatrix);

		// Measure Matrix H
		// [ 1 0 0 0 0 0 ]
		// [ 0 1 0 0 0 0 ]
		// [ 0 0 0 0 1 0 ]
		// [ 0 0 0 0 0 1 ]
		kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
		kf.measurementMatrix.at<float>(0) = 1.0f;
		kf.measurementMatrix.at<float>(7) = 1.0f;
		kf.measurementMatrix.at<float>(16) = 1.0f;
		kf.measurementMatrix.at<float>(23) = 1.0f;

		// Process Noise Covariance Matrix Q
		// [ Ex   0   0     0     0    0  ]
		// [ 0    Ey  0     0     0    0  ]
		// [ 0    0   Ev_x  0     0    0  ]
		// [ 0    0   0     Ev_y  0    0  ]
		// [ 0    0   0     0     Ew   0  ]
		// [ 0    0   0     0     0    Eh ]
		//setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
		kf.processNoiseCov.at<float>(0) = 1e-2;
		kf.processNoiseCov.at<float>(7) = 1e-2;
		kf.processNoiseCov.at<float>(14) = 5.0f;
		kf.processNoiseCov.at<float>(21) = 5.0f;
		kf.processNoiseCov.at<float>(28) = 1e-2;
		kf.processNoiseCov.at<float>(35) = 1e-2;

		// Measures Noise Covariance Matrix R
		setIdentity(kf.measurementNoiseCov, Scalar(1e-1));
		// <<<< Kalman Filter
		width = 550, height = 500;
		rect_backup = Point(700, 120);
		while (camera.read(origin))
		{
			if (!isClicked) {
				
				handDetect();
				if (contours.size() > 0)
				{
					drawContours(drawROI, contours, index, Scalar(0, 0, 255), 3); //skin color contours
					circle(drawROI, centerOfHand, radius, Scalar(0, 0, 255), 3); //손바닥 빨간색
					circle(drawROI, centerOfHand, radius * 2, Scalar(255, 0, 255), 3); //손가락 기준되는 원 보라색
					circle(drawROI, centerOfHand, 10, Scalar(255, 255, 0), 3); //중심점 파란색
				}
			}
			else handTracking();
			pre_radius = radius;
			imshow("Hough", origin);
			ch = waitKey(1);
			if (ch == 27) close();
		}
	}

	void handDetect() { //0 :ROI, 1: 전체영상
		int scale = 2;
		Mat hierarchy;
		resize(origin, origin, sz);
		flip(origin, origin, 1); // y축 기준으로 뒤집기
		cvtColor(origin, hand, COLOR_RGB2YCrCb);
		inRange(hand, Scalar(30, 75, 130), Scalar(255, 133, 180), hand);

		Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, hand.rows / 50));
		erode(hand, hand, verticalStructure);
		erode(hand, hand, verticalStructure);
		dilate(hand, hand, verticalStructure);
		dilate(hand, hand, verticalStructure);

		Mat center, dist_change;
		//if (!flag) { //0
			imgROI = Mat(hand, ROI[0]);
			drawROI = Mat(origin, ROI[0]);
			center = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1);
			dist_change = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1);
			findContours(imgROI, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE, Point(0, 0));
			rectangle(origin, ROI[0], Scalar(255, 255, 0), 3); //draw ROI
		//}
		//else {
		//	center = Mat::zeros(Size(origin.rows, origin.cols), CV_8UC1);
		//	dist_change = Mat::zeros(Size(origin.rows, origin.cols), CV_8UC1);
		//	findContours(hand, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE, Point(0, 0));
		//}

		int bigsize = 0;
		//calculate biggest contour
		if (contours.size() > 0) {
			for (int i = 0; i < contours.size(); i++)
			{
				double a = contourArea(contours[i], false);
				if (a > bigsize) {
					bigsize = a;
					index = i;
				}
			}
			if (index >= 0) {
				int j = 0;
				Mat cImg;
				//if (!flag) 
					cImg = Mat(Size(imgROI.rows, imgROI.cols), CV_8UC1, Scalar(0));
				//else 
				//	cImg = Mat(Size(hand.rows, hand.cols), CV_8UC1, Scalar(0));
				drawContours(center, contours, index, Scalar(255), -1);
				distanceTransform(center, dist_change, CV_DIST_L2, 5);
				int maxIdx[2];    //좌표 값을 얻어올 배열(행, 열 순으로 저장됨)
				minMaxIdx(dist_change, NULL, &radius, NULL, maxIdx, center);   //최소값은 사용 X
				centerOfHand = Point(maxIdx[1], maxIdx[0]);

				if (centerOfHand.x > 0 && centerOfHand.y > 0 && centerOfHand.x < 800 && centerOfHand.y < 720) {
					circle(cImg, centerOfHand, radius*scale, Scalar(255), 3);
					cImg = cImg & center;
				//	imshow("cImg", cImg);

					findContours(cImg, circle_contour, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

					int fingercount = 0, thre = 15;
					for (int i = 0; i < circle_contour.size(); i++)
						if (circle_contour[i].size() > thre) fingercount++;
					//if (pre_fingercount - fingercount >= 4 || fingercount==1) putText(origin, "click event!!", Point(10, 45), 0, 1, Scalar(0, 0, 255),2);
					if (pre_fingercount - fingercount >= 4) {
						putText(origin, "click event!!", Point(10, 45), 0, 1, Scalar(0, 0, 255), 2);
						isClicked = true;
					}
					str = "finger count : ";
					str += to_string(fingercount - 1);
					putText(origin, str, Point(10, 25), 0, 1, Scalar(0, 0, 255), 2);
					pre_fingercount = fingercount;

				}
				//if (!flag) {
					circle(drawROI, centerOfHand, radius, Scalar(0, 0, 255), 3); //손바닥 빨간색
					circle(drawROI, centerOfHand, radius * scale, Scalar(255, 0, 255), 3); //손가락 기준되는 원 보라색
					circle(drawROI, centerOfHand, 10, Scalar(255, 255, 0), 3); //중심점 파란색
				//}
				//else {
				//	circle(origin, centerOfHand, radius, Scalar(0, 0, 255), 3); //손바닥 빨간색
				//	circle(origin, centerOfHand, radius * scale, Scalar(255, 0, 255), 3); //손가락 기준되는 원 보라색
				//	circle(origin, centerOfHand, 10, Scalar(255, 255, 0), 3); //중심점 파란색
				//}
			}
		}
	}

	void handTracking() {
		double ticks = 0;
		int notFoundCount = 0;
		double precTick = ticks;
		ticks = (double)cv::getTickCount();
		double dT = (ticks - precTick) / getTickFrequency(); //seconds

		Mat res;

		if (rect_backup.x <0) rect_backup.x = 0;
		else if (rect_backup.x >1280) rect_backup.x = 700;
		if (rect_backup.y <0) rect_backup.y = 0;
		else if (rect_backup.y > 720) rect_backup.y = 120;
		if (rect_backup.x + width >1280) width = 1280 - rect_backup.x;
		if (rect_backup.y + height>720) height = 720 - rect_backup.y;
		ROI[0] = Rect(rect_backup.x, rect_backup.y, width, height);
		//ROI[0].x = rect_backup.x;
		//ROI[0].y = rect_backup.y;
		printf("rect_backup.x : %d, rect_backup.y :%d, width :%d, height :%d\n", rect_backup.x, rect_backup.y, width, height);
		handDetect();
		origin.copyTo(res);

		if (found)
		{
			// >>>> Matrix A
			kf.transitionMatrix.at<float>(2) = dT;
			kf.transitionMatrix.at<float>(9) = dT;
			// <<<< Matrix A

			cout << "dT:" << endl << dT << endl;

			state = kf.predict();
			cout << "State post:" << endl << state << endl;

			Rect predRect;
			predRect.width = state.at<float>(4);
			predRect.height = state.at<float>(5);
			predRect.x = state.at<float>(0);
			predRect.y = state.at<float>(1);

			Point center;
			center.x = state.at<float>(4);
			center.y = state.at<float>(5);
			circle(res, center, 2, CV_RGB(255, 0, 0), -1);
			rectangle(res, predRect, CV_RGB(255, 0, 0), 2);
			imshow("res", res);
		}

		// Thresholding viewing
//		cv::imshow("Threshold", hand);
		vector<vector<Point>> hands;
		vector<Rect> handsBox;
		if (contours.size() > 0) {
			// >>>>> Filtering

			//			for (size_t i = 0; i < contours.size(); i++)
			//			{
			Rect hBox;
			hBox = boundingRect(contours[index]);

			rect_backup = Point(state.at<float>(4), state.at<float>(5));
//			rect_backup = Point(centerOfPoint + radius);
			float ratio = (float)hBox.width / (float)hBox.height;
			if (ratio > 1.0f)
				ratio = 1.0f / ratio;

			// Searching for a bBox almost square
//			if (ratio > 0.75 && hBox.area() >= 400)
//			{
			hands.push_back(contours[index]);
			handsBox.push_back(hBox);
			width = hBox.width;
			height = hBox.height;
			//			}
						//			}
			// <<<<< Filtering

			cout << "Hands found:" << handsBox.size() << endl;

			// >>>>> Detection result
			//for (size_t i = 0; i < hands.size(); i++)
			//{

			drawContours(drawROI, contours, index, CV_RGB(0, 0, 255), 2);
			rectangle(drawROI, handsBox[0], CV_RGB(0, 255, 0), 2);

			/*Point center;
			center.x = handsBox[0].x + handsBox[0].width / 2;
			center.y = handsBox[0].y + handsBox[0].height / 2;*/
			circle(drawROI, centerOfHand, 2, CV_RGB(20, 150, 20), -1);

			stringstream sstr;
			sstr << "(" << centerOfHand.x << "," << centerOfHand.y << ")";
			putText(drawROI, sstr.str(), Point(centerOfHand.x + 3, centerOfHand.y - 3), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20, 150, 20), 2);
		}
		//			}
		// <<<<< Detection result

		// >>>>> Kalman Update
		if (hands.size() == 0)
		{
			notFoundCount++;
			cout << "notFoundCount:" << notFoundCount << endl;
			if (notFoundCount >= 100)
			{
				found = false;
			}
			rect_backup.x = 700, rect_backup.y = 120;
			isClicked = false;
		}
		else
		{
			notFoundCount = 0;

			meas.at<float>(0) = centerOfHand.x;
			meas.at<float>(1) = centerOfHand.y;
			meas.at<float>(2) = (float)handsBox[0].width;
			meas.at<float>(3) = (float)handsBox[0].height;

			if (!found) // First detection!
			{
				// >>>> Initialization
				kf.errorCovPre.at<float>(0) = 1; // px
				kf.errorCovPre.at<float>(7) = 1; // px
				kf.errorCovPre.at<float>(14) = 1;
				kf.errorCovPre.at<float>(21) = 1;
				kf.errorCovPre.at<float>(28) = 1; // px
				kf.errorCovPre.at<float>(35) = 1; // px

				state.at<float>(0) = meas.at<float>(0);
				state.at<float>(1) = meas.at<float>(1);
				state.at<float>(2) = 0;
				state.at<float>(3) = 0;
				state.at<float>(4) = meas.at<float>(2);
				state.at<float>(5) = meas.at<float>(3);
				// <<<< Initialization

				kf.statePost = state;

				found = true;
			}
			else
				kf.correct(meas); // Kalman Correction
								  //				cout << "Measure matrix:" << endl << meas << endl;
		}
		// <<<<< Kalman Update

	}

	void close() {
		camera.release();
		cv::destroyAllWindows();
	}
};

int main(int argc, _TCHAR* argv[])
{
	Video video;
	namedWindow("Hough", CV_WINDOW_AUTOSIZE);
	video.readCam();

	return 0;
}