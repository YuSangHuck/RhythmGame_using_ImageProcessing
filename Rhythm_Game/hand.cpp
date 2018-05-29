#include "includes.h"

//#define KALMAN_EXAMPLE
//#define DRAWING
#define CONVEXITY
//#define HAND_DETECT_VER1 // using convexHull & convexityDefect
#define HAND_DETECT_VER2 // using centerPonintOfHand & convexHull through line and circle
//#define HAND_DETECT_VER3 // using center & circle through and(binary) operation

using namespace cv;
using namespace std;

class Video {
private:
	VideoCapture camera;
	Mat origin, hand, labels, backUp, imgROI, drawROI;
	Size sz;
	float fps;
	float curTime; int kk = 1;
public:
	Video() {
		sz.width = 1280;
		sz.height = 720;
		camera = VideoCapture(0);
		if (!camera.isOpened())
			printf("Camera not connected!");
	}

	void readCam() {
		char ch = 0;
		char init = 0;
		//origin = imread("C:\\1.jpg", IMREAD_COLOR);

		while (camera.read(origin))
		{
			if (ch == 27) close();
			curTime = clock();
			if (init == 0) {
				backUp = origin.clone();
				///				resize(backUp, backUp, sz);
				hand = origin.clone();
				init++; continue;
			}
			handDetect();
			imshow("Hough", origin);
			ch = waitKey(1);
			if (ch == 27) close();
		}
	}

	//// #handDetect Ver3
#ifdef HAND_DETECT_VER3
	void handDetect() { //0이 왼쪽, 1이 오른쪽
		int scale = 2;
		vector<vector<Point>> contours;
		Mat hierarchy, bf;

		//      absdiff(origin, backUp, hand); //차영상
		//      imshow("hand", hand);
		//      backUp = origin.clone();
		resize(origin, origin, sz);
		flip(origin, origin, 1); // y축 기준으로 뒤집기
		cvtColor(origin, hand, COLOR_RGB2YCrCb);
		inRange(hand, Scalar(0, 77, 130), Scalar(255, 133, 180), hand);
		Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, hand.rows / 50));
		erode(hand, hand, verticalStructure);
	    dilate(hand, hand, verticalStructure);
		erode(hand, hand, verticalStructure);
	    dilate(hand, hand, verticalStructure);
		
		Rect ROI(250, 120, 550, 500);
		imgROI = Mat(hand, ROI);
		drawROI = Mat(origin, ROI);
		Mat center = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1);
		Mat dist_change = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8U);
		findContours(imgROI, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE, Point(0, 0));
		int bigsize = 0;
		int index = 0;
		rectangle(origin, ROI, Scalar(255, 255, 0), 3); //draw ROI

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
				Mat cImg(Size(imgROI.rows, imgROI.cols), CV_8UC1, Scalar(0));
#ifdef DRAWING
				drawContours(drawROI, contours, index, Scalar(0, 0, 255), 3); //skin color contours
#endif

				vector<vector<Point>> hull(contours.size());
				for (int i = 0; i < contours.size(); i++)
					convexHull(contours[index], hull[i], false); //false면 index, true면 point
				drawContours(center, contours, index, Scalar(255), -1);
				int j = 0;
				distanceTransform(center, dist_change, CV_DIST_L2, 5);
				imshow("center", center);
				int maxIdx[2];    //좌표 값을 얻어올 배열(행, 열 순으로 저장됨)
				double radius = 0;
				minMaxIdx(dist_change, NULL, &radius, NULL, maxIdx, center);   //최소값은 사용 X
				Point centerOfHand = Point(maxIdx[1], maxIdx[0]);
				if (centerOfHand.x > 0 && centerOfHand.y > 0 && centerOfHand.x < 800 && centerOfHand.y < 720) {
					circle(cImg, centerOfHand, radius*scale, Scalar(255));
					imshow("test", cImg);
					cImg = cImg & center;
					imshow("cImg", cImg);

					contours.clear();
					findContours(cImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
					center = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1); //초기화
					//               drawContours(center, hull, index, Scalar(255), -1);

					//손가락하나하나 line그리기
					//for (int i = 0; i < hull[index].size(); i++)
					//   line(center, centerOfHand, hull[index][i], Scalar(255), 20);

					//int fingerCount = 0;
					//if (contours.size() > 0) {
					//   for (int i = 1; i < contours[0].size(); i++)
					//   {
					//      Point p1 = contours[0][i - 1];
					//      Point p2 = contours[0][i];
					//      uchar* contour1 = center.ptr<uchar>(p1.y);
					//      uchar* contour2 = center.ptr<uchar>(p2.y);
					//      if (contour1[p1.x] == 0 && contour2[p2.x] > 1) fingerCount++;
					//   }
					//}
					//               circle(center, centerOfHand, radius*scale, Scalar(255));
					//               imshow("convex hull", center);
					//               printf("%dside finger count : %d\n", flag,fingerCount);
				}

#ifdef DRAWING
				drawContours(drawROI, hull, index, Scalar(0, 255, 255), 3); //convex hull
				circle(drawROI, centerOfHand, radius, Scalar(0, 0, 255), 3);
				circle(drawROI, centerOfHand, radius * scale, Scalar(255, 0, 255), 3);
				circle(drawROI, centerOfHand, 10, Scalar(255, 255, 0), 3);
#endif
			}
		}
	}
#endif

	// #handDetect Ver2
#ifdef HAND_DETECT_VER2
	void handDetect() {
		int scale = 2;
		vector<vector<Point>> contours;
		Mat hierarchy, bf;
		//      absdiff(origin, backUp, hand); //차영상
		//      imshow("hand", hand);
		//      backUp = origin.clone();
		resize(origin, origin, sz);
		flip(origin, origin, 1); // y축 기준으로 뒤집기
		cvtColor(origin, hand, COLOR_RGB2YCrCb);
		inRange(hand, Scalar(0, 77, 130), Scalar(255, 133, 180), hand);
		Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, hand.rows / 50));
		erode(hand, hand, verticalStructure);
		erode(hand, hand, verticalStructure);
		//      dilate(hand, hand, verticalStructure);
		//      dilate(hand, hand, verticalStructure);

		Rect ROI(250, 120, 550, 500);
		imgROI = Mat(hand, ROI);
		drawROI = Mat(origin, ROI);
		Mat center = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1);
		Mat dist_change = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1);
		findContours(imgROI, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE, Point(0, 0));
		int bigsize = 0;
		int index = 0;
		rectangle(origin, ROI, Scalar(255, 255, 0), 3); //draw ROI

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
				Mat cImg(imgROI.size(), CV_8U, Scalar(0));
#ifdef DRAWING
				drawContours(drawROI, contours, index, Scalar(0, 0, 255), 3); //skin color contours
#endif
				vector<vector<Point>> hull(contours.size());
				for (int i = 0; i < contours.size(); i++)
					//{
					convexHull(contours[index], hull[i], false); //false면 index, true면 point
				drawContours(center, contours, index, Scalar(255), -1);
				int j = 0;
				distanceTransform(center, dist_change, CV_DIST_L2, 5);
				//거리 변환 행렬에서 값(거리)이 가장 큰 픽셀의 좌표와, 값을 얻어온다.

				int maxIdx[2];    //좌표 값을 얻어올 배열(행, 열 순으로 저장됨)
				double radius = 0;
				minMaxIdx(dist_change, NULL, &radius, NULL, maxIdx, center);   //최소값은 사용 X
				Point centerOfHand = Point(maxIdx[1], maxIdx[0]);
				if (centerOfHand.x > 0 && centerOfHand.y > 0 && centerOfHand.x < 800 && centerOfHand.y < 620) {
					circle(cImg, centerOfHand, radius*scale, Scalar(255));
					imshow("cImg", cImg);
					contours.clear();
					findContours(cImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
					center = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1); //초기화
					//               drawContours(center, hull, index, Scalar(255), -1);
					//손가락하나하나 line그리기
					for (int i = 0; i < hull[index].size(); i++)
						line(center, centerOfHand, hull[index][i], Scalar(255), 5);
					int fingerCount = 0;
					if (contours.size() > 0) {
						for (int i = 1; i < contours[0].size(); i++)
						{
							Point p1 = contours[0][i - 1];
							Point p2 = contours[0][i];
							uchar* contour1 = center.ptr<uchar>(p1.y);
							uchar* contour2 = center.ptr<uchar>(p2.y);
							if (contour1[p1.x] == 0 && contour2[p2.x] > 1) fingerCount++;
						}
					}
					circle(center, centerOfHand, radius*scale, Scalar(255));
					imshow("convex hull", center);
					printf("finger count : %d\n", fingerCount);
				}

#ifdef DRAWING
				drawContours(drawROI, hull, index, Scalar(0, 255, 255), 3); //convex hull
#ifdef CONVEXITY
				for(int i = 0; i < hull[index].size(); i++)
					circle(drawROI, hull[index][i], 3, Scalar(0, 255, 0), 3);
#endif
				circle(drawROI, centerOfHand, radius, Scalar(0, 0, 255), 3);
				circle(drawROI, centerOfHand, radius * scale, Scalar(255, 0, 255), 3);
				circle(drawROI, centerOfHand, 10, Scalar(255, 255, 0), 3);
#endif
			}
		}
	}
#endif

	//// #handDetect Ver1
#ifdef HAND_DETECT_VER1
	void handDetect() {
		vector<vector<Point>> contours;
		Mat hierarchy, bf;
		Rect ROI(250, 120, 550, 500);
		//		absdiff(origin, backUp, hand); //차영상
		//		imshow("hand", hand);
		//		backUp = origin.clone();
		resize(origin, origin, sz);
		flip(origin, origin, 1); // y축 기준으로 뒤집기
		cvtColor(origin, hand, COLOR_RGB2YCrCb);
		inRange(hand, Scalar(0, 77, 130), Scalar(255, 133, 180), hand);
		Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, hand.rows / 50));
		erode(hand, hand, verticalStructure);
		erode(hand, hand, verticalStructure);
		//		dilate(hand, hand, verticalStructure);
		//		dilate(hand, hand, verticalStructure);

		imgROI = Mat(hand, ROI);
		drawROI = Mat(origin, ROI);
		Mat center = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1);
		Mat dist_change = Mat::zeros(Size(imgROI.rows, imgROI.cols), CV_8UC1);
		findContours(imgROI, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE, Point(0, 0));
		int bigsize = 0;
		int index = 0;
		rectangle(origin, ROI, Scalar(255, 255, 0), 3); //draw ROI

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
				vector<vector<Vec4i>> convexity(contours.size());
				vector<vector<int>> hullint(contours.size());
				vector<vector<Point>> hull(contours.size());
				for (int i = 0; i < contours.size(); i++) {
					convexHull(contours[index], hull[i], false); //false면 index, true면 point
					convexHull(contours[i], hullint[i], true);
					if (hullint[i].size() > 3)
						convexityDefects(contours[i], hullint[i], convexity[i]);
				}
				drawContours(center, contours, index, Scalar(255), -1); //skin color contours
				imshow("debug_center", center);
				int j = 0;
				distanceTransform(center, dist_change, CV_DIST_L2, 5);
				//거리 변환 행렬에서 값(거리)이 가장 큰 픽셀의 좌표와, 값을 얻어온다.

				int maxIdx[2];    //좌표 값을 얻어올 배열(행, 열 순으로 저장됨)
				double radius = 0;
				minMaxIdx(dist_change, NULL, &radius, NULL, maxIdx, center);   //최소값은 사용 X
				Point centerOfHand = Point(maxIdx[1], maxIdx[0]);
				imshow("center", dist_change);

				int mindistance = 720;
				for (int i = 0; i < contours.size(); i++)
				{
					for (const Vec4i& v : convexity[i]) {
						float depth = v[3] / 256;
						if (depth > 15) {
							//							int startidx = v[0]; Point ptstart(contours[i][startidx]);
							int endidx = v[1]; Point ptend(contours[i][endidx]);
							//							int faridx = v[2]; Point ptfar(contours[i][faridx]);
#ifdef DRAWING
							circle(drawROI, ptend, 5, Scalar(0, 255, 0), 3);
#endif

							int n = sqrt(pow(ptend.x - centerOfHand.x, 2) + pow(ptend.y - centerOfHand.y, 2));
							if (mindistance > n) mindistance = n;
						}
					}
				}

				// Draw on origin	
#ifdef DRAWING
				drawContours(drawROI, contours, index, Scalar(0, 0, 255), 3); //skin color contours
				drawContours(drawROI, hull, index, Scalar(0, 255, 255), 3); //convex hull

				circle(drawROI, centerOfHand, radius, Scalar(0, 0, 255), 3);
				circle(drawROI, centerOfHand, mindistance, Scalar(255, 0, 255), 3);
#endif

				//Mat sub = hand - andimg; //손가락 구하기
				//imshow("finger", sub);
				//andimg.zeros(hand.rows, hand.cols, hand.type()); //초기화해서 재사용
				//circle(andimg, centerOfHand, mindistance, Scalar(1)); //원
				//imshow("circle", andimg);
				//Mat merge = sub + andimg; //5번째 영상
				//imshow("merge finger and circle", merge);
				////convex hull
				//andimg.zeros(hand.rows, hand.cols, hand.type()); //초기화해서 재사용
				//for (int i = 0; i < hull.size(); i++)
				//	fillConvexPoly(andimg, hull[i], Scalar(255, 255, 0));
				//imshow("convex hull", andimg);

				//and 연산

				//			if (centerOfHand.x > 0 && centerOfHand.y > 0 && centerOfHand.x < 850 && centerOfHand.y < 620)
#ifdef DRAWING
				circle(drawROI, centerOfHand, 10, Scalar(255, 255, 0), 3);
#endif
			}
		}
	}
#endif

	void close() {
		camera.release();
		cv::destroyAllWindows();
	}
};

int main(int argc, _TCHAR* argv[])
{
	Video video;
	int thres = 95, num = 0;
	namedWindow("Hough", CV_WINDOW_AUTOSIZE);

	video.readCam();

	return 0;
}

#ifdef KALMAN_EXAMPLE
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
using namespace cv;
static inline Point calcPoint(Point2f center, double R, double angle)
{
	return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}
static void help()
{
	printf("\nExample of c calls to OpenCV's Kalman filter.\n"
		"   Tracking of rotating point.\n"
		"   Rotation speed is constant.\n"
		"   Both state and measurements vectors are 1D (a point angle),\n"
		"   Measurement is the real point angle + gaussian noise.\n"
		"   The real and the estimated points are connected with yellow line segment,\n"
		"   the real and the measured points are connected with red line segment.\n"
		"   (if Kalman filter works correctly,\n"
		"    the yellow segment should be shorter than the red one).\n"
		"\n"
		"   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
		"   Pressing ESC will stop the program.\n"
		);
}
int main(int, char**)
{
	help();
	Mat img(500, 500, CV_8UC3);
	KalmanFilter KF(2, 1, 0);
	Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
	Mat processNoise(2, 1, CV_32F);
	Mat measurement = Mat::zeros(1, 1, CV_32F);
	char code = (char)-1;
	for (;;)
	{
		randn(state, Scalar::all(0), Scalar::all(0.1));
		KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
		setIdentity(KF.measurementMatrix);
		setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
		setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
		setIdentity(KF.errorCovPost, Scalar::all(1));
		randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
		for (;;)
		{
			Point2f center(img.cols*0.5f, img.rows*0.5f);
			float R = img.cols / 3.f;
			double stateAngle = state.at<float>(0);
			Point statePt = calcPoint(center, R, stateAngle);
			Mat prediction = KF.predict();
			double predictAngle = prediction.at<float>(0);
			Point predictPt = calcPoint(center, R, predictAngle);
			randn(measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
			// generate measurement
			measurement += KF.measurementMatrix*state;
			double measAngle = measurement.at<float>(0);
			Point measPt = calcPoint(center, R, measAngle);
			// plot points
#define drawCross( center, color, d )                                        \
                line( img, Point( center.x - d, center.y - d ),                          \
                             Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
                line( img, Point( center.x + d, center.y - d ),                          \
                             Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )
			img = Scalar::all(0);
			drawCross(statePt, Scalar(255, 255, 255), 3);
			drawCross(measPt, Scalar(0, 0, 255), 3);
			drawCross(predictPt, Scalar(0, 255, 0), 3);
			line(img, statePt, measPt, Scalar(0, 0, 255), 3, LINE_AA, 0);
			line(img, statePt, predictPt, Scalar(0, 255, 255), 3, LINE_AA, 0);
			if (theRNG().uniform(0, 4) != 0)
				KF.correct(measurement);
			randn(processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
			state = KF.transitionMatrix*state + processNoise;
			imshow("Kalman", img);
			code = (char)waitKey(100);
			if (code > 0)
				break;
		}
		if (code == 27 || code == 'q' || code == 'Q')
			break;
	}

	return 0;
}
#endif