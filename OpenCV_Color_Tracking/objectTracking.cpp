#include <sstream>
#include<iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <deque>

#include "SerialPort.h"
#include "Windows.h"

using namespace std;
using namespace cv;

char buffer[16];

int XdeltaY;
int XdeltaX;

int YdeltaY;
int YdeltaX;

Mat intrinsicX = Mat::eye(3, 3, CV_64F);
Mat distCoeffsX;

Mat intrinsicY = Mat::eye(3, 3, CV_64F);
Mat distCoeffsY;

cv::Mat map1, map2;

int circleSize = 10;

Point point1, point2;
int drag = 0;
cv::Rect rectX;
cv::Rect rectY;
cv::Mat imgX;
cv::Mat imgY;
cv::Mat roiImg;
int select_flagX = 0;
int select_flagY = 0;

VideoCapture captureX;
VideoCapture captureY;

cv::Point ballPositionX(0, 0);
cv::Point deltaPointX(0, 0);

cv::Point ballPositionY(0, 0);
cv::Point deltaPointY(0, 0);

//ranges of trimmed rectangle 
Range xRangeX(INT_MIN, INT_MAX);
Range yRangeX(INT_MIN, INT_MAX);

Range xRangeY(INT_MIN, INT_MAX);
Range yRangeY(INT_MIN, INT_MAX);

//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 40;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;
//we'll have just one object to search for

//bounding rectangle of the object, we will use the center of this as its position.
Rect objectBoundingRectangleY = Rect(0, 0, 0, 0);
Rect objectBoundingRectangleX = Rect(0, 0, 0, 0);

char commport[] = "\\\\.\\COM4";

//int to string helper function
string intToString(int number) {

	//this function has a nuber input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void searchForMovementX(Mat& thresholdImage) {
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetectedX = false;
	Mat tempX;
	thresholdImage.copyTo(tempX);
	//these two vectors needed for output of findContours
	vector<vector<Point> > contoursX;
	vector<Vec4i> hierarchyX;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE );// retrieves a ll contours
	findContours(tempX, contoursX, hierarchyX, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if (contoursX.size() > 0)objectDetectedX = true;
	else objectDetectedX = false;

	if (objectDetectedX) {
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVecX;
		largestContourVecX.push_back(contoursX.at(contoursX.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangleX = boundingRect(largestContourVecX.at(0));
	}
	ballPositionX.x = objectBoundingRectangleX.x + objectBoundingRectangleX.width / 2;
	ballPositionX.y = objectBoundingRectangleX.y + objectBoundingRectangleX.height / 2;
}

void searchForMovementY(Mat& thresholdImage) {
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetectedY = false;
	Mat tempY;
	thresholdImage.copyTo(tempY);
	//these two vectors needed for output of findContours
	vector<vector<Point> > contoursY;
	vector<Vec4i> hierarchyY;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE );// retrieves a ll contours
	findContours(tempY, contoursY, hierarchyY, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if (contoursY.size() > 0)objectDetectedY = true;
	else objectDetectedY = false;

	if (objectDetectedY) {
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVecY;
		largestContourVecY.push_back(contoursY.at(contoursY.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangleY = boundingRect(largestContourVecY.at(0));
	}
	ballPositionY.x = objectBoundingRectangleY.x + objectBoundingRectangleY.width / 2;
	ballPositionY.y = objectBoundingRectangleY.y + objectBoundingRectangleY.height / 2;
}


//Weighted avgerage for predicitons 
cv::Point predictNextPositionX(std::vector<cv::Point>& positions) {
	cv::Point predictedPosition; // this will be the return value
	int numPositions;

	numPositions = positions.size();

	if (numPositions == 0) {

		std::cout << "error, predictNextPosition was called with zero points\n";

	}
	else if (numPositions == 1) {

		return(positions[0]);

	}
	else if (numPositions == 2) {

		XdeltaX = positions[1].x - positions[0].x;
		XdeltaY = positions[1].y - positions[0].y;

		predictedPosition.x = positions.back().x + XdeltaX;
		predictedPosition.y = positions.back().y + XdeltaY;

	}
	else if (numPositions == 3) {

		int sumOfXChanges = ((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		XdeltaX = (int)std::round((float)sumOfXChanges / 3.0);

		int sumOfYChanges = ((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		XdeltaY = (int)std::round((float)sumOfYChanges / 3.0);

		predictedPosition.x = positions.back().x + XdeltaX;
		predictedPosition.y = positions.back().y + XdeltaY;

	}
	else if (numPositions == 4) {

		int sumOfXChanges = ((positions[3].x - positions[2].x) * 3) +
			((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		XdeltaX = (int)std::round((float)sumOfXChanges / 6.0);

		int sumOfYChanges = ((positions[3].y - positions[2].y) * 3) +
			((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		XdeltaY = (int)std::round((float)sumOfYChanges / 6.0);

		predictedPosition.x = positions.back().x + XdeltaX;
		predictedPosition.y = positions.back().y + XdeltaY;

	}
	else if (numPositions >= 5) {

		int sumOfXChanges = ((positions[numPositions - 1].x - positions[numPositions - 2].x) * 4) +
			((positions[numPositions - 2].x - positions[numPositions - 3].x) * 3) +
			((positions[numPositions - 3].x - positions[numPositions - 4].x) * 2) +
			((positions[numPositions - 4].x - positions[numPositions - 5].x) * 1);

		XdeltaX = (int)std::round((float)sumOfXChanges / 10.0);

		int sumOfYChanges = ((positions[numPositions - 1].y - positions[numPositions - 2].y) * 4) +
			((positions[numPositions - 2].y - positions[numPositions - 3].y) * 3) +
			((positions[numPositions - 3].y - positions[numPositions - 4].y) * 2) +
			((positions[numPositions - 4].y - positions[numPositions - 5].y) * 1);

		XdeltaY = (int)std::round((float)sumOfYChanges / 10.0);

		predictedPosition.x = positions.back().x + XdeltaX;
		predictedPosition.y = positions.back().y + XdeltaY;


	}
	else {
		// should never get here
	}

	return(predictedPosition);
}

cv::Point predictNextPositionY(std::vector<cv::Point>& positions) {
	cv::Point predictedPosition; // this will be the return value
	int numPositions;

	numPositions = positions.size();

	if (numPositions == 0) {

		std::cout << "error, predictNextPosition was called with zero points\n";

	}
	else if (numPositions == 1) {

		return(positions[0]);

	}
	else if (numPositions == 2) {

		YdeltaX = positions[1].x - positions[0].x;
		YdeltaY = positions[1].y - positions[0].y;

		predictedPosition.x = positions.back().x + YdeltaX;
		predictedPosition.y = positions.back().y + YdeltaY;

	}
	else if (numPositions == 3) {

		int sumOfXChanges = ((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		YdeltaX = (int)std::round((float)sumOfXChanges / 3.0);

		int sumOfYChanges = ((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		YdeltaY = (int)std::round((float)sumOfYChanges / 3.0);

		predictedPosition.x = positions.back().x + YdeltaX;
		predictedPosition.y = positions.back().y + YdeltaY;

	}
	else if (numPositions == 4) {

		int sumOfXChanges = ((positions[3].x - positions[2].x) * 3) +
			((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		YdeltaX = (int)std::round((float)sumOfXChanges / 6.0);

		int sumOfYChanges = ((positions[3].y - positions[2].y) * 3) +
			((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		YdeltaY = (int)std::round((float)sumOfYChanges / 6.0);

		predictedPosition.x = positions.back().x + YdeltaX;
		predictedPosition.y = positions.back().y + YdeltaY;

	}
	else if (numPositions >= 5) {

		int sumOfXChanges = ((positions[numPositions - 1].x - positions[numPositions - 2].x) * 4) +
			((positions[numPositions - 2].x - positions[numPositions - 3].x) * 3) +
			((positions[numPositions - 3].x - positions[numPositions - 4].x) * 2) +
			((positions[numPositions - 4].x - positions[numPositions - 5].x) * 1);

		YdeltaX = (int)std::round((float)sumOfXChanges / 10.0);

		int sumOfYChanges = ((positions[numPositions - 1].y - positions[numPositions - 2].y) * 4) +
			((positions[numPositions - 2].y - positions[numPositions - 3].y) * 3) +
			((positions[numPositions - 3].y - positions[numPositions - 4].y) * 2) +
			((positions[numPositions - 4].y - positions[numPositions - 5].y) * 1);

		YdeltaY = (int)std::round((float)sumOfYChanges / 10.0);

		predictedPosition.x = positions.back().x + YdeltaX;
		predictedPosition.y = positions.back().y + YdeltaY;


	}
	else {
		// should never get here
	}

	return(predictedPosition);
}

//uses 9x6 chessboard to calibrate fisheye lens distortion 
void CalibrateX(VideoCapture capture)
{
	static int numBoards = 5;
	static int numCornersHor = 9;
	static int numCornersVer = 6;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);


	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int successes = 0;

	Mat image;
	Mat gray_image;
	capture >> image;

	vector<Point3f> obj;
	for (int j = 0; j < numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j % numCornersHor, 0.0f));

	while (successes < numBoards)
	{
		cvtColor(image, gray_image, COLOR_BGR2GRAY);

		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (found)
		{
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(image, board_sz, corners, found);
		}
		imshow("CalibrateME!", image);

		capture >> image;
		int key = waitKey(1);

		if (key == 27)

			break;

		if (key == ' ' && found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);

			printf("Snap stored!");

			successes++;

			if (successes >= numBoards)
				continue;
		}
	}

	vector<Mat> rvecs;
	vector<Mat> tvecs;
	intrinsicX.ptr<float>(0)[0] = 1;
	intrinsicX.ptr<float>(1)[1] = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsicX, distCoeffsX, rvecs, tvecs);
	cv::initUndistortRectifyMap(intrinsicX, distCoeffsX, Mat_<double>::eye(3, 3), intrinsicX, cv::Size(720, 480), CV_16SC2, map1, map2);
	cv::destroyWindow("CalibrateME!");
}

void CalibrateY(VideoCapture capture)
{
	static int numBoards = 5;
	static int numCornersHor = 9;
	static int numCornersVer = 6;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);


	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int successes = 0;

	Mat image;
	Mat gray_image;
	capture >> image;

	vector<Point3f> obj;
	for (int j = 0; j < numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j % numCornersHor, 0.0f));

	while (successes < numBoards)
	{
		cvtColor(image, gray_image, COLOR_BGR2GRAY);

		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (found)
		{
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(image, board_sz, corners, found);
		}
		imshow("CalibrateME!", image);

		capture >> image;
		int key = waitKey(1);

		if (key == 27)

			break;

		if (key == ' ' && found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);

			printf("Snap stored!");

			successes++;

			if (successes >= numBoards)
				continue;
		}
	}

	vector<Mat> rvecs;
	vector<Mat> tvecs;
	intrinsicY.ptr<float>(0)[0] = 1;
	intrinsicY.ptr<float>(1)[1] = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsicY, distCoeffsY, rvecs, tvecs);
	cv::initUndistortRectifyMap(intrinsicY, distCoeffsY, Mat_<double>::eye(3, 3), intrinsicY, cv::Size(720, 480), CV_16SC2, map1, map2);
	cv::destroyWindow("CalibrateME!");
}


bool FindPoint(int x1, int y1, int x2, int y2, int x, int y)
{
	if (x > x1&& x < x2 && y > y1&& y < y2) {
		return true;
	}
	return false;
}


void mouseHandlerX(int event, int x, int y, int flags, void* param)
{
	Mat img1X = imgX.clone();

	if (event == cv::EVENT_LBUTTONDOWN && !drag)
	{
		/* left button clicked. ROI selection begins */
		cv::remap(img1X, img1X, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

		select_flagX = 0;
		point1 = Point(x, y);
		drag = 1;
	}

	if (event == cv::EVENT_MOUSEMOVE && drag)
	{
		/* mouse dragged. ROI being selected */
		point2 = Point(x, y);
		rectangle(img1X, point1, point2, (255, 0, 0), 3, 8, 0);
		imshow("imageX", img1X);
	}

	if (event == cv::EVENT_LBUTTONUP && drag)
	{
		point2 = Point(x, y);
		rectX = Rect(point1.x, point1.y, x - point1.x, y - point1.y);
		drag = 0;
		roiImg = imgX(rectX);

		xRangeX.start = point1.x;
		xRangeX.end = point2.x;

		yRangeX.start = point1.y;
		yRangeX.end = point2.y;
	}

	if (event == cv::EVENT_LBUTTONUP)
	{
		/* ROI selected */
		select_flagX = 1;
		drag = 0;
	}

}

void mouseHandlerY(int event, int x, int y, int flags, void* param)
{
	Mat img1Y = imgY.clone();

	if (event == cv::EVENT_LBUTTONDOWN && !drag)
	{
		/* left button clicked. ROI selection begins */
		cv::remap(img1Y, img1Y, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

		select_flagY = 0;
		point1 = Point(x, y);
		drag = 1;
	}

	if (event == cv::EVENT_MOUSEMOVE && drag)
	{
		/* mouse dragged. ROI being selected */
		point2 = Point(x, y);
		rectangle(img1Y, point1, point2, (255, 0, 0), 3, 8, 0);
		imshow("imageY", img1Y);
	}

	if (event == cv::EVENT_LBUTTONUP && drag)
	{
		point2 = Point(x, y);
		rectY = Rect(point1.x, point1.y, x - point1.x, y - point1.y);
		drag = 0;
		roiImg = imgY(rectY);

		xRangeY.start = point1.x;
		xRangeY.end = point2.x;

		yRangeY.start = point1.y;
		yRangeY.end = point2.y;
	}

	if (event == cv::EVENT_LBUTTONUP)
	{
		/* ROI selected */
		select_flagY = 1;
		drag = 0;
	}

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {

	//some boolean variables for added functionality
	bool objectDetected = false;
	//these two can be toggled by pressing 'd' or 't'
	bool debugMode = false;
	bool trackingEnabled = false;
	//pause and resume code
	bool pause = false;
	//set up the matrices that we will need
	//the two frames we will be comparing
	Mat frame1X, frame2X;
	Mat frame1Y, frame2Y;
	//their grayscale images (needed for absdiff() function)
	Mat grayImage1X, grayImage2X;
	Mat grayImage1Y, grayImage2Y;
	//resulting difference image
	Mat differenceImageX;
	Mat differenceImageY;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImageX;
	Mat thresholdImageY;
	//final Mat
	Mat finalX;
	Mat finalY;

	captureX.open(0 + cv::CAP_DSHOW);
	//calls the calibrate and stores the coeffs and intrisics in their respected variables
	CalibrateX(captureX);

	captureY.open(1 + cv::CAP_DSHOW);
	//calls the calibrate and stores the coeffs and intrisics in their respected variables
	CalibrateY(captureY);

	SerialPort arduino(commport);
	if (arduino.isConnected()) {
		cout << "Connection made" << endl << endl;
	}
	else {
		cout << "Error in port name" << endl << endl;
	}

	if (!captureX.isOpened()) {
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return -1;
	}

	if (!captureY.isOpened()) {
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return -1;
	}

	captureX.set(cv::CAP_PROP_SETTINGS, 1);
	captureX.set(cv::CAP_PROP_FPS, 260);

	captureY.set(cv::CAP_PROP_SETTINGS, 1);
	captureY.set(cv::CAP_PROP_FPS, 260);


	std::vector<cv::Point> ballPositionsX;
	cv::Point predictedBallPositionX;
	cv::Point predictedBallPositionFarX;

	std::vector<cv::Point> ballPositionsY;
	cv::Point predictedBallPositionY;
	cv::Point predictedBallPositionFarY;

	captureX >> imgX;
	cv::remap(imgX, imgX, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
	imshow("imageX", imgX);

	captureY >> imgY;
	cv::remap(imgY, imgY, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
	imshow("imageY", imgY);

	while (1) {

		//read first frame
		captureX.read(frame1X);
		captureY.read(frame1Y);

		//copy second frame
		captureX.read(frame2X);
		captureY.read(frame2Y);

		//create trimmed mats
		Mat revelvantFrame1X(frame1X, yRangeX, xRangeX);
		Mat relevantFrame2X(frame2X, yRangeX, xRangeX);

		Mat revelvantFrame1Y(frame1Y, yRangeY, xRangeY);
		Mat relevantFrame2Y(frame2Y, yRangeY, xRangeY);

		//convert trimmed mats to gray scale for frame differencing
		cv::cvtColor(revelvantFrame1X, grayImage1X, COLOR_BGR2GRAY);
		cv::cvtColor(relevantFrame2X, grayImage2X, COLOR_BGR2GRAY);

		cv::cvtColor(revelvantFrame1Y, grayImage1Y, COLOR_BGR2GRAY);
		cv::cvtColor(relevantFrame2Y, grayImage2Y, COLOR_BGR2GRAY);

		//perform frame differencing with the sequential images. This will output an "intensity image"
		//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
		cv::absdiff(grayImage1X, grayImage2X, differenceImageX);
		cv::remap(differenceImageX, differenceImageX, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

		cv::absdiff(grayImage1Y, grayImage2Y, differenceImageY);
		cv::remap(differenceImageY, differenceImageY, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

		//threshold intensity image at a given sensitivity value
		cv::threshold(differenceImageX, thresholdImageX, SENSITIVITY_VALUE, 255, THRESH_BINARY);
		cv::threshold(differenceImageY, thresholdImageY, SENSITIVITY_VALUE, 255, THRESH_BINARY);

		//blur the image to get rid of the noise. This will output an intensity image
		cv::blur(thresholdImageX, thresholdImageX, cv::Size(BLUR_SIZE, BLUR_SIZE));
		cv::blur(thresholdImageY, thresholdImageY, cv::Size(BLUR_SIZE, BLUR_SIZE));

		//threshold again to obtain binary image from blur output
		cv::threshold(thresholdImageX, thresholdImageX, SENSITIVITY_VALUE, 255, THRESH_BINARY);
		cv::threshold(thresholdImageY, thresholdImageY, SENSITIVITY_VALUE, 255, THRESH_BINARY);

		if (debugMode == true) {

			//show the difference image and threshold image
			cv::imshow("Difference Image X", differenceImageX);
			cv::imshow("Threshold Image X", thresholdImageX);

			cv::imshow("Difference Image Y", differenceImageY);
			cv::imshow("Threshold Image Y", thresholdImageY);

			//show the threshold image after it's been "blurred"
			imshow("Final Threshold Image X", thresholdImageX);
			imshow("Final Threshold Image Y", thresholdImageY);
		}
		else {

			//if not in debug mode, destroy the windows so we don't see them anymore
			cv::destroyWindow("Difference Image X");
			cv::destroyWindow("Threshold Image X");

			cv::destroyWindow("Difference Image Y");
			cv::destroyWindow("Threshold Image Y");

			//if not in debug mode, destroy the windows so we don't see them anymore
			cv::destroyWindow("Final Threshold Image X");
			cv::destroyWindow("Final Threshold Image Y");
		}

		//if tracking enabled, search for contours in our thresholded image
		if (trackingEnabled) {

			searchForMovementX(thresholdImageX);

			searchForMovementY(thresholdImageY);

			//store ball position
			ballPositionsX.push_back(ballPositionX);
			ballPositionsY.push_back(ballPositionY);

			//predict next position based on previous 
			predictedBallPositionX = predictNextPositionX(ballPositionsX);
			predictedBallPositionY = predictNextPositionY(ballPositionsY);

			//estimate far position 
			predictedBallPositionFarX.x = predictedBallPositionX.x + XdeltaX * 1;
			predictedBallPositionFarX.y = predictedBallPositionX.y + XdeltaY * 1;

			predictedBallPositionFarY.x = predictedBallPositionY.x + YdeltaX * 1;
			predictedBallPositionFarY.y = predictedBallPositionY.y + YdeltaY * 1;

			//draw current and predicted on X frame
			circle(revelvantFrame1X, ballPositionX, circleSize, Scalar(0, 0, 255), 2, 8, 0);
			circle(revelvantFrame1X, predictedBallPositionFarX, circleSize, Scalar(0, 255, 0), 2, 8, 0);
			line(revelvantFrame1X, ballPositionX, predictedBallPositionFarX, Scalar(255, 0, 0), 2, 8, 0);

			//draw current and predicted on Y frame
			circle(revelvantFrame1Y, ballPositionY, circleSize, Scalar(0, 0, 255), 2, 8, 0);
			circle(revelvantFrame1Y, predictedBallPositionFarY, circleSize, Scalar(0, 255, 0), 2, 8, 0);
			line(revelvantFrame1Y, ballPositionY, predictedBallPositionFarY, Scalar(255, 0, 0), 2, 8, 0);

			//write to arduino
			sprintf_s(buffer, "%d,%d", ballPositionX.x, ballPositionY.y);
			arduino.writeSerialPort(buffer, strlen(buffer));

			cout << buffer << endl;
		}
		  
		//show our captured frame
		cv::setMouseCallback("imageX", mouseHandlerX, NULL);
		cv::setMouseCallback("imageY", mouseHandlerY, NULL);

		if (select_flagX == 1)
		{
			//show the image bounded by the box
			imshow("FINAL X", revelvantFrame1X);

			//remove trim mats
			cv::destroyWindow("imageX");
		}

		if (select_flagY == 1)
		{
			//show the image bounded by the box
			imshow("FINAL Y", revelvantFrame1Y);

			//remove trim mats
			cv::destroyWindow("imageY");
		}

		//check to see if a button has been pressed.
		//this 10ms delay is necessary for proper operation of this program
		//if removed, frames will not have enough time to referesh and a blank 
		//image will appear.
		switch (waitKey(10)) {

		case 27: //'esc' key has been pressed, exit program.
			return 0;
		case 116: //'t' has been pressed. this will toggle tracking
			trackingEnabled = !trackingEnabled;
			if (trackingEnabled == false) cout << "Tracking disabled." << endl;
			else cout << "Tracking enabled." << endl;
			break;
		case 100: //'d' has been pressed. this will debug mode
			debugMode = !debugMode;
			if (debugMode == false) cout << "Debug mode disabled." << endl;
			else cout << "Debug mode enabled." << endl;
			break;
		case 112: //'p' has been pressed. this will pause/resume the code.
			pause = !pause;
			if (pause == true) {
				cout << "Code paused, press 'p' again to resume" << endl;
				while (pause == true) {
					//stay in this loop until 
					switch (waitKey()) {
						//a switch statement inside a switch statement? Mind blown.
					case 112:
						//change pause back to false
						pause = false;
						cout << "Code Resumed" << endl;
						break;
					}
				}
			}



		}

	}

	return 0;

}
