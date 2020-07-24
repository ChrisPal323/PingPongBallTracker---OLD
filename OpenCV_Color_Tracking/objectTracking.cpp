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


//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 50;
//numeric camera postion 
int camPos = 0;


int deltaY;
int deltaX;

char buffer[16];

int newX;
int newY;

Mat intrinsic = Mat::eye(3, 3, CV_64F);
Mat distCoeffs;

cv::Mat map1, map2;

Point point1, point2;
int drag = 0;
cv::Rect rect;
cv::Mat img, roiImg;
int select_flag = 0;

VideoCapture capture;

cv::Point ballPosition(0, 0);
cv::Point deltaPoint(0, 0);

//ranges of trimmed rectangle 
Range RangeX(INT_MIN, INT_MAX);
Range RangeY(INT_MIN, INT_MAX);

//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;
//we'll have just one object to search for

//bounding rectangle of the object, we will use the center of this as its position.
Rect objectBoundingRectangle = Rect(0, 0, 0, 0);

char commport[] = "\\\\.\\COM4";

//int to string helper function
string intToString(int number) {

	//this function has a nuber input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void searchForMovement(Mat thresholdImage, Mat& cameraFeed) {
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if (contours.size() > 0)objectDetected = true;
	else objectDetected = false;

	if (objectDetected) {
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
	}
	ballPosition.x = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
	ballPosition.y = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
}


//Weighted avgerage for predicitons 
cv::Point predictNextPosition(std::vector<cv::Point>& positions) {
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

		deltaX = positions[1].x - positions[0].x;
		deltaY = positions[1].y - positions[0].y;

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;

	}
	else if (numPositions == 3) {

		int sumOfXChanges = ((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		deltaX = (int)std::round((float)sumOfXChanges / 3.0);

		int sumOfYChanges = ((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		deltaY = (int)std::round((float)sumOfYChanges / 3.0);

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;

	}
	else if (numPositions == 4) {

		int sumOfXChanges = ((positions[3].x - positions[2].x) * 3) +
			((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		deltaX = (int)std::round((float)sumOfXChanges / 6.0);

		int sumOfYChanges = ((positions[3].y - positions[2].y) * 3) +
			((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		deltaY = (int)std::round((float)sumOfYChanges / 6.0);

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;

	}
	else if (numPositions >= 5) {

		int sumOfXChanges = ((positions[numPositions - 1].x - positions[numPositions - 2].x) * 4) +
			((positions[numPositions - 2].x - positions[numPositions - 3].x) * 3) +
			((positions[numPositions - 3].x - positions[numPositions - 4].x) * 2) +
			((positions[numPositions - 4].x - positions[numPositions - 5].x) * 1);

		deltaX = (int)std::round((float)sumOfXChanges / 10.0);

		int sumOfYChanges = ((positions[numPositions - 1].y - positions[numPositions - 2].y) * 4) +
			((positions[numPositions - 2].y - positions[numPositions - 3].y) * 3) +
			((positions[numPositions - 3].y - positions[numPositions - 4].y) * 2) +
			((positions[numPositions - 4].y - positions[numPositions - 5].y) * 1);

		deltaY = (int)std::round((float)sumOfYChanges / 10.0);

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;


	}
	else {
		// should never get here
	}

	return(predictedPosition);
}


bool FindPoint(int x1, int y1, int x2, int y2, int x, int y)
{
	if (x > x1&& x < x2 && y > y1&& y < y2) {
		return true;
	}
	return false;
}


void mouseHandler(int event, int x, int y, int flags, void* param)
{

	Mat img1 = img.clone();

	if (event == cv::EVENT_LBUTTONDOWN && !drag)
	{
		/* left button clicked. ROI selection begins */
		//cv::remap(img1, img1, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

		select_flag = 0;
		point1 = Point(x, y);
		drag = 1;
	}

	if (event == cv::EVENT_MOUSEMOVE && drag)
	{
		/* mouse dragged. ROI being selected */
		point2 = Point(x, y);
		rectangle(img1, point1, point2, (255, 0, 0), 3, 8, 0);
		imshow("image", img1);
	}

	if (event == cv::EVENT_LBUTTONUP && drag)
	{
		point2 = Point(x, y);
		rect = Rect(point1.x, point1.y, x - point1.x, y - point1.y);
		drag = 0;
		roiImg = img(rect);

		RangeX.start = point1.x;
		RangeX.end = point2.x;

		RangeY.start = point1.y;
		RangeY.end = point2.y;
	}

	if (event == cv::EVENT_LBUTTONUP)
	{
		/* ROI selected */
		select_flag = 1;
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
	Mat frame1, frame2;
	//their grayscale images (needed for absdiff() function)
	Mat grayImage1, grayImage2;
	//resulting difference image
	Mat differenceImage;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImage;
	//final Mat
	Mat Final;
	//2D rendering mat
	Mat render;
	//adjusted scale values for trimmed frame and render
	int adjustX, adjustY;
	//2d render size ints
	int renderSizeX = 600;
	int renderSizeY = 500;

	capture.open(camPos + cv::CAP_DSHOW);

	if (!capture.isOpened()) {
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return -1;
	}

	SerialPort arduino(commport);
	if (arduino.isConnected()) {
		cout << "Connection made" << endl << endl;
	}
	else {
		cout << "Error in port name" << endl << endl;
	}

	capture.set(cv::CAP_PROP_SETTINGS, 1);
	capture.set(cv::CAP_PROP_FPS, 260);


	std::vector<cv::Point> ballPositions;
	cv::Point predictedBallPosition;
	cv::Point predictedBallPositionFar;

	capture >> img;
	imshow("image", img);

	while (1) {
		//read first frame
		capture.read(frame1);
		//copy second frame
		capture.read(frame2);

		//create trimmed mats
		Mat revelvantFrame1(frame1, RangeY, RangeX);
		Mat relevantFrame2(frame2, RangeY, RangeX);
		Mat render = Mat::zeros(Size(renderSizeX, renderSizeY), CV_8UC3);

		//draw ping pong table on render 
		rectangle(render, Point(0, 0), Point(renderSizeX, renderSizeY), Scalar(255, 255, 255), FILLED);
		rectangle(render, Point(renderSizeX / 2 + abs(RangeX.start - RangeX.end) / 2, renderSizeY / 2 + abs(RangeY.start - RangeY.end) / 2), Point(renderSizeX / 2 - abs(RangeX.start - RangeX.end) / 2, renderSizeY / 2 - abs(RangeY.start - RangeY.end) / 2), Scalar(0, 0, 0), 8);
		rectangle(render, Point(renderSizeX/2 + abs(RangeX.start - RangeX.end) / 2, renderSizeY/2 + abs(RangeY.start - RangeY.end) / 2), Point(renderSizeX/2 - abs(RangeX.start - RangeX.end) / 2, renderSizeY/2 - abs(RangeY.start - RangeY.end) / 2), Scalar(51, 255, 51), FILLED, 1);
		line(render, Point(renderSizeX/2 + abs(RangeX.start - RangeX.end) / 2, renderSizeY/2), Point(renderSizeX/2 - abs(RangeX.start - RangeX.end) / 2, renderSizeY/2), Scalar(255, 255, 255), 4);
		line(render, Point(renderSizeX/2, renderSizeY/2 + abs(RangeY.start - RangeY.end) / 2), Point(renderSizeX/2, renderSizeY/2 - abs(RangeY.start - RangeY.end) / 2), Scalar(255, 255, 255), 4);

		//convert trimmed mats to gray scale for frame dif23/*3ferencing
		cv::cvtColor(revelvantFrame1, grayImage1, COLOR_BGR2GRAY);
		cv::cvtColor(relevantFrame2, grayImage2, COLOR_BGR2GRAY);

		//perform frame differencing with the sequential images. This will output an "intensity image"
		//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
		cv::absdiff(grayImage1, grayImage2, differenceImage);

		//threshold intensity image at a given sensitivity value
		cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

		//blur the image to get rid of the noise. This will output an intensity image
		cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE));

		//threshold again to obtain binary image from blur output
		cv::threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);


		if (debugMode == true) {
			//show the threshold image after it's been "blurred"
			cv::imshow("Difference Image", differenceImage);
			cv::imshow("Threshold Image", thresholdImage);
			cv::imshow("Final Threshold Image", thresholdImage);

		}
		else {
			//if not in debug mode, destroy the windows so we don't see them anymore
			cv::destroyWindow("Final Threshold Image");
			cv::destroyWindow("Difference Image");
			cv::destroyWindow("Threshold Image");
		}

		//if tracking enabled, search for contours in our thresholded image
		if (trackingEnabled) {

			searchForMovement(thresholdImage, frame1);

			//store ball position
			ballPositions.push_back(ballPosition);

			//predict next position based on previous 
			predictedBallPosition = predictNextPosition(ballPositions);

			//estimate far position 
			predictedBallPositionFar.x = predictedBallPosition.x + deltaX * 1;
			predictedBallPositionFar.y = predictedBallPosition.y + deltaY * 1;

			//draw circles 
			circle(revelvantFrame1, ballPosition, 10, Scalar(0, 0, 255), 2, 8, 0);
			circle(revelvantFrame1, predictedBallPositionFar, 10, Scalar(0, 255, 0), 2, 8, 0);
			line(revelvantFrame1, ballPosition, predictedBallPositionFar, Scalar(255, 0, 0), 2, 8, 0);

			// X = [0 - RangeX.end] --> [0 - renderSizeX / 2 - abs(RangeX.start - RangeX.end) / 2]
			// Y = [0 - RangeY.end] --> [0 - renderSizeY / 2 - abs(RangeY.start - RangeY.end) / 2]

			adjustX = renderSizeX / 2 - abs(RangeX.start - RangeX.end) / 2;
			adjustY = renderSizeY / 2 - abs(RangeY.start - RangeY.end) / 2;

			//draw circles 
			circle(render, Point(ballPosition.x + adjustX, ballPosition.y + adjustY), 10, Scalar(0, 0, 0), 2, 8, 0);
			circle(render, Point(predictedBallPositionFar.x + adjustX, predictedBallPositionFar.y +adjustY), 10, Scalar(160, 160, 160), 2, 8, 0);
			line(render, Point(ballPosition.x + adjustX, ballPosition.y + adjustY), Point(predictedBallPositionFar.x + adjustX, predictedBallPositionFar.y + adjustY), Scalar(128, 128, 128), 2, 8, 0);

			//write to cout 
			cout << ballPosition << endl;

			//write to arduino
			sprintf_s(buffer, "%d,%d", ballPosition);
			arduino.writeSerialPort(buffer, strlen(buffer));
		}

		//show our captured frame
		cv::setMouseCallback("image", mouseHandler, NULL);

		if (select_flag == 1)
		{
			//show trimmed frame
			imshow("FINAL", revelvantFrame1);
			
			//show render frame
			imshow("2D Rendering", render);

			cv::destroyWindow("image");
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
