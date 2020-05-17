#include <sstream>
#include<iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/calib3d/calib3d_c.h>
#include <deque>

#include "SerialPort.h"
#include "Windows.h"

using namespace std;
using namespace cv;

char buffer[16];

Mat intrinsic = Mat::eye(3, 3, CV_64F);
Mat distCoeffs;

cv::Mat map1, map2;

VideoCapture capture;

cv::Point ballPosition(0, 0);

//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 40;
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
	cv::Point predictedPosition;        // this will be the return value
	int numPositions;

	numPositions = positions.size();

	if (numPositions == 0) {

		std::cout << "error, predictNextPosition was called with zero points\n";

	}
	else if (numPositions == 1) {

		return(positions[0]);

	}
	else if (numPositions == 2) {

		int deltaX = positions[1].x - positions[0].x;
		int deltaY = positions[1].y - positions[0].y;

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;

	}
	else if (numPositions == 3) {

		int sumOfXChanges = ((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		int deltaX = (int)std::round((float)sumOfXChanges / 3.0);

		int sumOfYChanges = ((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		int deltaY = (int)std::round((float)sumOfYChanges / 3.0);

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;

	}
	else if (numPositions == 4) {

		int sumOfXChanges = ((positions[3].x - positions[2].x) * 3) +
			((positions[2].x - positions[1].x) * 2) +
			((positions[1].x - positions[0].x) * 1);

		int deltaX = (int)std::round((float)sumOfXChanges / 6.0);

		int sumOfYChanges = ((positions[3].y - positions[2].y) * 3) +
			((positions[2].y - positions[1].y) * 2) +
			((positions[1].y - positions[0].y) * 1);

		int deltaY = (int)std::round((float)sumOfYChanges / 6.0);

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;

	}
	else if (numPositions >= 5) {

		int sumOfXChanges = ((positions[numPositions - 1].x - positions[numPositions - 2].x) * 4) +
			((positions[numPositions - 2].x - positions[numPositions - 3].x) * 3) +
			((positions[numPositions - 3].x - positions[numPositions - 4].x) * 2) +
			((positions[numPositions - 4].x - positions[numPositions - 5].x) * 1);

		int deltaX = (int)std::round((float)sumOfXChanges / 10.0);

		int sumOfYChanges = ((positions[numPositions - 1].y - positions[numPositions - 2].y) * 4) +
			((positions[numPositions - 2].y - positions[numPositions - 3].y) * 3) +
			((positions[numPositions - 3].y - positions[numPositions - 4].y) * 2) +
			((positions[numPositions - 4].y - positions[numPositions - 5].y) * 1);

		int deltaY = (int)std::round((float)sumOfYChanges / 10.0);

		predictedPosition.x = positions.back().x + deltaX;
		predictedPosition.y = positions.back().y + deltaY;

	}
	else {
		// should never get here
	}

	return(predictedPosition);
}

//uses 9x6 chessboard to calibrate fisheye lens distortion 
void Calibrate()
{
	static int numBoards = 2;
	static int numCornersHor = 9;
	static int numCornersVer = 6;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	capture = VideoCapture(0 + CAP_DSHOW);

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

		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK);

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
	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	cv::initUndistortRectifyMap(intrinsic, distCoeffs, Mat_<double>::eye(3, 3), intrinsic, cv::Size(720, 480), CV_16SC2, map1, map2);
	cv::destroyWindow("CalibrateME!");
}

int main() {

	//calls the calibrate and stores the coeffs and intrisics in their respected variables
	Calibrate();

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

	//cap image
	capture.open(0 + cv::CAP_DSHOW);

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

	while (1) {

		//read first frame
		capture.read(frame1);
		//undistorts each frame
		//convert frame1 to gray scale for frame differencing
		cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
		//copy second frame
		capture.read(frame2);
		//undistorts each frame
		//convert frame2 to gray scale for frame differencing
		cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);

		//perform frame differencing with the sequential images. This will output an "intensity image"
		//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
		cv::absdiff(grayImage1, grayImage2, differenceImage);

		//threshold intensity image at a given sensitivity value
		cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

		if (debugMode == true) {
			//show the difference image and threshold image
			cv::imshow("Difference Image", differenceImage);
			cv::imshow("Threshold Image", thresholdImage);

		}
		else {
			//if not in debug mode, destroy the windows so we don't see them anymore
			cv::destroyWindow("Difference Image");
			cv::destroyWindow("Threshold Image");
		}
		//blur the image to get rid of the noise. This will output an intensity image
		cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE));
		//threshold again to obtain binary image from blur output
		cv::threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

		if (debugMode == true) {
			//show the threshold image after it's been "blurred"

			imshow("Final Threshold Image", thresholdImage);

		}
		else {
			//if not in debug mode, destroy the windows so we don't see them anymore
			cv::destroyWindow("Final Threshold Image");
		}

		//if tracking enabled, search for contours in our thresholded image
		if (trackingEnabled) {

			searchForMovement(thresholdImage, frame1);

			ballPositions.push_back(ballPosition);
			predictedBallPosition = predictNextPosition(ballPositions);

			circle(frame1, ballPositions.back(), 30, Scalar(0, 0, 255), 2, 8, 0);
			line(frame1, ballPositions.back(), predictedBallPosition, Scalar(255, 0, 0), 2, 8, 0);
			circle(frame1, predictedBallPosition, 30, Scalar(0, 255, 0), 2, 8, 0);

			//write to arduino
			//sprintf_s(buffer, "%d,%d", 0, 0);
			//arduino.writeSerialPort(buffer, strlen(buffer));
		}

		//show our captured frame
		cv::remap(frame1, Final, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
		imshow("Final", Final);

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
