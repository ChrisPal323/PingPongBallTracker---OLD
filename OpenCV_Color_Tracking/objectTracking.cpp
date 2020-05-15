#include <sstream>
#include<iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <deque>

#include "SerialPort.h"
#include "Windows.h"

using namespace std;
using namespace cv;

char buffer[16];

#define SIZE 4
Vec2i Dirs[SIZE];
int front = 0;
int rear = 0;
int dirsCount;

cv::Point ballPosition(0, 0);

//rolling vector buffer
Vec2i curAvg;

//cur pos
int xCur;
int yCur;

//prev pos
int xPrev;
int yPrev;

//tracking distance sens
int distThres = 30;

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

Vec2i rollingAvg(Vec2f& sum, Vec2i removed, Vec2i added)
{
	if (removed[0] == -1 && removed[1] == -1) {
		//nothing here 
	}
	else {
		sum -= removed;
	}

	sum += added;

	return sum / (float)dirsCount;

}

/*
Enqueues a new direction vector into the global Vec array 

	Param: newDir - Direction to enqueue 
*/
Vec2i enqueue(Vec2i newDir)
{
	if (dirsCount == SIZE) {
		front = (front + 1) % SIZE;
	}
	else {
		dirsCount++;
	}

	//store soon to be removed Vec
	Vec2i removed = Dirs[rear];

	//insert element at rear
	Dirs[rear] = newDir;
	rear = (rear + 1) % SIZE;

	if (dirsCount == SIZE) {
		return removed;
	}
	else {
		return -1;
	}
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
	findContours(temp, contours, hierarchy, cv:: RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);// retrieves external contours

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
	//video capture object.
	VideoCapture capture1;

	//cap image
	capture1.open(0 + cv::CAP_DSHOW);

	if (!capture1.isOpened()) {
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

	Vec2f sum = 0;

	capture1.set(cv::CAP_PROP_FPS, 10);
	//capture1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'));
	//capture1.set(cv2.CAP_PROP_SETTINGS, 1);
	//capture1.set(cv2.CAP_PROP_EXPOSURE, -12);
	capture1.set(cv::CAP_PROP_FRAME_WIDTH, 352);
	capture1.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

	std::vector<cv::Point> ballPositions;
	cv::Point predictedBallPosition;

	while (1) {

		//we can loop the video by re-opening the capture every time the video reaches its last frame

		//check if the video has reach its last frame.
		//we add '-1' because we are reading two frames from the video at a time.
		//if this is not included, we get a memory error!

			//read first frame
			capture1.read(frame1);
			//convert frame1 to gray scale for frame differencing
			cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
			//copy second frame
			capture1.read(frame2);
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

				std::cout << "current position        = " << ballPositions.back().x << ", " << ballPositions.back().y << "\n";
				std::cout << "next predicted position = " << predictedBallPosition.x << ", " << predictedBallPosition.y << "\n";
				std::cout << "--------------------------------------------------\n";
			
				circle(frame1, ballPositions.back(), 30, Scalar(0, 0, 255), 2, 8, 0);
				circle(frame1, predictedBallPosition, 30, Scalar(0,255,0), 2, 8, 0);


					//write to arduino
					sprintf_s(buffer, "%d,%d", xCur, yCur);
					arduino.writeSerialPort(buffer, strlen(buffer));
			}

			//show our captured frame
			imshow("Frame1", frame1);

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