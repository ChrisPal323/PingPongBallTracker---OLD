#include <sstream>
#include<iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <GLFW/glfw3.h>
#include <deque>

#include "SerialPort.h"
#include "Windows.h"

using namespace std;
using namespace cv;

char buffer[16];

#define SIZE 16
Vec2i Dirs[SIZE];
int front = 0;
int rear = 0;
int dirsCount;

Vec2i curAvg;

int xCur;
int yCur;

int xPrev;
int yPrev;

//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 50;
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

		// circle the tracked object
		circle(cameraFeed, Point(objectBoundingRectangle.x + objectBoundingRectangle.width / 2, objectBoundingRectangle.y + objectBoundingRectangle.height / 2), 20, Scalar(0, 255, 0), 2);
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

	capture1.open(1);

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

				xPrev = xCur;
				yPrev = yCur;

				xCur = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
				yCur = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

				//
				Vec2i Dir(xCur - xPrev, yCur - yPrev);

				//calc rolling avg
				curAvg = rollingAvg(sum, enqueue(Dir), Dir);

				//output
				cout << "AVERAGE" << curAvg << endl;\

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