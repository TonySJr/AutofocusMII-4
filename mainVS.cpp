// Diplom.cpp : This file contains the 'main' function. Program execution begins and ends there.
//for graphs
#define _CRT_SECURE_NO_WARNINGS
#include <string.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstdio>
#include <windows.h>
#include "serial/serial.h"
#include <fstream>

//#define steps 200 // stepper motor steps
#define staticError 14 // some error may be

//using namespace std;
using namespace cv;
using std::string;
using std::exception;
using std::cout;
using std::cin;
using std::cerr;
using std::endl;
using std::vector;

volatile bool endflag = false;
bool readyflag = false; // flag for arduino ready

Mat frame; // cam matrix for main img
int framecount = 0;
int steps = 0;
// FOCUS VAL 
double F;
//----------------------------------functions---------------------------------------
void my_sleep(unsigned long milliseconds);
void enumerate_ports();
void print_usage();
void MyRect(Mat img, Point start, Point end);
bool arduinoready(serial::Serial& obj);
void send_A_steps(serial::Serial& my_serial, uint8_t step); // send instruction to arduino

//== list of ready functions==
double th_grad(Mat& img);
double sq_grad(Mat& img);
double SML(Mat& img); //derivative
double Var(Mat& img); // working SKO
double Entropy(Mat& img); //information

//ROI
int ROI = 0;
int x,y;
//======================
string FNameArray[] = { "th_grad", "sq_grad", "SML", "Var", "Entropy" };
// array pointer to a function;
double (*Functions[])(Mat& img) = { th_grad, sq_grad, SML, Var, Entropy };
const int numF = sizeof(Functions) / sizeof(Functions[0]); // number of functions
//double FocusArray[numF][steps]; // array with 'numF' colums and "steps" rows for values of focus functions
int fuc = 0;
int main(int argc, char **argv)
{
	//	connect arduino
	if (argc < 2) {
		print_usage();
		return 0;
	}

	// Argument 1 is the serial port or enumerate flag
	string port(argv[1]);

	if (port == "-e") {
		enumerate_ports();
		return 0;
	}
	//else if (argc < 3) {
	//	print_usage();
	//	return 1;
	//}

	// Argument 2 is the baudrate
	unsigned long baud = 9600;
//#if defined(WIN32) && !defined(__MINGW32__)
//	sscanf_s(argv[2], "%lu", &baud);
//#else
//	sscanf(argv[2], "%lu", &baud);
//#endif

	// port, baudrate, timeout in milliseconds
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(200));

	//send ready flag
	readyflag = arduinoready(my_serial);
	my_sleep(5000);

	//	connect cam
	VideoCapture cap;
	int deviceID = 1;             // 0 = open default camera, 1 = open next camera
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API
	cap.open(deviceID + apiID);

	cout << "	enter ROI (int)\n";
	cin >> ROI;
	cout << "	ROI = " << ROI << endl;
	cout << "	enter steps\n";
	cin >> steps;
	cout << "	steps = " << steps << endl;
	double* FocusArray = new double[steps];
	cout << "	choose function\n";
	for (int i = 0; i < numF; i++) 
	{
		cout << FNameArray[i] << "	";
	}
	cout << endl;
	for (int i = 0; i < numF; i++)
	{
		cout << i << "	";
	}
	cout << endl;
	cin >> fuc;
	cout << "	fuc is = " << fuc << endl;
	//	GRAB AND WRITE LOOP
	cout << "Start grabbing" << endl;

	if (readyflag == true && cap.isOpened() && steps != 0)
	{	
		//ROI
		x = (frame.cols / 2) - (ROI / 2);
		y = (frame.rows / 2) - (ROI / 2);
		Rect r(x, y, ROI, ROI);

		for (;;)
		{
			cap.read(frame);
			if (frame.empty()) {
				cerr << "ERROR! blank frame grabbed\n";
				break;
			}

			cvtColor(frame, frame, COLOR_BGR2GRAY);

			if (framecount < steps)
			{				
				Mat smallframe = frame(r);
				FocusArray[framecount] = Functions[fuc](smallframe);
				send_A_steps(my_serial, 2);
				framecount++;
				cout << "	" << framecount << endl;
			}
			else
				endflag = true;

			MyRect(frame, Point(x, y), Point(x + ROI, y + ROI));
			imshow("Live", frame);

			if (waitKey(5) >= 0 || endflag == true) break;
		}

		int hist_w = framecount, hist_h = 400;  // for frame hist

		//initialyze map variables
		double in_max = FocusArray[0];
		double in_min = FocusArray[0];
		double maxstep = FocusArray[0];
		double out_min = 0;

		Mat FocusPlot(hist_h, hist_w, CV_8UC1, Scalar(255, 0, 0));

		//find max,min
		for (int rows = 0; rows < framecount; rows++)
		{
			if (FocusArray[rows] > in_max)
			{
				in_max = FocusArray[rows];
				maxstep = rows;
			}
			else if (FocusArray[rows] < in_min)
				in_min = FocusArray[rows];
		}

		//map hight
		for (int i = 0; i < framecount;i++)
		{
			FocusArray[i] = ((FocusArray[i] - in_min) * (hist_h - out_min)) / (in_max - in_min) + out_min;
		}
		
		//draw and save hist
		for (int i = 1; i < framecount; i++)
			line(FocusPlot, Point((i - 1), hist_h - cvRound(FocusArray[i - 1])),
				Point(i, hist_h - cvRound(FocusArray[i])),
				Scalar(0, 0, 0), 1, 4, 0);

		imshow("FocusPLot", FocusPlot);
		imwrite("FocusImagePlot.jpg", FocusPlot);

		while (true) {
		if (waitKey(5) >= 0) break;
		}

		// go back to maxstep
		int focus = (int)maxstep - staticError;
		for (int i = framecount; i > focus; i--)
		{
			cap.read(frame);
			if (frame.empty()) {
				cerr << "ERROR! blank frame grabbed\n";
				break;
			}
			send_A_steps(my_serial, 1);
			imshow("Live", frame);
			if (waitKey(5) >= 0) break;
		}

		// live streaming
		while (true)
		{
			cap.read(frame);
			if (frame.empty()) {
			cerr << "ERROR! blank frame grabbed\n";
			break;
			}
			imshow("Live", frame);
			if (waitKey(5) >= 0) break;
		}
		cap.read(frame);
		imwrite("Img.jpg", frame);

		//go stepper back to 0
		for (int i = focus; i > 0; i--)
		{
			send_A_steps(my_serial, 1);
			if (waitKey(5) >= 0) break;
		}
	}
	else
	{
		cerr << "ERROR! Unable to open camera\n";
		return -1;
	}
	delete[] FocusArray;
	return 0;
}
//	Focus Functions-----------------------------------
//Derivative
double th_grad(Mat & smallframe)
{
	F = 0.0;
	double threshold = 30;
	for (int y = 0; y < smallframe.rows; y++)
	{
		for (int x = 0; x < smallframe.cols - 1; x++)
		{
			double f = std::abs(smallframe.at<uint8_t>(y, x + 1) - smallframe.at<uint8_t>(y, x));
			if (f >= threshold)
				F += f;
		}
	}
	return F;
}

double sq_grad(Mat & smallframe)
{
	F = 0.0;
	double threshold = 240;
	for (int y = 0; y < smallframe.rows; y++)
	{
		for (int x = 0; x < smallframe.cols - 1; x++)
		{
			double f = pow(smallframe.at<uint8_t>(y, x + 1) - smallframe.at<uint8_t>(y, x), 2);
			if (f >= threshold)
				F += f;
		}
	}
	return F;
}

double SML(Mat & smallframe)
{
	F = 0.0;
	for (int y = 1; y < smallframe.rows - 1; y++)
	{
		for (int x = 1; x < smallframe.cols - 1; x++)
		{
			F += abs(smallframe.at<uint8_t>(y, x + 1) + smallframe.at<uint8_t>(y, x - 1) - 2 * smallframe.at<uint8_t>(y, x))
				+ abs(smallframe.at<uint8_t>(y + 1, x) + smallframe.at<uint8_t>(y - 1, x) - 2 * smallframe.at<uint8_t>(y, x));
		}
	}
	return F;
}
//Statistical
double Var(Mat & smallframe)
{
	F = 0.0;
	double MN = 1.0 / ((double)smallframe.rows * smallframe.cols);
	double gmean = mean(smallframe).val[0];
	for (int y = 0; y < smallframe.rows; y++)
	{
		for (int x = 0; x < smallframe.cols; x++)
		{
			F += pow(smallframe.at<uint8_t>(y, x) - gmean, 2);
		}
	}
	return F *= MN;
}
//Histogram Based
double Entropy(Mat & smallframe)
{
	F = 0.0;
	double MN = (double)smallframe.rows * smallframe.cols;

	// calc hist
	const int histSize = 256;
	float range[] = { 0, 256 }; //the upper boundary is exclusive
	const float* histRange = { range };
	bool uniform = true, accumulate = false;

	Mat hist;
	calcHist(&smallframe, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
	normalize(hist, hist, 1, 256, NORM_MINMAX);

	for (int intensity = 0; intensity < histSize; intensity++)
	{
		double P = (double)(hist.at<float>(intensity) / MN);
		F += P * std::log2(P);
	}
	return -F;
}
//-----------------------------------------------------------
bool arduinoready(serial::Serial & my_serial)
{
	cout << "Is the serial port open?";
	if (my_serial.isOpen())
	{
		cout << " Yes." << endl;
		return 1;
	}else{
		cout << " No." << endl;
		return 0;
	}
}

void send_A_steps(serial::Serial & my_serial, uint8_t step) //	отправляем на ардуино количество шагов и ждем пока шаги выполнятся
{
	string test_string = std::to_string(step);
	//cout <<  endl;

	size_t bytes_wrote = my_serial.write(test_string);

	string result;
	int countfail = 0;
	while (result != test_string && countfail < 5)
	{
		result = my_serial.read(test_string.length());
		cout << "	read: " << result;
		countfail++;
	}
	if (countfail == 5) endflag = true;
}

void MyRect(Mat img, Point start, Point end)
{
	int thickness = 2;
	int lineType = LINE_8;
	cv::rectangle(img, start, end, Scalar(0, 0, 0), thickness, lineType);
}

void my_sleep(unsigned long milliseconds)
{
#ifdef _WIN32
	Sleep(milliseconds); // 100 ms
#else
	usleep(milliseconds * 1000); // 100 ms
#endif
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		serial::PortInfo device = *iter++;

		printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
			device.hardware_id.c_str());
	}
}

void print_usage()
{
	cerr << "Usage: test_serial {-e|<serial port address>} ";
	cerr << "<baudrate> [test string]" << endl;
}