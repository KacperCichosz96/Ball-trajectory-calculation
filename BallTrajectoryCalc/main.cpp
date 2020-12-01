#include <iostream>
#include <math.h>
#include <iomanip>
#include <vector>
#include <Windows.h>
#include <random>
#include "defs.h"

//This application allows to calculate a trajectory of table tennis ball in side view of a table (to get complete information about ball position,
//a front view photos are needed too). The calculations are performed based on 3 photos of the ball location (subsequent in time) after opponent shot
//and before the first bounce on the table. For a proper working of the app, used photos should meet some condition:
//- the background should be plain, without any people or staff
//- good lighting conditions should be ensured
//- at the bottom of the photo a ping pong table (in full length) should be seen
//
//The app was a steering part of a university project of table tennis robot.


int main()
{
	using std::cout;
	using std::cin;
	using std::endl;
	using cv::Mat;

	const std::string name1("BallPos1.jpg");
	const std::string name2("BallPos2.jpg");
	const std::string name3("BallPos3.jpg");

	Mat img[3];
	img[0] = cv::imread(name1);
	img[1] = cv::imread(name2);
	img[2] = cv::imread(name3);

	if (img[0].empty())
	{
		cout << "Failed to read: " << name1 << endl;
		cin.get();
		exit(EXIT_FAILURE);
	}

	if (img[1].empty())
	{
		cout << "Failed to read: " << name2 << endl;
		cin.get();
		exit(EXIT_FAILURE);
	}
	
	if (img[2].empty())
	{
		cout << "Failed to read: " << name3 << endl;
		cin.get();
		exit(EXIT_FAILURE);
	}

	display_all(img, "BallPos", false);

	Mat BGRchnls[3];
	Mat imgBlur[3];

	for (int i = 0; i < 3; i++)
	{
		split(img[i], BGRchnls);
		cv::blur(BGRchnls[0], imgBlur[i], cv::Size(5, 5), cv::Point(-1, -1), cv::BORDER_REPLICATE);
	}
	display_all(imgBlur, "BallPos blurred", false);

	Mat edges[3];
	const int cols = imgBlur[0].cols;
	const int rows = imgBlur[0].rows;
	unsigned int avrg_intens = 0;

	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			avrg_intens += imgBlur[0].at<uchar>(i, j);

	avrg_intens /= (cols*rows);
	
	cout << "Average pixel intensity on blurred image: " << avrg_intens << endl << endl;

	for (int i = 0; i < 3; i++)
		cv::Canny(imgBlur[i], edges[i], 0.5*avrg_intens, avrg_intens, 3, true); 
	display_all(edges, "BallPos edges", false);

	//finding value of X coordinate of left edge of table:
	Mat TableDetectImg = edges[0].clone();
	std::vector<int> L_edge;

	for (int i = 0; i < TableDetectImg.size().height; i++)
		for (int j = 0; j < TableDetectImg.size().width; j++)
		{
			if (TableDetectImg.at<uchar>(i, j) == 255)
			{
				L_edge.push_back(j);
				break;
			}
		}

	auto left_X = std::min_element(L_edge.begin(), L_edge.end());
	cout << "Table:" << endl;
	cout << "Left edge X coordinate: " << *left_X << endl;

	//finding value of X coordinate of right edge of table:
	std::vector<int> R_edge;

	for (int i = 0; i < TableDetectImg.size().height; i++)
		for (int j = TableDetectImg.size().width - 1; j >= 0; j--) {
			if (TableDetectImg.at<uchar>(i, j) == 255)
			{
				R_edge.push_back(j);
				break;
			}
		}
	
	auto right_X = std::max_element(R_edge.begin(), R_edge.end());
	cout << "Right edge X coordinate: " << *right_X << endl << endl;

	//finding level of table surface:
	std::vector<int> TabLevel;

	for (int i = 0; i < TableDetectImg.size().width; i++)
		for (int j = 0; j < TableDetectImg.size().height; j++)
		{
			if (TableDetectImg.at<uchar>(j, i) == 255)
			{
				TabLevel.push_back(j);
				break;
			}
		}
	
	//creating a histogram to find the most frequent value of table level:
	std::vector<int> TabHistogram(TableDetectImg.size().height, 0);

	for (int i = 0; i < TabLevel.size(); i++)
		++TabHistogram[TabLevel[i]];

	auto LevelValIter = std::max_element(TabHistogram.begin(), TabHistogram.end());
	int LevelVal = LevelValIter - TabHistogram.begin();
	cout << "Table level value: " << LevelVal << endl;

	//ball detection:
	std::vector<cv::Vec3f> Circ[3];

	for(int i=0; i<3;i++)
		HoughCircles(imgBlur[i], Circ[i], cv::HOUGH_GRADIENT, 1.0, 50, avrg_intens, 30);

	cout << "Found circles (amount): " << endl;
	cout << name1 << ": " << Circ[0].size() << endl;
	cout << name2 << ": " << Circ[1].size() << endl;
	cout << name3 << ": " << Circ[2].size() << endl << endl;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < Circ[i].size(); j++)
		{
			cv::Point center(cvRound(Circ[i][j][0]), cvRound(Circ[i][j][1]));
			int radius = cvRound(Circ[i][j][2]);
			circle(img[i], center, radius, cv::Scalar(0, 255, 255), cv::FILLED);
		}
	}
	display_all(img, "Found circles", false);

	//trajectory calculation:
	//coordinates of ball in subsequent locations:
	cv::Point Pos1(cvRound(Circ[0][0][0]), cvRound(Circ[0][0][1]));
	cv::Point Pos2(cvRound(Circ[1][0][0]), cvRound(Circ[1][0][1]));
	cv::Point Pos3(cvRound(Circ[2][0][0]), cvRound(Circ[2][0][1]));
	const int TabLeng = abs(*left_X - *right_X);

	int distDone = Pos1.x - Pos3.x;			//distance between position 1 and position 3 of the ball
	int distAhead = Pos3.x - *left_X;		//distance to edge toward which the ball is flying

	//symulating the time of flight:
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> TimeGen(0.1, 0.2);
	const double time = TimeGen(gen);					
	//
	const double speed = (abs((distDone) / time));		
	const double  time_ahead = (distAhead / speed);		//time to the moment when the ball will reach the end of the table

	cout << "Position 1: x = " << Pos1.x << " ; y = " << Pos1.y << endl;
	cout << "Position 2: x = " << Pos2.x << " ; y = " << Pos2.y << endl;
	cout << "Position 3: x = " << Pos3.x << " ; y = " << Pos3.y << endl << endl;
	//cout << "Table lenght: " << TabLeng << endl;
	//cout << "Speed of the ball (pixels per second): " << static_cast<int>(speed) << endl << endl;
	
	//movement equation:
	double A(0);		//parameters of the parabola equation
	double B(0);
	double C(0);

	parab_param_calc(A, B, C, Pos1, Pos2, Pos3);

	cout.setf(std::ios_base::showpos);
	cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	cout << "Move equation:\t" << "y = " << A << "x^2" << B << "x" << C << endl << endl;		//in the picture X coordinates incerase normally -
																								//towards right hand side, but Y coordinates
																								//increases downwards
	cout.unsetf(std::ios_base::showpos);
	cout.unsetf(std::ios_base::floatfield);

	int connectionX = bounce_point(A, B, C, LevelVal);	//X coordinate of the bounce point

	cv::circle(img[0], cv::Point(connectionX, LevelVal), 10, cv::Scalar(0, 0, 255), cv::FILLED);
	cv::namedWindow("The bounce point");
	cv::imshow("The bounce point", img[0]);
	cv::waitKey(0);
	cv::destroyWindow("The bounce point");

	//trajectory after the bounce
	double A2(0);	
	double B2(0);
	double C2(0);

	cv::Point bouncedP1((connectionX - abs(Pos1.x - connectionX)),
		(TableDetectImg.size().height - (0.7*(TableDetectImg.size().height - Pos1.y))));	//we take under consideration the energy losses that is
																							//why we include "0.7*..."
	cv::Point bouncedP2((connectionX - abs(Pos2.x - connectionX)),
		(TableDetectImg.size().height - (0.7*(TableDetectImg.size().height - Pos2.y))));
	cv::Point bouncedP3(connectionX, LevelVal);

	parab_param_calc(A2, B2, C2, bouncedP1, bouncedP2, bouncedP3);

	cout.setf(std::ios_base::showpos);
	cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	cout << "Move equation aftre the bounce:\t" << "y = " << A2 << "x^2" << B2 << "x" << C2 << endl << endl;
	cout.unsetf(std::ios_base::showpos);
	cout.unsetf(std::ios_base::floatfield);

	//determining the height of the ball in the moment when it should be bounced with a racket:
	int heightY;								//Y coordinate of final ball position
	int heightPixels;							//height in pixels
	double heightCms;							//height in centimeters
	const double RealLeng = 274.0;				//real length of the table in centimeters
	const double ratio = RealLeng / TabLeng;
	
	if (*left_X < connectionX)		//case when the ball bounced on the table
		heightY = move_equation(A2, B2, C2, *left_X, img[0].size().height);
	else							//case when the ball flies outside the table
		heightY = move_equation(A, B, C, *left_X, img[0].size().height);

	heightPixels = abs(LevelVal - heightY);
	heightCms = heightPixels * ratio;
	
	cout << "\n**************************************************" << endl;
	cout << "RESULTS:" << endl;
	cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	cout << "Time left: " << std::setprecision(2) << time_ahead << " sec" << endl;
	//cout << "Ratio: " << std::setprecision(3) << ratio << endl;
	cout << "Table level (y coord. value): " << LevelVal << endl;
	cout << "Height over the table (in pixels): " << heightPixels << endl;
	cout << "Height over the table (in centimeters): " << std::setprecision(1) << heightCms << endl;
	cout.unsetf(std::ios_base::floatfield);
	cout << "**************************************************" << endl;

	//drawing the calculated trajectory and the final position of the ball:
	cv::Point finalPos(*left_X, heightY);
	Mat finImg = img[0].clone();

	for (int i = 0; i < finImg.size().width; i++)
	{
		if (i < connectionX)
		{
			int y = move_equation(A2, B2, C2, i, finImg.size().height);
			if (y > 0)
				finImg.at<cv::Vec3b>(y, i) = cv::Vec3b(0, 0, 255);
		}
		else
		{
			int y = move_equation(A, B, C, i, finImg.size().height);
			if (y > 0)
				finImg.at<cv::Vec3b>(y, i) = cv::Vec3b(0, 0, 255);
		}
	}

	circle(finImg, finalPos, 20, cv::Scalar(0, 0, 255), cv::FILLED);

	cv::namedWindow("Trajectory");
	imshow("Trajectory", finImg);
	imwrite("Trajectory.jpg", finImg);
	cv::waitKey(0);
	cv::destroyWindow("Trajectory");

	cin.get();

	return 0;
}
