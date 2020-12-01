#include "defs.h"

void display_all(const cv::Mat* imgs, std::string name, bool to_save)
{
	std::string temp_name;
	for (int i = 0; i < 3; i++)
	{
		temp_name = name + " " + std::to_string(i + 1);
		cv::namedWindow(temp_name);
		cv::imshow(temp_name, imgs[i]);

		if(to_save)
			cv::imwrite((temp_name +".jpg"), imgs[i]);

		cv::waitKey(0);
	}
	cv::destroyAllWindows();
}

void parab_param_calc(double& a, double& b, double& c, const cv::Point& P1, const cv::Point& P2, const cv::Point& P3)
{
	double x1 = P1.x;
	double y1 = P1.y;
	double x2 = P2.x;
	double y2 = P2.y;
	double x3 = P3.x;
	double y3 = P3.y;

	a = ((x1 - x3)*(y2 - y3) + (y1 - y3)*(x3 - x2)) / ((x1 - x3)*(x2*x2 - x3 * x3) + (x3*x3 - x1 * x1)*(x2 - x3));
	b = (y1 - y3 + (a*(x3*x3 - x1 * x1))) / (x1 - x3);
	c = (y3 - a * x3*x3 - b * x3);
}

int bounce_point(double a, double b, double c, double y)  //calculating the common point of trajectory parabola and the line which states table surface
{
	c = c - y;

	double delta; 
	delta = b * b - 4 * a*c;

	int x1 = (-b - sqrt(delta)) / (2 * a);
	int x2 = (-b + sqrt(delta)) / (2 * a);

	if (x1 < x2)
		return x1;
	else
		return x2;
}

int move_equation(double a, double b, double c, int x, int maxY)
{
	int y = a * (x*x) + b * x + c;
	if (y >= 0 && y < maxY)
		return y;
	else
		return -1;
}