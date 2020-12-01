#ifndef DEFS_H_
#define DEFS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

void display_all(const cv::Mat* imgs, std::string name, bool to_save);
void parab_param_calc(double& a, double& b, double& c, const cv::Point& P1, const cv::Point& P2, const cv::Point& P3);
int bounce_point(double a, double b, double c, double y);
int move_equation(double a, double b, double c, int x, int maxY);

#endif // !DEFS_H_

