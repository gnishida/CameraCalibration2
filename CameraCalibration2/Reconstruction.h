#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

class Reconstruction {
public:
	Reconstruction();

	cv::Mat findFundamentalMat(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<uchar>& status);
	cv::Mat computeEpipole(cv::Mat& F, int whichImage);
	void computeProjectionMat(cv::Mat& E, cv::Mat& P1, cv::Mat& P2);

	double unprojectPoints(cv::Matx33d& cameraMatrix, const cv::Matx34d& P1, const cv::Matx34d& P2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d);
	Matx31d triangulate(Point3d u, Matx34d P, Point3d u1, Matx34d P1);
	Matx41d iterativeTriangulation(cv::Point3d u, Matx34d P, cv::Point3d u1, Matx34d P1);
};

