#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

class Reconstruction {
public:
	Reconstruction();

	cv::Mat findFundamentalMat(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<uchar>& status);
	cv::Mat computeEpipole(cv::Mat& F, int whichImage);
	void computeProjectionMat(Matx33d E, Mat_<double>& R1, Mat_<double>& T1, Mat_<double>& R2, Mat_<double>& T2);

	double unprojectPoints(Size& size, Mat_<double>& K, const Mat_<double>& R1, const Mat_<double>& T1, const Mat_<double>& R2, const Mat_<double>& T2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d);
	double unprojectPoints(Size& size, Mat_<double>& K, Mat_<double>& Kinv, const Matx34d& P, const Matx34d& P1, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, int& numFront);
	Matx31d triangulate(Point3d u, Matx34d P, Point3d u1, Matx34d P1);
	Matx41d iterativeTriangulation(cv::Point3d u, Matx34d P, cv::Point3d u1, Matx34d P1);

	bool decomposeEtoRandT(Mat_<double>& E, Mat_<double>& R1, Mat_<double>& R2, Mat_<double>& t1, Mat_<double>& t2);
};

