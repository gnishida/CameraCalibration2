#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

class Reconstruction {
public:
	Reconstruction();

	cv::Mat findFundamentalMat(std::vector<Point2f>& pts1, std::vector<Point2f>& pts2, std::vector<uchar>& status);
	cv::Mat computeEpipole(cv::Mat& F, int whichImage);
	void computeProjectionMat(Matx33d E, Mat_<double>& R1, Mat_<double>& T1, Mat_<double>& R2, Mat_<double>& T2);

	double unprojectPoints(const Mat_<double>& K, const Mat_<double>& R1, const Mat_<double>& T1, const Mat_<double>& R2, const Mat_<double>& T2, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, Mat_<double>& P1, Mat_<double>& P2);
	bool unprojectPoints(const Mat_<double>& K, const Mat_<double>& Kinv, const Mat_<double>& P, const Mat_<double>& P1, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, double& error);
	Mat_<double> triangulate(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1);
	Mat_<double> iterativeTriangulation(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1);

	void sampson(Mat_<double>& F, std::vector<Point2f>& pts1, std::vector<Point2f>& pts2);
	void bundleAdjustment(const Mat_<double>& F, const Mat_<double>& P1, const Mat_<double>& P2, const Mat_<double>& K, const std::vector<Point2f>& pts1, const std::vector<Point2f>& pts2, std::vector<Point3d>& pts3d);
	bool decomposeEtoRandT(const Mat_<double>& E, Mat_<double>& R1, Mat_<double>& R2, Mat_<double>& t1, Mat_<double>& t2);
};

