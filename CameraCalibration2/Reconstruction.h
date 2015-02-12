#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>

class Reconstruction {
public:
	Reconstruction();

	cv::Mat findFundamentalMat(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<uchar>& status);
	cv::Mat computeEpipole(cv::Mat& F, int whichImage);
	void computeProjectionMat(cv::Mat& E, cv::Mat& P1, cv::Mat& P2);

	double unprojectPoints(cv::Mat& cameraMatrix, cv::Mat& P1, cv::Mat& P2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3f>& pts3d);
	cv::Mat triangulate(cv::Point2f& pt1, cv::Mat& P1, cv::Point2f& pt2, cv::Mat& P2);
	cv::Mat iterativeTriangulation(cv::Point2f& pt1, cv::Mat& P1, cv::Point2f& pt2, cv::Mat& P2);
};

