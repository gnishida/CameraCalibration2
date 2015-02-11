#include "Reconstruction.h"

Reconstruction::Reconstruction() {
}

cv::Mat Reconstruction::findFundamentalMat(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<uchar>& status) {
	return cv::findFundamentalMat(pts1, pts2, status);
}

cv::Mat Reconstruction::computeEpipole(cv::Mat& F, int whichImage) {
	cv::Mat u, d, v;
	cv::SVD::compute(F, u, d, v);

	cv::Mat e(3, 1, CV_64F);
	if (whichImage == 1) {
		e.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
		e.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
		e.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	} else {
		e.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
		e.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
		e.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	}

	return e;
}

void Reconstruction::computeProjectionMat(cv::Mat& F, cv::Mat& P1, cv::Mat& P2) {
	// e2を計算(F^T e2 = 0より)
	cv::Mat e2 = cv::Mat(3, 1, CV_64F);
	cv::Mat u, d, v;
	cv::SVD::compute(F.t(), u, d, v);
	e2.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
	e2.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
	e2.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);

	// 行列Aを計算(A=[e2]x F)
	cv::Mat e2x = cv::Mat::zeros(3, 3, CV_64F);
	e2x.at<double>(0, 1) = -e2.at<double>(2, 0);
	e2x.at<double>(0, 2) = e2.at<double>(1, 0);
	e2x.at<double>(1, 0) = e2.at<double>(2, 0);
	e2x.at<double>(1, 2) = -e2.at<double>(0, 0);
	e2x.at<double>(2, 0) = -e2.at<double>(1, 0);
	e2x.at<double>(2, 1) = e2.at<double>(0, 0);
	cv::Mat A = e2x * F;

	// 射影行列P1を計算
	P1 = cv::Mat::eye(3, 4, CV_64F);

	// 射影行列P2を計算
	P2 = cv::Mat(3, 4, CV_64F);
	cv::Mat vec(1, 3, CV_64F);
	cv::randu(vec, 0, 10);
	cv::Mat tmpP(P2, cv::Rect(0, 0, 3, 3));
	tmpP = A + e2 * vec;
	double lambda = 1.0;
	P2.at<double>(0, 3) = e2.at<double>(0, 0) * lambda;
	P2.at<double>(1, 3) = e2.at<double>(1, 0) * lambda;
	P2.at<double>(2, 3) = e2.at<double>(2, 0) * lambda;
}

double Reconstruction::unprojectPoints(cv::Mat& P1, cv::Mat& P2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3f>& pts3d) {
	pts3d.clear();
	double error = 0.0;

	for (int i = 0; i < pts1.size(); ++i) {
		cv::Mat L(4, 4, CV_64F);

		L.at<double>(0, 0) = P1.at<double>(0, 0) - pts1[i].x * P1.at<double>(2, 0);
		L.at<double>(0, 1) = P1.at<double>(0, 1) - pts1[i].x * P1.at<double>(2, 1);
		L.at<double>(0, 2) = P1.at<double>(0, 2) - pts1[i].x * P1.at<double>(2, 2);
		L.at<double>(0, 3) = P1.at<double>(0, 3) - pts1[i].x * P1.at<double>(2, 3);

		L.at<double>(1, 0) = P1.at<double>(1, 0) - pts1[i].y * P1.at<double>(2, 0);
		L.at<double>(1, 1) = P1.at<double>(1, 1) - pts1[i].y * P1.at<double>(2, 1);
		L.at<double>(1, 2) = P1.at<double>(1, 2) - pts1[i].y * P1.at<double>(2, 2);
		L.at<double>(1, 3) = P1.at<double>(1, 3) - pts1[i].y * P1.at<double>(2, 3);

		L.at<double>(2, 0) = P2.at<double>(0, 0) - pts2[i].x * P2.at<double>(2, 0);
		L.at<double>(2, 1) = P2.at<double>(0, 1) - pts2[i].x * P2.at<double>(2, 1);
		L.at<double>(2, 2) = P2.at<double>(0, 2) - pts2[i].x * P2.at<double>(2, 2);
		L.at<double>(2, 3) = P2.at<double>(0, 3) - pts2[i].x * P2.at<double>(2, 3);

		L.at<double>(3, 0) = P2.at<double>(1, 0) - pts2[i].y * P2.at<double>(2, 0);
		L.at<double>(3, 1) = P2.at<double>(1, 1) - pts2[i].y * P2.at<double>(2, 1);
		L.at<double>(3, 2) = P2.at<double>(1, 2) - pts2[i].y * P2.at<double>(2, 2);
		L.at<double>(3, 3) = P2.at<double>(1, 3) - pts2[i].y * P2.at<double>(2, 3);

		cv::Mat u, d, v;
		cv::SVD::compute(L, u, d, v);
		std::cout << "V:" << std::endl;
		std::cout << v.row(v.rows - 1) << std::endl;
		cv::Point3f p = cv::Point3f(v.at<double>(v.rows - 1, 0) / v.at<double>(v.rows - 1, 3), v.at<double>(v.rows - 1, 1) / v.at<double>(v.rows - 1, 3), v.at<double>(v.rows - 1, 2) / v.at<double>(v.rows - 1, 3));
		pts3d.push_back(p);
		std::cout << "point:" << p << std::endl;

		// reprojection errorを計算する
		cv::Mat pMat(4, 1, CV_64F);
		pMat.at<double>(0, 0) = p.x;
		pMat.at<double>(1, 0) = p.y;
		pMat.at<double>(2, 0) = p.z;
		pMat.at<double>(3, 0) = 1.0;

		cv::Mat projected_pt = P1 * pMat;
		cv::Point2f projected_point;
		projected_point.x = projected_pt.at<double>(0, 0) / projected_pt.at<double>(2, 0);
		projected_point.y = projected_pt.at<double>(1, 0) / projected_pt.at<double>(2, 0);
		std::cout << "projected point1: " << projected_point << " (observed: " << pts1[i] << ")" << std::endl;
		error += cv::norm(pts1[i] - projected_point);
		
		projected_pt = P2 * pMat;
		projected_point.x = projected_pt.at<double>(0, 0) / projected_pt.at<double>(2, 0);
		projected_point.y = projected_pt.at<double>(1, 0) / projected_pt.at<double>(2, 0);
		std::cout << "projected point2: " << projected_point << " (observed: " << pts2[i] << ")" << std::endl;
		error += cv::norm(pts2[i] - projected_point);
	}

	return error;
}
