#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>

int main (int argc, char *argv[]) {
	cv::Mat img[2];
	img[0] = cv::imread("images/image1.jpg");
	img[1] = cv::imread("images/image2.jpg");

	std::vector<cv::Point2f> pts[2];

	// とりあえず、対応点を手動で設定
	{
		pts[0].push_back(cv::Point2f(96, 131));
		pts[1].push_back(cv::Point2f(110, 166));
		pts[0].push_back(cv::Point2f(234, 51));
		pts[1].push_back(cv::Point2f(217, 79));
		pts[0].push_back(cv::Point2f(467, 129));
		pts[1].push_back(cv::Point2f(467, 134));
		pts[0].push_back(cv::Point2f(327, 248));
		pts[1].push_back(cv::Point2f(388, 251));
		pts[0].push_back(cv::Point2f(104, 160));
		pts[1].push_back(cv::Point2f(116, 193));
		pts[0].push_back(cv::Point2f(333, 279));
		pts[1].push_back(cv::Point2f(390, 280));
		pts[0].push_back(cv::Point2f(468, 158));
		pts[1].push_back(cv::Point2f(468, 159));
		pts[0].push_back(cv::Point2f(128, 220));
		pts[1].push_back(cv::Point2f(136, 251));
		pts[0].push_back(cv::Point2f(463, 228));
		pts[1].push_back(cv::Point2f(458, 225));
		pts[0].push_back(cv::Point2f(340, 345));
		pts[1].push_back(cv::Point2f(388, 340));
	}

	// Fundamental matrixを計算
	std::vector<cv::Point2f> pts_inlier[2];
	std::vector<uchar> status;
	cv::Mat F = cv::findFundamentalMat(pts[0], pts[1], status);

	// e1, e2を計算(F^T e2 = 0より)
	cv::Mat e[2];
	for (int i = 0; i < 2; ++i) e[i] = cv::Mat(3, 1, CV_64F);
	cv::Mat u, d, v;
	cv::SVD::compute(F, u, d, v);
	e[0].at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
	e[0].at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
	e[0].at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	cv::SVD::compute(F.t(), u, d, v);
	e[1].at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
	e[1].at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
	e[1].at<double>(2, 0) = v.at<double>(v.rows - 1, 2);

	// 行列Aを計算(A=[e2]x F)
	cv::Mat e2x = cv::Mat::zeros(3, 3, CV_64F);
	e2x.at<double>(0, 1) = -e[1].at<double>(2, 0);
	e2x.at<double>(0, 2) = e[1].at<double>(1, 0);
	e2x.at<double>(1, 0) = e[1].at<double>(2, 0);
	e2x.at<double>(1, 2) = -e[1].at<double>(0, 0);
	e2x.at<double>(2, 0) = -e[1].at<double>(1, 0);
	e2x.at<double>(2, 1) = e[1].at<double>(0, 0);
	cv::Mat A = e2x * F;

	// 行列P1, P2の計算
	cv::Mat P[2];
	P[0] = cv::Mat::eye(3, 4, CV_64F);
	std::cout << "P1:\n" << P[0] << std::endl;
	P[1] = cv::Mat(3, 4, CV_64F);
	cv::Mat vec(1, 3, CV_64F);
	cv::randu(vec, 0, 10);
	cv::Mat tmpP(P[1], cv::Rect(0, 0, 3, 3));
	tmpP = A + e[1] * vec;
	double lambda = 1.0;
	P[1].at<double>(0, 3) = e[1].at<double>(0, 0) * lambda;
	P[1].at<double>(1, 3) = e[1].at<double>(1, 0) * lambda;
	P[1].at<double>(2, 3) = e[1].at<double>(2, 0) * lambda;
	std::cout << "P2:\n" << P[1] << std::endl;

	// 3D座標を計算する
	std::vector<cv::Point3f> pts3d;
	for (int i = 0; i < pts[0].size(); ++i) {
		cv::Mat L(4, 4, CV_64F);
		for (int j = 0; j < 2; ++j) {
			L.at<double>(j * 2 + 0, 0) = P[j].at<double>(0, 0) - pts[j][i].x * P[j].at<double>(2, 0);
			L.at<double>(j * 2 + 0, 1) = P[j].at<double>(0, 1) - pts[j][i].x * P[j].at<double>(2, 1);
			L.at<double>(j * 2 + 0, 2) = P[j].at<double>(0, 2) - pts[j][i].x * P[j].at<double>(2, 2);
			L.at<double>(j * 2 + 0, 3) = P[j].at<double>(0, 3) - pts[j][i].x * P[j].at<double>(2, 3);

			L.at<double>(j * 2 + 1, 0) = P[j].at<double>(1, 0) - pts[j][i].y * P[j].at<double>(2, 0);
			L.at<double>(j * 2 + 1, 1) = P[j].at<double>(1, 1) - pts[j][i].y * P[j].at<double>(2, 1);
			L.at<double>(j * 2 + 1, 2) = P[j].at<double>(1, 2) - pts[j][i].y * P[j].at<double>(2, 2);
			L.at<double>(j * 2 + 1, 3) = P[j].at<double>(1, 3) - pts[j][i].y * P[j].at<double>(2, 3);
		}

		cv::SVD::compute(L, u, d, v);
		cv::Point3f p = cv::Point3f(v.at<double>(v.rows - 1, 0) / v.at<double>(v.rows - 1, 3), v.at<double>(v.rows - 1, 1) / v.at<double>(v.rows - 1, 3), v.at<double>(v.rows - 1, 2) / v.at<double>(v.rows - 1, 3));
		pts3d.push_back(p);

		// reprojection errorを計算する
		cv::Mat pMat(4, 1, CV_64F);
		pMat.at<double>(0, 0) = p.x;
		pMat.at<double>(1, 0) = p.y;
		pMat.at<double>(2, 0) = p.z;
		pMat.at<double>(3, 0) = 1.0;
		cv::Point2f projected_point[2];
		for (int j = 0; j < 2; ++j) {
			cv::Mat projected_pt = P[j] * pMat;
			projected_point[j].x = projected_pt.at<double>(0, 0) / projected_pt.at<double>(2, 0);
			projected_point[j].y = projected_pt.at<double>(1, 0) / projected_pt.at<double>(2, 0);
		}
		double error = cv::norm(pts[0][i] - projected_point[0]) + cv::norm(pts[1][i] - projected_point[1]);
		printf("point 1 (error=%lf):\n", error);
		printf("(%lf, %lf) <-> (%lf, %lf)\n", pts[0][i].x, pts[0][i].y, projected_point[0].x, projected_point[0].y);
		printf("(%lf, %lf) <-> (%lf, %lf)\n", pts[1][i].x, pts[1][i].y, projected_point[1].x, projected_point[1].y);
	}
	
	// outlierの点を除外する
	for (int i = 0; i < pts[0].size(); ++i) {
		if (status[i] == 1) {
			pts_inlier[0].push_back(pts[0][i]);
			pts_inlier[1].push_back(pts[1][i]);
		}
	}

	// epilineの描画
	std::vector<cv::Point3f> lines[2];
	for (int n = 0; n < 2; ++n) {
		cv::computeCorrespondEpilines(pts_inlier[1-n], 2-n, F, lines[n]);
		for (int i = 0; i < pts_inlier[n].size(); ++i) {
			float x1 = 0;
			float y1 = -lines[n][i].z / lines[n][i].y;
			float x2 = img[n].cols - 1;
			float y2 = -(lines[n][i].x * (img[n].cols - 1) + lines[n][i].z) / lines[n][i].y;

			int r = i * 10 % 255;
			int g = i * 20 % 255;
			int b = i * 40 % 255;
			cv::line(img[n], cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(b, g, r), 1);
			cv::circle(img[n], pts_inlier[n][i], 5, cv::Scalar(b, g, r), 2);
		}

		cv::namedWindow("Epiline", CV_WINDOW_AUTOSIZE);
		char filename[255];
		sprintf(filename, "result_%d.jpg", n);
		cv::imwrite(filename, img[n]);
		cv::imshow("Epiline", img[n]);
		cv::waitKey(0);
	}

	return 0;
}

