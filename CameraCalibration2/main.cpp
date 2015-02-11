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
	for (int i = 0; i < pts[0].size(); ++i) {
		if (status[i] == 1) {
			pts_inlier[0].push_back(pts[0][i]);
			pts_inlier[1].push_back(pts[1][i]);
		}
	}

	// epilineの計算
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
			cv::circle(img[n], pts_inlier[n][i], 5, cv::Scalar(b, g, r), 3);
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

