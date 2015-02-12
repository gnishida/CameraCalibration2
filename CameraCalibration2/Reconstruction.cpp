#include "Reconstruction.h"

using namespace cv;

Reconstruction::Reconstruction() {
}

cv::Mat Reconstruction::findFundamentalMat(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<uchar>& status) {
	return cv::findFundamentalMat(pts1, pts2, status);
	//return cv::findFundamentalMat(pts1, pts2, CV_FM_8POINT);
}

cv::Mat Reconstruction::computeEpipole(cv::Mat& F, int whichImage) {
	cv::Mat e(3, 1, CV_64F);
	if (whichImage == 1) {
		cv::Mat u, d, v;
		cv::SVD::compute(F, u, d, v);
		e.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
		e.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
		e.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	} else {
		cv::Mat u, d, v;
		cv::SVD::compute(F.t(), u, d, v);
		e.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
		e.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
		e.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	}

	return e;
}

void Reconstruction::computeProjectionMat(cv::Mat& E, cv::Mat& P1, cv::Mat& P2) {
	/*
	// e2を計算(F^T e2 = 0より)
	cv::Mat e2 = cv::Mat(3, 1, CV_64F);
	cv::Mat u, d, v;
	cv::SVD::compute(F.t(), u, d, v);
	e2.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
	e2.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
	e2.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	cv::normalize(e2, e2);

	std::cout << "e2:\n" << e2 << std::endl;

	// 行列Aを計算(A=[e2]x F)
	cv::Mat e2x = cv::Mat::zeros(3, 3, CV_64F);
	e2x.at<double>(0, 1) = -e2.at<double>(2, 0);
	e2x.at<double>(0, 2) = e2.at<double>(1, 0);
	e2x.at<double>(1, 0) = e2.at<double>(2, 0);
	e2x.at<double>(1, 2) = -e2.at<double>(0, 0);
	e2x.at<double>(2, 0) = -e2.at<double>(1, 0);
	e2x.at<double>(2, 1) = e2.at<double>(0, 0);
	cv::Mat A = -e2x * F;
	*/

	// 射影行列P1を計算
	P1 = cv::Mat::eye(3, 4, CV_64F);

	// 射影行列P2を計算
	cv::SVD svd(E);
	std::cout << svd.u << std::endl;
	std::cout << svd.w << std::endl;
	std::cout << svd.vt << std::endl;

	cv::Mat W = (cv::Mat_<double>(3, 3) << 0, -1, 0,
											1, 0, 0,
											0, 0, 1);

	cv::Mat R = svd.u * W * svd.vt;
	cv::Mat t = svd.u.col(2);
	P2 = (cv::Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
									R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
									R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));


	/*
	P2 = (cv::Mat_<double>(3, 4) << A.at<double>(0, 0), A.at<double>(0, 1), A.at<double>(0, 2), e2.at<double>(0, 0),
									A.at<double>(1, 0), A.at<double>(1, 1), A.at<double>(1, 2), e2.at<double>(1, 0),
									A.at<double>(2, 0), A.at<double>(2, 1), A.at<double>(2, 2), e2.at<double>(2, 0));
									*/
	/*
	cv::Mat vec(1, 3, CV_64F);
	cv::randu(vec, 0, 10);
	cv::Mat tmpP(P2, cv::Rect(0, 0, 3, 3));
	tmpP = A + e2 * vec;
	double lambda = 1.0;
	P2.at<double>(0, 3) = e2.at<double>(0, 0) * lambda;
	P2.at<double>(1, 3) = e2.at<double>(1, 0) * lambda;
	P2.at<double>(2, 3) = e2.at<double>(2, 0) * lambda;
	*/
}

double Reconstruction::unprojectPoints(cv::Mat& cameraMatrix, cv::Mat& P1, cv::Mat& P2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3f>& pts3d) {
	pts3d.clear();
	double error = 0.0;
	cv::Mat Kinv = cameraMatrix.inv();

	for (int i = 0; i < pts1.size(); ++i) {
		cv::Mat L(4, 3, CV_64F);
		cv::Mat b(4, 1, CV_64F);

		cv::Mat p1 = (cv::Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1.0);
		cv::Mat p2 = (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, 1.0);

		cv::Mat p1b = Kinv * p1;
		cv::Mat p2b = Kinv * p2;

		std::cout << "P1:\n" << P1 << std::endl;
		std::cout << "p1b:\n" << p1b << std::endl;
		std::cout << "P2:\n" << P2 << std::endl;
		std::cout << "p2b:\n" << p2b << std::endl;


		//cv::Mat X = iterativeTriangulation(cv::Point2f(p1.at<double>(0, 0), p1.at<double>(1, 0)), P1, cv::Point2f(p2.at<double>(0, 0), p2.at<double>(1, 0)), P2);
		cv::Mat X = triangulate(cv::Point2f(p1b.at<double>(0, 0), p1b.at<double>(1, 0)), P1, cv::Point2f(p2b.at<double>(0, 0), p2b.at<double>(1, 0)), P2);

		std::cout << "X:\n" << X << std::endl;
		cv::Point3f p = cv::Point3f(X.at<double>(0, 0), X.at<double>(1, 0), X.at<double>(2, 0));
		pts3d.push_back(p);
		std::cout << "point:" << p << std::endl;

		// reprojection errorを計算する
		cv::Mat pMat(4, 1, CV_64F);
		pMat.at<double>(0, 0) = p.x;
		pMat.at<double>(1, 0) = p.y;
		pMat.at<double>(2, 0) = p.z;
		pMat.at<double>(3, 0) = 1.0;

		cv::Mat projected_pt = cameraMatrix * P1 * pMat;
		std::cout << "projected_pt:\n" << projected_pt << std::endl;
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

cv::Mat Reconstruction::triangulate(cv::Point2f& pt1, cv::Mat& P1, cv::Point2f& pt2, cv::Mat& P2) {
	cv::Mat A = (cv::Mat_<double>(4, 3) << pt1.x * P1.at<double>(2, 0) - P1.at<double>(0, 0), pt1.x * P1.at<double>(2, 1) - P1.at<double>(0, 1), pt1.x * P1.at<double>(2, 2) - P1.at<double>(0, 2),
											pt1.y * P1.at<double>(2, 0) - P1.at<double>(1, 0), pt1.y * P1.at<double>(2, 1) - P1.at<double>(1, 1), pt1.y * P1.at<double>(2, 2) - P1.at<double>(1, 2),
											pt2.x * P2.at<double>(2, 0) - P2.at<double>(0, 0), pt2.x * P2.at<double>(2, 1) - P2.at<double>(0, 1), pt2.x * P2.at<double>(2, 2) - P2.at<double>(0, 2),
											pt2.y * P2.at<double>(2, 0) - P2.at<double>(1, 0), pt2.y * P2.at<double>(2, 1) - P2.at<double>(1, 1), pt2.y * P2.at<double>(2, 2) - P2.at<double>(1, 2));

	cv::Mat b = (cv::Mat_<double>(4, 1) << -(pt1.x * P1.at<double>(2, 3) - P1.at<double>(0, 3)),
											-(pt1.y * P1.at<double>(2, 3) - P1.at<double>(1, 3)),
											-(pt2.x * P1.at<double>(2, 3) - P2.at<double>(0, 3)),
											-(pt2.y * P1.at<double>(2, 3) - P2.at<double>(1, 3)));
 
    cv::Mat X;
    cv::solve(A, b, X, cv::DECOMP_SVD);
 
    return X;
}



cv::Mat Reconstruction::iterativeTriangulation(cv::Point2f& pt1, cv::Mat& P1, cv::Point2f& pt2, cv::Mat& P2) {
	double wi1 = 1, wi2 = 1;
    cv::Mat X(4, 1, CV_64F);
	for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
		cv::Mat X_ = triangulate(pt1, P1, pt2, P2);
		X.at<double>(0, 0) = X_.at<double>(0, 0);
		X.at<double>(1, 0) = X_.at<double>(1, 0);
		X.at<double>(2, 0) = X_.at<double>(2, 0);
		X.at<double>(3, 0) = 1.0;
         
        //recalculate weights
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);
        double p2x2 = cv::Mat_<double>(cv::Mat_<double>(P2).row(2)*X)(0);
         
        //breaking point
        if(fabsf(wi1 - p2x1) <= 1e-7 && fabsf(wi2 - p2x2) <= 1e-7) break;
         
        wi1 = p2x1;
        wi2 = p2x2;
         
        //reweight equations and solve
        cv::Mat A = (cv::Mat_<double>(4, 3) <<
					(pt1.x*P1.at<double>(2,0)-P1.at<double>(0,0))/wi1, (pt1.x*P1.at<double>(2,1)-P1.at<double>(0,1))/wi1, (pt1.x*P1.at<double>(2,2)-P1.at<double>(0,2))/wi1,
					(pt1.y*P1.at<double>(2,0)-P1.at<double>(1,0))/wi1, (pt1.y*P1.at<double>(2,1)-P1.at<double>(1,1))/wi1, (pt1.y*P1.at<double>(2,2)-P1.at<double>(1,2))/wi1,
					(pt2.x*P2.at<double>(2,0)-P2.at<double>(0,0))/wi2, (pt2.x*P2.at<double>(2,1)-P2.at<double>(0,1))/wi2, (pt2.x*P2.at<double>(2,2)-P2.at<double>(0,2))/wi2,
					(pt2.y*P2.at<double>(2,0)-P2.at<double>(1,0))/wi2, (pt2.y*P2.at<double>(2,1)-P2.at<double>(1,1))/wi2, (pt2.y*P2.at<double>(2,2)-P2.at<double>(1,2))/wi2);
        cv::Mat B = (cv::Mat_<double>(4,1) <<
						-(pt1.x*P1.at<double>(2,3) - P1.at<double>(0,3)) / wi2,
                        -(pt1.y*P1.at<double>(2,3) - P1.at<double>(1,3)) / wi1,
                        -(pt2.x*P2.at<double>(2,3) - P2.at<double>(0,3)) / wi2,
                        -(pt2.y*P2.at<double>(2,3) - P2.at<double>(1,3)) / wi2);
         
        cv::solve(A,B,X_, cv::DECOMP_SVD);
        X.at<double>(0, 0) = X_.at<double>(0, 0);
		X.at<double>(1, 0) = X_.at<double>(1, 0);
		X.at<double>(2, 0) = X_.at<double>(2, 0);
		X.at<double>(3, 0) = 1.0;
    }
    return X;
}
