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

double Reconstruction::unprojectPoints(cv::Matx33d& cameraMatrix, const cv::Matx34d& P, const cv::Matx34d& P1, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d) {
	pts3d.clear();
	double error = 0.0;
	std::cout << cameraMatrix << std::endl;
	cv::Matx33d Kinv = cameraMatrix.inv();
	std::cout << Kinv << std::endl;


	for (int i = 0; i < pts1.size(); ++i) {
		Point3d u(pts1[i].x, pts1[i].y, 1.0);
		Matx31d um = Kinv * Matx31d(u);
		u.x = um(0); u.y = um(1); u.z = um(2);
		
		Point3d u1(pts2[i].x, pts2[i].y, 1.0);
		Matx31d um1 = Kinv * Matx31d(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		Matx41d X = iterativeTriangulation(u, P, u1, P1);

		std::cout << "X:\n" << X << std::endl;
		cv::Point3d p = cv::Point3d(X(0), X(1), X(2));
		pts3d.push_back(p);
		std::cout << "point:" << p << std::endl;
		
		// reprojection errorを計算する
		cv::Matx41d pMat(p.x, p.y, p.z, 1.0);

		cv::Matx31d projected_pt = P * pMat;
		cv::Point2f projected_point;
		projected_point.x = projected_pt(0, 0) / projected_pt(2, 0);
		projected_point.y = projected_pt(1, 0) / projected_pt(2, 0);
		std::cout << "projected point1: " << projected_point << " (observed: " << um << ")" << std::endl;
		error += cv::norm(pts1[i] - projected_point);
		
		projected_pt = P1 * pMat;
		projected_point.x = projected_pt(0, 0) / projected_pt(2, 0);
		projected_point.y = projected_pt(1, 0) / projected_pt(2, 0);
		std::cout << "projected point2: " << projected_point << " (observed: " << um1 << ")" << std::endl;
		error += cv::norm(pts2[i] - projected_point);
	}

	return error;
}

Matx31d Reconstruction::triangulate(cv::Point3d u, Matx34d P, Point3d u1, Matx34d P1) {
    Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
          u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
          u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
          u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3)),
                      -(u.y*P(2,3)  -P(1,3)),
                      -(u1.x*P1(2,3)    -P1(0,3)),
                      -(u1.y*P1(2,3)    -P1(1,3)));
 
    Matx31d X;
    solve(A,B,X,DECOMP_SVD);
	std::cout << X << std::endl;

    return X;
}



cv::Matx41d Reconstruction::iterativeTriangulation(cv::Point3d u, Matx34d P, cv::Point3d u1, Matx34d P1) {
   double wi = 1, wi1 = 1;
    Matx41d X(4,1);
    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        Matx31d X_ = triangulate(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
         
        //recalculate weights
        double p2x = Mat_<double>(P.row(2)*X)(0);
        double p2x1 = Mat_<double>(P1.row(2)*X)(0);
         
        //breaking point
        if(fabsf(wi - p2x) <= 1e-7 && fabsf(wi1 - p2x1) <= 1e-7) break;
         
        wi = p2x;
        wi1 = p2x1;
         
        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,    
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,    
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
         
        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }
    return X;
}
