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

void Reconstruction::computeProjectionMat(Matx33d E, Mat_<double>& R1, Mat_<double>& T1, Mat_<double>& R2, Mat_<double>& T2) {
	SVD svd(E);
	Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
	Matx33d Winv(0, 1, 0, -1, 0, 0, 0, 0, 1);
	R1 = svd.u * Mat(W) * svd.vt;
	R2 = svd.u * Mat(W.t()) * svd.vt;
	T1 = svd.u.col(2);
	T2 = -svd.u.col(2);
}

double Reconstruction::unprojectPoints(Size& size, cv::Mat_<double>& K, const Mat_<double>& R1, const Mat_<double>& T1, const Mat_<double>& R2, const Mat_<double>& T2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d) {
	std::cout << K << std::endl;
	cv::Mat_<double> Kinv = K.inv();
	std::cout << Kinv << std::endl;

	int numFront = 0;
	cv::Matx34d P(1, 0, 0, 0, 
				  0, 1, 0, 0,
				  0, 0, 1, 0);

	// 第1候補のチェック
	cv::Matx34d P1(R1(0, 0), R1(0, 1), R1(0, 2), T1(0, 0),
				  R1(1, 0), R1(1, 1), R1(1, 2), T1(1, 0),
				  R1(2, 0), R1(2, 1), R1(2, 2), T1(2, 0));	
	double error = unprojectPoints(size, K, Kinv, P, P1, pts1, pts2, pts3d, numFront);	
	if (numFront > pts1.size() * 0.5) return error;

	// 第2候補のチェック
	P1 = Matx34d(R1(0, 0), R1(0, 1), R1(0, 2), T2(0, 0),
				 R1(1, 0), R1(1, 1), R1(1, 2), T2(1, 0),
				 R1(2, 0), R1(2, 1), R1(2, 2), T2(2, 0));
	error = unprojectPoints(size, K, Kinv, P, P1, pts1, pts2, pts3d, numFront);	
	//if (numFront > pts1.size() * 0.5) return error;

	// 第3候補のチェック
	P1 = Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), T1(0, 0),
				 R2(1, 0), R2(1, 1), R2(1, 2), T1(1, 0),
				 R2(2, 0), R2(2, 1), R2(2, 2), T1(2, 0));
	error = unprojectPoints(size, K, Kinv, P, P1, pts1, pts2, pts3d, numFront);	
	//if (numFront > pts1.size() * 0.5) return error;
	return error;

	// 第4候補のチェック
	P1 = Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), T2(0, 0),
				 R2(1, 0), R2(1, 1), R2(1, 2), T2(1, 0),
				 R2(2, 0), R2(2, 1), R2(2, 2), T2(2, 0));
	error = unprojectPoints(size, K, Kinv, P, P1, pts1, pts2, pts3d, numFront);	
	if (numFront > pts1.size() * 0.5) return error;

	return error;
}

double Reconstruction::unprojectPoints(Size& size, Mat_<double>& K, Mat_<double>& Kinv, const Matx34d& P, const Matx34d& P1, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, int& numFront) {
	pts3d.clear();
	std::vector<double> error;

	numFront = 0;

	for (int i = 0; i < pts1.size(); ++i) {
		Point3d u(pts1[i].x, pts1[i].y, 1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u.x = um(0); u.y = um(1); u.z = um(2);
		
		Point3d u1(pts2[i].x, pts2[i].y, 1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		Matx41d X = iterativeTriangulation(u, P, u1, P1);

		std::cout << "X:\n" << X << std::endl;
		cv::Point3d p = cv::Point3d(X(0), X(1), X(2));
		pts3d.push_back(p);
		std::cout << "point:" << p << std::endl;
		
		// reprojection errorを計算する
		cv::Mat_<double> pt1_3d_hat = K * Mat_<double>(P) * Mat_<double>(X);
		std::cout << "projection to image1: " << pt1_3d_hat << std::endl;
		Point2f pt1_hat(pt1_3d_hat(0, 0) / pt1_3d_hat(2, 0), pt1_3d_hat(1, 0) / pt1_3d_hat(2, 0));
		std::cout << "projected point1: " << pt1_hat << " (observed: " << pts1[i] << ")" << std::endl;
		error.push_back(norm(pt1_hat - pts1[i]));

		cv::Mat_<double> pt2_3d_hat = K * Mat_<double>(P1) * Mat_<double>(X);
		std::cout << "projection to image1: " << pt2_3d_hat << std::endl;
		Point2f pt2_hat(pt2_3d_hat(0, 0) / pt2_3d_hat(2, 0), pt2_3d_hat(1, 0) / pt2_3d_hat(2, 0));
		std::cout << "projected point2: " << pt2_hat << " (observed: " << pts2[i] << ")" << std::endl;
		error.push_back(norm(pt2_hat - pts1[i]));

		if ((P * X)(2, 0) > 0 && (P1 * X)(2, 0) > 0) {
			numFront++;
		}
	}

	return mean(error)[0];
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
