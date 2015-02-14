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

double Reconstruction::unprojectPoints(const Mat_<double>& K, const Mat_<double>& R1, const Mat_<double>& T1, const Mat_<double>& R2, const Mat_<double>& T2, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, Mat_<double>& P1, Mat_<double>& P2) {
	cv::Mat_<double> Kinv = K.inv();

	double error = 0.0;

	P1 = (Mat_<double>(3, 4) << 1, 0, 0, 0, 
							  0, 1, 0, 0,
							  0, 0, 1, 0);

	std::cout << P1 << std::endl;

	// 第1候補のチェック
	P2 = (Mat_<double>(3, 4) << R1(0, 0), R1(0, 1), R1(0, 2), T1(0, 0),
				 				R1(1, 0), R1(1, 1), R1(1, 2), T1(1, 0),
								R1(2, 0), R1(2, 1), R1(2, 2), T1(2, 0));	
	std::cout << P2 << std::endl;
	if (unprojectPoints(K, Kinv, P1, P2, pts1, pts2, pts3d, error)) return error;

	// 第2候補のチェック
	P2 = (Mat_<double>(3, 4) << R1(0, 0), R1(0, 1), R1(0, 2), T2(0, 0),
        						R1(1, 0), R1(1, 1), R1(1, 2), T2(1, 0),
								R1(2, 0), R1(2, 1), R1(2, 2), T2(2, 0));
	if (unprojectPoints(K, Kinv, P1, P2, pts1, pts2, pts3d, error)) return error;

	// 第3候補のチェック
	P2 = (Mat_<double>(3, 4) << R2(0, 0), R2(0, 1), R2(0, 2), T1(0, 0),
								R2(1, 0), R2(1, 1), R2(1, 2), T1(1, 0),
								R2(2, 0), R2(2, 1), R2(2, 2), T1(2, 0));
	if (unprojectPoints(K, Kinv, P1, P2, pts1, pts2, pts3d, error)) return error;

	// 第4候補のチェック
	P2 = (Mat_<double>(3, 4) << R2(0, 0), R2(0, 1), R2(0, 2), T2(0, 0),
								R2(1, 0), R2(1, 1), R2(1, 2), T2(1, 0),
								R2(2, 0), R2(2, 1), R2(2, 2), T2(2, 0));
	if (unprojectPoints(K, Kinv, P1, P2, pts1, pts2, pts3d, error)) return error;

	return error;
}

bool Reconstruction::unprojectPoints(const Mat_<double>& K, const Mat_<double>& Kinv, const Mat_<double>& P, const Mat_<double>& P1, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, double& error) {
	pts3d.clear();
	std::vector<double> errors;

	int numFront = 0;

	for (int i = 0; i < pts1.size(); ++i) {
		Point3d u(pts1[i].x, pts1[i].y, 1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u.x = um(0); u.y = um(1); u.z = um(2);
		
		Point3d u1(pts2[i].x, pts2[i].y, 1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		Mat_<double> X = iterativeTriangulation(u, P, u1, P1);

		std::cout << "X:\n" << X << std::endl;
		cv::Point3d p = cv::Point3d(X(0), X(1), X(2));
		pts3d.push_back(p);
		
		// reprojection errorを計算する
		cv::Mat_<double> pt1_3d_hat = K * Mat_<double>(P) * Mat_<double>(X);
		Point2f pt1_hat(pt1_3d_hat(0, 0) / pt1_3d_hat(2, 0), pt1_3d_hat(1, 0) / pt1_3d_hat(2, 0));
		std::cout << "projected point1: " << pt1_hat << " (observed: " << pts1[i] << ") E=" << norm(pt1_hat - pts1[i]) << std::endl;
		errors.push_back(norm(pt1_hat - pts1[i]));

		cv::Mat_<double> pt2_3d_hat = K * Mat_<double>(P1) * Mat_<double>(X);
		Point2f pt2_hat(pt2_3d_hat(0, 0) / pt2_3d_hat(2, 0), pt2_3d_hat(1, 0) / pt2_3d_hat(2, 0));
		std::cout << "projected point2: " << pt2_hat << " (observed: " << pts2[i] << ") E=" << norm(pt2_hat - pts1[i]) << std::endl;
		errors.push_back(norm(pt2_hat - pts1[i]));

		Mat_<double> x1 = P * X;
		Mat_<double> x2 = P1 * X;
		if (x1(2, 0) > 0 && x2(2, 0) > 0) {
			numFront++;
		}
	}

	error = mean(errors)[0];

	if (numFront > (float)pts1.size() * 0.75) return true;
	else false;
}

Mat_<double> Reconstruction::triangulate(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1) {
    Mat_<double> A = (Mat_<double>(4, 3) << u.x*P(2,0)-P(0,0), u.x*P(2,1)-P(0,1), u.x*P(2,2)-P(0,2),
											u.y*P(2,0)-P(1,0), u.y*P(2,1)-P(1,1), u.y*P(2,2)-P(1,2),
											u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1), u1.x*P1(2,2)-P1(0,2),
											u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1), u1.y*P1(2,2)-P1(1,2));
    Mat_<double> B = (Mat_<double>(4, 1) << -(u.x*P(2,3) - P(0,3)),
											-(u.y*P(2,3) - P(1,3)),
											-(u1.x*P1(2,3) - P1(0,3)),
											-(u1.y*P1(2,3) - P1(1,3)));
 
    Mat_<double> X;
    solve(A,B,X,DECOMP_SVD);

    return X;
}

Mat_<double> Reconstruction::iterativeTriangulation(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1) {
	double wi = 1, wi1 = 1;
	Mat_<double> X(4,1);
    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = triangulate(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
         
        //recalculate weights
        double p2x = Mat_<double>(P.row(2)*X)(0);
        double p2x1 = Mat_<double>(P1.row(2)*X)(0);
         
        //breaking point
        if(fabsf(wi - p2x) <= 1e-7 && fabsf(wi1 - p2x1) <= 1e-7) break;
         
        wi = p2x;
        wi1 = p2x1;
         
        //reweight equations and solve
        Mat_<double> A = (Mat_<double>(4, 3) << (u.x*P(2,0)-P(0,0))/wi, (u.x*P(2,1)-P(0,1))/wi, (u.x*P(2,2)-P(0,2))/wi,
												(u.y*P(2,0)-P(1,0))/wi, (u.y*P(2,1)-P(1,1))/wi, (u.y*P(2,2)-P(1,2))/wi,
												(u1.x*P1(2,0)-P1(0,0))/wi1, (u1.x*P1(2,1)-P1(0,1))/wi1, (u1.x*P1(2,2)-P1(0,2))/wi1,
												(u1.y*P1(2,0)-P1(1,0))/wi1, (u1.y*P1(2,1)-P1(1,1))/wi1, (u1.y*P1(2,2)-P1(1,2))/wi1);
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

void Reconstruction::sampson(Mat_<double>& F, std::vector<Point2f>& pts1, std::vector<Point2f>& pts2) {
	for (int i = 0; i < pts1.size(); ++i) {
		Mat_<double> x1 = (Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1);
		Mat_<double> x2 = (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, 1);

		Mat_<double> xFx = x2.t() * F * x1;
		std::cout << xFx << std::endl;
		double Fx1_1 = F(0, 0) * pts1[i].x + F(0, 1) * pts1[i].y + F(0, 2);
		double Fx1_2 = F(1, 0) * pts1[i].x + F(1, 1) * pts1[i].y + F(1, 2);
		double Fx2_1 = F(0, 0) * pts2[i].x + F(1, 0) * pts2[i].y + F(2, 0);
		double Fx2_2 = F(0, 1) * pts2[i].x + F(1, 1) * pts2[i].y + F(2, 1);

		double denom = Fx1_1 * Fx1_1 + Fx1_2 * Fx1_2 + Fx2_1 * Fx2_1 + Fx2_2 * Fx2_2;

		pts1[i].x -= xFx(0, 0) / denom * Fx2_1;
		pts1[i].y -= xFx(0, 0) / denom * Fx2_2;
		pts2[i].x -= xFx(0, 0) / denom * Fx1_1;
		pts2[i].x -= xFx(0, 0) / denom * Fx1_2;
	}
}

void Reconstruction::bundleAdjustment(const Mat_<double>& F, const Mat_<double>& P1, const Mat_<double>& P2, const Mat_<double>& K, const std::vector<Point2f>& pts1, const std::vector<Point2f>& pts2, std::vector<Point3d>& pts3d) {
}

bool Reconstruction::decomposeEtoRandT(const Mat_<double>& E, Mat_<double>& R1, Mat_<double>& R2, Mat_<double>& t1, Mat_<double>& t2) {
	SVD svd(E);

	//check if first and second singular values are the same (as they should be)
	double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1));
	if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
	if (singular_values_ratio < 0.7) {
		std::cout << "singular values are too far apart\n";
		return false;
	}
	Matx33d W(0,-1,0, //HZ 9.13
			  1,0,0,
			  0,0,1);
	Matx33d Wt(0,1,0,
			  -1,0,0,
			   0,0,1);

	R1 = svd.u * Mat(W) * svd.vt; //HZ 9.19
	R2 = svd.u * Mat(Wt) * svd.vt; //HZ 9.19
	t1 = svd.u.col(2); //u3
	t2 = -svd.u.col(2); //u3

	return true;
}
