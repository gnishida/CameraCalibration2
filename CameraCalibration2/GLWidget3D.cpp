#include <iostream>
#include <fstream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>
#include "Reconstruction.h"
#include <opencv2/nonfree/features2d.hpp>

#define SQR(x)	((x) * (x))

using namespace cv;

GLWidget3D::GLWidget3D(MainWindow* mainWin) {
	this->mainWin = mainWin;

	// set up the camera
	camera.setLookAt(0.0f, 0.0f, 0.0f);
	camera.setYRotation(0);
	camera.setTranslation(0.0f, 0.0f, 1500.0f);
}

/**
 * This event handler is called when the mouse press events occur.
 */
void GLWidget3D::mousePressEvent(QMouseEvent *e)
{
	lastPos = e->pos();
}

/**
 * This event handler is called when the mouse release events occur.
 */
void GLWidget3D::mouseReleaseEvent(QMouseEvent *e)
{
	updateGL();
}

/**
 * This event handler is called when the mouse move events occur.
 */
void GLWidget3D::mouseMoveEvent(QMouseEvent *e)
{
	float dx = (float)(e->x() - lastPos.x());
	float dy = (float)(e->y() - lastPos.y());
	lastPos = e->pos();

	if (e->buttons() & Qt::LeftButton) {
		camera.changeXRotation(dy);
		camera.changeYRotation(dx);
	} else if (e->buttons() & Qt::RightButton) {
		camera.changeXYZTranslation(0, 0, -dy * camera.dz * 0.02f);
		if (camera.dz < -9000) camera.dz = -9000;
		if (camera.dz > 9000) camera.dz = 9000;
	} else if (e->buttons() & Qt::MidButton) {
		camera.changeXYZTranslation(-dx, dy, 0);
	}

	updateGL();
}

/**
 * This function is called once before the first call to paintGL() or resizeGL().
 */
void GLWidget3D::initializeGL()
{
	glClearColor(0.443, 0.439, 0.458, 0.0);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	static GLfloat lightPosition[4] = {0.0f, 0.0f, 100.0f, 0.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

/**
 * This function is called whenever the widget has been resized.
 */
void GLWidget3D::resizeGL(int width, int height)
{
	height = height?height:1;

	glViewport( 0, 0, (GLint)width, (GLint)height );
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)width/(GLfloat)height, 0.1f, 10000);
	glMatrixMode(GL_MODELVIEW);
}

/**
 * This function is called whenever the widget needs to be painted.
 */
void GLWidget3D::paintGL()
{
	glMatrixMode(GL_MODELVIEW);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	camera.applyCamTransform();

	drawScene();		
}

/**
 * Draw the scene.
 */
void GLWidget3D::drawScene() {
	// ワールド座標系の軸表示
	glPointSize(3);
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(500, 0, 0);
	glEnd();
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 500, 0);
	glEnd();
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 500);
	glEnd();


	if (pts3d.size() > 0) {
		glBegin(GL_TRIANGLES);
		drawTriangle(0, 1, 2);
		drawTriangle(0, 2, 3);
		drawTriangle(4, 5, 6);
		drawTriangle(4, 6, 1);
		drawTriangle(1, 6, 7);
		drawTriangle(1, 7, 8);
		glEnd();
		
		/*
		Subdiv2D subdiv(Rect(0, 0, 3000, 3000));
		for (int i = 0; i < pts3d.size(); ++i) {
			subdiv.insert(Point2f(pts[0][i].x, pts[0][i].y));
		}
		std::vector<Vec6f> triangleList;
		subdiv.getTriangleList(triangleList);
		glBegin(GL_TRIANGLES);
		for (int i = 0; i < triangleList.size(); ++i) {
			int edge = 0;
			int vertex[3] = {0, 0, 0};
			Point2f a(triangleList[i][0], triangleList[i][1]);
			vertex[0] = findPointIndex(pts[0], Point2f(triangleList[i][0], triangleList[i][1]));
			vertex[1] = findPointIndex(pts[0], Point2f(triangleList[i][2], triangleList[i][3]));
			vertex[2] = findPointIndex(pts[0], Point2f(triangleList[i][4], triangleList[i][5]));

			if (vertex[0] >= 0 && vertex[1] >= 0 && vertex[2] >= 0) {
				drawTriangle(vertex[0], vertex[1], vertex[2]);
			}
		}
		glEnd();
		*/
	}
}

QVector2D GLWidget3D::mouseTo2D(int x,int y) {
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];

	// retrieve the matrices
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	float z;
	glReadPixels(x, (float)viewport[3] - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
	
	// unproject the image plane coordinate to the model space
	GLdouble posX, posY, posZ;
	gluUnProject(x, (float)viewport[3] - y, z, modelview, projection, viewport, &posX, &posY, &posZ);

	return QVector2D(posX, posY);
}

void GLWidget3D::drawTriangle(int index1, int index2, int index3) {
	int b = (img[0].at<Vec3b>(pts[0][index1].y, pts[0][index1].x)[0] + img[1].at<Vec3b>(pts[1][index1].y, pts[1][index1].x)[0]) * 0.5;
	int g = (img[0].at<Vec3b>(pts[0][index1].y, pts[0][index1].x)[1] + img[1].at<Vec3b>(pts[1][index1].y, pts[1][index1].x)[1]) * 0.5;
	int r = (img[0].at<Vec3b>(pts[0][index1].y, pts[0][index1].x)[2] + img[1].at<Vec3b>(pts[1][index1].y, pts[1][index1].x)[2]) * 0.5;
	glColor3f((float)r / 255.0, (float)g / 255.0, (float)b / 255.0);
	glVertex3f(pts3d[index1].x, pts3d[index1].y, pts3d[index1].z);

	b = (img[0].at<Vec3b>(pts[0][index2].y, pts[0][index2].x)[0] + img[1].at<Vec3b>(pts[1][index2].y, pts[1][index2].x)[0]) * 0.5;
	g = (img[0].at<Vec3b>(pts[0][index2].y, pts[0][index2].x)[1] + img[1].at<Vec3b>(pts[1][index2].y, pts[1][index2].x)[1]) * 0.5;
	r = (img[0].at<Vec3b>(pts[0][index2].y, pts[0][index2].x)[2] + img[1].at<Vec3b>(pts[1][index2].y, pts[1][index2].x)[2]) * 0.5;
	glColor3f((float)r / 255.0, (float)g / 255.0, (float)b / 255.0);
	glVertex3f(pts3d[index2].x, pts3d[index2].y, pts3d[index2].z);

	b = (img[0].at<Vec3b>(pts[0][index3].y, pts[0][index3].x)[0] + img[1].at<Vec3b>(pts[1][index3].y, pts[1][index3].x)[0]) * 0.5;
	g = (img[0].at<Vec3b>(pts[0][index3].y, pts[0][index3].x)[1] + img[1].at<Vec3b>(pts[1][index3].y, pts[1][index3].x)[1]) * 0.5;
	r = (img[0].at<Vec3b>(pts[0][index3].y, pts[0][index3].x)[2] + img[1].at<Vec3b>(pts[1][index3].y, pts[1][index3].x)[2]) * 0.5;
	glColor3f((float)r / 255.0, (float)g / 255.0, (float)b / 255.0);
	glVertex3f(pts3d[index3].x, pts3d[index3].y, pts3d[index3].z);
}

void GLWidget3D::drawSphere(float x, float y, float z, float r, const QColor& color) {
	int slices = 16;
	int stacks = 8;

	glBegin(GL_QUADS);
	glColor3f(color.redF(), color.greenF(), color.blueF());
	for (int i = 0; i < slices; ++i) {
		float theta1 = M_PI * 2.0f / slices * i;
		float theta2 = M_PI * 2.0f / slices * (i + 1);

		for (int j = 0; j < stacks; ++j) {
			float phi1 = M_PI / stacks * j - M_PI * 0.5;
			float phi2 = M_PI / stacks * (j + 1) - M_PI * 0.5;

			QVector3D pt1 = QVector3D(cosf(theta1) * cosf(phi1), sinf(theta1) * cosf(phi1), sinf(phi1));
			QVector3D pt2 = QVector3D(cosf(theta2) * cosf(phi1), sinf(theta2) * cosf(phi1), sinf(phi1));
			QVector3D pt3 = QVector3D(cosf(theta2) * cosf(phi2), sinf(theta2) * cosf(phi2), sinf(phi2));
			QVector3D pt4 = QVector3D(cosf(theta1) * cosf(phi2), sinf(theta1) * cosf(phi2), sinf(phi2));

			glNormal3f(pt1.x(), pt1.y(), pt1.z());
			glVertex3f(x + pt1.x() * r, y + pt1.y() * r, z + pt1.z() * r);
			glNormal3f(pt2.x(), pt2.y(), pt2.z());
			glVertex3f(x + pt2.x() * r, y + pt2.y() * r, z + pt2.z() * r);
			glNormal3f(pt3.x(), pt3.y(), pt3.z());
			glVertex3f(x + pt3.x() * r, y + pt3.y() * r, z + pt3.z() * r);
			glNormal3f(pt4.x(), pt4.y(), pt4.z());
			glVertex3f(x + pt4.x() * r, y + pt4.y() * r, z + pt4.z() * r);
		}
	}
	glEnd();
}

void GLWidget3D::featureExtraction(std::vector<cv::Mat>& img) {
	std::vector<KeyPoint> keypoints[2];
	
	SurfFeatureDetector detector(400);

	detector.detect(img[0], keypoints[0]);
	detector.detect(img[1], keypoints[1]);

	SurfDescriptorExtractor extractor;

	Mat descriptors[2];

	extractor.compute(img[0], keypoints[0], descriptors[0]);
	extractor.compute(img[1], keypoints[1], descriptors[1]);

	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match(descriptors[0], descriptors[1], matches);

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors[0].rows; i++) {
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist );
	printf("-- Min dist : %f \n", min_dist );

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector< DMatch > good_matches;

	for( int i = 0; i < descriptors[0].rows; i++ ) {
		if (cv::norm(keypoints[0][i].pt - keypoints[1][matches[i].trainIdx].pt) < 100) {
		//if( matches[i].distance <= max(2*min_dist, 0.02) ) {
			good_matches.push_back( matches[i]);
		}
	}

	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches( img[0], keypoints[0], img[1], keypoints[1],
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//-- Show detected matches
	imwrite( "matches.jpg", img_matches );

	FILE* fp = fopen("matches.txt", "w");
	for (int i = 0; i < good_matches.size(); ++i) {
		fprintf(fp, "%lf,%lf,%lf,%lf\n", keypoints[0][good_matches[i].queryIdx].pt.x, keypoints[0][good_matches[i].queryIdx].pt.y, keypoints[1][good_matches[i].trainIdx].pt.x, keypoints[1][good_matches[i].trainIdx].pt.y);
	}
	fclose(fp);
}

void GLWidget3D::reconstruct() {
	img.resize(2);
	img[0] = imread("images/image1.jpg");
	img[1] = imread("images/image2.jpg");

	Mat_<double> K;
	cv::FileStorage fs;
	fs.open("camera_calibration.yml", cv::FileStorage::READ);
	fs["camera_matrix"] >> K;
	std::cout << "K:\n" << K << std::endl;

	// 対応点をファイルから読み込む
	pts.resize(2);
	pts[0].clear();
	pts[1].clear();
	std::ifstream ifs("matches.txt");
	char str[256];
	while (!ifs.eof()) {
		Point2f x1, x2;
		char delimitor;
		ifs >> x1.x >> delimitor >> x1.y >> delimitor >> x2.x >> delimitor >> x2.y;

		if (x1.x == 0 && x1.y == 0 && x2.x == 0 && x2.y == 0) continue;

		pts[0].push_back(x1);
		pts[1].push_back(x2);
	}

	// Y座標を反転させる
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < pts[i].size(); ++j) {
			pts[i][j].y = img[i].rows - pts[i][j].y;
		}
	}

	Reconstruction reconstruction;
	std::vector<uchar> status;
	cv::Mat F = reconstruction.findFundamentalMat(pts[0], pts[1], status);
	cv::Mat_<double> E = K.t() * F * K;

	std::cout << "E:" << E << std::endl;
	std::cout << "det(E) should be less than 1e-07." << std::endl;
	std::cout << "det(E): " << cv::determinant(E) << std::endl;

	Matx34d P, P1;
	Mat_<double> R1, R2, T1, T2;
	reconstruction.decomposeEtoRandT(E, R1, R2, T1, T2);

	if (determinant(R1) + 1.0 < 1e-09) {
		//according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
		cout << "det(R) == -1 ["<<determinant(R1)<<"]: flip E's sign" << endl;
		E = -E;
		reconstruction.decomposeEtoRandT(E, R1, R2, T1, T2);
	}

	double avg_error = reconstruction.unprojectPoints(img[0].size(), K, R1, T1, R2, T2, pts[0], pts[1], pts3d);
	printf("avg error: %lf\n", avg_error);	

	// compute bounding box
	double min_x = std::numeric_limits<float>::max();
	double min_y = std::numeric_limits<float>::max();
	double min_z = std::numeric_limits<float>::max();
	double max_x = -std::numeric_limits<float>::max();
	double max_y = -std::numeric_limits<float>::max();
	double max_z = -std::numeric_limits<float>::max();
	for (int i = 0; i < pts3d.size(); ++i) {
		min_x = std::min(min_x, pts3d[i].x);
		min_y = std::min(min_y, pts3d[i].y);
		min_z = std::min(min_z, pts3d[i].z);
		max_x = std::max(max_x, pts3d[i].x);
		max_y = std::max(max_y, pts3d[i].y);
		max_z = std::max(max_z, pts3d[i].z);
	}

	// translate to the origin
	for (int i = 0; i < pts3d.size(); ++i) {
		pts3d[i].x -= (min_x + max_x) * 0.5;
		pts3d[i].y -= (min_y + max_y) * 0.5;
		pts3d[i].z -= (min_z + max_z) * 0.5;
	}

	float scale_factor = 1000.0f;
	for (int i = 0; i < pts3d.size(); ++i) {
		pts3d[i].x *= scale_factor;
		pts3d[i].y *= scale_factor;
		pts3d[i].z *= -scale_factor;
	}

	updateGL();
}

void GLWidget3D::calibrateCamera(std::vector<cv::Mat>& img) {
	Mat_<double> K = Mat_<double>::eye(3, 3);
	Mat_<double> distCoeffs = Mat_<double>::zeros(1, 8);
	std::vector<Mat> rvecs;
	std::vector<Mat> tvecs;

	std::vector<std::vector<cv::Point3f> > objectPoints;
	objectPoints.resize(img.size());

	std::vector<std::vector<cv::Point2f> > pts;
	pts.resize(img.size());

	for (int i = 0; i < img.size(); ++i) {
		// ３Ｄ座標のセット
		for (int r = 0; r < 7; ++r) {
			for (int c = 0; c < 10; ++c) {
				objectPoints[i].push_back(cv::Point3f(c * 21.7, (6-r) * 21.7, 0.0f));
			}
		}

		// コーナー検出
		if (cv::findChessboardCorners(img[i], cv::Size(10, 7), pts[i])) {
			fprintf (stderr, "ok\n");
		} else {
			fprintf (stderr, "fail\n");
		}

		// サブピクセル精度のコーナー検出
		cv::Mat grayMat(img[i].size(), CV_8UC1);
		cv::cvtColor(img[i], grayMat, CV_RGB2GRAY);
		cv::cornerSubPix(grayMat, pts[i], cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

		for (int j = 0; j < pts[i].size(); ++j) {
			pts[i][j].y = img[i].rows - pts[i][j].y;
		}
	}

	cv::calibrateCamera(objectPoints, pts, img[0].size(), K, distCoeffs, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);

	cv::FileStorage fs;
	fs.open("camera_calibration.yml", cv::FileStorage::WRITE);
	fs << "camera_matrix" << K;
	fs << "distortion_coefficients" << distCoeffs;
}

int GLWidget3D::findPointIndex(std::vector<Point2f>& pts, Point2f& pt) {
	double min_dist = 100;
	int index = -1;
	for (int i = 0; i < pts.size(); ++i) {
		double dist = norm(pts[i] - pt);
		if (dist < min_dist) {
			min_dist = dist;
			index = i;
		}
	}

	return index;
}
