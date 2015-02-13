#include <iostream>
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
		drawTriangle(1, 4, 5);
		drawTriangle(1, 5, 2);

		drawTriangle(6, 7, 8);
		drawTriangle(7, 10, 8);
		drawTriangle(8, 10, 9);
		drawTriangle(9, 10, 11);
		glEnd();
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
	float r = (float)(index1 * 20) / 255.0;
	float g = (float)(index2 * 40) / 255.0;
	float b = (float)(index3 * 80) / 255.0;
	glColor3f(r, g, b);

	glVertex3f(pts3d[index1].x, pts3d[index1].y, pts3d[index1].z);
	glVertex3f(pts3d[index2].x, pts3d[index2].y, pts3d[index2].z);
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

void GLWidget3D::reconstruct(std::vector<cv::Mat>& img) {
	Mat_<double> K;
	cv::FileStorage fs;
	fs.open("camera_calibration.yml", cv::FileStorage::READ);
	fs["camera_matrix"] >> K;
	std::cout << "K:\n" << K << std::endl;

	// とりあえず、対応点を手動で設定
	pts.resize(2);
	{
		pts[0].clear();
		pts[1].clear();

		pts[0].push_back(cv::Point2f(590, 239));
		pts[1].push_back(cv::Point2f(636, 279));
		pts[0].push_back(cv::Point2f(615, 568));
		pts[1].push_back(cv::Point2f(647, 611));
		pts[0].push_back(cv::Point2f(1017, 662));
		pts[1].push_back(cv::Point2f(1082, 686));
		pts[0].push_back(cv::Point2f(1027, 297));
		pts[1].push_back(cv::Point2f(1105, 329));
		pts[0].push_back(cv::Point2f(291, 653));
		pts[1].push_back(cv::Point2f(279, 714));
		pts[0].push_back(cv::Point2f(1384, 936));
		pts[1].push_back(cv::Point2f(1457, 927));
		pts[0].push_back(cv::Point2f(720, 604));
		pts[1].push_back(cv::Point2f(810, 639));
		pts[0].push_back(cv::Point2f(666, 659));
		pts[1].push_back(cv::Point2f(768, 697));
		pts[0].push_back(cv::Point2f(708, 651));
		pts[1].push_back(cv::Point2f(810, 686));
		pts[0].push_back(cv::Point2f(802, 757));
		pts[1].push_back(cv::Point2f(802, 756));
		pts[0].push_back(cv::Point2f(649, 739));
		pts[1].push_back(cv::Point2f(753, 778));
		pts[0].push_back(cv::Point2f(696, 794));
		pts[1].push_back(cv::Point2f(777, 830));
		pts[0].push_back(cv::Point2f(709, 769));
		pts[1].push_back(cv::Point2f(613, 727));
	}
	
	// Y座標を反転させる
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < pts[i].size(); ++j) {
			pts[i][j].y = img[i].rows - pts[i][j].y;
		}
	}

	// 座標の正規化
	cv::Mat_<double> Kinv = K.inv();
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < pts[i].size(); ++j) {
			pts[i][j].x = (pts[i][j].x - K(0, 2)) / K(0, 0);
			pts[i][j].y = (pts[i][j].y - K(1, 2)) / K(1, 1);
		}
	}

	Reconstruction reconstruction;
	std::vector<uchar> status;
	//cv::Mat F = reconstruction.findFundamentalMat(pts[0], pts[1], status);
	cv::Matx33d E = cv::findFundamentalMat(pts[0], pts[1], cv::FM_RANSAC, 0.1, 0.99, status);
	//cv::Matx33d E = cameraMatrix.t() * F * cameraMatrix;


	std::cout << "det(E) should be less than 1e-07." << std::endl;
	std::cout << "det(E): " << cv::determinant(E) << std::endl;


	Matx34d P, P1;
	reconstruction.computeProjectionMat(E, P, P1);

	std::cout << "P:\n" << P1 << std::endl;

	double avg_error = reconstruction.unprojectPoints(K, P, P1, pts[0], pts[1], pts3d);
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
		pts3d[i].x *= -scale_factor;
		pts3d[i].y *= -scale_factor;
		pts3d[i].z *= scale_factor;
	}
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
