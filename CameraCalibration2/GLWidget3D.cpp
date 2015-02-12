﻿#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>
#include "Reconstruction.h"
#include <opencv2/nonfree/features2d.hpp>

#define SQR(x)	((x) * (x))

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
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_QUADS);
	glVertex3f(pts3d[0].x, pts3d[0].y, pts3d[0].z);
	glVertex3f(pts3d[3].x, pts3d[3].y, pts3d[3].z);
	glVertex3f(pts3d[2].x, pts3d[2].y, pts3d[2].z);
	glVertex3f(pts3d[1].x, pts3d[1].y, pts3d[1].z);

	glColor3f(1.0, 1.0, 0.0);
	glVertex3f(pts3d[0].x, pts3d[0].y, pts3d[0].z);
	glVertex3f(pts3d[7].x, pts3d[7].y, pts3d[7].z);
	glVertex3f(pts3d[8].x, pts3d[8].y, pts3d[8].z);
	glVertex3f(pts3d[3].x, pts3d[3].y, pts3d[3].z);

	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(pts3d[3].x, pts3d[3].y, pts3d[3].z);
	glVertex3f(pts3d[8].x, pts3d[8].y, pts3d[8].z);
	glVertex3f(pts3d[9].x, pts3d[9].y, pts3d[9].z);
	glVertex3f(pts3d[2].x, pts3d[2].y, pts3d[2].z);

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

void GLWidget3D::reconstruct() {
	cv::Mat img[2];
	img[0] = cv::imread("images/calib1.jpg");
	img[1] = cv::imread("images/calib2.jpg");

	std::vector<std::vector<cv::Point3f> > objectPoints;
	objectPoints.resize(2);
	pts.resize(2);
	for (int i = 0; i < 2; ++i) {
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

	// カメラキャリブレーション
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distCoeffs = cv::Mat::zeros(1, 8, CV_64F);
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	cv::calibrateCamera(objectPoints, pts, img[0].size(), cameraMatrix, distCoeffs, rvecs, tvecs);

	cv::Mat P[2];
	cv::Mat extrinsicMat[2];
	for (int i = 0; i < 2; ++i) {
		// 射影行列の計算
		cv::Mat rot33;
		cv::Rodrigues(rvecs[i], rot33);
		extrinsicMat[i] = (cv::Mat_<double>(3, 4) << rot33.at<double>(0, 0), rot33.at<double>(0, 1), rot33.at<double>(0, 2), tvecs[i].at<double>(0, 0),
														rot33.at<double>(1, 0), rot33.at<double>(1, 1), rot33.at<double>(1, 2), tvecs[i].at<double>(1, 0),
														rot33.at<double>(2, 0), rot33.at<double>(2, 1), rot33.at<double>(2, 2), tvecs[i].at<double>(2, 0));
		
		P[i] = cameraMatrix * extrinsicMat[i];

		std::cout << "P[" << i << "]:\n" << P[i] << std::endl;
	}

	// Hを計算
	cv::Mat H = P[1] * (P[0].t() * (P[0] * P[0].t()).inv());
	std::cout << "H\n" << H << std::endl;

	for (int j = 0; j < pts[0].size(); ++j) {
		cv::Mat x = (cv::Mat_<double>(3, 1) << pts[0][j].x, pts[0][j].y, 1);
		cv::Mat x2 = H * x;
		cv::Point2f pt(x2.at<double>(0, 0) / x2.at<double>(2, 0), x2.at<double>(1, 0) / x2.at<double>(2, 0));
		std::cout << pt << " <-> " << pts[1][j] << std::endl;
	}






	/*
	img[0] = cv::imread("images/church1.jpg");
	img[1] = cv::imread("images/church2.jpg");
	std::vector<cv::KeyPoint> keypoints[2];
	cv::SIFT sift;
	sift.detect(img[0], keypoints[0]);
    sift.detect(img[1], keypoints[1]);

	cv::SurfDescriptorExtractor surfDesc;

    // Extraction of the SURF descriptors
    cv::Mat descriptors[2];
    surfDesc.compute(img[0], keypoints[0], descriptors[0]);
    surfDesc.compute(img[1], keypoints[1], descriptors[1]);

	//cv::BruteForceMatcher<cv::L2<float> > matcher;
	cv::BFMatcher matcher;
	std::vector<cv::DMatch> matches;
    matcher.match(descriptors[0], descriptors[1], matches);

	cv::Mat img_matches;
	cv::drawMatches(img[0], keypoints[0], img[1], keypoints[1], matches, img_matches);
	cv::imwrite("matches.jpg", img_matches);

	std::vector<int> pts_index[2];
	for (std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it) {
         // Get the indexes of the selected matched keypoints
         pts_index[0].push_back(it->queryIdx);
         pts_index[1].push_back(it->trainIdx);
    }

    cv::KeyPoint::convert(keypoints[0], pts[0], pts_index[0]);
    cv::KeyPoint::convert(keypoints[1], pts[1], pts_index[1]);
	*/
	

	img[0] = cv::imread("images/image1.jpg");
	img[1] = cv::imread("images/image2.jpg");

	cv::Mat undistorted_img[2];
	cv::undistort(img[0], undistorted_img[0], cameraMatrix, distCoeffs);
	cv::imwrite("undistorted.jpg", undistorted_img[0]);

		// とりあえず、対応点を手動で設定
	{
		pts[0].clear();
		pts[1].clear();
		pts[0].push_back(cv::Point2f(95, 132));
		pts[1].push_back(cv::Point2f(110, 167));
		pts[0].push_back(cv::Point2f(234, 51));
		pts[1].push_back(cv::Point2f(217, 78));
		pts[0].push_back(cv::Point2f(467, 130));
		pts[1].push_back(cv::Point2f(467, 135));
		pts[0].push_back(cv::Point2f(328, 249));
		pts[1].push_back(cv::Point2f(388, 251));
		pts[0].push_back(cv::Point2f(104, 160));
		pts[1].push_back(cv::Point2f(117, 194));
		pts[0].push_back(cv::Point2f(333, 280));
		pts[1].push_back(cv::Point2f(389, 280));
		pts[0].push_back(cv::Point2f(469, 158));
		pts[1].push_back(cv::Point2f(468, 159));
		pts[0].push_back(cv::Point2f(129, 220));
		pts[1].push_back(cv::Point2f(137, 251));
		pts[0].push_back(cv::Point2f(341, 346));
		pts[1].push_back(cv::Point2f(387, 340));
		pts[0].push_back(cv::Point2f(462, 229));
		pts[1].push_back(cv::Point2f(458, 226));
	}
	
	// Y座標を反転させる
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < pts[i].size(); ++j) {
			pts[i][j].y = img[i].rows - pts[i][j].y;

			//pts[i][j].x = (pts[i][j].x - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0);
			//pts[i][j].y = (pts[i][j].y - cameraMatrix.at<double>(1, 2)) / cameraMatrix.at<double>(1, 1);
		}
	}



	Reconstruction reconstruction;
	std::vector<uchar> status;
	//cv::Mat F = reconstruction.findFundamentalMat(pts[0], pts[1], status);
	cv::Mat F = cv::findFundamentalMat(pts[0], pts[1], cv::FM_RANSAC, 0.1, 0.99, status);
	std::cout << "F:\n" << F << std::endl;

	cv::Mat E = cameraMatrix.t() * F * cameraMatrix;


	// epipoler lineを描画
	for (int i = 0; i < 2; ++i) {
		std::vector<cv::Point3f> lines;
		cv::computeCorrespondEpilines(pts[1-i], 2 - i, F, lines);
		
		for (int j = 0; j < pts[i].size(); ++j) {
			if (status[j] == 0) continue;

			float x1 = 0;
			float y1 = -lines[j].z / lines[j].y;
			float x2 = img[i].cols - 1;
			float y2 = -(lines[j].z + (img[i].cols - 1) * lines[j].x) / lines[j].y;

			cv::Scalar color(j * 10 % 255, j * 40 % 255, j * 80 % 255);
			cv::line(img[i], cv::Point(x1, img[i].rows - y1), cv::Point(x2, img[i].rows - y2), color, 1);

			cv::circle(img[i], cv::Point(pts[i][j].x, img[i].rows - pts[i][j].y), 5, color, 1);
		}

		char filename[255];
		sprintf(filename, "result%d.jpg", i);
		cv::imwrite(filename, img[i]);
	}

	reconstruction.computeProjectionMat(E, P[0], P[1]);


	double avg_error = reconstruction.unprojectPoints(cameraMatrix, P[0], P[1], pts[0], pts[1], pts3d);
	printf("avg error: %lf\n", avg_error);	

}