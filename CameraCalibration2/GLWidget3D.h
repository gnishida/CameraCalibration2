#pragma once

#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include "Camera.h"
#include <QVector3D>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

class MainWindow;

class GLWidget3D : public QGLWidget {
private:
	MainWindow* mainWin;
	Camera camera;
	QPoint lastPos;

	cv::Mat P[2];
	std::vector<std::vector<cv::Point2f> > pts;
	std::vector<cv::Point3d> pts3d;

public:
	GLWidget3D(MainWindow* mainWin);
	void drawScene();
	QVector2D mouseTo2D(int x,int y);
	void drawSphere(float x, float y, float z, float r, const QColor& color);
	void reconstruct();

protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();    
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);

};

