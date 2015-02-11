#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>
#include "Reconstruction.h"

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

	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glColor3f(1.0, 1.0, 1.0);
	for (int i = 0; i < pts3d.size(); ++i) {
		glVertex3f(pts3d[i].x, pts3d[i].y, pts3d[i].z);
	}
	glEnd();
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
	img[0] = cv::imread("images/image1.jpg");
	img[1] = cv::imread("images/image2.jpg");


	// とりあえず、対応点を手動で設定
	{
		pts[0].clear();
		pts[1].clear();
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

	for (int i = 0; i < pts[0].size(); ++i) {
		pts[0][i].y = 480 - pts[0][i].y;
		pts[1][i].y = 480 - pts[1][i].y;
	}

	Reconstruction reconstruction;
	std::vector<uchar> status;
	cv::Mat F = reconstruction.findFundamentalMat(pts[0], pts[1], status);

	cv::Mat P[2];
	reconstruction.computeProjectionMat(F, P[0], P[1]);

	double avg_error = reconstruction.unprojectPoints(P[0], P[1], pts[0], pts[1], pts3d);
	printf("avg error: %lf\n", avg_error);	
}
