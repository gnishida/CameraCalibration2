#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionFileOpen, SIGNAL(triggered()), this, SLOT(onFileOpen()));

	// setup the OpenGL widget
	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

void MainWindow::onFileOpen() {
	glWidget->reconstruct();
}
