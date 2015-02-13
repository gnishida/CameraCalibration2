#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	connect(ui.actionCalibration, SIGNAL(triggered()), this, SLOT(onCalibration()));
	connect(ui.actionReconstruction, SIGNAL(triggered()), this, SLOT(onReconstruction()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));

	// setup the OpenGL widget
	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

void MainWindow::onCalibration() {
    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(trUtf8("Image files (*.jpg *.png)"));
    QStringList fileNames;
    if (dialog.exec()) {
	    fileNames = dialog.selectedFiles();
		std::vector<Mat> img(fileNames.size());

		for (int i = 0; i < fileNames.size(); ++i) {
			img[i] = cv::imread(fileNames[i].toUtf8().data());
		}

		//glWidget->calibrateCamera(img);
	}
}

void MainWindow::onReconstruction() {
	std::vector<Mat> img(2);
	img[0] = cv::imread("images/image1.jpg");
	img[1] = cv::imread("images/image2.jpg");

	glWidget->reconstruct(img);
}
