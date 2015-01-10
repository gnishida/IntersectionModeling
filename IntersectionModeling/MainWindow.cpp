#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

MainWindow::~MainWindow()
{

}
