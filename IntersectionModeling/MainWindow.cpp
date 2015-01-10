#include "MainWindow.h"
#include <QFileDialog>
#include "GraphUtil.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));

	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

MainWindow::~MainWindow()
{

}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Street Map file..."), "", tr("StreetMap Files (*.gsm)"));
	if (filename.isEmpty()) return;

	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		std::cerr << "The file is not accessible: " << filename.toUtf8().constData() << endl;
		throw "The file is not accessible: " + filename;
	}

	glWidget->roads.clear();
	GraphUtil::loadRoads(glWidget->roads, filename);

	glWidget->roads.adaptToTerrain(&glWidget->vboRenderManager);
	glWidget->roads.updateRoadGraph(glWidget->vboRenderManager);

	glWidget->shadow.makeShadowMap(glWidget);

	glWidget->updateGL();
}
