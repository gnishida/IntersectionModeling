#pragma once

#include <glew.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QFile.h>
#include <QVector3D>
#include <QVector2D>

class FLayer{
public:
	void updateTexFromData(float minValue, float maxValue);
	cv::Mat layerData;

	bool initialized;
	QVector3D maxPos;
	QVector3D minPos;

	int imgResX;
	int imgResY;

	GLuint texData;

public:
	FLayer() : initialized(false), texData(0) {}

	void init(QVector3D _minPos, QVector3D _maxPos, int imgResX, int imgResY);

	// control
	float getValue(const QVector2D& pt);
};