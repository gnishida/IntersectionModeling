#include "FLayer.h"
//#include "VBOUtil.h"

void FLayer::init(QVector3D _minPos, QVector3D _maxPos, int _imgResX, int _imgResY) {
	minPos=_minPos;
	maxPos=_maxPos;
	imgResX=_imgResX;
	imgResY=_imgResY;
		
	// Matrixを初期化（全要素を0.0で初期化）
	layerData = cv::Mat(imgResY, imgResX, CV_32F, cv::Scalar(0.0));

	// とりあえずテクスチャを生成
	updateTexFromData(0, 1);

	initialized=true;
}

void FLayer::updateTexFromData(float minValue, float maxValue) {
	if (texData != 0) {
		glDeleteTextures(1, &texData);
		texData = 0;
	}

	GLubyte* data = new GLubyte [layerData.cols * layerData.rows * 3];
	{
		int index = 0;
		for (int r = 0; r < layerData.rows; ++r) {
			for (int c = 0; c < layerData.cols; ++c) {
				int v = (layerData.at<float>(r, c) - minValue) / (maxValue - minValue) * 255;
				if (v > 255) v = 255;
				data[index * 3] = (GLubyte)v;
				data[index * 3 + 1] = (GLubyte)v;
				data[index * 3 + 2] = (GLubyte)v;
				index++;
			}
		}
	}

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);

	glGenTextures(1, &texData);
	glBindTexture(GL_TEXTURE_2D, texData);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, layerData.cols, layerData.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	glGenerateMipmap(GL_TEXTURE_2D);

	delete [] data;
}

// return the interpolated value
float FLayer::getValue(const QVector2D& pt) {
	int c = (pt.x() - minPos.x()) / (maxPos.x() - minPos.x()) * imgResX;
	int r = (pt.y() - minPos.y()) / (maxPos.y() - minPos.y()) * imgResY;

	// ToDo: bilinear linterpolation

	return layerData.at<float>(r, c);
}
