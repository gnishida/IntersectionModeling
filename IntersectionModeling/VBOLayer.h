#pragma once


#include "FLayer.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"



class VBORenderManager;

class VBOLayer {



public:
	VBOLayer();

	void init(VBORenderManager& rendManager);

	void render(VBORenderManager& rendManager);

	// edit
	FLayer layer;
	bool initialized;
	int resolutionX;
	int resolutionY;
	QVector3D minPos;
	QVector3D maxPos;

private:
	GLuint vao;
	GLuint vbo;
	GLuint program;
};