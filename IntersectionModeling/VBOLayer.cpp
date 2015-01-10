#include "VBOLayer.h"
#include <QGLWidget>
#include "Util.h"
#include "global.h"
#include "VBORenderManager.h"

VBOLayer::VBOLayer() {
	initialized=false;

	/// resolution of vertices
	resolutionX=200;
	resolutionY=200;
}

void VBOLayer::init(VBORenderManager& rendManager){
	minPos=rendManager.minPos;
	maxPos=rendManager.maxPos;
		
	if (!initialized) {
		layer.init(minPos, maxPos, resolutionX, resolutionY);
	} else {
		layer.minPos=minPos;
		layer.maxPos=maxPos;
	}

	//////////////////
	// VERTICES
	/*if(initialized==true){
		glDeleteBuffers(1, &vbo);
		vbo=0;
	}*/

	program=rendManager.program;

	std::vector<Vertex> vert;
	float sideX=abs(maxPos.x()-minPos.x())/resolutionX;
	float sideY=abs(maxPos.y()-minPos.y())/resolutionY;
		
	// VERTEX
	float alpha = 0.3;
	vert.push_back(Vertex(minPos.x(), minPos.y(), 80, 0, 0, 0, alpha, 0, 0, 1, 0, 0, 0));
	vert.push_back(Vertex(maxPos.x(), minPos.y(), 80, 0, 0, 0, alpha, 0, 0, 1, 1, 0, 0));
	vert.push_back(Vertex(maxPos.x(), maxPos.y(), 80, 0, 0, 0, alpha, 0, 0, 1, 1, 1, 0));
	vert.push_back(Vertex(minPos.x(), maxPos.y(), 80, 0, 0, 0, alpha, 0, 0, 1, 0, 1, 0));

	glGenVertexArrays(1,&vao);
	glBindVertexArray(vao);

	// Crete VBO
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex)*vert.size(), vert.data(), GL_STATIC_DRAW);
	
	// Configure the attributes in the VAO.
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(4*sizeof(float)));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(8*sizeof(float)));
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(12*sizeof(float)));

	// Bind back to the default state.
	glBindVertexArray(0); 
	glBindBuffer(GL_ARRAY_BUFFER,0);

	initialized=true;
}

void VBOLayer::render(VBORenderManager& rendManager) {
	// デフォルトのテクスチャユニットを使用する
	glUniform1i(glGetUniformLocation (program, "tex0"), 0);//tex0: 0

	// テクスチャをバインド
	glBindTexture(GL_TEXTURE_2D, layer.texData);

	// テクスチャモード
	glUniform1i (glGetUniformLocation (program, "mode"), 2);

	// 描画
	glBindVertexArray(vao);
	glDrawArrays(GL_QUADS, 0, 4);

	// クリーンアップ
	glBindVertexArray(0);
}
