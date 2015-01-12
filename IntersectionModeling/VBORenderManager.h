#pragma once

#include "glew.h"
#include "VBOShader.h"
#include "VBOUtil.h"
#include "qmap.h"

#include "VBOTerrain.h"
#include "VBOModel.h"
#include "VBOLayer.h"
#include "Polygon3D.h"
#include "VBOModel_StreetElements.h"

#ifndef Q_MOC_RUN
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/multi/multi.hpp>
#include <boost/polygon/polygon.hpp>
#endif

const int mode_AdaptTerrain=0x0100;
const int mode_Lighting=0x0200;
const int mode_TexArray=0x0400;
const int mode_Tex3D=0x0800;

//0x0100 --> adapt vboRenderManager
//0x0200 --> lighting

struct RenderSt{
	uint texNum;//0 means use color
	GLuint vbo;
	GLuint vao;
	int numVertex;//defines if the vbo has been created
	std::vector<Vertex> vertices;

	GLenum geometryType;
	int shaderMode;

	RenderSt(uint _texNum,std::vector<Vertex> _vertices,GLenum geoT,int shModer){
		texNum=_texNum;
		vertices=_vertices;
		geometryType=geoT;
		shaderMode=shModer;
		numVertex=-1;
	}
	RenderSt(){
		numVertex=-1;
	}
};

typedef QHash<uint,RenderSt> renderGrid;

/////////////////////////////////////
// VBORenderManager

class VBORenderManager {

public:

	// POLYGON
	typedef boost::polygon::point_data<double> pointP;
	typedef boost::polygon::polygon_set_data<double> polygon_setP;
	typedef boost::polygon::polygon_with_holes_data<double> polygonP;
	typedef std::pair<pointP, pointP> edgeP;
	typedef std::vector<boost::polygon::polygon_data<double> > PolygonSetP;


	QMap<QString,int> geoNameToGeoNum;
	GLuint program;
	int currentIndexGeo;

	VBORenderManager();
	~VBORenderManager();

	void init();

	// layers & vboRenderManager
	bool editionMode;
	QVector3D mousePos3D;
	VBOTerrain vboTerrain;
	void changeTerrainDimensions(float terrainSide,int resolution);
	float getTerrainHeight(float xP,float xY);/// !!
	QVector3D minPos;
	QVector3D maxPos;
	float side;

	// textures
	QHash<QString,GLuint> nameToTexId;
	GLuint loadTexture(const QString fileName,bool mirrored=false);
	GLuint loadArrayTexture(QString texName,std::vector<QString> fileNames);

	//static
	bool addStaticGeometry(QString geoName,std::vector<Vertex>& vert,QString textureName,GLenum geometryType,int shaderMode);
	bool addStaticGeometry2(QString geoName,std::vector<QVector3D>& pos,float zShift,bool inverseLoop,QString textureName,GLenum geometryType,int shaderMode,QVector3D texScale,QColor color);
	bool removeStaticGeometry(QString geoName);
	void renderStaticGeometry(QString geoName);
	bool checkIfGeoNameInUse(QString geoName);

	// some convenient functions for drawing basic primitives
	void addTriangle(const QString& geoName, Loop3D& polygon, Loop3D& texCoord, const QString& textureName);
	void addTriangle(const QString& geoName, Loop3D& baseLine, float baseHeight, float topHeight, const QString& textureName);
	void addQuad(const QString& geoName, Loop3D& polygon, const QString& textureName);
	void addSphere(const QString& geoName, const QVector3D& center, float radius, const QColor& color);
	void addBox(const QString& geoName, const QVector3D& center, const QVector3D& size, const QColor& color);
	void addBox(const QString& geoName, const QVector3D& offset, const QVector3D& vec1, const QVector3D& vec2, float height, const QString& textureName, int faceNo = 0, float s0 = 0.0, float t0 = 0.0, float s1 = 1.0, float t1 = 1.0);
	void addLine(const QString& geoName, const QVector3D& pt1, const QVector3D& pt2, const QColor& color);
	void addPolygon(const QString& geoName, Loop3D& polygon, float z, const QString& textureName, const QVector3D& texScale);
	void addPolygon(const QString& geoName, Loop3D& polygon, float z, const QColor& color, bool inverseLoop = false);
	void addPrism(const QString& geoName, Loop3D& polygon, float baseHeight, float topHeight, const QString& textureName);
	void addPrism(const QString& geoName, Loop3D& polygon, float baseHeight, float topHeight, const QColor& color, bool addTopAndBase = true);
	void addWedge(const QString& geoName, Loop3D& polygon, float baseHeight, float topHeight, const QString& textureName);
	void addCylinder(const QString& geoName, const QVector3D& center, float baseRadius, float topRadius, float height, const QString& textureName);


	//models
	QHash<QString,std::vector<ModelSpec>> nameToVectorModels;
	bool initializedStreetElements;
	void addStreetElementModel(QString name,ModelSpec mSpec);
	void renderAllStreetElementName(QString name);
	void removeAllStreetElementName(QString name);

private:

	QHash<QString,QHash<int,renderGrid>> geoName2RenderGrids;
	QHash<QString,renderGrid> geoName2StaticRender;

	void renderVAO(RenderSt& renderSt,bool cleanVertex);
	bool createVAO(std::vector<Vertex>& vert,GLuint& vbo,GLuint& vao,int& numVertex);
	void cleanVAO(GLuint vbo,GLuint vao);

};
