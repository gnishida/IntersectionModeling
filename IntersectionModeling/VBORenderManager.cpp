#include "VBORenderManager.h"
//#include "triangle\triangle.c"

using namespace boost::polygon::operators;

VBORenderManager::VBORenderManager(){
	editionMode=false;
	side=2000.0f;
	minPos=QVector3D (-side/2.0f,-side/2.0f,0);
	maxPos=QVector3D (side/2.0f,side/2.0f,0);
	//initializedStreetElements=false;
}

VBORenderManager::~VBORenderManager() {
	Shader::cleanShaders();
}

void VBORenderManager::init(){
	// init program shader
	program=Shader::initShader(QString("../data/shaders/lc_vertex_sk.glsl"),QString("../data/shaders/lc_fragment_sk.glsl"));
	glUseProgram(program);

	vboTerrain.init(*this);

	nameToTexId[""]=0;

	printf("VBORenderManager\n");

}//

GLuint VBORenderManager::loadTexture(const QString fileName,bool mirrored){
	GLuint texId;
	if(nameToTexId.contains(fileName)){
		texId=nameToTexId[fileName];
	}else{
		texId=VBOUtil::loadImage(fileName,mirrored);
		nameToTexId[fileName]=texId;
	}
	return texId;
}//

GLuint VBORenderManager::loadArrayTexture(QString texName,std::vector<QString> fileNames){
	GLuint texId;
	if(nameToTexId.contains(texName)){
		texId=nameToTexId[texName];
	}else{
		texId=VBOUtil::loadImageArray(fileNames);
		nameToTexId[texName]=texId;
	}
	return texId;
}

// ATRIBUTES
// 0 Vertex
// 1 Color
// 2 Normal
// 3 UV

// UNIFORMS
// 0 mode
// 1 tex0

bool VBORenderManager::createVAO(std::vector<Vertex>& vert,GLuint& vbo,GLuint& vao,int& numVertex){
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
		
	
	numVertex=vert.size();
	
	return true;
}//

void VBORenderManager::renderVAO(RenderSt& renderSt,bool cleanVertex){
	//printf("renderVAO numVert %d texNum %d vao %d numVertVert %d\n",renderSt.numVertex,renderSt.texNum,renderSt.vao,renderSt.vertices.size());
	// 1. Create if necessary
	if(renderSt.numVertex!=renderSt.vertices.size()&&renderSt.vertices.size()>0){
		if(renderSt.numVertex!=-1){
			cleanVAO(renderSt.vbo,renderSt.vao);
		}
		// generate vao/vbo
		createVAO(renderSt.vertices,renderSt.vbo,renderSt.vao,renderSt.numVertex);
		if(cleanVertex)
			renderSt.vertices.clear();
	}
	// 2. Render
	// 2.1 TEX
	int mode=renderSt.shaderMode;
	if((mode&mode_TexArray)==mode_TexArray){
		// MULTI TEX
		mode=mode&(~mode_TexArray);//remove tex array bit
		glActiveTexture(GL_TEXTURE8);

		glBindTexture(GL_TEXTURE_2D,0); 
		glBindTexture(GL_TEXTURE_2D_ARRAY, renderSt.texNum);
		glActiveTexture(GL_TEXTURE0);
		glUniform1i (glGetUniformLocation (program, "tex_3D"), 8);
	}else{
		glBindTexture(GL_TEXTURE_2D, renderSt.texNum);
	}
	// 2.2 mode
	//if(renderSt.texNum==0){
		//glUniform1i (glGetUniformLocation (program, "mode"), 1|(renderSt.shaderMode&0xFF00));//MODE: same modifiers but just color (renderSt.shaderMode&0xFF00)
	//}else{
		glUniform1i (glGetUniformLocation (program, "mode"), mode);
	//}

	glUniform1i (glGetUniformLocation (program, "tex0"), 0);//tex0: 0

	glBindVertexArray(renderSt.vao);
	glDrawArrays(renderSt.geometryType,0,renderSt.numVertex);
	glBindVertexArray(0);
}

void VBORenderManager::cleanVAO(GLuint vbo,GLuint vao){
	glDeleteBuffers(1, &vbo);
	glDeleteVertexArrays(1, &vao);
}//

/**
	* If "actual" flag is on, then the actual elevation will be returned even if the 2D flat terrain is used.
	*/
float VBORenderManager::getTerrainHeight(float xP,float xY){
	float xM=1.0f-(side/2.0f-xP)/side;
	float yM=1.0f-(side/2.0f-xY)/side;
	return vboTerrain.getTerrainHeight(xM,yM);
}

void VBORenderManager::changeTerrainDimensions(float terrainSide,int resolution){
	side=terrainSide;
	minPos=QVector3D (-side/2.0f,-side/2.0f,0);
	maxPos=QVector3D (side/2.0f,side/2.0f,0);
	vboTerrain.resolutionX=resolution;
	vboTerrain.resolutionY=resolution;
	vboTerrain.init(*this);
}//

///////////////////////////////////////////////////////////////////
// STATIC
bool VBORenderManager::addStaticGeometry(QString geoName,std::vector<Vertex>& vert,QString texName,GLenum geometryType,int shaderMode){
	if(vert.size()<=0)
		return false;
	GLuint texId;
	if(nameToTexId.contains(texName)){
		texId=nameToTexId[texName];
	}else{
		printf("load img %s\n",texName.toAscii().constData());
		texId=VBOUtil::loadImage(texName);
		nameToTexId[texName]=texId;
	}
		
	if(geoName2StaticRender.contains(geoName)){
		// 1.1 If already in manager
		if(geoName2StaticRender[geoName].contains(texId)){
			if(geoName2StaticRender[geoName][texId].vertices.size()==0){
				//1.1.1 if also contains texture and the number of vertex=0--> vao created--> remove
				cleanVAO(geoName2StaticRender[geoName][texId].vbo,geoName2StaticRender[geoName][texId].vao);
				geoName2StaticRender[geoName][texId]=RenderSt(texId,vert,geometryType,shaderMode);
			}else{
				//1.1.1 if also contains texture and the number of vertex!=0--> vao no created--> just append
				if(geometryType==GL_TRIANGLE_STRIP){
					//vert.insert(vert.begin(),vert.front());
					vert.insert(vert.begin(),geoName2StaticRender[geoName][texId].vertices.back());
					vert.insert(vert.begin(),geoName2StaticRender[geoName][texId].vertices.back());
				}
				geoName2StaticRender[geoName][texId].vertices.insert(geoName2StaticRender[geoName][texId].vertices.end(),vert.begin(),vert.end());
			}
		}else{
			geoName2StaticRender[geoName][texId]=RenderSt(texId,vert,geometryType,shaderMode);
		}
		//printf("--> YES in manager %s\n",geoName.toAscii().constData());
	}else{
		// 1.2 No yet in manager
		geoName2StaticRender[geoName][texId]=RenderSt(texId,vert,geometryType,shaderMode);
		//renderStaticGeometry(geoName);
		//printf("--> It was not yet in manager %s\n",geoName.toAscii().constData());
	}
	return true;
}//

bool VBORenderManager::checkIfGeoNameInUse(QString geoName){
	return (geoName2StaticRender.contains(geoName));
}//

void VBORenderManager::addTriangle(const QString& geoName, Loop3D& polygon, Loop3D& texCoord, const QString& textureName) {
	QVector3D normal = QVector3D::crossProduct(polygon[1] - polygon[0], polygon[2] - polygon[0]).normalized();

	std::vector<Vertex> verts;
	verts.push_back(Vertex(polygon[0], QColor(), normal, texCoord[0]));
	verts.push_back(Vertex(polygon[1], QColor(), normal, texCoord[1]));
	verts.push_back(Vertex(polygon[2], QColor(), normal, texCoord[2]));
	verts.push_back(Vertex(polygon[2], QColor(), normal, texCoord[2]));

	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

void VBORenderManager::addTriangle(const QString& geoName, Loop3D& baseLine, float baseHeight, float topHeight, const QString& textureName) {
	QVector3D normal = QVector3D::crossProduct(baseLine[1] - baseLine[0], QVector3D(0, 0, 1)).normalized();
	float length = (baseLine[1] - baseLine[0]).length();

	std::vector<Vertex> verts;
	verts.push_back(Vertex(QVector3D(baseLine[0].x(), baseLine[0].y(), baseHeight), QColor(), normal, QVector3D(0, baseHeight, 0)));
	verts.push_back(Vertex(QVector3D(baseLine[1].x(), baseLine[1].y(), baseHeight), QColor(), normal, QVector3D(length, baseHeight, 0)));
	verts.push_back(Vertex(QVector3D((baseLine[0].x() + baseLine[1].x()) * 0.5, (baseLine[0].y() + baseLine[1].y()) * 0.5, topHeight), QColor(), normal, QVector3D(length * 0.5, topHeight, 0)));
	verts.push_back(Vertex(QVector3D((baseLine[0].x() + baseLine[1].x()) * 0.5, (baseLine[0].y() + baseLine[1].y()) * 0.5, topHeight), QColor(), normal, QVector3D(length * 0.5, topHeight, 0)));

	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

void VBORenderManager::addQuad(const QString& geoName, Loop3D& polygon, const QString& textureName) {
	QVector3D normal = QVector3D::crossProduct(polygon[1] - polygon[0], polygon[2] - polygon[0]).normalized();
	float width = (polygon[1] - polygon[0]).length();
	float height = (polygon[2] - polygon[1]).length();

	std::vector<Vertex> verts;
	verts.push_back(Vertex(polygon[0], QColor(), normal, QVector3D(0, 0, 0)));
	verts.push_back(Vertex(polygon[1], QColor(), normal, QVector3D(width, 0, 0)));
	verts.push_back(Vertex(polygon[2], QColor(), normal, QVector3D(width, height, 0)));
	verts.push_back(Vertex(polygon[3], QColor(), normal, QVector3D(0, height, 0)));

	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

void VBORenderManager::addSphere(const QString& geoName, const QVector3D& center, float radius, const QColor& color) {
	int slice = 8;
	int stack = 2;

	std::vector<Vertex> vert;

	for (int si = 0; si < slice; ++si) {
		int next_si = (si + 1) % slice;
		float theta1 = 2.0f * M_PI * (float)si / (float)slice;
		float theta2 = 2.0f * M_PI * (float)next_si / (float)slice;

		for (int ti = -stack; ti < stack; ++ti) {
			int next_ti = ti + 1;
			float phi1 = 0.5f * M_PI * (float)ti / (float)stack;
			float phi2 = 0.5f * M_PI * (float)next_ti / (float)stack;

			float dx = radius * cosf(phi1) * cosf(theta1);
			float dy = radius * cosf(phi1) * sinf(theta1);
			float dz = radius * sinf(phi1);
			QVector3D d(dx, dy, dz);
			vert.push_back(Vertex(center + d, color, d.normalized(), QVector3D()));

			d.setX(radius * cosf(phi1) * cosf(theta2));
			d.setY(radius * cosf(phi1) * sinf(theta2));
			vert.push_back(Vertex(center + d, color, d.normalized(), QVector3D()));

			d.setX(radius * cosf(phi2) * cosf(theta2));
			d.setY(radius * cosf(phi2) * sinf(theta2));
			d.setZ(radius * sinf(phi2));
			vert.push_back(Vertex(center + d, color, d.normalized(), QVector3D()));

			d.setX(radius * cosf(phi2) * cosf(theta1));
			d.setY(radius * cosf(phi2) * sinf(theta1));
			vert.push_back(Vertex(center + d, color, d.normalized(), QVector3D()));
		}
	}

	addStaticGeometry(geoName, vert, "", GL_QUADS, 1|mode_Lighting);
}

void VBORenderManager::addBox(const QString& geoName, const QVector3D& center, const QVector3D& size, const QColor& color) {
	std::vector<Vertex> vert;

	QVector3D pt(center - size * 0.5);
	vert.push_back(Vertex(pt, color, QVector3D(0, -1, 0), QVector3D()));
	pt.setX(pt.x() + size.x());
	vert.push_back(Vertex(pt, color, QVector3D(0, -1, 0), QVector3D()));
	pt.setZ(pt.z() + size.z());
	vert.push_back(Vertex(pt, color, QVector3D(0, -1, 0), QVector3D()));
	pt.setX(pt.x() - size.x());
	vert.push_back(Vertex(pt, color, QVector3D(0, -1, 0), QVector3D()));

	pt = center - size * 0.5;
	pt.setX(pt.x() + size.x());
	vert.push_back(Vertex(pt, color, QVector3D(1, 0, 0), QVector3D()));
	pt.setY(pt.y() + size.y());
	vert.push_back(Vertex(pt, color, QVector3D(1, 0, 0), QVector3D()));
	pt.setZ(pt.z() + size.z());
	vert.push_back(Vertex(pt, color, QVector3D(1, 0, 0), QVector3D()));
	pt.setY(pt.y() - size.y());
	vert.push_back(Vertex(pt, color, QVector3D(1, 0, 0), QVector3D()));

	pt = center + size * 0.5;
	pt.setZ(pt.z() - size.z());
	vert.push_back(Vertex(pt, color, QVector3D(0, 1, 0), QVector3D()));
	pt.setX(pt.x() - size.x());
	vert.push_back(Vertex(pt, color, QVector3D(0, 1, 0), QVector3D()));
	pt.setZ(pt.z() + size.z());
	vert.push_back(Vertex(pt, color, QVector3D(0, 1, 0), QVector3D()));
	pt.setX(pt.x() + size.x());
	vert.push_back(Vertex(pt, color, QVector3D(0, 1, 0), QVector3D()));

	pt = center - size * 0.5;
	pt.setY(pt.y() + size.y());
	vert.push_back(Vertex(pt, color, QVector3D(-1, 0, 0), QVector3D()));
	pt.setY(pt.y() - size.y());
	vert.push_back(Vertex(pt, color, QVector3D(-1, 0, 0), QVector3D()));
	pt.setZ(pt.z() + size.z());
	vert.push_back(Vertex(pt, color, QVector3D(-1, 0, 0), QVector3D()));
	pt.setY(pt.y() + size.y());
	vert.push_back(Vertex(pt, color, QVector3D(-1, 0, 0), QVector3D()));

	pt = center - size * 0.5;
	pt.setZ(pt.z() + size.z());
	vert.push_back(Vertex(pt, color, QVector3D(0, 0, 1), QVector3D()));
	pt.setX(pt.x() + size.x());
	vert.push_back(Vertex(pt, color, QVector3D(0, 0, 1), QVector3D()));
	pt.setY(pt.y() + size.y());
	vert.push_back(Vertex(pt, color, QVector3D(0, 0, 1), QVector3D()));
	pt.setX(pt.x() - size.x());
	vert.push_back(Vertex(pt, color, QVector3D(0, 0, 1), QVector3D()));

	addStaticGeometry(geoName, vert, "", GL_QUADS, 1|mode_Lighting);
}

void VBORenderManager::addBox(const QString& geoName, const QVector3D& offset, const QVector3D& vec1, const QVector3D& vec2, float height, const QString& textureName, int faceNo, float s0, float t0, float s1, float t1) {
	QVector3D vec3(0, 0, height);

	std::vector<QVector3D> pts;
	pts.push_back(offset);
	pts.push_back(offset + vec1);
	pts.push_back(offset + vec1 + vec2);
	pts.push_back(offset + vec2);

	pts.push_back(offset + vec3);
	pts.push_back(offset + vec1 + vec3);
	pts.push_back(offset + vec1 + vec2 + vec3);
	pts.push_back(offset + vec2 + vec3);

	std::vector<Vertex> verts;
	if (faceNo == 2 || faceNo == 0) { // 右面
		verts.push_back(Vertex(pts[1], QColor(), vec1.normalized(), QVector3D(s0, t0, 0)));
		verts.push_back(Vertex(pts[2], QColor(), vec1.normalized(), QVector3D(s1, t0, 0)));
		verts.push_back(Vertex(pts[6], QColor(), vec1.normalized(), QVector3D(s1, t1, 0)));
		verts.push_back(Vertex(pts[5], QColor(), vec1.normalized(), QVector3D(s0, t1, 0)));
	}
	if (faceNo == 4 || faceNo == 0) { // 左面
		verts.push_back(Vertex(pts[3], QColor(), -vec1.normalized(), QVector3D(s0, t0, 0)));
		verts.push_back(Vertex(pts[0], QColor(), -vec1.normalized(), QVector3D(s1, t0, 0)));
		verts.push_back(Vertex(pts[4], QColor(), -vec1.normalized(), QVector3D(s1, t1, 0)));
		verts.push_back(Vertex(pts[7], QColor(), -vec1.normalized(), QVector3D(s0, t1, 0)));
	}
	if (faceNo == 1 || faceNo == 0) { // 背面
		verts.push_back(Vertex(pts[2], QColor(), vec2.normalized(), QVector3D(s0, t0, 0)));
		verts.push_back(Vertex(pts[3], QColor(), vec2.normalized(), QVector3D(s1, t0, 0)));
		verts.push_back(Vertex(pts[7], QColor(), vec2.normalized(), QVector3D(s1, t1, 0)));
		verts.push_back(Vertex(pts[6], QColor(), vec2.normalized(), QVector3D(s0, t1, 0)));
	}
	if (faceNo == 3 || faceNo == 0) { // 正面
		verts.push_back(Vertex(pts[0], QColor(), -vec2.normalized(), QVector3D(s0, t0, 0)));
		verts.push_back(Vertex(pts[1], QColor(), -vec2.normalized(), QVector3D(s1, t0, 0)));
		verts.push_back(Vertex(pts[5], QColor(), -vec2.normalized(), QVector3D(s1, t1, 0)));
		verts.push_back(Vertex(pts[4], QColor(), -vec2.normalized(), QVector3D(s0, t1, 0)));
	}
	if (faceNo == 5 || faceNo == 0) { // 上面
		verts.push_back(Vertex(pts[4], QColor(), vec3.normalized(), QVector3D(s0, t0, 0)));
		verts.push_back(Vertex(pts[5], QColor(), vec3.normalized(), QVector3D(s1, t0, 0)));
		verts.push_back(Vertex(pts[6], QColor(), vec3.normalized(), QVector3D(s1, t1, 0)));
		verts.push_back(Vertex(pts[7], QColor(), vec3.normalized(), QVector3D(s0, t1, 0)));
	}
	if (faceNo == 6 || faceNo == 0) { // 底面
		verts.push_back(Vertex(pts[3], QColor(), -vec3.normalized(), QVector3D(s0, t0, 0)));
		verts.push_back(Vertex(pts[2], QColor(), -vec3.normalized(), QVector3D(s1, t0, 0)));
		verts.push_back(Vertex(pts[1], QColor(), -vec3.normalized(), QVector3D(s1, t1, 0)));
		verts.push_back(Vertex(pts[0], QColor(), -vec3.normalized(), QVector3D(s0, t1, 0)));
	}

	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

void VBORenderManager::addLine(const QString& geoName, const QVector3D& pt1, const QVector3D& pt2, const QColor& color) {
	std::vector<Vertex> vert;

	vert.push_back(Vertex(pt1.x(), pt1.y(), pt1.z(), color.redF(), color.greenF(), color.blueF(), color.alphaF(), 0, 0, 1, 0, 0, 0));
	vert.push_back(Vertex(pt2.x(), pt2.y(), pt2.z(), color.redF(), color.greenF(), color.blueF(), color.alphaF(), 0, 0, 1, 0, 0, 0));

	addStaticGeometry(geoName, vert, "", GL_LINES, 1);
}

/**
 * 指定されたポリゴンに基づいて、ジオメトリを生成する。（テクスチャ版）
 * 凹型のポリゴンにも対応するよう、ポリゴンは台形にtessellateする。
 */
void VBORenderManager::addPolygon(const QString& geoName, Loop3D& polygon, float z, const QString& textureName, const QVector3D& texScale) {
	if (polygon.size() < 3) return;

	VBORenderManager::PolygonSetP polySet;
	VBORenderManager::polygonP tempPolyP;

	// GEN (to find the OBB of the polygon)
	QVector3D size;
	QMatrix4x4 xformMat;
	Polygon3D::getLoopOBB2(polygon, size, xformMat);
	Loop3D xformPos;
	Polygon3D::transformLoop(polygon, xformPos, xformMat);

	float minX=FLT_MAX,minY=FLT_MAX;
	float maxX=-FLT_MAX,maxY=-FLT_MAX;

	// ポリゴンの頂点情報を、pointP型に変換する
	std::vector<VBORenderManager::pointP> vP;
	vP.resize(polygon.size());
	for (int pN = 0; pN < xformPos.size(); pN++) {
		vP[pN]=boost::polygon::construct<VBORenderManager::pointP>(polygon[pN].x(), polygon[pN].y());
		minX=std::min<float>(minX,xformPos[pN].x());
		minY=std::min<float>(minY,xformPos[pN].y());
		maxX=std::max<float>(maxX,xformPos[pN].x());
		maxY=std::max<float>(maxY,xformPos[pN].y());
	}
	// GEN



	// 台形にtessellateする
	boost::polygon::set_points(tempPolyP,vP.begin(),vP.end());
	polySet+=tempPolyP;
	std::vector<VBORenderManager::polygonP> allP;
	boost::polygon::get_trapezoids(allP,polySet);

	// 各台形について
	std::vector<Vertex> verts;
	for (int pN = 0; pN < allP.size(); pN++) {
		boost::polygon::polygon_with_holes_data<double>::iterator_type itPoly=allP[pN].begin();

		// 各台形について、pointsに頂点座標を格納する
		Polygon3D points;
		std::vector<QVector3D> texP;
		while (itPoly != allP[pN].end()) {
			VBORenderManager::pointP cP = *itPoly;
			points.push_back(QVector3D(cP.x(), cP.y(), z));

			QVector3D cP2 = xformMat * QVector3D(cP.x(), cP.y(), 0);
			texP.push_back(QVector3D((cP2.x() - minX) / size.x(), (cP2.y() - minY) / size.y(), 0.0f));
			itPoly++;
		}

		// 三角形の場合は、無理やり四角形にしちゃう
		while (points.contour.size() < 4) {
			points.push_back(points.contour.back());
			texP.push_back(texP.back());
		}

		// GEN 
		// Sometimes, the polygon is formed in CW order, so we have to reorient it in CCW order.
		points.correct();

		QVector3D normal = QVector3D::crossProduct(points[1] - points[0], points[2] - points[0]).normalized();
		verts.push_back(Vertex(points[0], QColor(), normal, texP[0]));
		verts.push_back(Vertex(points[1], QColor(), normal, texP[1]));
		verts.push_back(Vertex(points[2], QColor(), normal, texP[2]));
		verts.push_back(Vertex(points[3], QColor(), normal, texP[3]));
	}

	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

/**
 * 指定されたポリゴンに基づいて、ジオメトリを生成する。（カラー版）
 * 凹型のポリゴンにも対応するよう、ポリゴンは台形にtessellateする。
 */
void VBORenderManager::addPolygon(const QString& geoName, Loop3D& polygon, float z, const QColor& color, bool inverseLoop) {
	if (polygon.size() < 3) return;

	PolygonSetP polySet;
	polygonP tempPolyP;

	// ポリゴンの頂点情報を、pointP型に変換する
	std::vector<pointP> vP;
	vP.resize(polygon.size());
	for (int pN = 0; pN < polygon.size(); pN++) {
		vP[pN]=boost::polygon::construct<pointP>(polygon[pN].x(), polygon[pN].y());
	}

	// ポリゴンをclosedにする
	if (polygon.back().x() != polygon.front().x() && polygon.back().y() != polygon.front().y()) {
		vP.push_back(vP[0]);
	}

	// 台形にtessellateする
	boost::polygon::set_points(tempPolyP, vP.begin(), vP.end());
	polySet += tempPolyP;
	std::vector<polygonP> allP;
	boost::polygon::get_trapezoids(allP,polySet);
		
	// 各台形について
	std::vector<Vertex> verts;
	for (int pN = 0; pN < allP.size(); pN++) {
		boost::polygon::polygon_with_holes_data<double>::iterator_type itPoly = allP[pN].begin();

		// 各台形について、pointsに頂点座標を格納する
		Polygon3D points;
		while (itPoly != allP[pN].end()) {
			pointP cP = *itPoly;

			points.push_back(QVector3D(cP.x(), cP.y(), z));

			itPoly++;
		}
		if (points.contour.size() == 0) continue;

		// 三角形の場合は、無理やり四角形にしちゃう
		while (points.contour.size() < 4)
			points.push_back(points.contour.back());

		// GEN 
		// Sometimes, the polygon is formed in CW order, so we have to reorient it in CCW order.
		points.correct();
		
		QVector3D normal = QVector3D::crossProduct(points[1] - points[0], points[2] - points[0]).normalized();
		if (inverseLoop) {
			verts.push_back(Vertex(points[0], color, -normal, QVector3D()));
			verts.push_back(Vertex(points[3], color, -normal, QVector3D()));
			verts.push_back(Vertex(points[2], color, -normal, QVector3D()));
			verts.push_back(Vertex(points[1], color, -normal, QVector3D()));
		} else {
			verts.push_back(Vertex(points[0], color, normal, QVector3D()));
			verts.push_back(Vertex(points[1], color, normal, QVector3D()));
			verts.push_back(Vertex(points[2], color, normal, QVector3D()));
			verts.push_back(Vertex(points[3], color, normal, QVector3D()));
		}
	}

	addStaticGeometry(geoName, verts, "", GL_QUADS, 1|mode_Lighting);
}

/**
 * ポリゴンプリズムを構築する。（テクスチャ版）
 * テクスチャは側面のみ。上面と底面はなし。
 */
void VBORenderManager::addPrism(const QString& geoName, Loop3D& polygon, float baseHeight, float topHeight, const QString& textureName) {
	std::vector<Vertex> verts;

	float accuLength = 0.0f;
	for (int i = 0; i < polygon.size(); i++) {
		int next = (i + 1) % polygon.size();
		float length = (polygon[next] - polygon[i]).length();

		QVector3D normal = QVector3D::crossProduct(polygon[next] - polygon[i], QVector3D(0,0,1)).normalized();
		verts.push_back(Vertex(QVector3D(polygon[i].x(), polygon[i].y(), baseHeight), QColor(), normal, QVector3D(accuLength, baseHeight, 0)));
		verts.push_back(Vertex(QVector3D(polygon[next].x(), polygon[next].y(), baseHeight), QColor(), normal, QVector3D(accuLength + length, baseHeight, 0)));
		verts.push_back(Vertex(QVector3D(polygon[next].x(), polygon[next].y(), topHeight), QColor(), normal, QVector3D(accuLength + length, topHeight, 0)));
		verts.push_back(Vertex(QVector3D(polygon[i].x(), polygon[i].y(), topHeight), QColor(), normal, QVector3D(accuLength, topHeight, 0)));

		accuLength += length;
	}
	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

/**
 * ポリゴンプリズムを構築する。（カラー版）
 * addTopAndBase=trueなら、上面と底面を追加する。
 */
void VBORenderManager::addPrism(const QString& geoName, Loop3D& polygon, float baseHeight, float topHeight, const QColor& color, bool addTopAndBase) {
	std::vector<Vertex> verts;

	for (int i = 0; i < polygon.size(); i++) {
		int next = (i + 1) % polygon.size();

		QVector3D normal = QVector3D::crossProduct(polygon[next] - polygon[i], QVector3D(0,0,1)).normalized();
		verts.push_back(Vertex(QVector3D(polygon[i].x(), polygon[i].y(), baseHeight), color, normal, QVector3D()));
		verts.push_back(Vertex(QVector3D(polygon[next].x(), polygon[next].y(), baseHeight), color, normal, QVector3D()));
		verts.push_back(Vertex(QVector3D(polygon[next].x(), polygon[next].y(), topHeight), color, normal, QVector3D()));
		verts.push_back(Vertex(QVector3D(polygon[i].x(), polygon[i].y(), topHeight), color, normal, QVector3D()));			
	}
	addStaticGeometry(geoName, verts, "", GL_QUADS, 1|mode_Lighting);

	if (addTopAndBase) {
		addPolygon(geoName, polygon, baseHeight, color, true);
		addPolygon(geoName, polygon, topHeight, color, false);
	}
}

void VBORenderManager::addWedge(const QString& geoName, Loop3D& polygon, float baseHeight, float topHeight, const QString& textureName) {
	std::vector<Vertex> verts;

	{ // 手前の三角形
		QVector3D normal = (polygon[1] - polygon[2]).normalized();
		float length = (polygon[1] - polygon[0]).length();
		verts.push_back(Vertex(QVector3D(polygon[0].x(), polygon[0].y(), baseHeight), QColor(), normal, QVector3D(0, baseHeight, 0)));
		verts.push_back(Vertex(QVector3D(polygon[1].x(), polygon[1].y(), baseHeight), QColor(), normal, QVector3D(length, baseHeight, 0)));
		verts.push_back(Vertex(QVector3D((polygon[0].x() + polygon[1].x()) * 0.5, (polygon[0].y() + polygon[1].y()) * 0.5, topHeight), QColor(), normal, QVector3D(length * 0.5, topHeight, 0)));
		verts.push_back(Vertex(QVector3D((polygon[0].x() + polygon[1].x()) * 0.5, (polygon[0].y() + polygon[1].y()) * 0.5, topHeight), QColor(), normal, QVector3D(length * 0.5, topHeight, 0)));
	}

	{ // 奥の三角形
		QVector3D normal = (polygon[2] - polygon[1]).normalized();
		float length = (polygon[3] - polygon[2]).length();
		verts.push_back(Vertex(QVector3D(polygon[2].x(), polygon[2].y(), baseHeight), QColor(), normal, QVector3D(0, baseHeight, 0)));
		verts.push_back(Vertex(QVector3D(polygon[3].x(), polygon[3].y(), baseHeight), QColor(), normal, QVector3D(length, baseHeight, 0)));
		verts.push_back(Vertex(QVector3D((polygon[2].x() + polygon[3].x()) * 0.5, (polygon[2].y() + polygon[3].y()) * 0.5, topHeight), QColor(), normal, QVector3D(length * 0.5, topHeight, 0)));
		verts.push_back(Vertex(QVector3D((polygon[2].x() + polygon[3].x()) * 0.5, (polygon[2].y() + polygon[3].y()) * 0.5, topHeight), QColor(), normal, QVector3D(length * 0.5, topHeight, 0)));
	}

	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

/**
 * 円柱を構築する。（テクスチャ版）
* テクスチャは側面のみ。上面と底面はなし。
 */
void VBORenderManager::addCylinder(const QString& geoName, const QVector3D& center, float baseRadius, float topRadius, float height, const QString& textureName) {
	std::vector<Vertex> verts;

	int slices = 10;
	int stacks = 10;
	for (int i = 0; i < stacks; ++i) {
		float z1 = height / stacks * i;
		float z2 = height / stacks * (i + 1);
		float radius1 = (topRadius - baseRadius) / stacks * i + baseRadius;
		float radius2 = (topRadius - baseRadius) / stacks * (i + 1) + baseRadius;

		for (int j = 0; j < slices; ++j) {
			float theta1 = 2.0 * M_PI * j / slices;
			float theta2 = 2.0 * M_PI * (j + 1) / slices;
			float x1 = radius1 * cosf(theta1);
			float y1 = radius1 * sinf(theta1);
			float x2 = radius1 * cosf(theta2);
			float y2 = radius1 * sinf(theta2);
			float x3 = radius2 * cosf(theta2);
			float y3 = radius2 * sinf(theta2);
			float x4 = radius2 * cosf(theta1);
			float y4 = radius2 * sinf(theta1);

			verts.push_back(Vertex(QVector3D(x1, y1, z1) + center, QColor(), QVector3D(x1, y1, 0), QVector3D((float)j / slices, (float)i / stacks, 0)));
			verts.push_back(Vertex(QVector3D(x2, y2, z1) + center, QColor(), QVector3D(x2, y2, 0), QVector3D((float)(j+1) / slices, (float)i / stacks, 0)));
			verts.push_back(Vertex(QVector3D(x3, y3, z2) + center, QColor(), QVector3D(x3, y3, 0), QVector3D((float)(j+1) / slices, (float)(i+1) / stacks, 0)));
			verts.push_back(Vertex(QVector3D(x4, y4, z2) + center, QColor(), QVector3D(x4, y4, 0), QVector3D((float)j / slices, (float)(i+1) / stacks, 0)));
		}
	}

	addStaticGeometry(geoName, verts, textureName, GL_QUADS, 2|mode_Lighting);
}

/**
	* 指定されたポリゴンに基づいて、ジオメトリを生成する。
	* 凹型のポリゴンにも対応するよう、ポリゴンは台形にtessellateする。
	*/
bool VBORenderManager::addStaticGeometry2(QString geoName,std::vector<QVector3D>& pos,float zShift,bool inverseLoop,QString textureName,GLenum geometryType,int shaderMode,QVector3D texScale,QColor color){
	if(pos.size()<3){
		return false;
	}
	PolygonSetP polySet;
	polygonP tempPolyP;

	std::vector<pointP> vP;
	vP.resize(pos.size());
	float minX=FLT_MAX,minY=FLT_MAX;
	float maxX=-FLT_MAX,maxY=-FLT_MAX;

	for(int pN=0;pN<pos.size();pN++){
		vP[pN]=boost::polygon::construct<pointP>(pos[pN].x(),pos[pN].y());
		minX=std::min<float>(minX,pos[pN].x());
		minY=std::min<float>(minY,pos[pN].y());
		maxX=std::max<float>(maxX,pos[pN].x());
		maxY=std::max<float>(maxY,pos[pN].y());
	}
	if(pos.back().x()!=pos.front().x()&&pos.back().y()!=pos.front().y()){
		vP.push_back(vP[0]);
	}

	boost::polygon::set_points(tempPolyP,vP.begin(),vP.end());
	polySet+=tempPolyP;
	std::vector<polygonP> allP;
	boost::polygon::get_trapezoids(allP,polySet);
		
	std::vector<Vertex> vert;

	for(int pN=0;pN<allP.size();pN++){
		//glColor3ub(qrand()%255,qrand()%255,qrand()%255);
		boost::polygon::polygon_with_holes_data<double>::iterator_type itPoly=allP[pN].begin();

		Polygon3D points;
		//std::vector<QVector3D> points;
		std::vector<QVector3D> texP;
		while(itPoly!=allP[pN].end()){
			pointP cP=*itPoly;
			if(inverseLoop==false)
				points.push_back(QVector3D(cP.x(),cP.y(),pos[0].z()+zShift));
			else
				points.contour.insert(points.contour.begin(),QVector3D(cP.x(),cP.y(),pos[0].z()+zShift));

			//if(texZeroToOne==true){
				//texP.push_back(QVector3D((cP.x()-minX)/(maxX-minX),(cP.y()-minY)/(maxY-minY),0.0f));
			//}else{
				texP.push_back(QVector3D((cP.x()-minX)*texScale.x(),(cP.y()-minY)*texScale.y(),0.0f));
			//}
			itPoly++;
		}
		if(points.contour.size()==0)continue;
		while(points.contour.size()<4)
			points.push_back(points.contour.back());

		// GEN 
		// Sometimes, the polygon is formed in CW order, so we have to reorient it in CCW order.
		if(inverseLoop==false) {
			points.correct();
		}

		/*if(points.size()==4){//last vertex repited
			addTexTriang(texInd,points,texP,col,norm);
		}
		if(points.size()==5){
			addTexQuad(texInd,points,texP,col,norm);

		}
		if(points.size()==6){
			//addTexQuad(texInd,std::vector<QVector3D>(&points[0],&points[3]),std::vector<QVector3D>(&texP[0],&texP[3]),col,norm);

			addTexQuad(texInd,points,texP,col,norm);
			//addTexTriang(texInd,std::vector<QVector3D>(&points[3],&points[6]),std::vector<QVector3D>(&texP[3],&texP[6]),col,norm);
			//addTexTriang(texInd,std::vector<QVector3D>(&points[4],&points[6]),std::vector<QVector3D>(&texP[4],&texP[6]),col,norm);
		}*/
		vert.push_back(Vertex(points[0],color,QVector3D(0,0,1),texP[0]));//texScale is a hack to define a color when it is not texture
		vert.push_back(Vertex(points[1],color,QVector3D(0,0,1),texP[1]));
		vert.push_back(Vertex(points[2],color,QVector3D(0,0,1),texP[2]));
		vert.push_back(Vertex(points[3],color,QVector3D(0,0,1),texP[3]));
	}

	return addStaticGeometry(geoName,vert,textureName,geometryType,shaderMode);
}//

bool VBORenderManager::removeStaticGeometry(QString geoName){
	if(geoName2StaticRender.contains(geoName)){
		// iterate and remove
		renderGrid::iterator i;
		for (i = geoName2StaticRender[geoName].begin(); i != geoName2StaticRender[geoName].end(); ++i){
			cleanVAO(geoName2StaticRender[geoName][i.key()].vbo,geoName2StaticRender[geoName][i.key()].vao);
		}
		geoName2StaticRender[geoName].clear();
		geoName2StaticRender.remove(geoName);
	}else{
		//printf("ERROR: Remove Geometry %s but it did not exist\n",geoName.toAscii().constData());
		return false;
	}

	return true;
}//

void VBORenderManager::renderStaticGeometry(QString geoName){

	if(geoName2StaticRender.contains(geoName)){
		// iterate and remove
		renderGrid::iterator i;
		for (i = geoName2StaticRender[geoName].begin(); i != geoName2StaticRender[geoName].end(); ++i){
			renderVAO(i.value(),false);
		}
	}else{
		//printf("ERROR: Render Geometry %s but it did not exist\n",geoName.toAscii().constData());
		return;
	}
}//

////////////////////////////////////////////////////////////////////
// MODEL
void VBORenderManager::addStreetElementModel(QString name,ModelSpec mSpec){
	nameToVectorModels[name].push_back(mSpec);
}//
void VBORenderManager::renderAllStreetElementName(QString name){
	for(int i=0;i<nameToVectorModels[name].size();i++){
		VBOModel_StreetElements::renderOneStreetElement(program,nameToVectorModels[name][i]);
	}
	//printf("name %s --> %d\n",name.toAscii().constData(),nameToVectorModels[name].size());
}//
void VBORenderManager::removeAllStreetElementName(QString name){
	nameToVectorModels[name].clear();
}
	
