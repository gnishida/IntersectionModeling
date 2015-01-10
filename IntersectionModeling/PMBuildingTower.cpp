/************************************************************************************************
*		Procedural City Generation: Buildings geometry
*		@author igarciad
************************************************************************************************/

#include "PMBuildingTower.h"
#include "qdir.h"

const float storyHeight = 4.2f;

bool PMBuildingTower::initialized = false;

std::vector<QString> PMBuildingTower::facadeTex;
std::vector<QVector3D> PMBuildingTower::facadeScale;
std::vector<QString> PMBuildingTower::windowTex;
std::vector<QString> PMBuildingTower::roofTex;

void PMBuildingTower::initialize() {
	if (initialized) return;

	QString pathName="../data/textures/LC/";
	QStringList nameFilter;
	nameFilter << "*.png" << "*.jpg" << "*.gif";
	// 1. facade
	QDir directory(pathName+"facade/");
	QStringList list = directory.entryList( nameFilter, QDir::Files );
	for(int lE=0;lE<list.size();lE++){
		facadeTex.push_back(pathName+"/facade/"+list[lE]);
		QStringList scaleS=list[lE].split("_");
		//printf("*********** scaleS %d\n",scaleS.size());
		if(scaleS.size()!=4)
			facadeScale.push_back(QVector3D(1.0f,1.0f,0));
		else{
			facadeScale.push_back(QVector3D(scaleS[1].toFloat(),scaleS[2].toFloat(),0));
			//printf("Scale %s -->%f %f\n",list[lE].toAscii().constData(),scaleS[1].toFloat(),scaleS[2].toFloat());
		}
	}

	// 2. windows
	QDir directoryW(pathName+"wind/");
	list = directoryW.entryList( nameFilter, QDir::Files );
	for(int lE=0;lE<list.size();lE++){
		windowTex.push_back(pathName+"wind/"+list[lE]);
	}

	// 3. roof
	QDir directoryR(pathName+"roof/");
	list = directoryR.entryList( nameFilter, QDir::Files );
	for(int lE=0;lE<list.size();lE++){
		roofTex.push_back(pathName+"roof/"+list[lE]);
	}

	initialized = true;
}

void PMBuildingTower::calculateColumnContour(std::vector<QVector3D>& currentContour, std::vector<QVector3D>& columnContour) {
	QVector3D pos1,pos2;
	for(int sN=0;sN<currentContour.size();sN++){
		int ind1=sN;
		int ind2=(sN+1)%currentContour.size();
		pos1=currentContour[ind1];
		pos2=currentContour[ind2];
		QVector3D dirV=(pos2-pos1);
		float leng=(dirV).length();
		dirV/=leng;
		if(leng>7.0f){
			QVector3D perDir=QVector3D::crossProduct(dirV,QVector3D(0,0,1.0f));

			float remindingL=leng-1.0f-1.5f;
			int numWindows=remindingL/(3.0f+1.5f);
			float windowWidth=(leng-1.0f-1.5f*(1+numWindows))/numWindows;

			columnContour.push_back(pos1);
			//first col
			columnContour.push_back(pos1+0.5f*dirV);// first col
			columnContour.push_back(pos1+0.5f*dirV+0.8f*perDir);
			columnContour.push_back(pos1+(0.5f+1.5f)*dirV+0.8f*perDir);
			columnContour.push_back(pos1+(0.5f+1.5f)*dirV);
			QVector3D cPos=pos1+(0.5f+1.5f)*dirV;
			for(int nW=0;nW<numWindows;nW++){
				//window
				columnContour.push_back(cPos+(windowWidth)*dirV);
				//column
				columnContour.push_back(cPos+(windowWidth)*dirV+0.8f*perDir);
				columnContour.push_back(cPos+(windowWidth+1.5f)*dirV+0.8f*perDir);
				columnContour.push_back(cPos+(windowWidth+1.5f)*dirV);
				cPos+=dirV*(windowWidth+1.5f);
			}

		}else{
			columnContour.push_back(pos1);
		}
	}
}//

void PMBuildingTower::addWindow(VBORenderManager& rendManager, const QString& geoName, int windowTexId, const QVector3D& initPoint, const QVector3D& dirR, const QVector3D& dirUp, float width, float height) {
	std::vector<Vertex> vertWind;

	float depth = 2.0f;
	// IN: TOP
	QVector3D perI = QVector3D::crossProduct(dirUp,dirR);//note direction: to inside
	QVector3D vert[8];
	vert[0]=initPoint;
	vert[1]=initPoint+perI*depth;
	vert[2]=initPoint+perI*depth+dirUp*height;
	vert[3]=initPoint+dirUp*height;

	vert[4]=initPoint+perI*depth+dirR*width;
	vert[5]=initPoint+dirR*width;
	vert[6]=initPoint+dirUp*height+dirR*width;
	vert[7]=initPoint+perI*depth+dirUp*height+dirR*width;

	QColor color = QColor(0.5f,0.5f,0.5f);
	// LEFT
	QVector3D norm;
	norm=QVector3D::crossProduct(vert[1]-vert[0],vert[3]-vert[0]);
	vertWind.push_back(Vertex(vert[0],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[1],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[2],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[3],color,norm,QVector3D()));
	// RIGHT
	norm=QVector3D::crossProduct(vert[5]-vert[4],vert[7]-vert[4]);
	vertWind.push_back(Vertex(vert[4],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[5],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[6],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[7],color,norm,QVector3D()));
	// TOP
	norm=QVector3D::crossProduct(vert[7]-vert[2],vert[3]-vert[2]);
	vertWind.push_back(Vertex(vert[2],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[7],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[6],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[3],color,norm,QVector3D()));
	// BOT
	norm=QVector3D::crossProduct(vert[5]-vert[0],vert[1]-vert[0]);
	vertWind.push_back(Vertex(vert[0],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[5],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[4],color,norm,QVector3D()));
	vertWind.push_back(Vertex(vert[1],color,norm,QVector3D()));
	rendManager.addStaticGeometry(geoName, vertWind, "", GL_QUADS, 1|mode_Lighting);
	// BACK
	vertWind.clear();
	norm=QVector3D::crossProduct(vert[4]-vert[1],vert[2]-vert[1]);
	vertWind.push_back(Vertex(vert[1],color,norm,QVector3D(0,0,0)));
	vertWind.push_back(Vertex(vert[4],color,norm,QVector3D(1,0,0)));
	vertWind.push_back(Vertex(vert[7],color,norm,QVector3D(1,1,0)));
	vertWind.push_back(Vertex(vert[2],color,norm,QVector3D(0,1,0)));

	rendManager.addStaticGeometry(geoName, vertWind, windowTex[windowTexId], GL_QUADS, 2|mode_Lighting);
}

void PMBuildingTower::addColumnGeometry(VBORenderManager& rendManager, const QString& geoName, std::vector<QVector3D>& columnContour, int randomFacade, int windowTexId, float uS, float vS, float height, int numFloors) {
	std::vector<Vertex> vert;

	float verticalHoleSize = 0.5;
	float horHoleSize = 0.5;
	float accPerimeter = 0;
	QVector3D norm;
	for (int sN = 0; sN < columnContour.size(); sN++) {
		int ind1 = sN;
		int ind2 = (sN+1) % columnContour.size();
		std::vector<QVector3D> em;
		float sideLenght=(columnContour[ind1]-columnContour[ind2]).length();
		if (sideLenght <= 3.0f) {
			float heightB=height;
			float heightT=numFloors*storyHeight+height;

			QVector3D norm=QVector3D::crossProduct(columnContour[ind2]-columnContour[ind1],QVector3D(0,0,1.0f));
			vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,heightB),QColor(),norm,QVector3D(accPerimeter*uS,heightB*vS,0.0f)));
			vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,heightB),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,heightB*vS,0.0f)));
			vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,heightT),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,heightT*vS,0.0f)));
			vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,heightT),QColor(),norm,QVector3D((accPerimeter)*uS,heightT*vS,0.0f)));
		} else {
			for (int numF = 0; numF < numFloors; numF++) {
				float h0 = numF*storyHeight+height;
				float h3 = (numF+1)*storyHeight+height;
				float h1 = h0+verticalHoleSize;
				float h2 = h3-verticalHoleSize;
				norm=QVector3D::crossProduct(columnContour[ind2]-columnContour[ind1],QVector3D(0,0,1.0f));
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h0),QColor(),norm,QVector3D(accPerimeter*uS,h0*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h0),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,h0*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h1),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,h1*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h1),QColor(),norm,QVector3D((accPerimeter)*uS,h1*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h2),QColor(),norm,QVector3D(accPerimeter*uS,h2*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h2),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,h2*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h3),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,h3*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h3),QColor(),norm,QVector3D((accPerimeter)*uS,h3*vS,0.0f)));
				// LEFT
				QVector3D dirW = columnContour[ind2] - columnContour[ind1];
				dirW /= sideLenght;
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h1),QColor(),norm,QVector3D(accPerimeter*uS,h1*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h1)+dirW*horHoleSize,QColor(),norm,QVector3D((accPerimeter+horHoleSize)*uS,h1*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h2)+dirW*horHoleSize,QColor(),norm,QVector3D((accPerimeter+horHoleSize)*uS,h2*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind1]+QVector3D(0,0,h2),QColor(),norm,QVector3D((accPerimeter)*uS,h2*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h1)-dirW*horHoleSize,QColor(),norm,QVector3D((accPerimeter+sideLenght-horHoleSize)*uS,h1*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h1),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,h1*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h2),QColor(),norm,QVector3D((accPerimeter+sideLenght)*uS,h2*vS,0.0f)));
				vert.push_back(Vertex(columnContour[ind2]+QVector3D(0,0,h2)-dirW*horHoleSize,QColor(),norm,QVector3D((accPerimeter+sideLenght-horHoleSize)*uS,h2*vS,0.0f)));

				////////// INSIDE
				addWindow(rendManager, geoName, windowTexId, columnContour[ind1]+QVector3D(0,0,h1)+dirW*horHoleSize, dirW, QVector3D(0,0,1.0f), sideLenght-2*horHoleSize, h2-h1);
			}
		}
		accPerimeter+=sideLenght;
	}
	rendManager.addStaticGeometry(geoName,vert,facadeTex[randomFacade],GL_QUADS,2|mode_Lighting);
}

void PMBuildingTower::generate(VBORenderManager& rendManager, const QString& geoName, Building& building) {
	initialize();

	float z = building.buildingFootprint.contour[0].z();
	float boxSize = 1.0f;
	float firstFloorHeight = 4.8f;
	float buildingHeight = (building.numStories - 1) * storyHeight + firstFloorHeight + boxSize;//just one box size (1st-2nd)

	// roofOffContを作成 (footprintを少し大きくする)
	Loop3D roofContour;
	building.buildingFootprint.computeInset(-boxSize, roofContour, false); 

	// １階部分を構築
	rendManager.addPrism(geoName, building.buildingFootprint.contour, z, z + firstFloorHeight, building.color, false);
	rendManager.addPrism(geoName, roofContour, z + firstFloorHeight, z + firstFloorHeight + boxSize, building.color, true);

	// ファサードのcontourを計算する
	std::vector<QVector3D> columnContour;
	calculateColumnContour(building.buildingFootprint.contour, columnContour);

	// ファサードを追加する
	int randomFacade = qrand()%facadeTex.size();
	float uS = facadeScale[randomFacade].x();
	float vS = facadeScale[randomFacade].y();
	int windowTexId = ((int)qrand()) % windowTex.size();
	addColumnGeometry(rendManager, geoName, columnContour, randomFacade, windowTexId, uS, vS, firstFloorHeight + boxSize, building.numStories-1);

	// 屋根を追加する
	rendManager.addPrism(geoName, roofContour, z + buildingHeight, z + buildingHeight + boxSize, building.color, false);
	rendManager.addPolygon(geoName, roofContour, z + buildingHeight, building.color, true);
	rendManager.addPolygon(geoName, roofContour, z + buildingHeight + boxSize, roofTex[rand()%roofTex.size()], QVector3D(1, 1, 1));
}

