#include "PMBuildingFactory.h"

bool PMBuildingFactory::initialized = false;
std::vector<QString> PMBuildingFactory::textures;
int PMBuildingFactory::NUM_SUBTYPE = 3;

void PMBuildingFactory::initialize() {
	if (initialized) return;
	
	textures.push_back("../data/buildings/IHFawley/top.jpg");//1
	textures.push_back("../data/buildings/IHFawley/tube.jpg");//2
	textures.push_back("../data/buildings/IHFawley/buld2.jpg");//4
	textures.push_back("../data/buildings/IHFawley/roof1.jpg");//6
	textures.push_back("../data/buildings/IHFawley/roof2.jpg");//7
	textures.push_back("../data/buildings/IHFawley/roof3.jpg");//8

	for(int i=0;i<NUM_SUBTYPE;i++){
		textures.push_back("../data/buildings/IHFawley/chimney_"+QString::number(i)+".jpg");//0
		textures.push_back("../data/buildings/IHFawley/buld1_"+QString::number(i)+".jpg");//1
		textures.push_back("../data/buildings/IHFawley/buld3_"+QString::number(i)+".jpg");//2
		textures.push_back("../data/buildings/IHFawley/buld12_"+QString::number(i)+".jpg");//3
	}

	initialized = true;
}

void PMBuildingFactory::generate(VBORenderManager& rendManager, const QString& geoName, Building& building) {
	initialize();

	Loop3D rectangle = building.buildingFootprint.inscribedOBB();

	float dx = (rectangle[1] - rectangle[0]).length();
	float dy = (rectangle[2] - rectangle[1]).length();
	QVector3D vec1 = (rectangle[1] - rectangle[0]).normalized();
	QVector3D vec2 = (rectangle[2] - rectangle[1]).normalized();		

	float blockX = dx;
	float blockY = dy;
	float scale = 1.0f;

	if (dx > 30.0f) {
		scale = dx / 30.0f;
	}
	if (dy < ((1.05f/2+1.05f+4.0f+2.8f) * scale)) {
		scale = dy / ((1.05f/2+1.05f+4.0f+2.8f)*scale);
	}
			
	float towerRadius = 1.05f * scale;
	float towerHeigh = 36.0f * scale;

	float distBetweenTow = (12.0 + towerRadius * 4.1f) *scale;
	float tubesWidth = 0.8f * scale;
	float distTubes = 4.0f * scale;
			
	float buld1TexWidth = 3.3f * scale;
	float buld1Height = 6.4f * scale;
	float buld1Y = 2.8f * scale;
	float roofBuld1Width = 5.5f * scale;
	float margin1X = 1.0f * scale;

	float buld2TexWidth = 3.3f * scale;
	float buld2Height = 7.6f * scale;
	float buld2Y = 2.5f * scale;
	float roofBuld2Width = 1.8f * scale;

	float buld3TexWidth = 3.3f * scale;
	float buld3Height = 5.2f * scale;
	float buld3Y = 6.0f * scale;
	float roofBuld3Width = 2.0f * scale;
	float margin3X = scale;

	QVector3D offset = rectangle[0];

	// 歩道の分、底上げする
	offset.setZ(offset.z() + 1.0f);

	int numY = 1;
	bool drawBld2, drawBld3, drawBld3Dupl;
	if (floor(dy / (towerRadius / 2 + towerRadius + distTubes + buld1Y)) >= 2.0f) {
		blockY = dy / 2.0f;
		numY = 2;
	}
			
	if (blockY > (towerRadius / 2 + towerRadius + distTubes + buld1Y + buld2Y)) {
		drawBld2 = true;
	} else {
		drawBld2 = false;
	}

	if (blockY > (towerRadius/2 + towerRadius + distTubes + buld1Y + buld2Y + buld3Y)) {
		drawBld3 = true;
		drawBld3Dupl = true;
	} else {
		if (numY == 2 && dy >= (towerRadius + 2 * towerRadius + 2 * distTubes + 2 * buld1Y + 2 * buld2Y + buld3Y)) {
			drawBld3 = true;
			drawBld3Dupl = false;
		} else {
			drawBld3 = false;
			drawBld3Dupl = false;
		}
	}

	for(int sidesY=0;sidesY<numY;sidesY++){
		int numCylinders = floor(blockX/distBetweenTow) + 1;
		numCylinders = numCylinders>20?20:numCylinders;
		float shiftY = 0;
		for (int i = 0; i < numCylinders; i++) {
			rendManager.addCylinder(geoName, (i+1)*blockX/(numCylinders+1) * vec1 + towerRadius * 0.5 * vec2 + offset, towerRadius, towerRadius * 0.5f, towerHeigh, textures[6+building.subType*4]);
			Loop3D pts;
			QVector3D pt = ((i+1)*blockX/(numCylinders+1)-tubesWidth/2.0) * vec1 + towerRadius * 0.5 * vec2 + offset;
			pts.push_back(pt);
			pts.push_back(pt + tubesWidth * vec1);
			pts.push_back(pt + tubesWidth * vec1 + (towerRadius * 0.5 + distTubes) * vec2);
			pts.push_back(pt + (towerRadius * 0.5 + distTubes) * vec2);
			rendManager.addBox(geoName, offset + ((i+1)*blockX/(numCylinders+1)-tubesWidth/2.0f) * vec1 + towerRadius/2 * vec2, vec1 * tubesWidth, vec2 * (towerRadius+distTubes), tubesWidth, textures[1]);
		}
		shiftY += towerRadius / 2 + towerRadius + distTubes;

		// ビル１
		for (int f = 1; f < 5; f++) {
			if (f == 2 || f == 4) {
				rendManager.addBox(geoName, offset + margin1X * vec1 + shiftY * vec2, vec1 * (blockX-margin1X*2), vec2 * (buld1Y), buld1Height, textures[6+building.subType*4+3], f);
			} else {
				rendManager.addBox(geoName, offset + margin1X * vec1 + shiftY * vec2, vec1 * (blockX-margin1X*2), vec2 * (buld1Y), buld1Height, textures[6+building.subType*4+1], f, 0, 0, floor(blockX/buld1TexWidth)+1.0f, 1);
			}
		}
		rendManager.addBox(geoName, offset + margin1X * vec1 + shiftY * vec2, vec1 * (blockX-margin1X*2), vec2 * (buld1Y), buld1Height, textures[3], 5, 0, 0, floor(blockX/roofBuld1Width)+1.0f, 1);
		shiftY += buld1Y;

		// ビル２
		if (drawBld2) {
			for (int f = 1; f < 5; f++) {
				if (f == 2 || f == 4) {
					rendManager.addBox(geoName, offset + shiftY * vec2, vec1 * blockX, vec2 * buld2Y, buld2Height, textures[2], f);
				} else {
					rendManager.addBox(geoName, offset + shiftY * vec2, vec1 * blockX, vec2 * buld2Y, buld2Height, textures[2], f, 0, 0, floor(blockX/buld2TexWidth)+1.0f, 1);
				}
			}
			rendManager.addBox(geoName, offset + shiftY * vec2, vec1 * blockX, vec2 * buld2Y, buld2Height, textures[4], 5, 0, 0, floor(blockX/buld2TexWidth)+1.0f, 1);
			shiftY += buld2Y;
		}

		// ビル３
		if ((drawBld3 && sidesY == 0) || (drawBld3Dupl && sidesY == 1)) {
			for (int f= 1; f < 5; f++) {
				if (f == 2 || f == 4) {
					rendManager.addBox(geoName, offset + margin3X * vec1 + shiftY * vec2, vec1 * (blockX-margin3X*2), vec2 * buld3Y, buld3Height, textures[6+building.subType*4+2], f);
				} else {
					rendManager.addBox(geoName, offset + margin3X * vec1 + shiftY * vec2, vec1 * (blockX-margin3X*2), vec2 * buld3Y, buld3Height, textures[6+building.subType*4+2], f, 0, 0, floor(blockX/buld3TexWidth)+1.0f, 1);
				}
			}
			rendManager.addBox(geoName, offset + margin3X * vec1 + shiftY * vec2, vec1 * (blockX-margin3X*2), vec2 * buld3Y, buld3Height, textures[5], 5, 0, 0, floor(blockX/buld3TexWidth)+1.0f, 1);
			shiftY += buld3Y;
		}
	}
}