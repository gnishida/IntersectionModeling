#include "PMBuildingRB.h"

bool PMBuildingRB::initialized = false;
std::vector<QString> PMBuildingRB::textures;
int PMBuildingRB::NUM_SUBTYPE = 2;

void PMBuildingRB::initialize() {
	if (initialized) return;
	
	textures.push_back("../data/buildings/RB/side_0.jpg");//0
	textures.push_back("../data/buildings/RB/back0_0.jpg");//1
	textures.push_back("../data/buildings/RB/back1_0.jpg");//2
	textures.push_back("../data/buildings/RB/frontL_0.jpg");//3
	textures.push_back("../data/buildings/RB/frontR_0.jpg");//4
	textures.push_back("../data/buildings/RB/front_0.jpg");//5

	textures.push_back("../data/buildings/RB/frontL_1.jpg");//0
	textures.push_back("../data/buildings/RB/frontL2_1.jpg");//1
	textures.push_back("../data/buildings/RB/frontL3_1.jpg");//2
	textures.push_back("../data/buildings/RB/side0_1.jpg");//3
	textures.push_back("../data/buildings/RB/side1_1.jpg");//4
	textures.push_back("../data/buildings/RB/frontL4_1.jpg");//5
	textures.push_back("../data/buildings/RB/frontR0_1.jpg");//6
	textures.push_back("../data/buildings/RB/frontR1_1.jpg");//7
	textures.push_back("../data/buildings/RB/back0_1.jpg");//8
	textures.push_back("../data/buildings/RB/rightSide_1.jpg");//9
	textures.push_back("../data/buildings/RB/leftSide_1.jpg");//10

	textures.push_back("../data/buildings/roof/roof2.jpg");

	initialized = true;
}

void PMBuildingRB::generate(VBORenderManager& rendManager, const QString& geoName, Building& building) {
	initialize();

	if (building.subType == 0) {
		generateType0(rendManager, geoName, building);
	} else {
		generateType1(rendManager, geoName, building);
	}
}

void PMBuildingRB::generateType0(VBORenderManager& rendManager, const QString& geoName, Building& building) {
	Loop3D rectangle = building.buildingFootprint.inscribedOBB();

	float dx = (rectangle[1] - rectangle[0]).length();
	float dy = (rectangle[2] - rectangle[1]).length();
	QVector3D vec1 = (rectangle[1] - rectangle[0]).normalized();
	QVector3D vec2 = (rectangle[2] - rectangle[1]).normalized();	

	float dyOrig = dy;
	float dxOrig = dx;
	QVector3D offset = rectangle[0];

	// 歩道の分、底上げする
	offset.setZ(offset.z() + 1.0f);

	////
	//roofTex=bestTextureForRoof(dxOrig,dyOrig);
	////

	bool back1 = false;
	bool front = false;

	float baseHeight=10.5f;
	float frontHeight=11.48f;
	float sideWidth=baseHeight*432/374;
	float back0Width=baseHeight*25/201;
	float back1Width=baseHeight*293/201;
	float frontLWidth=baseHeight*164/201;
	float frontRWidth=frontHeight*277/249;
	float frontWidth=baseHeight*26/227;

	if (dx > back1Width) back1 = true;
	if (dx> frontLWidth + frontRWidth) front = true;

	//11 front
	rendManager.addBox(geoName, offset, (dx-(frontLWidth+frontRWidth)*front)/2.0f * vec1, dyOrig * vec2, baseHeight, textures[5], 3, 0, 0, 1.0f*(floor((dx-(frontLWidth+frontRWidth)*front)/2.0f/frontWidth)+1.0f), 1.0f);
	//renderFace(3,new QVector3D(0,0,0),(dx-(frontLWidth+frontRWidth)*front)/2.0f,dyOrig,baseHeight,&normals,&textures,5*subType+5,1.0f*(floor((dx-(frontLWidth+frontRWidth)*front)/2.0f/frontWidth)+1.0f),1.0f);
	if (front) {
		rendManager.addBox(geoName, offset + (dx-(frontLWidth+frontRWidth))/2.0f * vec1, frontLWidth * vec1, dyOrig * vec2, baseHeight, textures[3], 3);
		//renderFace(3,new QVector3D((dx-(frontLWidth+frontRWidth))/2.0f,0,0),frontLWidth,dyOrig,baseHeight,&normals,&textures,5*subType+3,1.0f,1.0f);
		for (int f = 1; f < 6; f++) {
			if (f == 3) {
				rendManager.addBox(geoName, offset + ((dx-(frontLWidth+frontRWidth))/2.0f+frontLWidth) * vec1, frontRWidth * vec1, 1.3 * vec2, frontHeight, textures[4], f);
				//renderFace(f,new QVector3D((dx-(frontLWidth+frontRWidth))/2.0f+frontLWidth,0,0),frontRWidth,0.3f,frontHeight,&normals,&textures,5*subType+4,1.0f,1.0f);
			} else {
				rendManager.addBox(geoName, offset + ((dx-(frontLWidth+frontRWidth))/2.0f+frontLWidth) * vec1, frontRWidth * vec1, 1.3 * vec2, frontHeight, textures[17], f, 0, 0, 0.1, 0.1);
				//renderFace(f,new QVector3D((dx-(frontLWidth+frontRWidth))/2.0f+frontLWidth,0,0),frontRWidth,0.3f,frontHeight,&normals,&BuildingRenderer::roofTextures,roofTex,0.1,0.1f);
			}
		}

	}

	rendManager.addBox(geoName, offset + ((dx-(frontLWidth+frontRWidth)*front)/2.0f+(frontLWidth+frontRWidth)*front) * vec1, (dx-(frontLWidth+frontRWidth)*front)/2.0f * vec1, dyOrig * vec2, baseHeight, textures[5], 3, 0, 0, 1.0f*(floor((dx-(frontLWidth+frontRWidth)*front)/2.0f/frontWidth)+1.0f), 1.0f);
	//renderFace(3,new QVector3D((dx-(frontLWidth+frontRWidth)*front)/2.0f+(frontLWidth+frontRWidth)*front,0,0),(dx-(frontLWidth+frontRWidth)*front)/2.0f,dyOrig,baseHeight,&normals,&textures,5*subType+5,1.0f*(floor((dx-(frontLWidth+frontRWidth)*front)/2.0f/frontWidth)+1.0f),1.0f);

	// 2 back
	if (back1)
		rendManager.addBox(geoName, offset, back1Width * vec1, dyOrig * vec2, baseHeight, textures[5 * building.subType + 2], 1);
		//renderFace(1,new QVector3D(0,0,0),back1Width,dyOrig,baseHeight,&normals,&textures,5*subType+2,1.0f,1.0f);
	rendManager.addBox(geoName, offset + back1 * back1Width * vec1, (dxOrig-back1*back1Width) * vec1, dyOrig * vec2, baseHeight, textures[1], 1, 0, 0, 1.0f*(floor(dx/back0Width)+1.0f),1.0f);
	//renderFace(1,new QVector3D(back1*back1Width,0,0),dxOrig-back1*back1Width,dyOrig,baseHeight,&normals,&textures,5*subType+1,1.0f*(floor(dx/back0Width)+1.0f),1.0f);

	// 3 sides
	for(int i=2;i<=4;i=i+2){
		rendManager.addBox(geoName, offset, dxOrig * vec1, dyOrig * vec2, baseHeight, textures[0], i, 0, 0, 1.0f*(floor(dy/sideWidth)+1.0f),1.0f);
		//renderFace(i,new QVector3D(0,0,0),dxOrig,dyOrig,baseHeight,&normals,&textures,5*subType+0,1.0f*(floor(dy/sideWidth)+1.0f),1.0f);
	}

	//4 roof
	rendManager.addBox(geoName, offset, dxOrig * vec1, dyOrig * vec2, baseHeight, textures[17], 5);
	//renderFace(5,new QVector3D(0,0,0),dxOrig,dyOrig,baseHeight,&normals,&BuildingRenderer::roofTextures,roofTex,1.0f,1.0f);//roof
}

void PMBuildingRB::generateType1(VBORenderManager& rendManager, const QString& geoName, Building& building) {
	Loop3D rectangle = building.buildingFootprint.inscribedOBB();

	float dx = (rectangle[1] - rectangle[0]).length();
	float dy = (rectangle[2] - rectangle[1]).length();
	QVector3D vec1 = (rectangle[1] - rectangle[0]).normalized();
	QVector3D vec2 = (rectangle[2] - rectangle[1]).normalized();	

	float dyOrig = dy;
	float dxOrig = dx;
	QVector3D offset = rectangle[0];

	// 歩道の分、底上げする
	offset.setZ(offset.z() + 1.0f);

	Polygon3D allRoof;

	float frontHeight=9.4f;
	float shiftX=0;
	float coverDepth=2.0f;
	float pharmWidth=frontHeight*256/158;
	float pharmDepth=frontHeight*63/158;;
	float frontLWidth=frontHeight*334/187;
	float sideWidth=frontHeight*56/158;
	float frontSideWidth=frontHeight*221/158;;

	float frontRHe=frontHeight*137.0/187;
	float frontR=frontRHe*264/137;
	float frontR2=frontHeight*31/137;
	bool pharm = true;
	dx -= (pharmWidth+frontLWidth*309/334+frontR);
	if (dxOrig < 45.0f) {
		pharm = false;
		dx += pharmWidth;
	}

	//left side
	allRoof.contour.push_back(offset + shiftX * vec1 + (sideWidth+pharmDepth+coverDepth) * vec2);
	allRoof.contour.push_back(offset + (shiftX+dx*2/3) * vec1 + (sideWidth+pharmDepth+coverDepth) * vec2);
	rendManager.addBox(geoName, offset + shiftX * vec1 + (sideWidth+pharmDepth+coverDepth) * vec2, dx*2/3*vec1, pharmDepth * vec2, frontHeight, textures[11], 3, 0, 0, floor(dx/(2.0f*frontSideWidth))+1,1.0f);
	//renderFace(3,new QVector3D(shiftX,sideWidth+pharmDepth+coverDepth,0),dx*2/3,pharmDepth,frontHeight,&normals,&textures,6*subType+5,floor(dx/(2.0f*frontSideWidth))+1,1.0f);//pharmacyFront
	shiftX+=dx*2/3;

	//pharmacy

	rendManager.addBox(geoName, offset + shiftX * vec1 + (sideWidth+coverDepth) * vec2, pharmWidth * vec1, pharmDepth * vec2, frontHeight, textures[9], 4, 0, 0, 63.0f/375, 1.0f);
	//renderFace(4,new QVector3D(shiftX,sideWidth+coverDepth,0),pharmWidth,pharmDepth,frontHeight,&normals,&textures,6*subType+3,0,63.0f/375,0,1.0f);//side
	allRoof.contour.push_back(offset + shiftX * vec1 + (sideWidth+coverDepth) * vec2);
	if (pharm) {
		allRoof.contour.push_back(offset + shiftX * vec1 + (sideWidth+coverDepth) * vec2);
		rendManager.addBox(geoName, offset + shiftX * vec1 + (sideWidth+coverDepth) * vec2, pharmWidth * vec1, pharmDepth * vec2, frontHeight, textures[9], 3, 63.0f/375, 0, 319.0f/375, 1.0f);
		//renderFace(3,new QVector3D(shiftX,sideWidth+coverDepth,0),pharmWidth,pharmDepth,frontHeight,&normals,&textures,6*subType+3,63.0f/375,319.0f/375,0,1.0f);//pharmacyFront
		shiftX+=pharmWidth-frontLWidth*25/334;
	} else {
		shiftX -= frontLWidth*25/334;
	}

	//front side
	rendManager.addBox(geoName, offset + (shiftX+frontLWidth*25/334) * vec1 + coverDepth * vec2, frontLWidth * vec1, sideWidth * vec2, frontHeight, textures[9], 4, 319.0f/375, 0.0f, 1.0f, 1.0f);
	//renderFace(4,new QVector3D(shiftX+frontLWidth*25/334,coverDepth,0),frontLWidth,sideWidth,frontHeight,&normals,&textures,6*subType+3,319.0f/375,1.0f,0,1.0f);//side


	//front
	allRoof.contour.push_back(offset + (shiftX+frontLWidth*25/334) * vec1 + (coverDepth+0.4f) * vec2);
	allRoof.contour.push_back(offset + (shiftX+frontLWidth) * vec1 + (coverDepth+0.4f) * vec2);

	rendManager.addBox(geoName, offset + (shiftX+frontLWidth*25/334) * vec1 + coverDepth * vec2, (frontLWidth-frontLWidth*25/334) * vec1, dyOrig * vec2, frontHeight*51/187, textures[6], 3, 25.0f/334, 0, 1.0f, 51.0f/201.0f);
	//renderFace(3,new QVector3D(shiftX+frontLWidth*25/334,coverDepth,0),frontLWidth-frontLWidth*25/334,dyOrig,frontHeight*51/187,&normals,&textures,6*subType+0,25.0f/334,1.0f,0,51.0f/201.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + coverDepth * vec2 + QVector3D(0, 0, frontHeight*51/187), frontLWidth * vec1, dyOrig * vec2, frontHeight*136/187, textures[6], 3, 0, 51.0f/201.0f, 1.0f, 187.0f/201.0f);
	//renderFace(3,new QVector3D(shiftX+0,coverDepth,frontHeight*51/187),frontLWidth,dyOrig,frontHeight*136/187,&normals,&textures,6*subType+0,0,1.0f,51.0f/201.0f,187.0f/201.0f);

	for (int f = 1; f < 6; f++) {
		if (f == 3 || f == 2) continue;
		rendManager.addBox(geoName, offset + shiftX * vec1 + coverDepth * vec2 + QVector3D(0, 0, frontHeight*51/187), frontLWidth * vec1, 0.4f * vec2, frontHeight*136/187, textures[10], f);
		//renderFace(f,new QVector3D(shiftX+0,coverDepth,frontHeight*51/187),frontLWidth,0.4f,frontHeight*136/187,&normals,&textures,6*subType+4,1.0f,1.0f);
	}

	/*if(RBRenderer::cylinder==0){
		RBRenderer::cylinder = gluNewQuadric();
		gluQuadricTexture(cylinder, GL_TRUE);
	}
	glPushMatrix();
	glTranslatef(shiftX+3.8f,0.6f+coverDepth,8.25f);//7.95
	glRotatef(90.f, 1.0f, 0.0f, 0.0f);  
	renderCylinder(RBRenderer::cylinder,2.2f,2.2f,1.2f,10,1,&textures,6*subType+2,6*subType+1,true);//2,2
	glPopMatrix();
	*/

	shiftX+=frontLWidth;

	//front right
	rendManager.addBox(geoName, offset + shiftX * vec1 + coverDepth * vec2, vec1 * 0.1, sideWidth * vec2, frontHeight, textures[9], 2, 319.0f/375, 0, 1.0f, 1.0f);
	//renderFace(2,new QVector3D(shiftX,coverDepth,0),0,sideWidth,frontHeight,&normals,&textures,6*subType+3,319.0f/375,1.0f,0,1.0f);//side -0.4 for the front border
	rendManager.addBox(geoName, offset + shiftX * vec1 + coverDepth * vec2, vec1 * frontR, sideWidth * vec2, frontRHe, textures[12], 3);
	//renderFace(3,new QVector3D(shiftX,0+coverDepth,0),frontR,sideWidth,frontRHe,&normals,&textures,6*subType+6,1.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + coverDepth * vec2, vec1 * frontR, sideWidth * vec2, frontRHe, textures[12], 2, 0, 0, 85.0f/264, 1.0f);
	//renderFace(2,new QVector3D(shiftX,0+coverDepth,0),frontR,sideWidth,frontRHe,&normals,&textures,6*subType+6,0,85.0f/264,0,1.0f);//side

	for(int f=1;f<7;f++){//front right horizontal box
		if(f<5)
			rendManager.addBox(geoName, offset + (shiftX-2.0f) * vec1 + QVector3D(0, 0, frontRHe*115/137), (frontR+2*2.0f) * vec1, (coverDepth+sideWidth) * vec2, frontRHe*22/137, textures[12], f, 0, 115.0f/137.0f, 1.0f, 1.0f);
			//renderFace(f,new QVector3D(shiftX-2.0f,0,frontRHe*115/137),frontR+2*2.0f,coverDepth+sideWidth,frontRHe*22/137,&normals,&textures,6*subType+6,0,1.0f,115.0f/137.0f,1.0f);
		else
			rendManager.addBox(geoName, offset + (shiftX-2.0f) * vec1 + QVector3D(0, 0, frontRHe*115/137), (frontR+2*2.0f) * vec1, (coverDepth+sideWidth) * vec2, frontRHe*22/137, textures[12], f, 0, 121.0f/137.0f, 1.0f, 135.0f/137.0f);
			//renderFace(f,new QVector3D(shiftX-2.0f,0,frontRHe*115/137),frontR+2*2.0f,coverDepth+sideWidth,frontRHe*22/137,&normals,&textures,6*subType+6,0,1.0f,121.0f/137.0f,135.0f/137.0f);
	}


	//back front right
	allRoof.contour.push_back(offset + shiftX * vec1 + (sideWidth+coverDepth) * vec2);
	allRoof.contour.push_back(offset + (shiftX+dx/3.0f+frontR) * vec1 + (sideWidth+coverDepth) * vec2);

	rendManager.addBox(geoName, offset + shiftX * vec1 + (sideWidth+coverDepth) * vec2, (dx/3.0f+frontR) * vec1, sideWidth * vec2, frontHeight, textures[13], 3, 0, 0, floor(dx/(2.0f*frontR2))+1.0f, 1.0f);
	//renderFace(3,new QVector3D(shiftX,sideWidth+coverDepth,0),dx/3.0f+frontR,sideWidth,frontHeight,&normals,&textures,6*subType+7,floor(dx/(2.0f*frontR2))+1.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + (sideWidth+coverDepth) * vec2, (dx/3.0f+frontR) * vec1, pharmDepth * vec2, frontHeight, textures[13], 2, 0, 0, floor(pharmDepth/(frontR2))+1.0f, 1.0f);
	//renderFace(2,new QVector3D(shiftX,sideWidth+coverDepth,0),dx/3.0f+frontR,pharmDepth,frontHeight,&normals,&textures,6*subType+7,0,floor(pharmDepth/(frontR2))+1.0f,0,1.0f);//side
	shiftX+=dx/3.0f+frontR;

	float dyDepth=dyOrig-sideWidth-coverDepth-pharmDepth;
	rendManager.addBox(geoName, offset + (sideWidth+coverDepth+pharmDepth) * vec2, dxOrig * vec1, dyDepth * vec2, frontHeight, textures[14], 1, 0, 0, floor(dxOrig*130/(frontHeight*123))+1.0f,1.0f);
	//renderFace(1,new QVector3D(0,sideWidth+coverDepth+pharmDepth,0),dxOrig,dyDepth,frontHeight,&normals,&textures,6*subType+8,floor(dxOrig*130/(frontHeight*123))+1.0f,1.0f);
	rendManager.addBox(geoName, offset + (sideWidth+coverDepth+pharmDepth) * vec2, dxOrig * vec1, dyDepth * vec2, frontHeight, textures[15], 2, 0, 0, floor(dyDepth*131/(frontHeight*160))+1.0f,1.0f);
	//renderFace(2,new QVector3D(0,sideWidth+coverDepth+pharmDepth,0),dxOrig,dyDepth,frontHeight,&normals,&textures,6*subType+9,floor(dyDepth*131/(frontHeight*160))+1.0f,1.0f);
	rendManager.addBox(geoName, offset + (sideWidth+coverDepth+pharmDepth) * vec2, dxOrig * vec1, dyDepth * vec2, frontHeight, textures[16], 4, 0, 0, floor(dyDepth*136/(frontHeight*319))+1.0f,1.0f);
	//renderFace(4,new QVector3D(0,sideWidth+coverDepth+pharmDepth,0),dxOrig,dyDepth,frontHeight,&normals,&textures,6*subType+10,floor(dyDepth*136/(frontHeight*319))+1.0f,1.0f);

	allRoof.contour.push_back(offset + dxOrig * vec1 + dyOrig * vec2);
	allRoof.contour.push_back(offset + dyOrig * vec2);
	/*if(roofTex==-1){
		roofTex=bestTextureForRoof(dxOrig,dyOrig);
	}*/
	//renderFlatRoof(&allRoof,&roofTextures,roofTex);
	rendManager.addPolygon(geoName, allRoof.contour, offset.z() + frontHeight, textures[17], QVector3D(1, 1, 0));
}