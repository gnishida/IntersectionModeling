#include "PMBuildingSchool.h"

bool PMBuildingSchool::initialized = false;
std::vector<QString> PMBuildingSchool::textures;
int PMBuildingSchool::NUM_SUBTYPE = 2;

void PMBuildingSchool::initialize() {
	if (initialized) return;
	
	textures.push_back("../data/buildings/SC/0_back0.jpg");//0
	textures.push_back("../data/buildings/SC/0_back1.jpg");//1
	textures.push_back("../data/buildings/SC/0_front.jpg");//2
	textures.push_back("../data/buildings/SC/0_frontSides0.jpg");//3
	textures.push_back("../data/buildings/SC/0_frontSides1.jpg");//4
	textures.push_back("../data/buildings/SC/0_left0.jpg");//5
	textures.push_back("../data/buildings/SC/0_left1.jpg");//6
	textures.push_back("../data/buildings/SC/0_left2.jpg");//7
	textures.push_back("../data/buildings/SC/0_left3.jpg");//8
	textures.push_back("../data/buildings/SC/0_right0.jpg");//9
	textures.push_back("../data/buildings/SC/0_right1.jpg");//10
	textures.push_back("../data/buildings/SC/0_center0.jpg");//11

	textures.push_back("../data/buildings/SC/1_front.jpg");//0
	textures.push_back("../data/buildings/SC/1_frontBack.jpg");//1
	textures.push_back("../data/buildings/SC/1_back.jpg");//2 
	textures.push_back("../data/buildings/SC/1_inside.jpg");//3
	textures.push_back("../data/buildings/SC/1_sides.jpg");//4
	textures.push_back("../data/buildings/SC/1_frontFarSides.jpg");//5
	textures.push_back("../data/buildings/SC/1_sidesWall.jpg");//6

	textures.push_back("../data/buildings/roof/roof2.jpg");

	initialized = true;
}

void PMBuildingSchool::generate(VBORenderManager& rendManager, const QString& geoName, Building& building) {
	initialize();

	if (building.subType == 0) {
		generateType0(rendManager, geoName, building);
	} else {
		generateType0(rendManager, geoName, building);
		//generateType1(rendManager, geoName, building);
	}
}

void PMBuildingSchool::generateType0(VBORenderManager& rendManager, const QString& geoName, Building& building) {
	Loop3D rectangle = building.buildingFootprint.inscribedOBB();

	float dx = (rectangle[1] - rectangle[0]).length();
	float dy = (rectangle[2] - rectangle[1]).length();
	QVector3D vec1 = (rectangle[1] - rectangle[0]).normalized();
	QVector3D vec2 = (rectangle[2] - rectangle[1]).normalized();	

	float dyOrig=dy;
	float dxOrig=dx;
	QVector3D offset = rectangle[0];

	// 歩道の分、底上げする
	offset.setZ(offset.z() + 1.0f);

	int texShift=12;
	float frontHeigh=4.47f;
	float frontSideDepth=6.0f;
	float colDepth=0.4f;
	float shiftX=0;
	float frontWidth=frontHeigh*679.0f/200;
	float farSide=1.0f;

	int roofTex=-1;
	int roofTex2=-1;

	// 1. LEFT
	dx-=frontWidth;
	dx-=farSide*2;//far sides
	dx/=2.0f;//left and right


	rendManager.addBox(geoName, offset + vec1 * shiftX + vec2 * frontSideDepth, farSide * vec1, vec2 * (dyOrig-frontSideDepth*2), frontHeigh, textures[texShift+5], 3, 0, 0, farSide/(frontHeigh*49/200), 1.0f);
	//renderFace(3,new QVector3D(shiftX,frontSideDepth,0),farSide,dyOrig-frontSideDepth*2,frontHeigh,&normals,&textures,texShift+5,0,farSide/(frontHeigh*49/200),0.0f,1.0f);
	shiftX+=farSide;

	//columns
	float columnSpan = 5.6f + frontHeigh * 56.0f / 200;
	int numColumns = floor(dx / columnSpan);
	columnSpan = dx / numColumns;
	float colShift = 0.0f;
	for(int c = 0; c < numColumns; c++) {
		rendManager.addBox(geoName, offset + vec1 * (shiftX + colShift) + vec2 * frontSideDepth, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 1, 0, 0, 56.0f/679, 122.0f/200.0f);
		//renderFace(1,new QVector3D(shiftX+colShift,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
		rendManager.addBox(geoName, offset + vec1 * (shiftX + colShift) + vec2 * frontSideDepth, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 3, 0, 0, 56.0f/679, 122.0f/200.0f);
		//renderFace(3,new QVector3D(shiftX+colShift,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
		rendManager.addBox(geoName, offset + vec1 * (shiftX + colShift) + vec2 * frontSideDepth, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 2, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
		//renderFace(2,new QVector3D(shiftX+colShift,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
		rendManager.addBox(geoName, offset + vec1 * (shiftX + colShift) + vec2 * frontSideDepth, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 4, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
		//renderFace(4,new QVector3D(shiftX+colShift,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
		colShift += columnSpan;
	}

	//top
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), dx * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 3, 0, 122.0f/200.0f, dx/(frontWidth), 1.0f);
	//renderFace(3,new QVector3D(shiftX,frontSideDepth,frontHeigh*122.0f/200.0f),dx,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,dx/(frontWidth),122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), dx * vec1, frontSideDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 6, 0, 122.0f/200.0f, 1.0f, 1.0f);
	//renderFace(6,new QVector3D(shiftX,frontSideDepth,frontHeigh*122.0f/200.0f),dx,frontSideDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,1.0f,122.0f/200.0f,1.0f);

	//back
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * 2.0f * vec2, dx * vec1, (dyOrig-frontSideDepth * 2.0) * vec2, frontHeigh*122.0f/200.0f, textures[texShift + 3], 3, 0, 0.5f, dx*200*200.0f/(frontHeigh*1429*122.0f), 1.0f);
	//renderFace(3,new QVector3D(shiftX,frontSideDepth*2.0f,0),dx,dyOrig-frontSideDepth*2,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+3,0,dx*200*200.0f/(frontHeigh*1429*122.0f),0.5f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + (frontSideDepth+colDepth) * vec2, vec1 * 0.1, (frontSideDepth-colDepth) * vec2, frontHeigh*122.0f/200.0f, textures[texShift + 4], 2, 270.0f/696, 0, 452.0f/696, 176.0f/200);
	//renderFace(2,new QVector3D(shiftX,frontSideDepth+colDepth,0),0,frontSideDepth-colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+4,270.0f/696,452.0f/696,0,176.0f/200);
	rendManager.addBox(geoName, offset + (shiftX + dx) * vec1 + frontSideDepth * vec2, vec1, frontSideDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift + 4], 4, 270.0f/696, 0, 452.0f/696, 176.0f/200);
	//renderFace(4,new QVector3D(shiftX+dx,frontSideDepth,0),0,frontSideDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+4,270.0f/696,452.0f/696,0,176.0f/200);
	shiftX += dx;

	// 2. MAIN
	//front
	//left
	rendManager.addBox(geoName, offset + shiftX * vec1, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 1, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(1,new QVector3D(shiftX,0,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 3, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(3,new QVector3D(shiftX,0,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 2, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(2,new QVector3D(shiftX,0,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
	//right
	rendManager.addBox(geoName, offset + (shiftX+frontHeigh*299.0f/200.0f) * vec1, frontHeigh*115.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 1, 299.0f/679.0f, 0, 414.0f/679, 122.0f/200.0f);
	//renderFace(1,new QVector3D(shiftX+frontHeigh*299.0f/200.0f,0,0),frontHeigh*115.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,299.0f/679.0f,414.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontHeigh*299.0f/200.0f) * vec1, frontHeigh*115.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 3, 299.0f/679.0f, 0, 414.0f/679, 122.0f/200.0f);
	//renderFace(3,new QVector3D(shiftX+frontHeigh*299.0f/200.0f,0,0),frontHeigh*115.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,299.0f/679.0f,414.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontHeigh*299.0f/200.0f) * vec1, frontHeigh*115.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 2, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
	//renderFace(2,new QVector3D(shiftX+frontHeigh*299.0f/200.0f,0,0),frontHeigh*115.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontHeigh*299.0f/200.0f) * vec1, frontHeigh*115.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 4, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
	//renderFace(4,new QVector3D(shiftX+frontHeigh*299.0f/200.0f,0,0),frontHeigh*115.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);

	//top
	rendManager.addBox(geoName, offset + shiftX * vec1 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), frontHeigh*679.0f/200 * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 1, 0, 122.0f/200.0f, 1, 1);
	//renderFace(1,new QVector3D(shiftX,0,frontHeigh*122.0f/200.0f),frontHeigh*679.0f/200,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,1.0f,122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), frontHeigh*679.0f/200 * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 3, 0, 122.0f/200.0f, 1, 1);
	//renderFace(3,new QVector3D(shiftX,0,frontHeigh*122.0f/200.0f),frontHeigh*679.0f/200,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,1.0f,122.0f/200.0f,1.0f);

	rendManager.addBox(geoName, offset + shiftX * vec1 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), frontHeigh*679.0f/200 * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 5, 0, 122.0f/200.0f, 1, 1);
	//renderFace(5,new QVector3D(shiftX,0,frontHeigh*122.0f/200.0f),frontHeigh*679.0f/200,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,1.0f,122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), frontHeigh*679.0f/200 * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 6, 0, 122.0f/200.0f, 1, 1);
	//renderFace(6,new QVector3D(shiftX,0,frontHeigh*122.0f/200.0f),frontHeigh*679.0f/200,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,1.0f,122.0f/200.0f,1.0f);

	rendManager.addBox(geoName, offset + shiftX * vec1 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), frontHeigh*679.0f/200 * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 2, 0, 122.0f/200.0f, colDepth/(frontHeigh*679.0f/200), 1);
	//renderFace(2,new QVector3D(shiftX,0,frontHeigh*122.0f/200.0f),frontHeigh*679.0f/200,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*679.0f/200)),122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), frontHeigh*679.0f/200 * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 4, 0, 122.0f/200.0f, colDepth/(frontHeigh*679.0f/200), 1);
	//renderFace(4,new QVector3D(shiftX,0,frontHeigh*122.0f/200.0f),frontHeigh*679.0f/200,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*679.0f/200)),122.0f/200.0f,1.0f);

	//side
	//top
	rendManager.addBox(geoName, offset + shiftX * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 2, 0, 122.0f/200.0f, (frontSideDepth-colDepth)/(frontHeigh*679.0f/200), 1);
	//renderFace(2,new QVector3D(shiftX,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(frontSideDepth-colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 4, 0, 122.0f/200.0f, (frontSideDepth-colDepth)/(frontHeigh*679.0f/200), 1);
	//renderFace(4,new QVector3D(shiftX,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(frontSideDepth-colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);

	rendManager.addBox(geoName, offset + shiftX * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 5, 0, 122.0f/200.0f, (colDepth)/(frontHeigh*679.0f/200), 1);
	//renderFace(5,new QVector3D(shiftX,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 6, 0, 122.0f/200.0f, (colDepth)/(frontHeigh*679.0f/200), 1);
	//renderFace(6,new QVector3D(shiftX,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);

	//column
	rendManager.addBox(geoName, offset + shiftX * vec1 + colDepth * vec2, colDepth * vec1, (frontHeigh*56.0f/200-colDepth) * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 2, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(2,new QVector3D(shiftX,colDepth,0),colDepth,frontHeigh*56.0f/200-colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1, colDepth * vec1, frontHeigh*56.0f/200 * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 4, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(4,new QVector3D(shiftX,0,0),colDepth,frontHeigh*56.0f/200,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + colDepth * vec2, colDepth * vec1, (frontHeigh*56.0f/200-colDepth) * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 1, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
	//renderFace(1,new QVector3D(shiftX,colDepth,0),colDepth,frontHeigh*56.0f/200-colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);

	// front back
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * vec2, frontHeigh*679.0f/200 * vec1, (dyOrig-frontSideDepth) * vec2, frontHeigh, textures[texShift+1], 3);
	//renderFace(3,new QVector3D(shiftX,frontSideDepth,0),frontHeigh*679.0f/200,dyOrig-frontSideDepth,frontHeigh,&normals,&textures,texShift+1,1.0f,1.0f);
	//front right
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-frontHeigh*56.0f/200) * vec1, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 1, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(1,new QVector3D(shiftX+frontWidth-frontHeigh*56.0f/200,0,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-frontHeigh*56.0f/200) * vec1, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 3, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(3,new QVector3D(shiftX+frontWidth-frontHeigh*56.0f/200,0,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-frontHeigh*56.0f/200) * vec1, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 4, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
	//renderFace(4,new QVector3D(shiftX+frontWidth-frontHeigh*56.0f/200,0,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
	//top
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-colDepth) * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 2, 0, 122.0f/200.0f, (frontSideDepth-colDepth)/(frontHeigh*679.0f/200), 1.0f);
	//renderFace(2,new QVector3D(shiftX+frontWidth-colDepth,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(frontSideDepth-colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-colDepth) * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 4, 0, 122.0f/200.0f, (frontSideDepth-colDepth)/(frontHeigh*679.0f/200), 1.0f);
	//renderFace(4,new QVector3D(shiftX+frontWidth-colDepth,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(frontSideDepth-colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);
	
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-colDepth) * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 5, 0, 122.0f/200.0f, (colDepth)/(frontHeigh*679.0f/200), 1.0f);
	//renderFace(5,new QVector3D(shiftX+frontWidth-colDepth,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-colDepth) * vec1 + colDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), colDepth * vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 6, 0, 122.0f/200.0f, (colDepth)/(frontHeigh*679.0f/200), 1.0f);
	//renderFace(6,new QVector3D(shiftX+frontWidth-colDepth,colDepth,frontHeigh*122.0f/200.0f),colDepth,frontSideDepth-colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth)/(frontHeigh*679.0f/200),122.0f/200.0f,1.0f);

	//column
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-colDepth) * vec1 + colDepth * vec2, colDepth * vec1, (frontHeigh*56.0f/200-colDepth) * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 4, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(4,new QVector3D(shiftX+frontWidth-colDepth,colDepth,0),colDepth,frontHeigh*56.0f/200-colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-colDepth) * vec1, colDepth * vec1, (frontHeigh*56.0f/200-colDepth) * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 2, 0, 0, 56.0f/679, 122.0f/200.0f);
	//renderFace(2,new QVector3D(shiftX+frontWidth-colDepth,0,0),colDepth,frontHeigh*56.0f/200,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
	rendManager.addBox(geoName, offset + (shiftX+frontWidth-colDepth) * vec1 + colDepth * vec2, colDepth * vec1, (frontHeigh*56.0f/200-colDepth) * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 1, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
	//renderFace(1,new QVector3D(shiftX+frontWidth-colDepth,colDepth,0),colDepth,frontHeigh*56.0f/200-colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
	shiftX += frontWidth;

	// 2. RIGHT
	//columns
	colShift = 0.0f;
	for (int c = 0; c < numColumns; c++) {
		rendManager.addBox(geoName, offset + (shiftX+dx-colShift-frontHeigh*56.0f/200) * vec1 + frontSideDepth * vec2, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 1, 0, 0, 56.0f/679, 122.0f/200.0f);
		//renderFace(1,new QVector3D(shiftX+dx-(colShift)-frontHeigh*56.0f/200,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
		rendManager.addBox(geoName, offset + (shiftX+dx-colShift-frontHeigh*56.0f/200) * vec1 + frontSideDepth * vec2, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 3, 0, 0, 56.0f/679, 122.0f/200.0f);
		//renderFace(3,new QVector3D(shiftX+dx-(colShift)-frontHeigh*56.0f/200,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,56.0f/679,0.0f,122.0f/200.0f);
		rendManager.addBox(geoName, offset + (shiftX+dx-colShift-frontHeigh*56.0f/200) * vec1 + frontSideDepth * vec2, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 2, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
		//renderFace(2,new QVector3D(shiftX+dx-(colShift)-frontHeigh*56.0f/200,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
		rendManager.addBox(geoName, offset + (shiftX+dx-colShift-frontHeigh*56.0f/200) * vec1 + frontSideDepth * vec2, frontHeigh*56.0f/200 * vec1, colDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift], 4, 0, 0, (colDepth/(frontHeigh*56.0f/200))*56.0f/679, 122.0f/200.0f);
		//renderFace(4,new QVector3D(shiftX+dx-(colShift)-frontHeigh*56.0f/200,frontSideDepth,0),frontHeigh*56.0f/200,colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+0,0,(colDepth/(frontHeigh*56.0f/200))*56.0f/679,0.0f,122.0f/200.0f);
		colShift += columnSpan;
	}

	//top
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), dx * vec1, colDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 3, 0, 122.0f/200.0f, dx/(frontWidth), 1.0f);
	//renderFace(3,new QVector3D(shiftX,frontSideDepth,frontHeigh*122.0f/200.0f),dx,colDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,dx/(frontWidth),122.0f/200.0f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * vec2 + QVector3D(0, 0, frontHeigh*122.0f/200.0f), dx * vec1, frontSideDepth * vec2, frontHeigh*78.0f/200.0f, textures[texShift], 6, 0, 122.0f/200.0f, 1, 1.0f);
	//renderFace(6,new QVector3D(shiftX,frontSideDepth,frontHeigh*122.0f/200.0f),dx,frontSideDepth,frontHeigh*78.0f/200.0f,&normals,&textures,texShift+0,0,1.0f,122.0f/200.0f,1.0f);
	//back
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * 2.0f * vec2, dx * vec1, (dyOrig-frontSideDepth*2) * vec2, frontHeigh*122.0f/200.0f, textures[texShift+3], 3, 0, 0.5f, dx*200*200.0f/(frontHeigh*1429*122.0f), 1.0f);
	//renderFace(3,new QVector3D(shiftX,frontSideDepth*2.0f,0),dx,dyOrig-frontSideDepth*2,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+3,0,dx*200*200.0f/(frontHeigh*1429*122.0f),0.5f,1.0f);
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * vec2, vec1 * 0.1, frontSideDepth * vec2, frontHeigh*122.0f/200.0f, textures[texShift+4], 2, 270.0f/696, 0, 452.0f/696, 176.0f/200);
	//renderFace(2,new QVector3D(shiftX,frontSideDepth,0),0,frontSideDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+4,270.0f/696,452.0f/696,0,176.0f/200);
	rendManager.addBox(geoName, offset + (shiftX + dx) * vec1 + (frontSideDepth+colDepth) * vec2, vec1, (frontSideDepth-colDepth) * vec2, frontHeigh*122.0f/200.0f, textures[texShift+4], 4, 270.0f/696, 0, 452.0f/696, 176.0f/200);
	//renderFace(4,new QVector3D(shiftX+dx,frontSideDepth+colDepth,0),0,frontSideDepth-colDepth,frontHeigh*122.0f/200.0f,&normals,&textures,texShift+4,270.0f/696,452.0f/696,0,176.0f/200);
	shiftX += dx;
	rendManager.addBox(geoName, offset + shiftX * vec1 + frontSideDepth * vec2, farSide * vec1, (dyOrig-frontSideDepth*2) * vec2, frontHeigh, textures[texShift+5], 3, 0, 0, farSide/(frontHeigh*49/200), 1);
	//renderFace(3,new QVector3D(shiftX,frontSideDepth,0),farSide,dyOrig-frontSideDepth*2,frontHeigh,&normals,&textures,texShift+5,0,farSide/(frontHeigh*49/200),0.0f,1.0f);
	shiftX += farSide;



	// 3. BACK
	float texBackStart=0,texBackEnd=0;
	float backWidth=frontHeigh*1340/200;
	float backX=dxOrig;
	if(dxOrig>backWidth){
		texBackEnd=floor(dxOrig/backWidth);
		backX-=texBackEnd*backWidth;
	}
	if(backX>(backWidth*400/1340)){
		texBackEnd+=400.0f/1340;
		backX-=texBackEnd*backWidth;
	}
	rendManager.addBox(geoName, offset, dxOrig * vec1, dyOrig * vec2, frontHeigh, textures[texShift+2], 1, texBackStart, 0, texBackEnd, 1);
	//renderFace(1,new QVector3D(0,0,0),dxOrig,dyOrig,frontHeigh,&normals,&textures,texShift+2,texBackStart,texBackEnd,0,1.0f);

	// 4. SIDES
	//left
	float doorDepth=frontHeigh*244.0f/200;
	float dyHalf=(dyOrig-frontSideDepth-doorDepth)/2.0f;
	rendManager.addBox(geoName, offset + vec2 * frontSideDepth, dxOrig * vec1, dyHalf * vec2, frontHeigh, textures[texShift+6], 4, dyHalf/(frontHeigh*182/200), 0, 1, 1);
	//renderFace(4,new QVector3D(0,frontSideDepth,0),dxOrig,dyHalf,frontHeigh,&normals,&textures,texShift+6,dyHalf/(frontHeigh*182/200),1.0f,0,1.0f);
	rendManager.addBox(geoName, offset + vec2 * (frontSideDepth+dyHalf), dxOrig * vec1, doorDepth * vec2, frontHeigh, textures[texShift+4], 4, 452.0f/696.0f, 0, 1, 1);
	//renderFace(4,new QVector3D(0,frontSideDepth+dyHalf,0),dxOrig,doorDepth,frontHeigh,&normals,&textures,texShift+4,452.0f/696.0f,1.0f,0,1.0f);
	rendManager.addBox(geoName, offset + vec2 * (dyOrig-dyHalf), dxOrig * vec1, dyHalf * vec2, frontHeigh, textures[texShift+6], 4, dyHalf/(frontHeigh*182/200), 0, 1, 1);
	//renderFace(4,new QVector3D(0,dyOrig-dyHalf,0),dxOrig,dyHalf,frontHeigh,&normals,&textures,texShift+6,dyHalf/(frontHeigh*182/200),1.0f,0,1.0f);


	rendManager.addBox(geoName, offset + vec2 * frontSideDepth, dxOrig * vec1, dyHalf * vec2, frontHeigh, textures[texShift+6], 2, dyHalf/(frontHeigh*182/200), 0, 1, 1);
	//renderFace(2,new QVector3D(0,frontSideDepth,0),dxOrig,dyHalf,frontHeigh,&normals,&textures,texShift+6,dyHalf/(frontHeigh*182/200),1.0f,0,1.0f);
	rendManager.addBox(geoName, offset + vec2 * (frontSideDepth+dyHalf), dxOrig * vec1, doorDepth * vec2, frontHeigh, textures[texShift+4], 2, 452.0f/696.0f, 0, 1, 1);
	//renderFace(2,new QVector3D(0,frontSideDepth+dyHalf,0),dxOrig,doorDepth,frontHeigh,&normals,&textures,texShift+4,452.0f/696.0f,1.0f,0,1.0f);
	rendManager.addBox(geoName, offset + vec2 * (dyOrig-dyHalf), dxOrig * vec1, dyHalf * vec2, frontHeigh, textures[texShift+6], 2, dyHalf/(frontHeigh*182/200), 0, 1, 1);
	//renderFace(2,new QVector3D(0,dyOrig-dyHalf,0),dxOrig,dyHalf,frontHeigh,&normals,&textures,texShift+6,dyHalf/(frontHeigh*182/200),1.0f,0,1.0f);

	// 5. ROOF
	if(roofTex==-1){
		//roofTex=bestTextureForRoof(dxOrig,dyOrig-frontSideDepth);
	}
	rendManager.addBox(geoName, offset + vec2 * frontSideDepth, dxOrig * vec1, (dyOrig-frontSideDepth) * vec2, frontHeigh, textures[19], 5);
	//renderFace(5,new QVector3D(0,frontSideDepth,0),dxOrig,dyOrig-frontSideDepth,frontHeigh,&normals,&BuildingRenderer::roofTextures,roofTex,1.0f,1.0f);//roof

}

/*void PMBuildingSchool::generateType1(VBORenderManager& rendManager, Building& building) {
	float dx = (building.footprint.contour[1] - building.footprint.contour[0]).length();
	float dy = (building.footprint.contour[2] - building.footprint.contour[1]).length();
	QVector3D vec1 = (building.footprint.contour[1] - building.footprint.contour[0]).normalized();
	QVector3D vec2 = (building.footprint.contour[2] - building.footprint.contour[1]).normalized();	

	float dyOrig=dy;
	float dxOrig=dx;


	if (building.numStories < 2) building.numStories = 2;
	float frontHeigh=3.0f;
	float frontTopHeight=0.83f;
	float frontTopDepth=1.0f;
	float frontWidth=(frontHeigh+frontTopHeight)*524/200;

	float frontSideHeight=7.64f;
	float frontSideWidth=frontSideHeight*447/250;

	float depthOneBuilding=22.0f;
	float backWidth=frontSideHeight*549/250;
	float mainSideWidth=(frontHeigh+frontSideHeight)*549/250;

	float leftWidth=frontSideHeight*472/250;
	float rightWidth=frontSideHeight*251/250;
	float backWidth2=frontSideHeight*549/250;

	float corridorWidth=(dyOrig-frontTopDepth)/3.0f;
	if (corridorWidth > depthOneBuilding) corridorWidth = depthOneBuilding;

	float shiftX = 0;
	dx -= frontWidth;

	// front left
	float shiftY=0;
	renderFace(3,new QVector3D(shiftX,frontTopDepth,0),dx/2.0f,dyOrig,frontSideHeight*104/250.0f,&normals,&textures,3,0,floor(dx/(2*frontSideWidth))+1.0f,0.0f,104.0f/250.0f);
	shiftY+=frontSideHeight*104/250.0f;
	int extraStories=stories-2;
	if(extraStories>0){
		renderFace(3,new QVector3D(shiftX,frontTopDepth,shiftY),dx/2.0f,dyOrig,extraStories*frontSideHeight*104/250.0f,&normals,&textures,4,0,floor(dx/(2*frontSideWidth))+1.0f,0.0f,1.0f*extraStories);
		shiftY+=extraStories*frontSideHeight*104/250.0f;
	}
	renderFace(3,new QVector3D(shiftX,frontTopDepth,shiftY),dx/2.0f,dyOrig,frontSideHeight*146.0f/250.0f,&normals,&textures,3,0,floor(dx/(2*frontSideWidth))+1.0f,104.0f/250.0f,1.0f);
	shiftX+=dx/2.0f;


	//main entrance
	renderFace(3,new QVector3D(shiftX,frontTopDepth,0),frontWidth,dyOrig,frontHeigh,&normals,&textures,2,0,1.0f,0.0f,140.f/200.0f);
	for(int f=1;f<7;f++){
		if(f<5){
			if(f==1||f==3)//front
				renderFace(f,new QVector3D(shiftX,0,frontHeigh),frontWidth,frontTopDepth,frontTopHeight,&normals,&textures,2,0,1.0f,161.0f/200,1.0f);
			else//sides
				renderFace(f,new QVector3D(shiftX,0,frontHeigh),frontWidth,frontTopDepth,frontTopHeight,&normals,&textures,2,0,50.0f/524.0f,161.0f/200,1.0f);
		}
		else
			renderFace(f,new QVector3D(shiftX,0,frontHeigh),frontWidth,frontTopDepth,frontTopHeight,&normals,&textures,2,0,1.0f,140.f/200,161.0f/200);
	}
	renderFace(2,new QVector3D(shiftX,frontTopDepth,0),frontWidth,dyOrig-corridorWidth-frontTopDepth,frontHeigh+frontTopHeight,&normals,&textures,0,0,floor(dy/mainSideWidth)+1.0f,0.0f,1.0f);
	renderFace(4,new QVector3D(shiftX,frontTopDepth,0),frontWidth,dyOrig-corridorWidth-frontTopDepth,frontHeigh+frontTopHeight,&normals,&textures,0,0,floor(dy/mainSideWidth)+1.0f,0.0f,1.0f);
	if(roofTex2==-1){
		roofTex2=bestTextureForRoof(frontWidth,dyOrig-corridorWidth-frontTopDepth);
	}
	renderFace(5,new QVector3D(shiftX,frontTopDepth,0),frontWidth,dyOrig-corridorWidth-frontTopDepth,frontHeigh+frontTopHeight,&normals,&BuildingRenderer::roofTextures,roofTex2,0,1.0f,0.0f,1.0f);
	shiftX+=frontWidth;

	// front right
	shiftY=0;
	renderFace(3,new QVector3D(shiftX,frontTopDepth,0),dx/2.0f,dyOrig,frontSideHeight*104/250.0f,&normals,&textures,3,0,floor(dx/(2*frontSideWidth))+1.0f,0.0f,104.0f/250.0f);
	shiftY+=frontSideHeight*104/250.0f;
	extraStories=stories-2;
	if(extraStories>0){
		renderFace(3,new QVector3D(shiftX,frontTopDepth,shiftY),dx/2.0f,dyOrig,extraStories*frontSideHeight*104/250.0f,&normals,&textures,4,0,floor(dx/(2*frontSideWidth))+1.0f,0.0f,1.0f*extraStories);
		shiftY+=extraStories*frontSideHeight*104/250.0f;
	}
	renderFace(3,new QVector3D(shiftX,frontTopDepth,shiftY),dx/2.0f,dyOrig,frontSideHeight*146.0f/250.0f,&normals,&textures,3,0,floor(dx/(2*frontSideWidth))+1.0f,104.0f/250.0f,1.0f);

	// left

	shiftX=0.0f;
	shiftY=0;
	renderFace(4,new QVector3D(shiftX,frontTopDepth,0),dxOrig,dyOrig-frontTopDepth,frontSideHeight*98/250.0f,&normals,&textures,5,0,floor((dyOrig-frontTopDepth)/(2*leftWidth))+1.0f,0.0f,98.0f/250.0f);
	shiftY+=frontSideHeight*98/250.0f;
	extraStories=stories-2;
	if(extraStories>0){
		renderFace(4,new QVector3D(shiftX,frontTopDepth,shiftY),dxOrig,dyOrig-frontTopDepth,extraStories*frontSideHeight*104/250.0f,&normals,&textures,6,0,floor((dyOrig-frontTopDepth)/(2*leftWidth))+1.0f,0.0f,1.0f*extraStories);
		shiftY+=extraStories*frontSideHeight*104/250.0f;
	}
	renderFace(4,new QVector3D(shiftX,frontTopDepth,shiftY),dxOrig,dyOrig-frontTopDepth,frontSideHeight*152.0f/250.0f,&normals,&textures,5,0,floor((dyOrig-frontTopDepth)/(2*leftWidth))+1.0f,98.0f/250.0f,1.0f);

	// right
	shiftX=0.0f;
	shiftY=0;
	renderFace(2,new QVector3D(shiftX,frontTopDepth,0),dxOrig,dyOrig-frontTopDepth,frontSideHeight*128.0f/250.0f,&normals,&textures,9,0,floor((dyOrig-frontTopDepth)/(2*rightWidth))+1.0f,0.0f,128.0f/250.0f);
	shiftY+=frontSideHeight*128/250.0f;
	extraStories=stories-2;
	if(extraStories>0){
		renderFace(2,new QVector3D(shiftX,frontTopDepth,shiftY),dxOrig,dyOrig-frontTopDepth,extraStories*frontSideHeight*104/250.0f,&normals,&textures,10,0,floor((dyOrig-frontTopDepth)/(2*rightWidth))+1.0f,0.0f,1.0f*extraStories);
		shiftY+=extraStories*frontSideHeight*104/250.0f;
	}
	renderFace(2,new QVector3D(shiftX,frontTopDepth,shiftY),dxOrig,dyOrig-frontTopDepth,frontSideHeight*122.0f/250.0f,&normals,&textures,9,0,floor((dyOrig-frontTopDepth)/(2*rightWidth))+1.0f,122.0f/250.0f,1.0f);

	// back

	shiftX=0.0f;
	shiftY=0;
	renderFace(1,new QVector3D(shiftX,frontTopDepth,0),dxOrig,dyOrig-frontTopDepth,frontSideHeight*127.0f/250.0f,&normals,&textures,0,0,floor((dxOrig)/(2*backWidth2))+1.0f,0.0f,127.0f/250.0f);
	shiftY+=frontSideHeight*127/250.0f;
	extraStories=stories-2;
	if(extraStories>0){
		renderFace(1,new QVector3D(shiftX,frontTopDepth,shiftY),dxOrig,dyOrig-frontTopDepth,extraStories*frontSideHeight*104/250.0f,&normals,&textures,1,0,floor((dxOrig-frontTopDepth)/(2*backWidth2))+1.0f,0.0f,1.0f*extraStories);
		shiftY+=extraStories*frontSideHeight*104/250.0f;
	}
	renderFace(1,new QVector3D(shiftX,frontTopDepth,shiftY),dxOrig,dyOrig-frontTopDepth,frontSideHeight*123.0f/250.0f,&normals,&textures,0,0,floor((dxOrig-frontTopDepth)/(2*backWidth2))+1.0f,123.0f/250.0f,1.0f);

	// center
	//corridor from main to corridor side
	bool corridorFromMain=false;
	if((dx/2.0f)>corridorWidth){
		corridorFromMain=true;
		for(int sid=0;sid<2;sid++){
			if(sid==0){
				shiftX=corridorWidth;
			}
			else{
				shiftX=(dx/2.0f)+frontWidth;
			}
			shiftY=0;
			renderFace(1,new QVector3D(shiftX,frontTopDepth,shiftY),(dx/2.0f)-corridorWidth,corridorWidth,frontSideHeight*127.0f/250.0f,&normals,&textures,0,0,floor(((dx/2.0f)-corridorWidth)/(2*backWidth2))+1.0f,0.0f,127.0f/250.0f);
			shiftY+=frontSideHeight*127/250.0f;
			extraStories=stories-2;
			if(extraStories>0){
				renderFace(1,new QVector3D(shiftX,frontTopDepth,shiftY),(dx/2.0f)-corridorWidth,corridorWidth,extraStories*frontSideHeight*104/250.0f,&normals,&textures,1,0,floor(((dx/2.0f)-corridorWidth)/(2*backWidth2))+1.0f,0.0f,1.0f*extraStories);
				shiftY+=extraStories*frontSideHeight*104/250.0f;
			}
			renderFace(1,new QVector3D(shiftX,frontTopDepth,shiftY),(dx/2.0f)-corridorWidth,corridorWidth,frontSideHeight*123.0f/250.0f,&normals,&textures,0,0,floor(((dx/2.0f)-corridorWidth)/(2*backWidth2))+1.0f,123.0f/250.0f,1.0f);
		}

		//left and right corridor
		float depth=frontTopDepth+corridorFromMain*corridorWidth;
		float leftRightCorWidth=dyOrig-frontTopDepth-corridorFromMain*2.0f*corridorWidth;
		for(int sid=0;sid<2;sid++){
			int face;
			shiftY=0;
			if(sid==0){
				shiftX=0;
				face=2;
			}
			else{
				shiftX=(dx/2.0f)+frontWidth+corridorFromMain*((dx/2.0f)-corridorWidth);
				face=4;
			}
			shiftY=0;
			renderFace(face,new QVector3D(shiftX,depth,shiftY),corridorWidth,leftRightCorWidth,frontSideHeight*127.0f/250.0f,&normals,&textures,0,0,floor((leftRightCorWidth)/(2*backWidth2))+1.0f,0.0f,127.0f/250.0f);
			shiftY+=frontSideHeight*127/250.0f;
			extraStories=stories-2;
			if(extraStories>0){
				renderFace(face,new QVector3D(shiftX,depth,shiftY),corridorWidth,leftRightCorWidth,extraStories*frontSideHeight*104/250.0f,&normals,&textures,1,0,floor((leftRightCorWidth)/(2*backWidth2))+1.0f,0.0f,1.0f*extraStories);
				shiftY+=extraStories*frontSideHeight*104/250.0f;
			}
			renderFace(face,new QVector3D(shiftX,depth,shiftY),corridorWidth,leftRightCorWidth,frontSideHeight*123.0f/250.0f,&normals,&textures,0,0,floor((leftRightCorWidth)/(2*backWidth2))+1.0f,123.0f/250.0f,1.0f);
		}
		//brick sides
		for(int sid=0;sid<2;sid++){
			int face;
			shiftY=0;
			if(sid==0){
				face=2;
				shiftX=corridorWidth;
			}
			else{
				shiftX=(dx/2.0f)+frontWidth;
				face=4;
			}
			shiftY=0;
			renderFace(face,new QVector3D(shiftX,frontTopDepth,shiftY),(dx/2.0f)-corridorWidth,corridorWidth,frontSideHeight+frontSideHeight*extraStories*104.0f/250.0f,&normals,&textures,11,0,floor(corridorWidth)/(3.35f)+1.0f,0.0f,(frontSideHeight+frontSideHeight*extraStories*104.f/250.0f)/(10.0f));
		}
		//front of the back corridor
		shiftX=corridorWidth;
		float backCorridorWidth=dxOrig-2*corridorWidth;
		shiftY=0;
		renderFace(3,new QVector3D(shiftX,dyOrig-corridorWidth,shiftY),backCorridorWidth,corridorWidth,frontSideHeight*127.0f/250.0f,&normals,&textures,0,0,floor((backCorridorWidth)/(2*backWidth2))+1.0f,0.0f,127.0f/250.0f);
		shiftY+=frontSideHeight*127/250.0f;
		extraStories=stories-2;
		if(extraStories>0){
			renderFace(3,new QVector3D(shiftX,dyOrig-corridorWidth,shiftY),backCorridorWidth,corridorWidth,frontSideHeight*extraStories*104/250.0f,&normals,&textures,1,0,floor((backCorridorWidth)/(2*backWidth2))+1.0f,0.0f,1.0f*extraStories);
			shiftY+=extraStories*frontSideHeight*104/250.0f;
		}
		renderFace(3,new QVector3D(shiftX,dyOrig-corridorWidth,shiftY),backCorridorWidth,corridorWidth,frontSideHeight*123.0f/250.0f,&normals,&textures,0,0,floor((backCorridorWidth)/(2*backWidth2))+1.0f,123.0f/250.0f,1.0f);


	}else{
		//there is not square center
		for(int sid=0;sid<2;sid++){
			int face;
			shiftY=0;
			if(sid==0){
				face=2;
				shiftX=0;
			}
			else{
				shiftX=(dx/2.0f)+frontWidth;
				face=4;
			}
			shiftY=0;
			renderFace(face,new QVector3D(shiftX,frontTopDepth,shiftY),(dx/2.0f),dyOrig-corridorWidth-frontTopDepth,frontSideHeight+frontSideHeight*extraStories*104.0f/250.0f,&normals,&textures,11,0,floor(dyOrig-corridorWidth-frontTopDepth)/(3.35f)+1.0f,0.0f,(frontSideHeight+frontSideHeight*extraStories*104.f/250.0f)/(10.0f));
		}


		//front of the back corridor
		shiftX=dx/2.0f;
		float backCorridorWidth=dxOrig-frontWidth;
		shiftY=0;
		renderFace(3,new QVector3D(shiftX,dyOrig-corridorWidth,shiftY),frontWidth,corridorWidth,frontSideHeight*127.0f/250.0f,&normals,&textures,0,0,floor((backCorridorWidth)/(2*backWidth2))+1.0f,0.0f,127.0f/250.0f);
		shiftY+=frontSideHeight*127/250.0f;
		extraStories=stories-2;
		if(extraStories>0){
			renderFace(3,new QVector3D(shiftX,dyOrig-corridorWidth,shiftY),frontWidth,corridorWidth,frontSideHeight*extraStories*104/250.0f,&normals,&textures,1,0,floor((backCorridorWidth)/(2*backWidth2))+1.0f,0.0f,1.0f*extraStories);
			shiftY+=extraStories*frontSideHeight*104/250.0f;
		}
		renderFace(3,new QVector3D(shiftX,dyOrig-corridorWidth,shiftY),frontWidth,corridorWidth,frontSideHeight*123.0f/250.0f,&normals,&textures,0,0,floor((backCorridorWidth)/(2*backWidth2))+1.0f,123.0f/250.0f,1.0f);



	}

	MTC::misctools::Polygon3D allRoof;
	float toltaH=frontSideHeight+frontSideHeight*extraStories*104.0f/250.0f;
	allRoof.contour.push_back(QVector3D(dxOrig,frontTopDepth,toltaH));
	allRoof.contour.push_back(QVector3D(dxOrig,dyOrig,toltaH));
	allRoof.contour.push_back(QVector3D(0,dyOrig,toltaH));
	allRoof.contour.push_back(QVector3D(0,frontTopDepth,toltaH));
	allRoof.contour.push_back(QVector3D(dx/2.0f,frontTopDepth,toltaH));
	if((dx/2.0f)>corridorWidth){
		//brick
		allRoof.contour.push_back(QVector3D(dx/2.0f,frontTopDepth+corridorWidth,toltaH));
		//central sq
		float centerCor=(dx/2.0f)-corridorWidth;
		allRoof.contour.push_back(QVector3D((dx/2.0f)-centerCor,frontTopDepth+corridorWidth,toltaH));

		allRoof.contour.push_back(QVector3D((dx/2.0f)-centerCor,dyOrig-corridorWidth,toltaH));
		allRoof.contour.push_back(QVector3D((dx/2.0f)+centerCor+frontWidth,dyOrig-corridorWidth,toltaH));

		allRoof.contour.push_back(QVector3D((dx/2.0f)+centerCor+frontWidth,frontTopDepth+corridorWidth,toltaH));
		//
		//brick
		allRoof.contour.push_back(QVector3D(dx/2.0f+frontWidth,frontTopDepth+corridorWidth,toltaH));


	}else{
		allRoof.contour.push_back(QVector3D(dx/2.0f,dyOrig-corridorWidth,toltaH));
		allRoof.contour.push_back(QVector3D(dx/2.0f+frontWidth,dyOrig-corridorWidth,toltaH));

	}
	allRoof.contour.push_back(QVector3D(dx/2.0f+frontWidth,frontTopDepth,toltaH));
	if(roofTex==-1){
		roofTex=bestTextureForRoof(dxOrig,dyOrig);
	}
	renderFlatRoof(&allRoof,&roofTextures,roofTex);

}
*/