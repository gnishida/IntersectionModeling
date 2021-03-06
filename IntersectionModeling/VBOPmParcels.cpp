/************************************************************************************************
 *		Procedural City Generation: Parcel
 *		@author igarciad
 ************************************************************************************************/

#include "VBOPmParcels.h"
#include "qmatrix4x4.h"


void subdivideBlockIntoParcels(Block &block);
bool subdivideParcel(Block &block, Parcel parcel, float areaMean, float areaMin, float areaVar, float splitIrregularity, std::vector<Parcel> &outParcels); 

bool VBOPmParcels::generateParcels(VBORenderManager& rendManager, std::vector< Block > &blocks) {
	srand(0);
	for (int i = 0; i < blocks.size(); ++i) {
		subdivideBlockIntoParcels(blocks[i]);

		blocks[i].adaptToTerrain(&rendManager);
	}

	return true;
}

void subdivideBlockIntoParcels(Block &block) {
	//srand(block.randSeed);
	std::vector<Parcel> tmpParcels;

	//Empty parcels in block
	block.myParcels.clear();

	//Make the initial parcel of the block be the block itself
	Parcel tmpParcel;
	tmpParcel.setContour(block.blockContour);

	subdivideParcel(block, tmpParcel, block.zone.parcel_area_mean, block.zone.parcel_area_min, block.zone.parcel_area_deviation, block.zone.parcel_split_deviation, tmpParcels);

	Block::parcelGraphVertexDesc tmpPGVD;
	for(int i=0; i<tmpParcels.size(); ++i){
		//assign a zone type to parcel
		tmpParcels[i].zone = block.zone;

		//add parcel to block parcels graph
		tmpPGVD = boost::add_vertex(block.myParcels);
		block.myParcels[tmpPGVD] = tmpParcels[i];
	}
}

/**
* Parcel subdivision
* @desc: Recursive subdivision of a parcel using OBB technique
* @return: true if parcel was successfully subdivided, false otherwise
* @areaMean: mean target area of parcels after subdivision
* @areaVar: variance of parcels area after subdivision (normalized in 0-1)
* @splitIrregularity: A normalized value 0-1 indicating how far
*					from the middle point the split line should be
**/
bool subdivideParcel(Block &block, Parcel parcel, float areaMean, float areaMin, float areaStd,
	float splitIrregularity, std::vector<Parcel> &outParcels)
{
	//printf("subdivideParcel\n");
	//check if parcel is subdividable
	float thresholdArea = areaMean + areaStd*areaMean*(((float)qrand()/RAND_MAX)*2.0f-1.0f);//LC::misctools::genRand(-1.0f, 1.0f)
	
	if( (fabs(boost::geometry::area(parcel.bg_parcelContour))) <= std::max(thresholdArea, areaMin)) {
		//printf("a: %.3f %.3f", boost::geometry::area(parcel.bg_parcelContour));
		//boost::geometry::correct(parcel.bg_parcelContour);
		//printf("a: %.3f %.3f", boost::geometry::area(parcel.bg_parcelContour));
		outParcels.push_back(parcel);
		return true;
	}

	//compute OBB
	QVector3D obbSize;
	QMatrix4x4 obbMat;
	parcel.parcelContour.getMyOBB(obbSize, obbMat);

	//compute split line passing through center of OBB TODO (+/- irregularity)
	//		and with direction parallel/perpendicular to OBB main axis
	QVector3D slEndPoint;
	QVector3D dirVectorInit, dirVector, dirVectorOrthogonal;
	QVector3D midPt(0.0f, 0.0f, 0.0f);
	QVector3D auxPt(1.0f, 0.0f, 0.0f);
	QVector3D midPtNoise(0.0f, 0.0f, 0.0f);
	std::vector<QVector3D> splitLine;	

	midPt = midPt*obbMat;

	dirVectorInit = (auxPt*obbMat - midPt);
	dirVectorInit.normalize();
	if(obbSize.x() > obbSize.y()){
		dirVector.setX( -dirVectorInit.y() );
		dirVector.setY(  dirVectorInit.x() );
	} else {
		dirVector = dirVectorInit;
	}

	midPtNoise.setX( splitIrregularity*(((float)qrand()/RAND_MAX)*20.0f-10.0f));//LC::misctools::genRand(-10.0f, 10.0f) );
	midPtNoise.setY( splitIrregularity*(((float)qrand()/RAND_MAX)*20.0f-10.0f));//LC::misctools::genRand(-10.0f, 10.0f) );
	midPt = midPt + midPtNoise;

	slEndPoint = midPt + 10000.0f*dirVector;
	splitLine.push_back(slEndPoint);
	slEndPoint = midPt - 10000.0f*dirVector;
	splitLine.push_back(slEndPoint);

	//split parcel with line and obtain two new parcels
	Polygon3D pgon1, pgon2;

	float kDistTol = 0.01f;

	std::vector<Polygon3D> pgons;
	if (parcel.parcelContour.split(splitLine, pgons)) {
		for (int i = 0; i < pgons.size(); ++i) {
			Parcel parcel;
			parcel.setContour(pgons[i]);

			subdivideParcel(block, parcel, areaMean, areaMin, areaStd, splitIrregularity, outParcels);
		}
	}
	/*
	if (parcel.parcelContour.splitMeWithPolyline(splitLine, pgon1.contour, pgon2.contour)) {
		Parcel parcel1;
		Parcel parcel2;

		parcel1.setContour(pgon1);
		parcel2.setContour(pgon2);

		//call recursive function for both parcels
		subdivideParcel(block, parcel1, areaMean, areaMin, areaStd, splitIrregularity, outParcels);
		subdivideParcel(block, parcel2, areaMean, areaMin, areaStd, splitIrregularity, outParcels);
	} else {
		return false;
	}
	*/

	return true;
}

bool compareFirstPartTuple (const std::pair<float,Parcel*> &i, const std::pair<float,Parcel*> &j) {
	return (i.first<j.first);
}

void VBOPmParcels::assignZoneType(Block& block) {
	Block::parcelGraphVertexIter vi, viEnd;
	for (boost::tie(vi, viEnd) = boost::vertices(block.myParcels); vi != viEnd; ++vi) {
		block.myParcels[*vi].zone = block.zone;
	}
}

