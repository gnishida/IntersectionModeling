/************************************************************************************************
*		VBO Parcel
*		@author igarciad
************************************************************************************************/
#pragma once

#include <QSettings>
#include "VBOBuilding.h"
#include <QTextStream>
#include "Polygon3D.h"
#include "ZoneType.h"

const int PAR_PARK=0;
const int PAR_WITH_BLDG=1;

/**
* Parcel.
**/
class Parcel {
public:
	Polygon3D parcelContour;
	Polygon3D parcelBuildableAreaContour;

	ZoneType zone;

	Building myBuilding;

	BBox3D bbox;
	boost::geometry::ring_type<Polygon3D>::type bg_parcelContour;

public:
	Parcel();

	/** Set Contour */
	inline void setContour(Polygon3D &inContour)
	{
		this->parcelContour = inContour;
		if(parcelContour.contour.size()>0){
			boost::geometry::correct(parcelContour.contour); // GEN ???
			boost::geometry::assign(bg_parcelContour, parcelContour.contour);
		}
		initializeParcel();
	}

	/**
	* Initialize parcel
	**/
	void initializeParcel()
	{
		if(parcelContour.contour.size()>0){
			boost::geometry::assign(bg_parcelContour, parcelContour.contour);
			boost::geometry::correct(parcelContour.contour);
		}
		/*QVector3D minPt,maxPt;

		parcelContour.getBBox3D(minPt, maxPt);
		bbox.minPt=minPt.toVector2D();
		bbox.maxPt=maxPt.toVector2D();*/
		parcelContour.getBBox3D(bbox.minPt, bbox.maxPt);
	}


	/**
	* @brief: Returns true if parcels are adjacent			
	**/
	bool intersectsParcel(Parcel &other);

	/**			
	* @brief: Compute union of this parcel with a given parcel. The contour of the current parcel is modified. The other parcel is left unchanged.
	* @param[in] other: Parcel to union this parcel with.
	* @param[out] bool: True if union was computed. False if union fails. Union fails if parcels are not adjacent.
	**/
	int unionWithParcel(Parcel &other);

	/** Compute Parcel Buildable Area */
	float computeBuildableArea(float frontSetback, float rearSetback, float sideSetback, std::vector<int> &frontEdges, std::vector<int> &rearEdges, std::vector<int> &sideEdges, Loop3D &pgonInset);
};

