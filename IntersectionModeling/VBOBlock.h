/************************************************************************************************
*		VBO Block Class
*		@author igarciad
************************************************************************************************/
#pragma once

#ifndef Q_MOC_RUN
#include <boost/graph/adjacency_list.hpp>
#endif

#include "VBORenderManager.h"
#include "VBOParcel.h"
#include <QVector3D>
#include "Polygon3D.h"
#include "ZoneType.h"

/**
* Block.
**/

class Block {
public:
	/**
	* BGL Graph of parcels into which block is subdivided.
	**/
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Parcel> parcelGraph;
	typedef boost::graph_traits<parcelGraph>::vertex_descriptor parcelGraphVertexDesc;
	typedef boost::graph_traits<parcelGraph>::vertex_iterator parcelGraphVertexIter;
	typedef boost::graph_traits<parcelGraph>::edge_iterator parcelGraphEdgeIter;
	typedef boost::graph_traits<parcelGraph>::adjacency_iterator parcelGraphAdjIter;// Carlos


public:
	parcelGraph myParcels;

	BBox3D bbox;

	int randSeed;
	ZoneType zone;

	/** Contour of the block */
	Polygon3D blockContour;

	/** contour of the sidewalk */
	Polygon3D sidewalkContour;

	/** Boundary road widths */
	std::vector<float> sidewalkContourRoadsWidths;

public:
	/** Constructor */
	Block() {}

	/** Clear */
	void clear(void);

	void computeMyBBox3D(void);

	/** Compute parcel adjacency graph */
	void computeParcelAdjacencyGraph(void);

	void buildableAreaMock(void);

	static void findParcelFrontAndBackEdges(Block &inBlock, Parcel &inParcel,
		std::vector<int> &frontEdges,
		std::vector<int> &rearEdges,
		std::vector<int> &sideEdges );


	bool splitBlockParcelsWithRoadSegment(std::vector<QVector3D> &roadSegmentGeometry,
		float roadSegmentWidth, BBox3D roadSegmentBBox3D, std::list<Parcel> &blockParcels);

	bool areParcelsAdjacent(parcelGraphVertexIter &p0, parcelGraphVertexIter &p1);
	
	void adaptToTerrain(VBORenderManager* vboRenderManager);
};

