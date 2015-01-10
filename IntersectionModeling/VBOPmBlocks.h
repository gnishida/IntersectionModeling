/************************************************************************************************
 *		Procedural City Generation: Blocks
 *		@author igarciad
 ************************************************************************************************/

#pragma once

#include <boost/graph/planar_face_traversal.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>

#include "VBOBlock.h"
#include "VBOParcel.h"
#include "VBOBuilding.h"
#include "RoadGraph.h"
#include "BlockSet.h"
#include "Zoning.h"

class VBORenderManager;

class VBOPmBlocks
{
public:

	//Generate Blocks
	static bool generateBlocks(Zoning& zones, RoadGraph &roadGraph, BlockSet &blocks);

	static void buildEmbedding(RoadGraph &roads, std::vector<std::vector<RoadEdgeDesc> > &embedding);
	static void assignZonesToBlocks(Zoning& zoning, BlockSet& blocks);
};


