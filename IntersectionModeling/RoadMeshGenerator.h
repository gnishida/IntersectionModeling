#pragma once

#include "VBORenderManager.h"
#include "RoadGraph.h"

class RoadMeshGenerator {
public:
	RoadMeshGenerator() {}

	static void generateRoadMesh(VBORenderManager& rendManager, RoadGraph& roads);
};

