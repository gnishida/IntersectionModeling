#pragma once

#include <QString>
#include <vector>
#include "VBORenderManager.h"
#include "VBOBuilding.h"

class PMBuildingFactory {
public:
	static std::vector<QString> textures;
	static bool initialized;
	static int NUM_SUBTYPE;

public:
	static void initialize();
	static void generate(VBORenderManager& rendManager, const QString& geoName, Building& building);
};

