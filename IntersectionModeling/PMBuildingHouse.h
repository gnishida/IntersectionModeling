#pragma once

#include <QString>
#include <vector>
#include "VBORenderManager.h"
#include "VBOBuilding.h"

class PMBuildingHouse {
public:
	static std::vector<QString> textures;
	static bool initialized;

public:
	static void initialize();
	static void generate(VBORenderManager& rendManager, const QString& geoName, Building& building);
};

