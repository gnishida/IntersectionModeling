#pragma once

#include "VBOBlock.h"
#include "VBOModel_StreetElements.h"
#include "Zoning.h"

class VBORenderManager;

class VBOVegetation
{
public:
	VBOVegetation(void);

	~VBOVegetation(void);

	static bool generateVegetation(VBORenderManager& rendManager, Zoning& zones, std::vector< Block > &blocks);

};
