#pragma once

#include "ColorTable.h"
#include <qcolor.h>

class HeatMapColorTable : public ColorTable {
protected:
	float maxValue;
	float minValue;

public:
	HeatMapColorTable(float minValue, float maxValue);
	~HeatMapColorTable();
	QColor getColor(double value);
};
