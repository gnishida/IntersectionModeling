#include "HeatMapColorTable.h"
#include <algorithm>

HeatMapColorTable::HeatMapColorTable(float minValue, float maxValue) : ColorTable() {
	this->maxValue = maxValue;
	this->minValue = minValue;
}

HeatMapColorTable::~HeatMapColorTable() {
}

QColor HeatMapColorTable::getColor(double value) {
	double r, g, b;

	double t;
	if (scale == ColorTable::SCALE_LINEAR) {
		t = (value - minValue) / (maxValue - minValue);
	} else {
		t = log(value - minValue) / log(maxValue - minValue);
	}	

	if (t < 0.0) t = 0.0;
	else if (t > 1.0) t = 1.0;

	if (t < 0.25f) {
		r = 0.0f;
		g = t * 4.0f;
		b = 1.0f;
	} else if (t < 0.5f) {
		r = 0.0f;
		g = 1.0f;
		b = 1.0f - (t - 0.25f) * 4.0f;
	} else if (t < 0.75f) {
		r = (t - 0.5f) * 4.0f;
		g = 1.0f;
		b = 0.0f;
	} else {
		r = 1.0f;
		g = 1.0f - (t - 0.75f) * 4.0f;
		b = 0.0f;
	}

	return QColor(r * 255, g * 255, b * 255);
}
