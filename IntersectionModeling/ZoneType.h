#pragma once

#include <QMap>
#include <QString>
#include "Polygon2D.h"

class ZoneType
{
public:
	static enum { TYPE_RESIDENTIAL = 0, TYPE_COMMERCIAL, TYPE_MANUFACTURING, TYPE_PARK, TYPE_AMUSEMENT, TYPE_PUBLIC };

private:
	int _type;
	int _level;	// 1/2/3

public:
	float park_percentage;
	float parcel_area_mean;
	float parcel_area_min;
	float parcel_area_deviation;
	float parcel_split_deviation;
	float parcel_setback_front;
	float parcel_setback_sides;
	float parcel_setback_rear;
	float building_stories_mean;
	float building_stories_deviation;
	float building_max_depth;
	float building_max_frontage;
	float sidewalk_width;
	float tree_setback;
	//int building_type;

public:
	ZoneType() : _type(TYPE_RESIDENTIAL), _level(0) { init(); }
	ZoneType(int type, int level) : _type(type), _level(level) { init(); }
	void init();
	int type() { return _type; }
	void setType(int type);
	int level() { return _level; }
};

