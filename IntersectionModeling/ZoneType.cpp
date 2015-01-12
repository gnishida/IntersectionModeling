#include <QFile>
#include <QDomDocument>
#include "Util.h"
#include "BlockSet.h"

void ZoneType::init() {
	if (_type == TYPE_RESIDENTIAL) {
		if (_level == 1) {
			park_percentage = 0.0f;
			parcel_area_mean = 200;
			parcel_area_min = 100;
			parcel_area_deviation = 1;
			parcel_split_deviation = 0.2;
			parcel_setback_front = 2;
			parcel_setback_sides = 2;
			parcel_setback_rear = 2;
			building_stories_mean = 2;
			building_stories_deviation = 50;
			building_max_depth = 0;
			building_max_frontage = 0;
			sidewalk_width = 2;
			tree_setback = 1;
		} else if (_level == 2) {
			park_percentage = 0.0f;
			parcel_area_mean = 2000;
			parcel_area_min = 1000;
			parcel_area_deviation = 1;
			parcel_split_deviation = 0.2;
			parcel_setback_front = 8;
			parcel_setback_sides = 6;
			parcel_setback_rear = 8;
			building_stories_mean = 4;
			building_stories_deviation = 50;
			building_max_depth = 0;
			building_max_frontage = 0;
			sidewalk_width = 3;
			tree_setback = 1;
		} else if (_level == 3) {
			park_percentage = 0.0f;
			parcel_area_mean = 4000;
			parcel_area_min = 2000;
			parcel_area_deviation = 1;
			parcel_split_deviation = 0.2;
			parcel_setback_front = 10;
			parcel_setback_sides = 8;
			parcel_setback_rear = 12;
			building_stories_mean = 12;
			building_stories_deviation = 50;
			building_max_depth = 0;
			building_max_frontage = 0;
			sidewalk_width = 4;
			tree_setback = 1;
		}
	} else if (_type == TYPE_COMMERCIAL) {
		if (_level == 1) {
			park_percentage = 0.0f;
			parcel_area_mean = 2500;
			parcel_area_min = 1250;
			parcel_area_deviation = 1;
			parcel_split_deviation = 0.2;
			parcel_setback_front = 8;
			parcel_setback_sides = 6;
			parcel_setback_rear = 8;
			building_stories_mean = 4;
			building_stories_deviation = 50;
			building_max_depth = 0;
			building_max_frontage = 0;
			sidewalk_width = 3;
			tree_setback = 1;
		} else if (_level == 2) {
			park_percentage = 0.0f;
			parcel_area_mean = 2500;
			parcel_area_min = 1250;
			parcel_area_deviation = 1;
			parcel_split_deviation = 0.2;
			parcel_setback_front = 8;
			parcel_setback_sides = 6;
			parcel_setback_rear = 8;
			building_stories_mean = 4;
			building_stories_deviation = 50;
			building_max_depth = 0;
			building_max_frontage = 0;
			sidewalk_width = 3;
			tree_setback = 1;
		} else if (_level == 3) {
			park_percentage = 0.0f;
			parcel_area_mean = 4000;
			parcel_area_min = 2000;
			parcel_area_deviation = 1;
			parcel_split_deviation = 0.2;
			parcel_setback_front = 10;
			parcel_setback_sides = 8;
			parcel_setback_rear = 12;
			building_stories_mean = 12;
			building_stories_deviation = 50;
			building_max_depth = 0;
			building_max_frontage = 0;
			sidewalk_width = 4;
			tree_setback = 1;
		}
	} else if (_type == TYPE_MANUFACTURING) {
		park_percentage = 0.0f;
		parcel_area_mean = 8000;
		parcel_area_min = 2000;
		parcel_area_deviation = 1;
		parcel_split_deviation = 0.2;
		parcel_setback_front = 7;
		parcel_setback_sides = 4;
		parcel_setback_rear = 4;
		building_stories_mean = 12;
		building_stories_deviation = 50;
		building_max_depth = 0;
		building_max_frontage = 0;
		sidewalk_width = 4;
		tree_setback = 1;
	} else if (_type == TYPE_PARK) {
		park_percentage = 1.0f;
		parcel_area_mean = 4000;
		parcel_area_min = 2000;
		parcel_area_deviation = 1;
		parcel_split_deviation = 0.2;
		parcel_setback_front = 7;
		parcel_setback_sides = 4;
		parcel_setback_rear = 4;
		building_stories_mean = 1;
		building_stories_deviation = 50;
		building_max_depth = 0;
		building_max_frontage = 0;
		sidewalk_width = 4;
		tree_setback = 1;
	} else if (_type == TYPE_AMUSEMENT) {
		park_percentage = 0.0f;
		parcel_area_mean = 4000;
		parcel_area_min = 2000;
		parcel_area_deviation = 1;
		parcel_split_deviation = 0.2;
		parcel_setback_front = 8;
		parcel_setback_sides = 6;
		parcel_setback_rear = 8;
		building_stories_mean = 4;
		building_stories_deviation = 50;
		building_max_depth = 0;
		building_max_frontage = 0;
		sidewalk_width = 4;
		tree_setback = 1;
	} else if (_type == TYPE_PUBLIC) {
		park_percentage = 0.0f;
		parcel_area_mean = 6000;
		parcel_area_min = 2000;
		parcel_area_deviation = 1;
		parcel_split_deviation = 0.2;
		parcel_setback_front = 8;
		parcel_setback_sides = 6;
		parcel_setback_rear = 8;
		building_stories_mean = 3;
		building_stories_deviation = 50;
		building_max_depth = 0;
		building_max_frontage = 0;
		sidewalk_width = 4;
		tree_setback = 1;
	}
}

void ZoneType::setType(int type) {
	_type = type;
	init();
}

