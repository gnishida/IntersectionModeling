#pragma once

#include <QMap>
#include <QString>
#include "Polygon2D.h"
#include "BlockSet.h"
#include "ZoneType.h"

class Zoning {
public:
	std::vector<std::pair<Polygon2D, ZoneType> > zones;
	int* zones2;
	int zone_size;

	// 改善案
	std::vector<float> zoneTypeDistribution;

public:
	Zoning();
	size_t size() { return zones.size(); }
	int getZone(const QVector2D& pt) const;
	void load(const QString& filename);
	void save(const QString& filename);
	void generate(Polygon2D& targetArea);
	
	// 改善案
	void randomlyAssignZoneType(BlockSet& blocks);

private:
	std::pair<Polygon2D, ZoneType> defaultZone();
};

class CompareZoning
{
public:
    // Compare two Foo structs.
    bool operator()(const std::pair<float, Zoning>& x, const std::pair<float, Zoning>& y) const
    {
        return x.first < y.first;
    }
};

class CompareZoningReverse
{
public:
    // Compare two Foo structs.
    bool operator()(const std::pair<float, Zoning>& x, const std::pair<float, Zoning>& y) const
    {
        return x.first > y.first;
    }
};