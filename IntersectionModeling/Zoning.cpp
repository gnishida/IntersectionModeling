#include "Zoning.h"
#include <QFile>
#include <QDomDocument>
#include "Util.h"
#include "BlockSet.h"

Zoning::Zoning() {
	// residential
	zoneTypeDistribution.push_back(0.2f);
	zoneTypeDistribution.push_back(0.38f);
	zoneTypeDistribution.push_back(0.2f);

	// commercial
	zoneTypeDistribution.push_back(0.06f);
	zoneTypeDistribution.push_back(0.05f);
	zoneTypeDistribution.push_back(0.03f);

	// manufacturing
	zoneTypeDistribution.push_back(0.02f);
	zoneTypeDistribution.push_back(0.01f);
	zoneTypeDistribution.push_back(0.01f);

	// park
	zoneTypeDistribution.push_back(0.02f);
	zoneTypeDistribution.push_back(0);
	zoneTypeDistribution.push_back(0);

	// amusement
	zoneTypeDistribution.push_back(0.01f);
	zoneTypeDistribution.push_back(0);
	zoneTypeDistribution.push_back(0);

	// public
	zoneTypeDistribution.push_back(0.01f);
	zoneTypeDistribution.push_back(0);
	zoneTypeDistribution.push_back(0);
}

int Zoning::getZone(const QVector2D& pt) const {
	int zoneId = -1;

	for (int i = 0; i < zones.size(); ++i) {
		if (zones[i].first.contains(QVector2D(pt))) {
			zoneId = i;
		}
	}

	return zoneId;
}

void Zoning::load(const QString& filename) {
	zones.clear();

	QFile file(filename);

	QDomDocument doc;
	doc.setContent(&file, true);
	QDomElement root = doc.documentElement();

	QDomNode node = root.firstChild();
	while (!node.isNull()) {
		if (node.toElement().tagName() == "zone") {
			int type = node.toElement().attribute("type").toInt();
			int level = node.toElement().attribute("level").toInt();

			// to make level between [1,3]
			if (level < 1 || level > 3) level = 1;

			ZoneType zone(type, level);

			Polygon2D polygon;
			QDomNode polygonNode = node.childNodes().at(0);
			for (int i = 0; i < polygonNode.childNodes().size(); ++i) {
				QDomNode pointNode = polygonNode.childNodes().at(i);
				float x = pointNode.toElement().attribute("x").toFloat();
				float y = pointNode.toElement().attribute("y").toFloat();
				polygon.push_back(QVector2D(x, y));
			}

			zones.push_back(std::make_pair(polygon, zone));
		}

		node = node.nextSibling();
	}
}

void Zoning::save(const QString& filename) {
	QFile file(filename);
     
    if (!file.open(QFile::WriteOnly| QFile::Truncate)) return;

	QDomDocument doc;
	QDomElement root = doc.createElement("zoning");
	doc.appendChild(root);

	for (int i = 0; i < zones.size(); ++i) {
		QDomElement zoneNode = doc.createElement("zone");
		zoneNode.setAttribute("type", zones[i].second.type());
		zoneNode.setAttribute("level", zones[i].second.level());
		
		QDomElement polygonNode = doc.createElement("polygon");
		for (int j = 0; j < zones[i].first.size(); ++j) {
			QDomElement pointNode = doc.createElement("point");
			pointNode.setAttribute("x", zones[i].first.at(j).x());
			pointNode.setAttribute("y", zones[i].first.at(j).y());
			polygonNode.appendChild(pointNode);
		}
		zoneNode.appendChild(polygonNode);

		root.appendChild(zoneNode);
	}

	QTextStream out(&file);
	doc.save(out, 4);
}

/**
 * Randomly generate zoning plan.
 * Each type of zone is assigned to at least one block.
 */
void Zoning::generate(Polygon2D& targetArea) {
	BBox bbox = targetArea.envelope();

	while (true) {
		zones.clear();

		int histogram[7] = {};

		zones.push_back(std::make_pair(targetArea, ZoneType(ZoneType::TYPE_RESIDENTIAL, 1)));

		float step = 200.0f;
		for (int u = 0; u < bbox.dx()  / step; ++u) {
			float x = (float)u * step + bbox.minPt.x();
			for (int v = 0; v < bbox.dy() / step; ++v) {
				float y = (float)v * step + bbox.minPt.y();

				if (!targetArea.contains(QVector2D(x, y))) continue;

				Polygon2D polygon;
				polygon.push_back(QVector2D(x, y));
				polygon.push_back(QVector2D(x + step, y));
				polygon.push_back(QVector2D(x + step, y + step));
				polygon.push_back(QVector2D(x, y + step));

				ZoneType zone;
				float r = Util::genRand(0, 100);
				if (r <= 20) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_RESIDENTIAL, 1)));
					histogram[ZoneType::TYPE_RESIDENTIAL]++;
				} else if (r <= 58) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_RESIDENTIAL, 2)));
					histogram[ZoneType::TYPE_RESIDENTIAL]++;
				} else if (r <= 78) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_RESIDENTIAL, 3)));
					histogram[ZoneType::TYPE_RESIDENTIAL]++;
				} else if (r <= 84) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_COMMERCIAL, 1)));
					histogram[ZoneType::TYPE_COMMERCIAL]++;
				} else if (r <= 89) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_COMMERCIAL, 2)));
					histogram[ZoneType::TYPE_COMMERCIAL]++;
				} else if (r <= 92) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_COMMERCIAL, 3)));
					histogram[ZoneType::TYPE_COMMERCIAL]++;
				} else if (r <= 94) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_MANUFACTURING, 1)));
					histogram[ZoneType::TYPE_MANUFACTURING]++;
				} else if (r <= 95) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_MANUFACTURING, 2)));
					histogram[ZoneType::TYPE_MANUFACTURING]++;
				} else if (r <= 96) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_MANUFACTURING, 3)));
					histogram[ZoneType::TYPE_MANUFACTURING]++;
				} else if (r <= 98) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_PARK, 1)));
					histogram[ZoneType::TYPE_PARK]++;
				} else if (r <= 99) {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_AMUSEMENT, 1)));
					histogram[ZoneType::TYPE_AMUSEMENT]++;
				} else {
					zones.push_back(std::make_pair(polygon, ZoneType(ZoneType::TYPE_PUBLIC, 1)));
					histogram[ZoneType::TYPE_PUBLIC]++;
				}			
			}
		}

		// check if there is at least one block for each zone type
		bool valid = true;
		for (int i = 1; i < 7; ++i) {
			if (histogram[i] == 0) valid = false;
		}
		if (valid) break;
	}
}

/**
 * 指定されたdistributionに従い、ブロックにゾーンタイプを割り当てる。
 */
void Zoning::randomlyAssignZoneType(BlockSet& blocks) {
	zones.clear();
	zones.push_back(defaultZone());

	float totalArea = 0.0f;

	QVector3D size;
	QMatrix4x4 xformMat;
	for (int i = 0; i < blocks.size(); ++i) {
		totalArea += blocks[i].blockContour.area();
	}

	float Z = 0.0f;
	for (int i = 0; i < zoneTypeDistribution.size(); ++i) {
		Z += zoneTypeDistribution[i];
	}

	std::vector<float> remainedArea;
	for (int i = 0; i < zoneTypeDistribution.size(); ++i) {
		remainedArea.push_back(zoneTypeDistribution[i] / Z * totalArea);
	}

	for (int i = 0; i < blocks.size(); ++i) {
		Z = 0.0f;
		for (int type = 0; type < zoneTypeDistribution.size(); ++type) {
			if (remainedArea[type] < 0) continue;
			Z += remainedArea[type];
		}

		float r = Util::genRand(0, Z);
		Z = 0.0f;
		for (int type = 0; type < zoneTypeDistribution.size(); ++type) {
			if (remainedArea[type] < 0) continue;

			Z += remainedArea[type];
			if (r < Z) {
				blocks[i].zone = ZoneType(type / 3, (type % 3) + 1);
				remainedArea[type] -= blocks[i].blockContour.area();
				Polygon2D polygon;
				for (int k = 0; k < blocks[i].blockContour.contour.size(); ++k) {
					polygon.push_back(QVector2D(blocks[i].blockContour.contour[k]));
				}
				zones.push_back(std::make_pair(polygon, blocks[i].zone));
				break;
			}
		}
	}
}

std::pair<Polygon2D, ZoneType> Zoning::defaultZone() {
	Polygon2D polygon;
	polygon.push_back(QVector2D(-100000, -100000));
	polygon.push_back(QVector2D(100000, -100000));
	polygon.push_back(QVector2D(100000, 100000));
	polygon.push_back(QVector2D(-100000, 100000));

	ZoneType zone(ZoneType::TYPE_RESIDENTIAL, 1);
	zone.init();

	return std::make_pair(polygon, zone);
}
