#include "RoadGraph.h"
#include <QGLWidget>
#include "GraphUtil.h"
#include "Util.h"
#include "global.h"
#include "RoadMeshGenerator.h"

RoadGraph::RoadGraph() {
	modified = false;
}

RoadGraph::~RoadGraph() {
}

void RoadGraph::generateMesh(VBORenderManager& renderManager) {
	if (!modified) return;

	modified = false;

	RoadMeshGenerator::generateRoadMesh(renderManager, *this);
}

void RoadGraph::clear() {
	graph.clear();

	modified = true;
}

/**
 * adapt this road graph to the vboRenderManager.
 */
void RoadGraph::adaptToTerrain(VBORenderManager* vboRenderManager) {
	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
		float z = vboRenderManager->getTerrainHeight(graph[*vi]->pt.x(), graph[*vi]->pt.y());
		graph[*vi]->pt3D = QVector3D(graph[*vi]->pt.x(), graph[*vi]->pt.y(), z + 1);
	}

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(graph); ei != eend; ++ei) {
		RoadVertexDesc src = boost::source(*ei, graph);
		RoadVertexDesc tgt = boost::target(*ei, graph);
		graph[*ei]->polyline3D.clear();

		//Polyline2D polyline = GraphUtil::finerEdge(graph[*ei]->polyline, 10.0f);
		for (int i = 0; i < graph[*ei]->polyline.size(); ++i) {
			float z = vboRenderManager->getTerrainHeight(graph[*ei]->polyline[i].x(), graph[*ei]->polyline[i].y());
			graph[*ei]->polyline3D.push_back(QVector3D(graph[*ei]->polyline[i].x(), graph[*ei]->polyline[i].y(), z + 1));
		}
	}

	setModified();
}

