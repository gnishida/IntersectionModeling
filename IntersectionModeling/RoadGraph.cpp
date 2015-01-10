#include "RoadGraph.h"
#include <QGLWidget>
#include "GraphUtil.h"
#include "Util.h"
#include "global.h"


bool compare2ndPartTuple2 (const std::pair<std::pair<QVector3D,QVector2D>,float> &i, const std::pair<std::pair<QVector3D,QVector2D>,float> &j) {
	return (i.second<j.second);
}

RoadGraph::RoadGraph() {
	modified = false;
}

RoadGraph::~RoadGraph() {
}

void RoadGraph::generateMesh(VBORenderManager& renderManager) {
	if (!modified) return;

	modified = false;

	renderManager.removeStaticGeometry("3d_roads");
	renderManager.removeStaticGeometry("3d_roads_interCom");
	renderManager.removeStaticGeometry("3d_roads_inter");

	updateRoadGraph(renderManager);
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

void RoadGraph::updateRoadGraph(VBORenderManager& rendManager) {
	const int renderRoadType=2;
	float deltaZ=G::global().getFloat("3d_road_deltaZ");//avoid road intersect with terrain
	// 0 No polylines No intersection
	// 1 Polylines Circle Intersection --> GOOD
	// 2 Polylines Circle+Poly Intersection
	// 3 Polylines Circle+Complex --> GOOD

	//////////////////////////////////////////
	// TYPE=1/2/3 POLYLINES 
	if(renderRoadType==1||renderRoadType==2||renderRoadType==3){
		float const maxSegmentLeng=5.0f;//5.0f

		RoadEdgeIter ei, eiEnd;
		QVector3D p0,p1;

		std::vector<Vertex> vertROAD[2];
		QVector3D a0,a1,a2,a3;
		QVector3D per,dir,lastDir;
		float length;
		for (boost::tie(ei, eiEnd) = boost::edges(graph); ei != eiEnd; ++ei) {
			if (!graph[*ei]->valid) continue;

			RoadEdgePtr edge = graph[*ei];
			float hWidth=graph[*ei]->getWidth()/2.0f;//magic 1.1f !!!! (make roads go below buildings

			int type;
			switch (graph[*ei]->type) {
			case RoadEdge::TYPE_HIGHWAY:
				type=1;//should have its texture!!! TODO
				break;
			case RoadEdge::TYPE_BOULEVARD:
			case RoadEdge::TYPE_AVENUE:
				type=1;
				break;
			case RoadEdge::TYPE_STREET:
				type=0;
				break;
			default:
				type=0;
				break;
			}

			// GEN
			type = 0;
			if (graph[*ei]->getWidthUnit() > 2) {
				type = 1;
			}
			
			float lengthMovedL=0;//road texture dX
			float lengthMovedR=0;//road texture dX

			for (int pL = 0; pL < edge->polyline3D.size()-1; pL++) {
				bool bigAngle=false;
				p0 = edge->polyline3D[pL];
				p1 = edge->polyline3D[pL+1];
				//p0.setZ(deltaZ);
				//p1.setZ(deltaZ);

				lastDir=dir;//save to compare

				dir=(p1-p0);//.normalized();
				length=dir.length();
				dir/=length;
				
				per = QVector3D(-dir.y(), dir.x(), 0.0f).normalized();
				if (pL == 0) {
					a0 = p0 - per * hWidth;
					a3 = p0 + per * hWidth;
				}
				a1 = p1 - per * hWidth;
				a2 = p1 + per * hWidth;

				if (pL < edge->polyline3D.size() - 2) {
					QVector3D p2 = edge->polyline3D[pL + 2];
					//p2.setZ(deltaZ);
					
					Util::getIrregularBisector(p0, p1, p2, hWidth, hWidth, a2);
					Util::getIrregularBisector(p0, p1, p2, -hWidth, -hWidth, a1);
					//a1.setZ(deltaZ);
					//a2.setZ(deltaZ);
				}
				
				float middLenghtR=length;
				float middLenghtL=length;
				float segmentLengR, segmentLengL;
				int numSegments=ceil(length/5.0f);

				float dW=7.5f;//tex size in m

				QVector3D b0, b3;
				QVector3D b1 = a0;
				QVector3D b2 = a3;
				QVector3D vecR = a1 - a0;
				QVector3D vecL = a2 - a3;

				for(int nS=0;nS<numSegments;nS++){
					segmentLengR=std::min(maxSegmentLeng,middLenghtR);
					segmentLengL=std::min(maxSegmentLeng,middLenghtL);

					b0 = b1;
					b3 = b2;
					if (nS < numSegments - 1) {
						b1 += dir * segmentLengR;
						b2 += dir * segmentLengL;
					} else {
						b1 = a1;
						b2 = a2;
					}
					//printf("a %f %f b %f %f %f %f\n",a2.z(),a1.z(),b0.z(),b1.z(),b2.z(),b3.z());
					vertROAD[type].push_back(Vertex(b0,QColor(),QVector3D(0,0,1.0f),QVector3D(1,lengthMovedR / dW,0)));
					vertROAD[type].push_back(Vertex(b1,QColor(),QVector3D(0,0,1.0f),QVector3D(1,(lengthMovedR + segmentLengR) / dW,0)));
					vertROAD[type].push_back(Vertex(b2,QColor(),QVector3D(0,0,1.0f),QVector3D(0,(lengthMovedL + segmentLengL) / dW,0)));
					vertROAD[type].push_back(Vertex(b3,QColor(),QVector3D(0,0,1.0f),QVector3D(0,lengthMovedL / dW,0)));

					lengthMovedR+=segmentLengR;
					lengthMovedL+=segmentLengL;
					middLenghtR-=segmentLengR;
					middLenghtL-=segmentLengL;
				}				

				a3 = a2;
				a0 = a1;
			}
		}

		// add all geometry
		rendManager.addStaticGeometry("3d_roads",vertROAD[0],"../data/textures/roads/road_2lines.jpg",GL_QUADS,2);
		rendManager.addStaticGeometry("3d_roads",vertROAD[1],"../data/textures/roads/road_4lines.jpg",GL_QUADS,2);
	}

	//////////////////////////////////////////
	// TYPE=1 Circle Intersection

	if(renderRoadType==1){
		// INTERSECTIONS
		std::vector<Vertex> intersectCirclesV;
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
			if (!graph[*vi]->valid) continue;

			// get the largest width of the outing edges
			float max_r = 0;
			int max_roadType = 0;
			float offset = 0.3f;
			RoadOutEdgeIter oei, oeend;
			int numO=0;
			for (boost::tie(oei, oeend) = boost::out_edges(*vi, graph); oei != oeend; ++oei) {
				if (!graph[*oei]->valid) continue;

				float r = graph[*oei]->getWidth();
				if (r > max_r) {
					max_r = r;
				}

				if (graph[*oei]->type > max_roadType) {
					max_roadType = graph[*oei]->type;
				}
				numO++;
			}
			QVector3D center=graph[*vi]->pt3D;
			if(numO<=2)
				center.setZ(deltaZ-0.1f);//below
			else
				center.setZ(deltaZ+0.1f);//above

			float radi1 = max_r /2.0f;
			if(numO==2)radi1*=1.10f;
			if(numO>3)radi1*=1.2f;

			const float numSides=20;
			const float deltaAngle=( 1.0f / numSides ) * 3.14159f * 2.0f;
			float angle=0.0f;
			QVector3D nP,oP;
			oP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );//init point
			for(int i=0;i<numSides+1;i++){
				angle=deltaAngle*i;
				nP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );

				intersectCirclesV.push_back(Vertex(center,center/7.5f));
				intersectCirclesV.push_back(Vertex(center+oP,(center+oP)/7.5f));
				intersectCirclesV.push_back(Vertex(center+nP,(center+nP)/7.5f));
				oP=nP;
			}


		}//all vertex
		rendManager.addStaticGeometry("3d_roads_inter",intersectCirclesV,"../data/textures/roads/road_0lines.jpg",GL_TRIANGLES,2|mode_AdaptTerrain);

	}

	//////////////////////////////////////////
	// TYPE=3  Circle+Complex
	if (renderRoadType == 2) {
		// 2. INTERSECTIONS
		std::vector<Vertex> intersectCirclesV;

		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
			if (!graph[*vi]->valid) continue;

			int outDegree=0;//boost::out_degree(*vi,roadGraph.graph);
			RoadOutEdgeIter oei, oeend;
			for (boost::tie(oei, oeend) = boost::out_edges(*vi, graph); oei != oeend; ++oei) {
				if (!graph[*oei]->valid) continue;
				outDegree++;
			}
			////////////////////////
			// 2.1 JUST TWO OR LESS--> CIRCLE BELOW
			if(outDegree<=3){//if(outDegree<=2){
				// get the largest width of the outing edges
				float max_r = 0;
				int max_roadType = 0;
				float offset = 0.3f;
				RoadOutEdgeIter oei, oeend;
				for (boost::tie(oei, oeend) = boost::out_edges(*vi, graph); oei != oeend; ++oei) {
					if (!graph[*oei]->valid) continue;

					float r = graph[*oei]->getWidth();
					if (r > max_r) {
						max_r = r;
					}

					if (graph[*oei]->type > max_roadType) {
						max_roadType = graph[*oei]->type;
					}
				}
				QVector3D center=graph[*vi]->pt3D;
				if(outDegree<=2)
					center.setZ(center.z() - 0.1f);//deltaZ-0.1f);//below
				else
					center.setZ(center.z() + 0.1f);//deltaZ+0.1f);//above

				float radi1 = max_r /2.0f;
				//if(outDegree==2)radi1*=1.10f;

				const float numSides=20;
				const float deltaAngle=( 1.0f / numSides ) * 3.14159f * 2.0f;
				float angle=0.0f;
				QVector3D nP,oP;
				oP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );//init point
				for(int i=0;i<numSides+1;i++){
					angle=deltaAngle*i;
					nP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );

					intersectCirclesV.push_back(Vertex(center,center/7.5f));
					intersectCirclesV.push_back(Vertex(center+oP,(center+oP)/7.5f));
					intersectCirclesV.push_back(Vertex(center+nP,(center+nP)/7.5f));
					oP=nP;
				}

			}
		}//all vertex

		rendManager.addStaticGeometry("3d_roads_inter",intersectCirclesV,"../data/textures/roads/road_0lines.jpg",GL_TRIANGLES,2);
	}


	//////////////////////////////////////////
	// TYPE=3  Circle+Complex
	if (renderRoadType == 3) {
		// 2. INTERSECTIONS
		std::vector<Vertex> intersectCirclesV;

		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
			if (!graph[*vi]->valid) continue;

			int outDegree=0;//boost::out_degree(*vi,roadGraph.graph);
			RoadOutEdgeIter oei, oeend;
			for (boost::tie(oei, oeend) = boost::out_edges(*vi, graph); oei != oeend; ++oei) {
				if (!graph[*oei]->valid) continue;
				outDegree++;
			}
			////////////////////////
			// 2.1 JUST TWO OR LESS--> CIRCLE BELOW
			if(outDegree<=3){//if(outDegree<=2){
				// get the largest width of the outing edges
				float max_r = 0;
				int max_roadType = 0;
				float offset = 0.3f;
				RoadOutEdgeIter oei, oeend;
				for (boost::tie(oei, oeend) = boost::out_edges(*vi, graph); oei != oeend; ++oei) {
					if (!graph[*oei]->valid) continue;

					float r = graph[*oei]->getWidth();
					if (r > max_r) {
						max_r = r;
					}

					if (graph[*oei]->type > max_roadType) {
						max_roadType = graph[*oei]->type;
					}
				}
				QVector3D center=graph[*vi]->pt3D;
				if(outDegree<=2)
					center.setZ(center.z() - 0.1f);//deltaZ-0.1f);//below
				else
					center.setZ(center.z() + 0.1f);//deltaZ+0.1f);//above

				float radi1 = max_r /2.0f;
				//if(outDegree==2)radi1*=1.10f;

				const float numSides=20;
				const float deltaAngle=( 1.0f / numSides ) * 3.14159f * 2.0f;
				float angle=0.0f;
				QVector3D nP,oP;
				oP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );//init point
				for(int i=0;i<numSides+1;i++){
					angle=deltaAngle*i;
					nP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );

					intersectCirclesV.push_back(Vertex(center,center/7.5f));
					intersectCirclesV.push_back(Vertex(center+oP,(center+oP)/7.5f));
					intersectCirclesV.push_back(Vertex(center+nP,(center+nP)/7.5f));
					oP=nP;
				}

			}else{
				////////////////////////
				// 2.2 FOUR OR MORE--> COMPLEX INTERSECTION

				//printf("a\n");
				////////////
				// 2.2.1 For each vertex find edges and short by angle
				QVector2D referenceVector(0,1);
				QVector2D p0,p1;
				std::vector<std::pair<std::pair<QVector3D,QVector2D>,float>> edgeAngleOut;
				int numOutEdges=0;
				RoadOutEdgeIter Oei, Oei_end;
				float angleRef=atan2(referenceVector.y(),referenceVector.x());
				//printf("a1\n");
				for(boost::tie(Oei, Oei_end) = boost::out_edges(*vi,graph); Oei != Oei_end; ++Oei){
					if (!graph[*Oei]->valid) continue;
					// find first segment 
					RoadEdgePtr edge = graph[*Oei];

					Polyline2D polyline = GraphUtil::orderPolyLine(*this, *Oei, *vi);
					p0 = polyline[0];
					p1 = polyline[1];

					QVector2D edgeDir=(p1-p0).normalized();// NOTE p1-p0
					p1=p0+edgeDir*30.0f;//expand edge to make sure it is long enough

					float angle=angleRef-atan2(edgeDir.y(),edgeDir.x());
					float width=edge->getWidth();//*1.1f;//1.1 (since in render this is also applied)
					edgeAngleOut.push_back(std::make_pair(std::make_pair(QVector3D(p0.x(),p0.y(),width),p1),angle));//z as width
					numOutEdges++;
				}
				//printf("a2\n");
				if(edgeAngleOut.size()>0){
					std::sort( edgeAngleOut.begin(), edgeAngleOut.end(), compare2ndPartTuple2);
				}
				//printf("b\n");
				// 2.2.2 Create intersection geometry of the given edges
				QVector3D ed1p0,ed1p1;
				QVector3D ed1p0L,ed1p1L,ed1p0R,ed1p1R;
				QVector3D ed2p0L,ed2p1L,ed2p0R,ed2p1R;//right
				QVector3D ed1Dir,ed1Per;
				QVector3D ed2DirL,ed2DirR,ed2PerL,ed2PerR;
				float ed1W;
				float ed2WL,ed2WR;
				std::vector<QVector3D> interPoints;
				std::vector<Vertex> interVertex;
				std::vector<Vertex> interPedX;
				std::vector<Vertex> interPedXLineR;
				//printf("c\n");
				for(int eN=0;eN<edgeAngleOut.size();eN++){
					//printf("** eN %d\n",eN);
					// a) ED1: this edge
					ed1W=edgeAngleOut[eN].first.first.z();//use z as width
					ed1p0=edgeAngleOut[eN].first.first;
					ed1p0.setZ(0);
					ed1p1=edgeAngleOut[eN].first.second.toVector3D();
					// compute right side
					ed1Dir=(ed1p0-ed1p1).normalized();//ends in 0
					ed1Per=(QVector3D::crossProduct(ed1Dir,QVector3D(0,0,1.0f)).normalized());//need normalized()?
					ed1p0R=ed1p0+ed1Per*ed1W/2.0f;
					ed1p1R=ed1p1+ed1Per*ed1W/2.0f;
					// compute left side
					ed1p0L=ed1p0-ed1Per*ed1W/2.0f;
					ed1p1L=ed1p1-ed1Per*ed1W/2.0f;

					// b) ED2: next edge
					int lastEdge=eN-1;
					if(lastEdge<0)lastEdge=edgeAngleOut.size()-1;
					//printf("last eN %d\n",lastEdge);
					ed2WL=edgeAngleOut[lastEdge].first.first.z();//use z as width
					ed2p0L=edgeAngleOut[lastEdge].first.first;
					ed2p0L.setZ(0);
					ed2p1L=edgeAngleOut[lastEdge].first.second.toVector3D();
					// compute left side
					ed2DirL=(ed2p0L-ed2p1L).normalized();//ends in 0
					ed2PerL=(QVector3D::crossProduct(ed2DirL,QVector3D(0,0,1.0f)).normalized());//need normalized()?
					ed2p0L-=ed2PerL*ed2WL/2.0f;
					ed2p1L-=ed2PerL*ed2WL/2.0f;

					// c) ED2: last edge
					int nextEdge=(eN+1)%edgeAngleOut.size();
					//printf("next eN %d\n",nextEdge);

					ed2WR=edgeAngleOut[nextEdge].first.first.z();//use z as width
					ed2p0R=edgeAngleOut[nextEdge].first.first;
					ed2p0R.setZ(0);
					ed2p1R=edgeAngleOut[nextEdge].first.second.toVector3D();
					// compute left side
					ed2DirR=(ed2p0R-ed2p1R).normalized();//ends in 0
					ed2PerR=(QVector3D::crossProduct(ed2DirR,QVector3D(0,0,1.0f)).normalized());//need normalized()?
					ed2p0R+=ed2PerR*ed2WR/2.0f;
					ed2p1R+=ed2PerR*ed2WR/2.0f;

					//////////////////////////////////////////
					// d) Computer interior coordinates
					// d.1 computer intersection left
					QVector3D intPoint1(FLT_MAX,0,0);
					if(acos(QVector3D::dotProduct(ed1Dir,ed2DirL))<(170.0f*0.0174532925f)){//angle smaller than 45 degrees
						float tab,tcd;
						//printf("ED1 %f %f --> %f %f  ED2 %f %f --> %f %f\n",ed1p0R.x(),ed1p0R.y(),ed1p1R.x(),ed1p1R.y(), ed2p0L.x(),ed2p0L.y(),ed2p1L.x(),ed2p1L.y());
						if(Util::segmentSegmentIntersectXY3D(ed1p0R,ed1p1R,ed2p0L,ed2p1L,&tab,&tcd, false,intPoint1)==false&&false){
							printf("ERROR: Parallel!!!\n");
							continue;
						}else{
							//printf("Int %f %f\n",intPoint1.x(),intPoint1.y());
							//printf("ADD: No Parallel!!!\n");
						}
					}
					// d.2 computer intersecion right
					QVector3D intPoint2(FLT_MAX,0,0);
					if(acos(QVector3D::dotProduct(ed1Dir,ed2DirR))<(170.0f*0.0174532925f)){//angle smaller than 45 degrees
						float tab,tcd;
						//printf("ED1 %f %f --> %f %f  ED2 %f %f --> %f %f\n",ed1p0L.x(),ed1p0L.y(),ed1p1L.x(),ed1p1L.y(), ed2p0R.x(),ed2p0R.y(),ed2p1R.x(),ed2p1R.y());
						if(Util::segmentSegmentIntersectXY3D(ed1p0L,ed1p1L,ed2p0R,ed2p1R,&tab,&tcd, false,intPoint2)==false){
							printf("ERROR: Parallel!!!\n");
							continue;
						}else{
							//printf("Int %f %f\n",intPoint2.x(),intPoint2.y());
							//printf("ADD: No Parallel!!!\n");
						}
					}
					if(intPoint1.x()==FLT_MAX&&intPoint2.x()==FLT_MAX){
						printf("ERROR: No intersect both sides\n");
						printf("angle1 %f\n",acos(QVector3D::dotProduct(ed1Dir,ed2DirR))/0.0174532925f);
						printf("angle2 %f\n",acos(QVector3D::dotProduct(ed1Dir,ed2DirL))/0.0174532925f);
						exit(0);//for now exit program
					}
					if(intPoint1.x()==FLT_MAX){
						intPoint1=intPoint2-ed1Per*ed1W;
					}
					if(intPoint2.x()==FLT_MAX){
						intPoint2=intPoint1+ed1Per*ed1W;
					}

					// middle
					float zOff=0.1f;
					intPoint2.setZ(graph[*vi]->pt3D.z() + zOff);//deltaZ+zOff);
					intPoint1.setZ(graph[*vi]->pt3D.z() + zOff);//deltaZ+zOff);
					
					interPoints.push_back(intPoint1);
					interPoints.push_back(intPoint2);

					// EDIT: to make the stop line perpendicular to the road direction
					if (QVector3D::dotProduct(intPoint1 - intPoint2, ed1Dir) >= 0) {
						intPoint1 -= ed1Dir * QVector3D::dotProduct(intPoint1 - intPoint2, ed1Dir);
					} else {
						intPoint2 += ed1Dir * QVector3D::dotProduct(intPoint1 - intPoint2, ed1Dir);
					}

					// pedX
					interPedX.push_back(Vertex(intPoint1,QVector3D(0-0.07f,0,0)));
					interPedX.push_back(Vertex(intPoint2,QVector3D(ed1W/7.5f+0.07f,0,0)));
					interPedX.push_back(Vertex(intPoint2-ed1Dir*3.5f,QVector3D(ed1W/7.5f+0.07f,1.0f,0)));
					interPedX.push_back(Vertex(intPoint1-ed1Dir*3.5f,QVector3D(0.0f-0.07f,1.0f,0)));
					// Line in right lines
					QVector3D midPoint=(intPoint2+intPoint1)/2.0f+0.2f*ed1Per;


					interPedXLineR.push_back(Vertex(intPoint1-ed1Dir*3.5f,QVector3D(0,0.0f,0)));
					interPedXLineR.push_back(Vertex(midPoint-ed1Dir*3.5f,QVector3D(1.0f,0.0f,0)));
					interPedXLineR.push_back(Vertex(midPoint-ed1Dir*4.25f,QVector3D(1.0f,1.0f,0)));
					interPedXLineR.push_back(Vertex(intPoint1-ed1Dir*4.25f,QVector3D(0.0f,1.0f,0)));

				}

				if(interPoints.size()>2){
						
					{
						for(int iP=0;iP<interPoints.size()-1;iP++){//remove duplicated
							if((interPoints[iP]-interPoints[iP+1]).lengthSquared()<0.5f){
								interPoints.erase(interPoints.begin()+iP);
								iP--;
							}
						}
					}

					rendManager.addStaticGeometry2("3d_roads_interCom",interPoints,0.0f,false,"../data/textures/roads/road_0lines.jpg",GL_QUADS,2,QVector3D(1.0f/7.5f,1.0f/7.5f,1),QColor());//0.0f (moved before)
				}
				rendManager.addStaticGeometry("3d_roads_interCom",interPedX,"../data/textures/roads/road_pedX.jpg",GL_QUADS,2);
				rendManager.addStaticGeometry("3d_roads_interCom",interPedXLineR,"../data/textures/roads/road_pedXLineR.jpg",GL_QUADS,2);
			}
		}//all vertex

		rendManager.addStaticGeometry("3d_roads_inter",intersectCirclesV,"../data/textures/roads/road_0lines.jpg",GL_TRIANGLES,2);
	}


}//
