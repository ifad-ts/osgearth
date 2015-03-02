#include <cassert>
#include <algorithm>
#include <iostream>
#include <osgEarthFeatures/RoofBuilder>

#define LC "[RoofBuilder] "

using namespace osgEarth;
using namespace osgEarth::Features;

#define EPSILON 0.001f

float min(float f1, float f2){return f1<f2 ? f1:f2;}
float max(float f1, float f2){return f1>f2 ? f1:f2;}

float	defaultLength = FLT_MAX;


bool test2DRayRay(osg::Vec2 p1, osg::Vec2 d1, osg::Vec2 p2, osg::Vec2 d2, osg::Vec2& intersect)
{
	osg::Vec2 n(-d2.y(), d2.x());
	if(fabs(n*d1) < EPSILON ) //parallel rays
		return false;
	
	float d = (p2-p1)*n;
	if(fabs(d) < EPSILON)
	{
		intersect = p1;
	} else{
		float projd1 = n*d1;
		intersect = p1 + d1*(d/projd1);
	}
	return true;
}

bool test2DSegmentSegment(osg::Vec2 p1, osg::Vec2 d1, float l1, osg::Vec2 p2, osg::Vec2 d2, float l2, float& t, osg::Vec2& intersect)
{
	bool b = test2DRayRay(p1,d1,p2,d2,intersect);
	if(!b)
		return false;
	
	osg::Vec2 d = intersect - p2;
	float dot = d2*d;
	if(dot < -EPSILON || dot - l2 > EPSILON )
		return false;

	d = intersect - p1;
	dot = d1*d;
	if(dot < -EPSILON || dot - l1 > EPSILON )
		return false;

	t = dot/l1;
	return true;
}


//////////////////////////////////////////////////////////////////////////

RoofFaceEdge::RoofFaceEdge(unsigned int _edgeIndex, std::vector<RoofFace>& roofFaces, unsigned int _leftFaceIndex, unsigned int _rightFaceIndex):
edgeIndex(_edgeIndex),
leftFaceIndex(_leftFaceIndex),
rightFaceIndex(_rightFaceIndex),
bActive(true),
createTime(0.0)
{
	unsigned int edgeSize = roofFaces[leftFaceIndex].rightEdge.size();
	vStart = roofFaces[leftFaceIndex].rightEdge[edgeSize-1]; // last point in edge

	if(edgeSize==1)
	{
		vDir	= roofFaces[leftFaceIndex].edgeNormal + roofFaces[rightFaceIndex].edgeNormal;
		vDir.normalize(); 
	}
	else
	{
		osg::Vec3 leftNormal(roofFaces[leftFaceIndex].edgeNormal.x(),roofFaces[leftFaceIndex].edgeNormal.y(),1.0f);
		osg::Vec3 rightNormal(roofFaces[rightFaceIndex].edgeNormal.x(),roofFaces[rightFaceIndex].edgeNormal.y(),1.0f);
		osg::Vec3 dir3D = leftNormal^rightNormal;
		dir3D.z() = 0.0f;
		dir3D.normalize();

		vDir.x() = dir3D.x();
		vDir.y() = dir3D.y();


		osg::Vec2 testDir = roofFaces[rightFaceIndex].edgeDir;
		if((vDir*testDir) < 0.0f)
			vDir *= -1.0;
	}

	length = defaultLength;
}

osg::Vec2 RoofFaceEdge::getEndPoint()
{
	return vStart + vDir*length;
}

void RoofFaceEdge::clipEdgeToOutline(std::vector<RoofFace>& roofFaces)
{
	int numFaces = roofFaces.size();
	for(int i=0;i<numFaces;i++)
	{
		if(i==leftFaceIndex || i==rightFaceIndex)
			continue;
		RoofFace& currentFace = roofFaces[i];
		float t;
		osg::Vec2 intersect;
		if(test2DSegmentSegment(vStart, vDir, length, currentFace.p1, currentFace.edgeDir, currentFace.edgeLength, t, intersect))
		{
			//reduce length to max possible between these 2 edges
			//it's the point on the edge, with the same perpendicular distance to the 2 face edges
			// p(t) = vStart + t*vDir
			// n1 * (p(t)-currentFace.p1) = n2 * (p(t)-roofFaces[leftFaceIndex].p1)
			osg::Vec2 n1 = currentFace.edgeNormal;
			osg::Vec2 n2 = roofFaces[leftFaceIndex].edgeNormal;
			float tNew = (n1*(vStart-currentFace.p1) - n2*(vStart-roofFaces[leftFaceIndex].p1))/(vDir * (n2-n1));
			
			if(tNew >= 0 && tNew <= length)
				length = tNew;

			evt.evtType = SkeletonEvent::SPLIT;
			evt.intersection = intersect;
			evt.time = currentFace.edgeNormal * (getEndPoint() - currentFace.p1);
			evt.faceIndex = i;
		}
	}
}

void RoofFaceEdge::clipToEdge(std::vector<RoofFace>& roofFaces, RoofFaceEdge& edge)
{
	float t;
	osg::Vec2 intersect;
	if(test2DSegmentSegment(vStart, vDir, length, edge.vStart, edge.vDir, edge.length, t, intersect))
	{
		setIntersection(roofFaces,edge,intersect);
	}
}

// clips edge to first intersection with neighbor edges
void RoofFaceEdge::clipToEdgeToNeighbors(std::vector<RoofFace>& roofFaces,std::vector<RoofFaceEdge>& faceEdges)
{
	RoofFaceEdge* pLeftEdge = NULL;
	RoofFaceEdge* pRightEdge = NULL;
	int nEdges = faceEdges.size();
	for(int i=0;i<nEdges;i++)
	{
		RoofFaceEdge& currentEdge = faceEdges[i];
		if(currentEdge.bActive)
		{
			if(currentEdge.rightFaceIndex == leftFaceIndex)
			{
				pLeftEdge = &currentEdge;
			}
			else if(currentEdge.leftFaceIndex == rightFaceIndex)
			{
				pRightEdge = &currentEdge;
			}
		}
	}

	if(pLeftEdge==NULL || pRightEdge == NULL)
	{
		Disable();
		return;
	}

	RoofFaceEdge& leftEdge = *pLeftEdge;
	float leftT = FLT_MAX;
	osg::Vec2 leftIntersection;
	bool bIntersectLeft = test2DSegmentSegment(vStart, vDir, length, leftEdge.vStart, leftEdge.vDir, leftEdge.length, leftT, leftIntersection);
	
	RoofFaceEdge& rightEdge = *pRightEdge;
	float rightT = FLT_MAX;
	osg::Vec2 rightIntersection;
	bool bIntersectRight = test2DSegmentSegment(vStart, vDir, length, rightEdge.vStart, rightEdge.vDir, rightEdge.length, rightT, rightIntersection);
	
	if(bIntersectLeft && bIntersectRight)
	{
		if(leftT < rightT)
		{
			setIntersection(roofFaces,leftEdge, leftIntersection);
		}
		else
		{
			setIntersection(roofFaces,rightEdge, rightIntersection);
		}
	}
	else if(bIntersectLeft)
	{
		setIntersection(roofFaces,leftEdge, leftIntersection);
	}
	else if(bIntersectRight)
	{
		setIntersection(roofFaces,rightEdge, rightIntersection);
	}
}

void RoofFaceEdge::setIntersection(std::vector<RoofFace>& roofFaces, RoofFaceEdge& edge, osg::Vec2 intersection)
{
	length = (intersection-vStart).length();
	edge.length = (intersection-edge.vStart).length();

	evt.evtType = SkeletonEvent::EDGE;
	edge.evt.evtType = SkeletonEvent::EDGE;

	evt.intersection = intersection;
	edge.evt.intersection = intersection;

	evt.time = roofFaces[leftFaceIndex].edgeNormal * (getEndPoint() - roofFaces[leftFaceIndex].p1);
	edge.evt.time = evt.time;

	evt.edgeIndex = edge.edgeIndex;
	edge.evt.edgeIndex = edgeIndex;
}

RoofFace::RoofFace(unsigned int index, const osg::Vec2& _p1, const osg::Vec2& _p2):
faceIndex(index),
p1(_p1),
p2(_p2)
{
	leftEdge.push_back(p1);
	rightEdge.push_back(p2);
	initEdgeData();

	textureOrigo = p1;
}

void RoofFace::initEdgeData()
{
	edgeNormal.x() = -(p2.y()-p1.y());
	edgeNormal.y() =  (p2.x()-p1.x());
	edgeNormal.normalize();

	edgeDir = p2-p1;
	edgeLength = edgeDir.normalize();
}

RoofFace& RoofFace::splitFace(std::vector<RoofFace>& roofFaces, osg::Vec2 splitPoint)
{
	roofFaces.push_back(RoofFace(roofFaces.size(),splitPoint, p2));
	RoofFace& newFace = roofFaces[roofFaces.size()-1];

	newFace.rightEdge.clear();
	newFace.rightEdge.insert(newFace.rightEdge.begin(), rightEdge.begin(), rightEdge.end());
	
	rightEdge.clear();
	rightEdge.push_back(splitPoint);
	p2 = splitPoint;
	initEdgeData();

	newFace.textureOrigo = textureOrigo;

	return newFace;
}

osg::Vec2 RoofFace::getLeftEndPoint()
{
	return leftEdge[leftEdge.size()-1];
}

osg::Vec2 RoofFace::getRightEndPoint()
{
	return rightEdge[rightEdge.size()-1];
}

float RoofFace::getPointDist(osg::Vec2 p)
{
	return fabs(edgeNormal*(p-p1));
}

void RoofFace::generateGeometry(osg::Vec3Array* roofVerts, osg::Vec3Array* roofNormals, osg::Vec3Array* roofTexCoords, float texSpanY, float roofStartZ, float roofAngle)
{
	float roofAngleRad = osg::DegreesToRadians(roofAngle);
	float tangent = tan(roofAngleRad);

	std::vector<osg::Vec3> outline;
	outline.push_back(osg::Vec3(p1.x(),p1.y(),roofStartZ));
	outline.push_back(osg::Vec3(p2.x(),p2.y(),roofStartZ));
	int rightEdgeSize = rightEdge.size();
	for(int i=1; i<rightEdgeSize;i++) //skip rightEdge[0] since it's p2'
	{
		float height = roofStartZ + getPointDist(rightEdge[i])*tangent;
		outline.push_back(osg::Vec3(rightEdge[i].x(), rightEdge[i].y(), height));
	}
	
	int leftEdgeSize = leftEdge.size();
	for(int i=leftEdgeSize-1; i>0;i--) // skip leftEdge[0] since it's p1
	{
		float height =  roofStartZ + getPointDist(leftEdge[i])*tangent;
		outline.push_back(osg::Vec3(leftEdge[i].x(), leftEdge[i].y(), height));
	}
	int nPoints = outline.size();
	osg::Vec3 roofNormal = osg::Vec3(0,0,sin(roofAngleRad)) -osg::Vec3(edgeNormal.x(),edgeNormal.y(),0.0f)*cos(roofAngleRad);
	osg::Vec3 uAxis = osg::Vec3(edgeNormal.x(),edgeNormal.y(),0.0f)*cos(roofAngleRad) + osg::Vec3(0,0,sin(roofAngleRad));
	osg::Vec3 vAxis = osg::Vec3(edgeDir.x(),edgeDir.y(),0.0f);
	osg::Vec3 textureOrigo3D = osg::Vec3(textureOrigo.x(),textureOrigo.y(), roofStartZ); 
	for(int i=0;i<nPoints;i++)
	{
		roofVerts->push_back(outline[i]);
		roofNormals->push_back( roofNormal);

		osg::Vec3 dir = outline[i]-textureOrigo3D;
		float u = (uAxis * dir)/texSpanY;
		float v = (vAxis * dir)/texSpanY;
		roofTexCoords->push_back(osg::Vec3(u,v,0));
	}

}

RoofBuilder2D::RoofBuilder2D(std::vector<osg::Vec3> outline, float roofAngle)
{
	int numPoints = outline.size();
	std::vector<osg::Vec2> outline2D;
	for(int i=0;i<numPoints;i++)
	{
		osg::Vec2 p(outline[i].x(), outline[i].y());
		outline2D.push_back(p);
	}

	roofStartZ = outline[0].z();
	//test
	//outline2D.clear();
	
	/*outline2D.push_back(osg::Vec2(0,0));
	outline2D.push_back(osg::Vec2(10,0));
	outline2D.push_back(osg::Vec2(10,10));
	outline2D.push_back(osg::Vec2(0,10));*/

	/*outline2D.push_back(osg::Vec2(0,0));
	outline2D.push_back(osg::Vec2(10,0));
	outline2D.push_back(osg::Vec2(10,10));
	outline2D.push_back(osg::Vec2(5,2));
	outline2D.push_back(osg::Vec2(0,10));*/

	/*outline2D.push_back(osg::Vec2(0,0));
	outline2D.push_back(osg::Vec2(7,0));
	outline2D.push_back(osg::Vec2(7,9));
	outline2D.push_back(osg::Vec2(18,9));
	outline2D.push_back(osg::Vec2(18,17));
	outline2D.push_back(osg::Vec2(0,17));

	numPoints = outline2D.size();*/

	roofFaces.reserve(1000);

	vMin = osg::Vec2(FLT_MAX,FLT_MAX);
	vMax = osg::Vec2(FLT_MIN,FLT_MIN);	

	for(int i=0; i<numPoints; i++)
	{
		int iNext = (i+1)%numPoints;
		roofFaces.push_back(RoofFace(i,outline2D[i], outline2D[iNext]));

		vMin.x() = min(vMin.x(), outline2D[i].x());
		vMin.y() = min(vMin.y(), outline2D[i].y());

		vMax.x() = max(vMax.x(), outline2D[i].x());
		vMax.y() = max(vMax.y(), outline2D[i].y());
	}
	defaultLength = (vMax-vMin).length();

	faceEdges.reserve(1000);
	
	for(int i=0; i<numPoints; i++)
	{
		int iNext = (i+1)%numPoints;
		faceEdges.push_back(RoofFaceEdge(i,roofFaces,i,iNext));
	}

	clipEdgesToOutline();

	clipEdgesToEdges();

	int minEdgeIndex = getMinEdgeIndex();
	while(minEdgeIndex != -1)
	{
		RoofFaceEdge& minEdge = faceEdges[minEdgeIndex];
		if(minEdge.evt.evtType == SkeletonEvent::EDGE)
		{
			edgeEvent(minEdge);
		}
		else if(minEdge.evt.evtType == SkeletonEvent::SPLIT)
		{
			splitEvent(minEdge);
		}
		minEdgeIndex = getMinEdgeIndex();

		if(roofFaces.size()>1000)
			int a=0;
	}
}

void RoofBuilder2D::clipEdgesToOutline()
{
	int numEdges = faceEdges.size();
	for(int i=0;i<numEdges;i++)
	{
		if(!faceEdges[i].bActive)
			continue;

		faceEdges[i].clipEdgeToOutline(roofFaces);
	}
}

void RoofBuilder2D::clipEdgesToEdges()
{
	int numEdges = faceEdges.size();
	for(int i=0;i<numEdges;i++)
	{
		RoofFaceEdge& edge1 = faceEdges[i];
		
		if(!edge1.bActive)
			continue;

		for(int j=i+1 ; j<numEdges; j++)
		{
			RoofFaceEdge& edge2 = faceEdges[j];
			
			if(!edge2.bActive)
				continue;

			if(!(edge1.leftFaceIndex == edge2.rightFaceIndex || edge1.rightFaceIndex == edge2.leftFaceIndex))
				continue;

			edge1.clipToEdge(roofFaces, edge2);
		}
	}
}

int RoofBuilder2D::getMinEdgeIndex()
{
	float minTime = FLT_MAX;
	int numEdges = faceEdges.size();
	int minEdgeIndex = -1;
	for(int i=0; i<numEdges; i++)
	{
		if(!faceEdges[i].bActive || (faceEdges[i].createTime > faceEdges[i].evt.time))
			continue;
		if(faceEdges[i].evt.time < minTime)
		{
			if(faceEdges[i].evt.evtType == SkeletonEvent::EDGE && faceEdges[faceEdges[i].evt.edgeIndex].bActive==false)
				continue;
			minTime = faceEdges[i].evt.time;
			minEdgeIndex = i;
		}
	}
	return minEdgeIndex;
}

void RoofBuilder2D::edgeEvent(RoofFaceEdge& edge)
{
	osg::Vec2 endPoint = edge.getEndPoint();
	RoofFaceEdge& otherEdge = faceEdges[edge.evt.edgeIndex];

	edge.Disable();
	otherEdge.Disable();

	//common face
	bool bLeftEdge = edge.rightFaceIndex == otherEdge.leftFaceIndex;
	unsigned int commonFaceIndex =  bLeftEdge ? edge.rightFaceIndex : edge.leftFaceIndex;
	//common faces neighbors
	unsigned int leftNeighborFace = bLeftEdge ? edge.leftFaceIndex : otherEdge.leftFaceIndex;
	unsigned int rightNeighborFace = bLeftEdge ? otherEdge.rightFaceIndex : edge.rightFaceIndex;

	// doesn't really matter which edge in commonface we add point to, since the face is done after this
	
	// endpoint may already be in edges , if multiple edge events happen on the same point, f.ex on a square roof, where all edges meet in the same point
	osg::Vec2 rightEndPoint = roofFaces[commonFaceIndex].getRightEndPoint();
	osg::Vec2 leftEndPoint = roofFaces[commonFaceIndex].getLeftEndPoint();
	if((endPoint-leftEndPoint).length() > EPSILON && (endPoint-rightEndPoint).length() > EPSILON  )
		roofFaces[commonFaceIndex].leftEdge.push_back(endPoint);
	
	
	rightEndPoint = roofFaces[leftNeighborFace].getRightEndPoint();
	if((endPoint-rightEndPoint).length() > EPSILON)
		roofFaces[leftNeighborFace].rightEdge.push_back(endPoint);

	leftEndPoint = roofFaces[rightNeighborFace].getLeftEndPoint();
	if((endPoint-leftEndPoint).length() > EPSILON )
		roofFaces[rightNeighborFace].leftEdge.push_back(endPoint);

	// create new edge with left and right neighbor faces
	faceEdges.push_back(RoofFaceEdge(faceEdges.size(),roofFaces,leftNeighborFace,rightNeighborFace));

	faceEdges[faceEdges.size()-1].createTime = edge.evt.time;

	//clipEdgesToOutline();
	faceEdges[faceEdges.size()-1].clipEdgeToOutline(roofFaces);
	faceEdges[faceEdges.size()-1].clipToEdgeToNeighbors(roofFaces, faceEdges);
	//clipEdgesToEdges();
}

void RoofBuilder2D::splitEvent(RoofFaceEdge& edge)
{
	edge.Disable();

	osg::Vec2 intersectPoint = edge.evt.intersection;
	osg::Vec2 splitPoint = edge.getEndPoint();

	roofFaces[edge.leftFaceIndex].rightEdge.push_back(splitPoint);
	roofFaces[edge.rightFaceIndex].leftEdge.push_back(splitPoint);
	
	RoofFace& faceToSplit = roofFaces[edge.evt.faceIndex];
	RoofFace& splitFace = faceToSplit.splitFace(roofFaces, intersectPoint);
	splitFace.leftEdge.push_back(splitPoint);
	faceToSplit.rightEdge.push_back(splitPoint);

	// find right edge of faceToSplit and update leftFaceIndex to splitface' index
	int numEdges = faceEdges.size();
	for(int i=0;i<numEdges;i++)
	{
		if(faceEdges[i].bActive)
		{
			if(faceEdges[i].leftFaceIndex == faceToSplit.faceIndex)
			{
				faceEdges[i].leftFaceIndex = splitFace.faceIndex;
				break;
			}
		}
	}

	// create new split edges
	faceEdges.push_back(RoofFaceEdge(faceEdges.size(), roofFaces, edge.leftFaceIndex, splitFace.faceIndex));
	faceEdges.push_back(RoofFaceEdge(faceEdges.size(), roofFaces, faceToSplit.faceIndex, edge.rightFaceIndex));

	// reclip edges that used to split FaceToSplit
	for(int i=0;i<numEdges;i++)
	{
		if(faceEdges[i].bActive && faceEdges[i].evt.evtType == SkeletonEvent::SPLIT)
		{
			if(faceEdges[i].evt.faceIndex == faceToSplit.faceIndex)
			{
				faceEdges[i].length = defaultLength;
				faceEdges[i].clipEdgeToOutline(roofFaces);
				faceEdges[i].clipToEdgeToNeighbors(roofFaces, faceEdges);
			}
		}
	}

	//clipEdgesToOutline();

	faceEdges[faceEdges.size()-2].createTime = edge.evt.time;
	faceEdges[faceEdges.size()-1].createTime = edge.evt.time;

	faceEdges[faceEdges.size()-2].clipEdgeToOutline(roofFaces);
	faceEdges[faceEdges.size()-1].clipEdgeToOutline(roofFaces);

	faceEdges[faceEdges.size()-2].clipToEdgeToNeighbors(roofFaces, faceEdges);
	faceEdges[faceEdges.size()-1].clipToEdgeToNeighbors(roofFaces, faceEdges);
	//clipEdgesToEdges();
}

void RoofBuilder2D::generateRoofGeometry(osg::Geometry* roofGeometry, osg::Vec3Array*  roofVerts, osg::Vec3Array* roofNormals, osg::Vec3Array* roofTexcoords, float roofTexSpanY, float roofAngle)
{
	int numRoofFaces= roofFaces.size();
	for(int i=0; i<numRoofFaces; i++)
	{
		int iStart = roofVerts->size();
		roofFaces[i].generateGeometry(roofVerts, roofNormals, roofTexcoords, roofTexSpanY, roofStartZ, roofAngle);
		int numVertsAdded = roofVerts->size() - iStart;

		roofGeometry->addPrimitiveSet( new osg::DrawArrays(GL_POLYGON, iStart, numVertsAdded) );
	}
}