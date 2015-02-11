#include <cassert>
#include <algorithm>
#include <iostream>
#include <osgEarthFeatures/RoofBuilder>

#define LC "[RoofBuilder] "

using namespace osgEarth;
using namespace osgEarth::Features;

#define EPSILON 0.001f

RoofBuildStruct::RoofBuildStruct(const osg::Vec3& _p1, const osg::Vec3& _p2, float _roofAngle):
index1(-1),
index2(-1),
dist1(FLT_MAX),
dist2(FLT_MAX)
{
	init(_p1,_p2,_roofAngle);
}

RoofBuildStruct::RoofBuildStruct(const osg::Vec3& _p1, const osg::Vec3& _p2, RoofBuildStruct& parent):
index1(-1),
index2(-1),
dist1(FLT_MAX),
dist2(FLT_MAX)
{
	init(_p1,_p2,parent.roofAngle);	

	// bottom normal should be perpendicular to roof normal and have maximum z coord
	bottomNormal = osg::Z_AXIS - parent.roofNormal*(osg::Z_AXIS*roofNormal);
	bottomNormal.normalize();
	bottomPlane = osg::Plane(bottomNormal,p1);

	roofNormal = parent.roofNormal;
	roofPlane = parent.roofPlane;
	textureOrigo = parent.textureOrigo;
	uAxis = parent.uAxis;
	vAxis = parent.vAxis;
}

void RoofBuildStruct::init(const osg::Vec3& _p1, const osg::Vec3& _p2, float _roofAngle)
{
	roofAngle = _roofAngle;
	p1 = _p1;
	p2 = _p2;

	edgeDir = p2-p1;
	edgeDir.normalize();

	bottomNormal.x() = -edgeDir.y();
	bottomNormal.y() = edgeDir.x();
	bottomNormal.z() = 0;
	bottomNormal.normalize();

	float cosRoofAngle = cos(osg::DegreesToRadians(_roofAngle));
	float sinRoofAngle = sin(osg::DegreesToRadians(_roofAngle));

	bottomNormal *= cosRoofAngle;
	bottomNormal.z() = sinRoofAngle;

	bottomPlane = osg::Plane(bottomNormal,p1);

	roofNormal = edgeDir^bottomNormal;
	roofNormal.normalize();

	roofPlane = osg::Plane(roofNormal,p1);

	textureOrigo = p1;
	uAxis = bottomNormal;
	vAxis = edgeDir;
}

//Initiazes edge2 and edge1 of next RoofBuildStruct, resulting in all edges being initialized 
//after looping through all RoofBuildStructs once
void RoofBuildStruct::inializeRoofEdges(RoofBuildStruct& next)
{
	edgeDir2 = roofNormal^next.roofNormal;

	if(edgeDir2.length2()<EPSILON) // parallel roof vectors
	{
		edgeDir2 = bottomNormal;
	}

	edgeDir2.normalize();

	//check facing
	osg::Vec3 test = edgeDir^next.edgeDir;
	if(test.z() > 0) // inward corner
	{
		if((edgeDir.x()*edgeDir2.x() +  edgeDir.y()*edgeDir2.y())> 0.0f)
			edgeDir2 *= -1.0f;
	}
	else
	{
		if((edgeDir.x()*edgeDir2.x() +  edgeDir.y()*edgeDir2.y()) < 0.0f)
			edgeDir2 *= -1.0f;
	}

	//if(( edgeDir2.x() * bottomNormal.x() + edgeDir2.y() * bottomNormal.y() ) < 0.0f) // make sure edgedir points in direction of bottom normal
	//	edgeDir2 *= -1.0f;

	osg::Vec3 edge2Normal = roofNormal^edgeDir2; //points inward
	edge2Normal.normalize();
	plane2 = osg::Plane(edge2Normal,p2);

	next.edgeDir1 = edgeDir2;
	osg::Vec3 edge1Normal = next.edgeDir1^next.roofNormal;
	edge1Normal.normalize();
	next.plane1 = osg::Plane(edge1Normal,next.p1);
}

bool RoofBuildStruct::pointInsideRoofFace(const osg::Vec3& point)
{
	if(bottomPlane.distance(point) < -EPSILON)
		return false;
	if(plane1.distance(point) < -EPSILON)
		return false;
	if(plane2.distance(point) < -EPSILON)
		return false;
	return true;
}

bool RoofBuildStruct::rayIntersectsRoofPlane(const osg::Vec3& point, const osg::Vec3& dir, float& dist, osg::Vec3& intersection)
{
	assert(fabs(dir.length()-1.0f) < EPSILON);
	float d = roofPlane.distance(point);
	float d2 = roofPlane.distance(point + dir);
	float delta = d2-d;
	//check if ray points away from roof
	if(d >= -EPSILON && (d2-d) >= -EPSILON) 
		return false;
	else if(d <= EPSILON && (d2-d) <= EPSILON)
		return false;

	float dot = dir*roofPlane.getNormal();

	intersection = point - dir * d/dot;
	dist = -d/dot;

	return true;
}

bool RoofBuildStruct::rayIntersectsRoofFace(const osg::Vec3& point, const osg::Vec3& dir, float& dist, osg::Vec3& intersection)
{
	float roofPlaneDist;
	osg::Vec3 intersect;

	if(!rayIntersectsRoofPlane(point, dir, roofPlaneDist, intersect))
		return false;
	
	if(!pointInsideRoofFace(intersect))
		return false;

	dist = roofPlaneDist; 
	intersection = intersect;

	return true;
}



//RoofBuildStruct RoofBuildStruct::split(const osg::Vec3 splitPoint)
//{
//	// always base splitplane on plane2, since splitting always happens from splitpoints closest to plane 2
//	osg::Plane splitPlane = osg::Plane(plane2.getNormal(),splitPoint);
//
//	//calculate intersection with bottom edge
//	float distP1 = fabs(splitPlane.distance(p1));
//	float distP2 = fabs(splitPlane.distance(p2));
//	float distP1P2 = distP1+distP2;
//	osg::Vec3 intersection = p1 + (p2 - p1) * distP1/distP1P2;
//
//	RoofBuildStruct newRBS(intersection,p2,roofAngle);
//	newRBS.plane2 = plane2;
//	newRBS.edgeDir2 = edgeDir2;
//	newRBS.dist2 = dist2;
//	newRBS.edgeDir1 = edgeDir2;
//	newRBS.plane1 = splitPlane;
//	newRBS.plane1.flip();
//	newRBS.dist1 = (newRBS.p1-splitPoint).length();
//	newRBS.textureOrigo = textureOrigo;
//
//	p2 = intersection;
//	plane2 = splitPlane;
//	dist2 = newRBS.dist1;
//	
//	return newRBS;
//}

RoofBuildStruct RoofBuildStruct::split(const osg::Vec3 splitPoint)
{
	// always base splitplane on p2, since splitting always happens from splitpoints closest to plane 2
	osg::Vec3 splitEdgeDir = splitPoint-p2;
	splitEdgeDir.normalize();
	osg::Vec3 splitPlaneNormal = splitEdgeDir^roofNormal;
	osg::Plane splitPlane = osg::Plane(splitPlaneNormal,splitPoint);

	RoofBuildStruct newRBS(p2,p2+edgeDir,roofAngle);
	newRBS.p2 = p2;
	newRBS.plane2 = plane2;
	newRBS.edgeDir2 = edgeDir2;
	newRBS.dist2 = dist2;
	newRBS.edgeDir1 = splitEdgeDir;
	newRBS.plane1 = splitPlane;

	newRBS.dist1 = (newRBS.p1-splitPoint).length();
	newRBS.textureOrigo = textureOrigo;
	newRBS.vAxis = vAxis;
	newRBS.uAxis = uAxis;

	plane2 = splitPlane;
	plane2.flip();
	edgeDir2 = splitEdgeDir;
	dist2 = newRBS.dist1;

	return newRBS;
}

std::vector<RoofBuildStruct> RoofBuildStruct::doSplits()
{
	std::vector<RoofBuildStruct> splits;
	if(splitPoints.size() == 0)
		return splits;

	std::sort(splitPoints.begin(), splitPoints.end(),*this);

	osg::Vec3 prevPoint(FLT_MAX,FLT_MAX,FLT_MAX);
	int nPoints = splitPoints.size();
	
	for(int i=0;i<nPoints;i++)
	{
		if((prevPoint-splitPoints[i]).length2()>EPSILON)
		{
			splits.push_back(split(splitPoints[i]));
		}
		prevPoint = splitPoints[i];
	}
	std::reverse(splits.begin(),splits.end());
	return splits;
}


void RoofBuildStruct::checkSplit(int myIndex, std::vector<RoofBuildStruct>& roof)
{
	if(index1 < 0 || index2 < 0)
		return;

	// no split if this is a gable
	osg::Vec3 intersect[2] = { p1 + edgeDir1*dist1, p2 + edgeDir2*dist2};
	if((intersect[0]-intersect[1]).length2() < EPSILON)
		return;

	int indx[2]={index1,index2};
	osg::Vec3 p[2]={p1,p2};
	osg::Vec3 edgeDir[2]={edgeDir1,edgeDir2};
	float dist[2]={dist1,dist2};

	for(int i=0;i<2;i++)
	{
		float d1,d2,d3;
		osg::Vec3 intersect2;
		bool bColi2 = roof[myIndex].rayIntersectsRoofPlane(roof[indx[i]].p1, roof[indx[i]].edgeDir1,d2,intersect2);

		osg::Vec3 intersect3;
		bool bColi3 = roof[myIndex].rayIntersectsRoofPlane(roof[indx[i]].p2, roof[indx[i]].edgeDir2,d3,intersect3);

		osg::Vec3 roofTopDir = roof[myIndex].roofPlane.getNormal()^roof[indx[i]].roofPlane.getNormal();

		d1 = intersect[i]*roofTopDir;
		if(bColi2)
		{
			d2 = intersect2*roofTopDir;
		}
		else
		{
			if(roofTopDir*roof[indx[i]].edgeDir1 > 0)
				d2 = FLT_MAX;
			else
				d2 = FLT_MIN;
		}
		if(bColi3)
		{
			d3 = intersect3*roofTopDir;
		}
		else
		{
			if(roofTopDir*roof[indx[i]].edgeDir2 > 0)
				d3 = FLT_MAX;
			else
				d3 = FLT_MIN;
		}
		

		float dMin = d2<d3 ? d2:d3;
		float dMax = d2>d3 ? d2:d3;

		if( d1-dMin > EPSILON && dMax-d1 > EPSILON) // intersect1 is between intersect2 and 3, split 
			roof[indx[i]].addSplitPoint(intersect[i]);
	}
}

void RoofBuildStruct::generateGeometry(osg::Vec3Array* roofVerts, osg::Vec3Array* roofNormals, osg::Vec3Array* roofTexCoords, float texSpanY)
{
	if(dist1 == FLT_MAX || dist2 == FLT_MAX)
		return;
	osg::Vec3 roofPoints[4]={p1,p2,p2+edgeDir2*dist2, p1+edgeDir1*dist1};
	int nPoints = (roofPoints[3]-roofPoints[2]).length2() < EPSILON ? 3:4;
	for(int i=0;i<nPoints;i++)
	{
		roofVerts->push_back(roofPoints[i]);
		roofNormals->push_back(roofPlane.getNormal());

		osg::Vec3 dir = roofPoints[i]-textureOrigo;
		float u = (uAxis * dir)/texSpanY;
		float v = (vAxis * dir)/texSpanY;
		roofTexCoords->push_back(osg::Vec3(u,v,0));
	}
	
}


RoofBuilder::RoofBuilder(std::vector<osg::Vec3> outline, float roofAngle)
{
	std::vector<RoofBuildStruct> roof = initialize(outline,roofAngle);

	calculateRoofFaces(roof);

	roofParts.push_back(roof);

	std::vector<std::vector<roofEdge>> outlines = getOutlines(roof);

	while(outlines.size()>0)
	{
		std::vector<std::vector<roofEdge>> nextOutlines;

		for(unsigned int i=0;i<outlines.size();i++)
		{
			std::vector<RoofBuildStruct> newRoofPart = initialize(outlines[i],roof);

			calculateRoofFaces(newRoofPart);

			roofParts.push_back(newRoofPart);

			std::vector<std::vector<roofEdge>> newOutlines=getOutlines(newRoofPart);
			for(unsigned int j=0;j<newOutlines.size();j++)
				nextOutlines.push_back(newOutlines[j]);
		}
		outlines = nextOutlines;
	}
	

	int a=0;
}

std::vector<std::vector<roofEdge>> RoofBuilder::getOutlines(std::vector<RoofBuildStruct>& roof)
{
	std::vector<roofEdge> roofEdges;
	int numVerts = roof.size();
	for(int i=0;i<numVerts;i++)
	{
		osg::Vec3 p1Top = roof[i].p1+roof[i].edgeDir1*roof[i].dist1;
		osg::Vec3 p2Top = roof[i].p2+roof[i].edgeDir2*roof[i].dist2;
		if((p1Top-p2Top).length2()<EPSILON)
			continue;
		roofEdges.push_back(roofEdge(p1Top,p2Top,i));
	}

	//filter out mirrored edges
	int numEdges = roofEdges.size();
	for(int i=0; i<numEdges;i++)
	{
		if(roofEdges[i].bDelete)
			continue;
		for(int j=i+1;j<numEdges;j++)
		{
			if(roofEdges[j].bDelete)
				continue;
			bool bDelete = (((roofEdges[i].p1-roofEdges[j].p2).length2()<EPSILON) && ((roofEdges[i].p2-roofEdges[j].p1).length2()<EPSILON) );
			if(bDelete)
			{
				roofEdges[i].bDelete = true;
				roofEdges[j].bDelete = true;
				break;
			}
		}
	}

	std::vector<roofEdge> finalroofEdges;
	for(int i=0;i<numEdges;i++)
	{
		if(!roofEdges[i].bDelete)
			finalroofEdges.push_back(roofEdges[i]);
	}

	//find hole outlines
	std::vector<std::vector<roofEdge>> outlines;
	numEdges = finalroofEdges.size();
	for(int i=0;i<numEdges;i++)
	{
		if(!finalroofEdges[i].bDelete)
		{
			std::vector<roofEdge> outline;
			osg::Vec3 loopTarget = finalroofEdges[i].p1;
			osg::Vec3 next = finalroofEdges[i].p2;
			outline.push_back(finalroofEdges[i]);
			finalroofEdges[i].bDelete = true;
			for(int j=i+1;j<numEdges;j++)
			{
				if((finalroofEdges[j].p1-next).length2() < EPSILON)
				{
					outline.push_back(finalroofEdges[j]);
					finalroofEdges[j].bDelete = true;
					next = finalroofEdges[j].p2;
				}
				if((finalroofEdges[j].p2-loopTarget).length2() < EPSILON)
				{
					outlines.push_back(outline);
					break;
				}
			}
		}
	}

	return outlines;
}

std::vector<RoofBuildStruct> RoofBuilder::initialize(std::vector<osg::Vec3>& outline, float roofAngle)
{
	std::vector<RoofBuildStruct> roof;

	int numVerts = outline.size(); 
	for(int i=0; i<numVerts; i++)
	{
		int iNext = (i+1)%numVerts;
		roof.push_back(RoofBuildStruct(outline[i],outline[iNext],roofAngle));
	}

	for(int i=0; i<numVerts; i++)
	{
		int iNext = (i+1)%numVerts;
		roof[i].inializeRoofEdges(roof[iNext]);
	}

	return roof;
}

std::vector<RoofBuildStruct> RoofBuilder::initialize(std::vector<roofEdge>& outline, std::vector<RoofBuildStruct>& parentRoof)
{
	std::vector<RoofBuildStruct> roof;

	int numVerts = outline.size(); 
	for(int i=0; i<numVerts; i++)
	{
		roof.push_back(RoofBuildStruct(outline[i].p1,outline[i].p2,parentRoof[outline[i].indx]));
	}

	for(int i=0; i<numVerts; i++)
	{
		int iNext = (i+1)%numVerts;
		roof[i].inializeRoofEdges(roof[iNext]);
	}

	return roof;
}

void RoofBuilder::calculateRoofFaces(std::vector<RoofBuildStruct>& roof)
{
	int numFaces = roof.size();

	if(numFaces == 3)
	{
		for(int i=0;i<numFaces;i++)
		{
			osg::Vec3 intersect;
			roof[i].index1 = (i+1)%numFaces;
			roof[roof[i].index1].rayIntersectsRoofPlane(roof[i].p1, roof[i].edgeDir1, roof[i].dist1, intersect);
			roof[i].index2 = (i+2)%numFaces;
			roof[roof[i].index2].rayIntersectsRoofPlane(roof[i].p2, roof[i].edgeDir2, roof[i].dist2, intersect);
		}
	}
	else
	{
		for(int i=0;i<numFaces;i++)
		{
			for(int j=0;j<numFaces;j++)
			{
				if(i==j)
					continue;

				osg::Vec3 intersect;
				float dist;
				if(roof[i].rayIntersectsRoofFace(roof[j].p1,roof[j].edgeDir1, dist, intersect))
				{
					if(roof[j].dist1>dist)
					{
						roof[j].dist1 = dist;
						roof[j].index1 = i;
					}
				}

				if(roof[i].rayIntersectsRoofFace(roof[j].p2,roof[j].edgeDir2, dist, intersect))
				{
					if(roof[j].dist2>dist)
					{
						roof[j].dist2 = dist;
						roof[j].index2 = i;
					}
				}
			}
		}
	}
	

	for(int i=0; i<numFaces; i++)
	{
		roof[i].checkSplit(i,roof);
	}

	for(int i=0; i<numFaces; i++)
	{
		std::vector<RoofBuildStruct> splits = roof[i].doSplits();
		int numSplits = splits.size();
		if(numSplits>0)
		{
			roof.insert(roof.begin()+i+1,splits.begin(),splits.end());
			numFaces += numSplits;
			i += numSplits;
		}
	}
}

void RoofBuilder::generateRoofGeometry(osg::Geometry* roofGeometry, osg::Vec3Array*  roofVerts, osg::Vec3Array* roofNormals, osg::Vec3Array* roofTexcoords, float roofTexSpanY)
{
	int numRoofParts = roofParts.size();
	for(int j=0;j<numRoofParts;j++)
	{
		std::vector<RoofBuildStruct>& roof = roofParts[j];
		int numRoofQuads = roof.size();
		for(int i=0; i<numRoofQuads; i++)
		{ 
			int iStart = roofVerts->size();
			roof[i].generateGeometry(roofVerts, roofNormals, roofTexcoords, roofTexSpanY);
			int numVertsAdded = roofVerts->size() - iStart;

			if(numVertsAdded>=3)
			{
				osg::DrawElementsUInt* idx = new osg::DrawElementsUInt( GL_TRIANGLES );

				idx->push_back(iStart);
				idx->push_back(iStart+1);
				idx->push_back(iStart+2);
				if(numVertsAdded == 4)
				{
					idx->push_back(iStart);
					idx->push_back(iStart+2);
					idx->push_back(iStart+3);
				}
				roofGeometry->addPrimitiveSet( idx );
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////
//2d version
//////////////////////////////////////////////////////////////////////////

RoofBuildStruct2D::RoofBuildStruct2D(const osg::Vec3& _p1, const osg::Vec3& _p2)
{
	p1.x() = _p1.x(); p1.y() = _p1.y();
	p2.x() = _p2.x(); p2.y() = _p2.y();

	initEdgeAndNormal();
}

RoofBuildStruct2D::RoofBuildStruct2D(const osg::Vec2& _p1, const osg::Vec2& _p2)
{
	p1 = _p1;
	p2 = _p2;

	initEdgeAndNormal();
}

void RoofBuildStruct2D::initEdgeAndNormal()
{
	edgeDir = p2-p1;
	edgeLength = edgeDir.normalize();

	edgeNormal.x() = -edgeDir.y();
	edgeNormal.y() = edgeDir.x();
}

//Initializes edge2 and edge1 of next RoofBuildStruct, resulting in all edges being initialized 
//after looping through all RoofBuildStructs once
void RoofBuildStruct2D::inializeRoofEdges(RoofBuildStruct2D& next)
{
	edgeDir2 = (edgeNormal + next.edgeNormal);
	edgeDir2.normalize();
	edgeLength2 = 1000.0f;
	intersectIndex2 = -1;

	next.edgeDir1 = edgeDir2;
	next.edgeDir1.normalize();
	next.edgeLength1 = 1000.0f;
	next.intersectIndex1 = -1;
}


RoofBuilder2D::RoofBuilder2D(std::vector<osg::Vec3> outline, float roofAngle)
{
	std::vector<RoofBuildStruct2D> roof = initialize(outline);
	m_roofAngle = roofAngle;
	m_roofBaseZ = outline[0].z();
}

std::vector<RoofBuildStruct2D> RoofBuilder2D::initialize(std::vector<osg::Vec3>& outline){
	std::vector<RoofBuildStruct2D> roof;

	int numVerts = outline.size(); 
	for(int i=0; i<numVerts; i++)
	{
		int iNext = (i+1)%numVerts;
		roof.push_back(RoofBuildStruct2D(outline[i],outline[iNext]));
	}

	for(int i=0; i<numVerts; i++)
	{
		int iNext = (i+1)%numVerts;
		roof[i].inializeRoofEdges(roof[iNext]);
	}

	initializeRoofEdgeLengths(roof);
	
	return roof;
}

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

//float signed2DTriArea(osg::Vec2 a, osg::Vec2 b, osg::Vec2 c)
//{
//	return (a.x()-c.x())*(b.y()-c.y()) - (a.y()-c.y())*(b.x()-c.x());
//}
//
//bool test2DSegmentSegment(osg::Vec2 a, osg::Vec2 b, osg::Vec2 c, osg::Vec2 d, float& t, osg::Vec2& intersect)
//{
//	float a1 = signed2DTriArea(a,b,d);
//	float a2 = signed2DTriArea(a,b,c);
//	if(a1 * a2 <= 0.0f)
//	{
//		float a3 = signed2DTriArea(c,d,a);
//		float a4 = a3 + a2 - a1;
//		if(a3 * a4 <= 0.0f)
//		{
//			t = a3 / (a3 - a4);
//			intersect = a + (b - a) * t;
//			return true;
//		}
//	}
//	return false;
//}

// distance from roof edges should be approximately the same
bool validateIntersection(RoofBuildStruct2D& r1, RoofBuildStruct2D& r2, osg::Vec2& intersect)
{
	float d1 = (intersect-r1.p1)*r1.edgeNormal;
	float d2 = (intersect-r2.p1)*r2.edgeNormal;
	return fabs(d1-d2) < EPSILON;
}

void RoofBuilder2D::initializeRoofEdgeLengths(std::vector<RoofBuildStruct2D>& roof)
{
	int numEdges = roof.size();
	//run through all outline edges to establish maximum edge lengths
	for(int i=0; i<numEdges; i++ )
	{
		int iPrev = i-1 < 0 ? numEdges-1 : i-1;
		int iNext = (i+1)%numEdges;

		for(int j=0; j<numEdges;j++)
		{
			if(j==i) continue;

			float t;
			osg::Vec2 intersect;

			if(j!=iNext && test2DSegmentSegment(roof[i].p2, roof[i].edgeDir2 ,roof[i].edgeLength2, roof[j].p1, roof[j].edgeDir, roof[j].edgeLength , t, intersect ))
			{
				roof[i].edgeLength2 = (t * roof[i].edgeLength2);
				roof[i].tempLength2 = roof[i].edgeLength2;
				roof[i].intersectIndex2 = j;
				roof[iNext].edgeLength1 = roof[i].edgeLength2;
				roof[iNext].tempLength1 = roof[i].edgeLength2;
				roof[iNext].intersectIndex1 = j;
			}
		}
	}

	for(int i=0; i<numEdges; i++ )
	{
		int iNext = (i+1)%numEdges;
		osg::Vec2 n1 = roof[roof[i].intersectIndex2].edgeNormal;
		osg::Vec2 n2 = roof[i].edgeNormal;
		float t = (roof[i].p2 * n1 - roof[roof[i].intersectIndex2].p2 * n1) / (roof[i].edgeDir2 *(n2-n1));
		
		roof[i].edgeLength2 = t;
		roof[i].tempLength2 = t;
		roof[iNext].edgeLength1 = t;
		roof[iNext].tempLength1 = t;
	}

	// check roof edges against each other
	for(int i=0; i<numEdges; i++ )
	{
		int iPrev = i-1 < 0 ? numEdges-1 : i-1;
		int iNext = (i+1)%numEdges;

		for(int j=0; j<numEdges;j++)
		{
			if(j==i) continue;

			float t;
			osg::Vec2 intersect;

			if(test2DSegmentSegment(roof[i].p2, roof[i].edgeDir2 ,roof[i].edgeLength2, roof[j].p2, roof[j].edgeDir2, roof[j].edgeLength2, t, intersect ))
			{
				if(validateIntersection(roof[i], roof[j], intersect))
				{
					float length = (t * roof[i].edgeLength2);
					if(length < roof[i].tempLength2+EPSILON)
					{
						roof[i].tempLength2 = length;
						roof[i].intersectIndex2 = -1;
						roof[iNext].tempLength1 = length;
						roof[iNext].intersectIndex1 = -1;
					}
				}
			}
		}
	}

	for(int i=0; i<numEdges; i++ )
	{
		roof[i].edgeLength1 = roof[i].tempLength1;
		roof[i].edgeLength2 = roof[i].tempLength2;
	}
}

