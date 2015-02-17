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

	initializeRoofFaces(outline);
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

//////////////////////////////////////////////////////////////////////////
// faceEdge approach
//////////////////////////////////////////////////////////////////////////

void RoofFaceEdgeList::clone(const RoofFaceEdgeList& other)
{
	roofFace			= other.roofFace;
	neighborEdge		= other.neighborEdge;
	currentEdgeDir		= other.currentEdgeDir;
	currentEdgeLength	= other.currentEdgeLength;
	nextEventTime		= other.nextEventTime;
	evtType				= other.evtType;
	splitFace			= other.splitFace;
	splitPoint			= other.splitPoint;
	bIsLeftEdge			= other.bIsLeftEdge;

	edgePoints.insert(edgePoints.begin(), other.edgePoints.begin(), other.edgePoints.end());
}

void RoofFaceEdgeList::initialize(struct RoofFace* ownerFace, struct RoofFace* _neighborFace)
{
	roofFace			= ownerFace;
	bIsLeftEdge			= (ownerFace->p1 == _neighborFace->p2); 

	neighborEdge		= bIsLeftEdge ? &(_neighborFace->getP2EdgeList()) : &(_neighborFace->getP1EdgeList());

	if(bIsLeftEdge)
		edgePoints.push_back(ownerFace->p1);
	else
		edgePoints.push_back(ownerFace->p2);

	currentEdgeDir		= osg::Vec2(0,0);
	currentEdgeDir		= calculateEdgeDir(_neighborFace);
	currentEdgeLength	= defaultLength;
	nextEventTime		= defaultLength;
	evtType				= NA;

}

void RoofFaceEdgeList::getCurrentEdge(osg::Vec2& start, osg::Vec2& dir, float& length)
{
	start	= edgePoints[edgePoints.size()-1];
	dir		= currentEdgeDir;
	length	= currentEdgeLength;
}

void RoofFaceEdgeList::setCurrentEdge(const osg::Vec2& start, const osg::Vec2& dir, float length)
{
	edgePoints[edgePoints.size()-1] = start;
	currentEdgeDir		= dir;
	currentEdgeLength	= length;
	float prevEventTime = nextEventTime;
	nextEventTime		= roofFace->edgeNormal * ((start + dir*currentEdgeLength) - roofFace->p1);

	if(nextEventTime < prevEventTime)
		int a=0;
}

void RoofFaceEdgeList::advanceCurrentEdge(const osg::Vec2& start, const osg::Vec2& dir, float length)
{
	edgePoints.push_back(start);
	currentEdgeDir		= dir;
	currentEdgeLength	= length;
	float prevEventTime = nextEventTime;
	nextEventTime		= roofFace->edgeNormal * ((start + dir*currentEdgeLength) - roofFace->p1);
	
	if(nextEventTime < prevEventTime)
		int a=0;
}

osg::Vec2 RoofFaceEdgeList::calculateEdgeDir(RoofFace* neighborFace)
{
	// calculate edgeDir based on roofFace & neighbor roofFace edgeNormals
	osg::Vec2 dir;
	osg::Vec3 testNormal(roofFace->edgeNormal.x(),roofFace->edgeNormal.y(),1.0f);
	osg::Vec3 testNormal2(neighborFace->edgeNormal.x(),neighborFace->edgeNormal.y(),1.0f);
	osg::Vec3 testDir = testNormal^testNormal2;
	testDir.z() = 0.0f;
	float normalLength = testDir.normalize();
	if(normalLength < EPSILON) // parallel normals
	{
		dir.x() = -roofFace->edgeNormal.y();
		dir.y() = roofFace->edgeNormal.x();
	}
	else
	{
		dir.x() = testDir.x();
		dir.y() = testDir.y();
	}

	osg::Vec2 prevEdgeDir = bIsLeftEdge ? roofFace->p1-roofFace->p2 : roofFace->p2-roofFace->p1;
	if(currentEdgeDir.length2()>EPSILON)
	{
		prevEdgeDir = currentEdgeDir;
	}
	
	float edgeCrossZ = prevEdgeDir.x()*dir.y() - prevEdgeDir.y()*dir.x(); 

	if( ( bIsLeftEdge && edgeCrossZ > 0.0f) || (!bIsLeftEdge && edgeCrossZ < 0.0f))
	{
		dir *= -1.0f;
	} 
	

	return dir;
}

void RoofFaceEdgeList::splitEvent()
{
	if(evtType != SPLIT)
		return;

	osg::Vec2 splitDir = roofFace->edgeNormal + splitFace->edgeNormal;
	if(splitDir.length2() < EPSILON) // parallel normals
	{
		splitDir.x() = -roofFace->edgeNormal.y();
		splitDir.y() = roofFace->edgeNormal.x();
	}
	splitDir.normalize();
	osg::Vec2 newStart = edgePoints[edgePoints.size()-1] + currentEdgeDir*currentEdgeLength;
	if(bIsLeftEdge) // p1 edge
	{
		//splitDir should point in edge direction
		if(splitDir*roofFace->edgeDir < 0)
			splitDir *= -1.0f;
	} 
	else //p2 edge
	{	
		//splitDir should point in opposite direction of edge direction
		if(splitDir*roofFace->edgeDir > 0)
			splitDir *= -1.0f;	
	}
	advanceCurrentEdge(newStart,splitDir, defaultLength);
	evtType = NA;
	neighborEdge->splitEvent();
}

void RoofFaceEdgeList::vertexEvent()
{
	//close face and update neighbor edges
	roofFace->bDone = true;
	
	RoofFaceEdgeList* p1Neighbor = NULL;
	RoofFaceEdgeList* p2Neighbor = NULL;

	if(bIsLeftEdge) // p1 edge
	{
		p1Neighbor = neighborEdge;
		p2Neighbor = roofFace->getP2EdgeList().neighborEdge;
	}
	else //p2 edge
	{
		p1Neighbor = roofFace->getP1EdgeList().neighborEdge;
		p2Neighbor = neighborEdge;	
	}

	p1Neighbor->neighborEdge = p2Neighbor;
	p2Neighbor->neighborEdge = p1Neighbor;


	osg::Vec2 edgeDir = p2Neighbor->calculateEdgeDir(p1Neighbor->roofFace);
	osg::Vec2 vStart,vDummy;
	float l;
	getCurrentEdge(vStart,vDummy,l);
	vStart = vStart + vDummy*l;
	if(p1Neighbor->evtType != RoofFaceEdgeList::VERTEX)	
		p1Neighbor->advanceCurrentEdge(vStart,edgeDir,defaultLength);
	if(p2Neighbor->evtType != RoofFaceEdgeList::VERTEX)
		p2Neighbor->advanceCurrentEdge(vStart,edgeDir,defaultLength);
}

RoofFace::RoofFace(unsigned int index, const osg::Vec2& _p1, const osg::Vec2& _p2):
faceIndex(index),
p1(_p1),
p2(_p2),
bDone(false)
{
	initEdgeData();
}

void RoofFace::initEdgeData()
{
	edgeNormal.x() = -(p2.y()-p1.y());
	edgeNormal.y() =  (p2.x()-p1.x());
	edgeNormal.normalize();

	edgeDir = p2-p1;
	edgeLength = edgeDir.normalize();
}

void RoofFace::initializeEdgeLists(RoofFace* pPrevface, RoofFace* pNextFace)
{
	p1Edge.initialize(this, pPrevface);	
	p2Edge.initialize(this, pNextFace);
}

void RoofFace::splitFace(std::vector<RoofFace>& roofFaces, RoofFaceEdgeList& splitEdge)
{
	osg::Vec2 splitPoint = splitEdge.splitPoint;
	RoofFace newFaceOrg(roofFaces.size(),splitPoint, p2);
	roofFaces.push_back(newFaceOrg);

	RoofFace& newFace = roofFaces[roofFaces.size()-1];

	p2 = splitPoint;
	initEdgeData();

	newFace.p2Edge.clone(p2Edge);
	newFace.getP1EdgeList().bIsLeftEdge = true;

	newFace.getP1EdgeList().roofFace = &newFace;
	newFace.getP2EdgeList().roofFace = &newFace;
	newFace.getP1EdgeList().neighborEdge = &p2Edge;

	//update neighbor edges neighbor
	p2Edge.neighborEdge->neighborEdge = &(newFace.getP2EdgeList());
	p2Edge.neighborEdge = &(newFace.getP1EdgeList());

	// reset p2Edge, and newfaces p1Edge
	p2Edge.edgePoints.clear();

	osg::Vec2 start,dir;
	float length;
	splitEdge.getCurrentEdge(start,dir,length);

	osg::Vec2 edgePoint = start + dir * length;

	dir = edgePoint-splitPoint;
	length = dir.normalize();
	p2Edge.advanceCurrentEdge(splitPoint, dir, length);
	newFace.getP1EdgeList().advanceCurrentEdge(splitPoint, dir, length);

	//update newFace p1 edge to reflect split
	osg::Vec3 testNormal(edgeNormal.x(),edgeNormal.y(),1.0f);
	osg::Vec3 testNormal2(splitEdge.roofFace->edgeNormal.x(),splitEdge.roofFace->edgeNormal.y(),1.0f);
	osg::Vec3 testDir = testNormal^testNormal2;
	testDir.z() = 0.0f;
	float normalLength = testDir.normalize();
	if(normalLength < EPSILON) // parallel normals
	{
		dir.x() = -edgeNormal.y();
		dir.y() = edgeNormal.x();
	}
	else
	{
		dir.x() = testDir.x();
		dir.y() = testDir.y();
	}
	// dir should point in same direction of edgedir
	if(dir * edgeDir < 0.0f)
		dir *= -1.0f;

	newFace.getP1EdgeList().advanceCurrentEdge(edgePoint, dir, defaultLength);
	splitEdge.advanceCurrentEdge(edgePoint, dir, defaultLength);

	//update this face's p2 edge to reflect split
	testNormal = osg::Vec3 (edgeNormal.x(),edgeNormal.y(),1.0f);
	testNormal2 = osg::Vec3 (splitEdge.neighborEdge->roofFace->edgeNormal.x(),splitEdge.neighborEdge->roofFace->edgeNormal.y(),1.0f);
	testDir = testNormal^testNormal2;
	testDir.z() = 0.0f;
	normalLength = testDir.normalize();
	if(normalLength < EPSILON) // parallel normals
	{
		dir.x() = -edgeNormal.y();
		dir.y() = edgeNormal.x();
	}
	else
	{
		dir.x() = testDir.x();
		dir.y() = testDir.y();
	}
	// dir should point in opposite direction of edgedir
	if(dir * edgeDir > 0.0f)
		dir *= -1.0f;

	p2Edge.advanceCurrentEdge(edgePoint, dir, defaultLength);
	splitEdge.neighborEdge->advanceCurrentEdge(edgePoint, dir, defaultLength);

	splitEdge.evtType = RoofFaceEdgeList::NA;
	splitEdge.neighborEdge->evtType = RoofFaceEdgeList::NA;

	p2Edge.evtType = RoofFaceEdgeList::NA;
	newFace.getP1EdgeList().evtType = RoofFaceEdgeList::NA;

	RoofFaceEdgeList* oldNeighbor = splitEdge.neighborEdge;

	splitEdge.neighborEdge = &(newFace.getP1EdgeList());
	newFace.getP1EdgeList().neighborEdge = &splitEdge;

	oldNeighbor->neighborEdge = &p2Edge;
	p2Edge.neighborEdge = oldNeighbor;
}

bool RoofFace::clipEdges()
{
	osg::Vec2 p1Start, p1Dir;
	float p1Length;

	p1Edge.getCurrentEdge(p1Start,p1Dir,p1Length);

	osg::Vec2 p2Start, p2Dir;
	float p2Length;

	p2Edge.getCurrentEdge(p2Start,p2Dir, p2Length);

	float t;
	osg::Vec2 intersect;
	if(test2DSegmentSegment(p1Start, p1Dir, p1Length, p2Start, p2Dir, p2Length, t, intersect))
	{
		float length1 = (p1Start-intersect).length();
		p1Edge.setCurrentEdge(p1Start,p1Dir,length1);
		float length2 = (p2Start-intersect).length();
		p2Edge.setCurrentEdge(p2Start,p2Dir,length2);
		
		p1Edge.evtType = RoofFaceEdgeList::VERTEX;
		p2Edge.evtType = RoofFaceEdgeList::VERTEX;

		if(length1 <= p1Edge.neighborEdge->currentEdgeLength)
		{
			p1Edge.neighborEdge->setCurrentEdge(p1Start,p1Dir,length1);
			p1Edge.neighborEdge->evtType = RoofFaceEdgeList::NA;
		}
		if(length2 <= p2Edge.neighborEdge->currentEdgeLength)
		{
			p2Edge.neighborEdge->setCurrentEdge(p2Start,p2Dir,length2);
			p2Edge.neighborEdge->evtType = RoofFaceEdgeList::NA;
		}

		return true;
	}
	return false;
}


void RoofFace::generateRoofFace()
{
	//verify that edges end in same point
	osg::Vec2 start1,dir1;
	float length1;
	p1Edge.getCurrentEdge(start1,dir1,length1);
	
	osg::Vec2 start2,dir2;
	float length2;
	p2Edge.getCurrentEdge(start2,dir2,length2);

	osg::Vec2 edge1End = start1 + dir1 * length1;
	osg::Vec2 edge2End = start2 + dir2 * length2;

	assert(bDone && (edge1End-edge2End).length2()< EPSILON);
	
	faceOutline.push_back(p1);
	int numPoint = p2Edge.edgePoints.size();
	for(int i=0; i<numPoint; i++)
	{
		faceOutline.push_back(p2Edge.edgePoints[i]);
	}

	faceOutline.push_back(edge2End);

	numPoint = p1Edge.edgePoints.size();
	for(int i=numPoint-1; i >= 1; i--)  // skip first point since it's equal to p1, and was already added
	{
		faceOutline.push_back(p1Edge.edgePoints[i]);
	}
}

void RoofBuilder2D::clipEdgeToOutline(RoofFaceEdgeList& edge)
{
	osg::Vec2 edgeStart, edgeDir;
	float edgeLength;

	edge.getCurrentEdge(edgeStart, edgeDir, edgeLength);

	int numEdges = roofFaces.size();
	for(int i=0; i<numEdges; i++ )
	{
		RoofFace& currentFace = roofFaces[i];
		// don't clip with segment containing edge start point
		if(edgeStart==currentFace.p1 || edgeStart==currentFace.p2)
			continue;	
		float t;
		osg::Vec2 intersect;
		if(test2DSegmentSegment(edgeStart, edgeDir, edgeLength, currentFace.p1, currentFace.edgeDir, currentFace.edgeLength, t, intersect))
		{
			//reduce length to max possible between these 2 edges
			//it's the point on the edge, with the same projection on the 2 face edgeNormals
			osg::Vec2 n1 = currentFace.edgeNormal;
			osg::Vec2 n2 = edge.roofFace->edgeNormal;
			t = (edgeStart * n1 - currentFace.p2 * n1) / (edgeDir *(n2-n1));

			edgeLength = t;
			edge.evtType = RoofFaceEdgeList::SPLIT;
			edge.splitPoint = intersect;
			edge.splitFace = &(currentFace);
		}
	}
	edge.setCurrentEdge(edgeStart, edgeDir, edgeLength);
}

RoofFaceEdgeList* RoofBuilder2D::getMinEdge()
{
	float minEventTime = defaultLength;
	float minLength = FLT_MAX;
	RoofFaceEdgeList* pEdge=NULL;
	int numFaces = roofFaces.size();
	for(int i=0; i<numFaces;i++)
	{
		if(roofFaces[i].bDone)
			continue;

		float nextEvtTime;
		if(roofFaces[i].getP1EdgeList().evtType != RoofFaceEdgeList::NA)
		{
			nextEvtTime = roofFaces[i].getP1EdgeList().nextEventTime;
			if(nextEvtTime < minEventTime)
			{
				minEventTime = nextEvtTime;
				pEdge = &(roofFaces[i].getP1EdgeList());
				minLength = pEdge->currentEdgeLength;
			} 
			else if(nextEvtTime < minEventTime+EPSILON && minLength > roofFaces[i].getP1EdgeList().currentEdgeLength)
			{
				pEdge = &(roofFaces[i].getP1EdgeList());
				minLength = pEdge->currentEdgeLength;
			}
		}
		

		if(roofFaces[i].getP2EdgeList().evtType != RoofFaceEdgeList::NA)
		{
			nextEvtTime = roofFaces[i].getP2EdgeList().nextEventTime;
			if(nextEvtTime < minEventTime)
			{
				minEventTime = nextEvtTime;
				pEdge = &(roofFaces[i].getP2EdgeList());
			}
			else if(nextEvtTime < minEventTime+EPSILON && minLength > roofFaces[i].getP2EdgeList().currentEdgeLength)
			{
				pEdge = &(roofFaces[i].getP2EdgeList());
				minLength = pEdge->currentEdgeLength;
			}
		}
	}
	return pEdge;
}

void RoofBuilder2D::clipEdges()
{
	int numPoints = roofFaces.size();
	for(int i=0; i<numPoints; i++)
	{
		roofFaces[i].clipEdges();
	}
}

void RoofBuilder2D::initializeRoofFaces(std::vector<osg::Vec3>& outline)
{
	int numPoints = outline.size();
	std::vector<osg::Vec2> outline2D;
	for(int i=0;i<numPoints;i++)
	{
		osg::Vec2 p(outline[i].x(), outline[i].y());
		outline2D.push_back(p);
	}

	//test
	outline2D.clear();
	
	/*outline2D.push_back(osg::Vec2(0,0));
	outline2D.push_back(osg::Vec2(10,0));
	outline2D.push_back(osg::Vec2(10,10));
	outline2D.push_back(osg::Vec2(0,10));*/

	/*outline2D.push_back(osg::Vec2(0,0));
	outline2D.push_back(osg::Vec2(10,0));
	outline2D.push_back(osg::Vec2(10,10));
	outline2D.push_back(osg::Vec2(5,2));
	outline2D.push_back(osg::Vec2(0,10));*/

	outline2D.push_back(osg::Vec2(0,0));
	outline2D.push_back(osg::Vec2(7,0));
	outline2D.push_back(osg::Vec2(7,9));
	outline2D.push_back(osg::Vec2(18,9));
	outline2D.push_back(osg::Vec2(18,17));
	outline2D.push_back(osg::Vec2(0,17));

	numPoints = outline2D.size();

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

	for(int i=0; i<numPoints; i++)
	{
		int iPrev = i==0 ? numPoints-1:i-1;
		int iNext = (i+1)%numPoints;
		
		roofFaces[i].initializeEdgeLists(&(roofFaces[iPrev]), &(roofFaces[iNext]));
	}

	for(int i=0; i<numPoints; i++)
	{
		clipEdgeToOutline(roofFaces[i].getP2EdgeList());
		int iNext = (i+1)%numPoints;
		osg::Vec2 start,dir;
		float length;
		roofFaces[i].getP2EdgeList().getCurrentEdge(start,dir,length);
		roofFaces[iNext].getP1EdgeList().setCurrentEdge(start,dir,length);
		roofFaces[iNext].getP1EdgeList().evtType = roofFaces[i].getP2EdgeList().evtType;
		roofFaces[iNext].getP1EdgeList().splitPoint = roofFaces[i].getP2EdgeList().splitPoint;
		roofFaces[iNext].getP1EdgeList().splitFace = roofFaces[i].getP2EdgeList().splitFace;
	}

	clipEdges();
	
	RoofFaceEdgeList* pEdge= getMinEdge();
	while(pEdge)
	{
		if(pEdge->evtType == RoofFaceEdgeList::SPLIT)
		{
			RoofFaceEdgeList* pOldNeighbor = pEdge->neighborEdge;
			pEdge->splitFace->splitFace(roofFaces,*pEdge);

			clipEdgeToOutline(*pEdge);
			clipEdgeToOutline(*(pEdge->neighborEdge));
			clipEdgeToOutline(*pOldNeighbor);
			clipEdgeToOutline(*(pOldNeighbor->neighborEdge));

			clipEdges();
		}
		else if(pEdge->evtType == RoofFaceEdgeList::VERTEX)
		{
			pEdge->vertexEvent();
			clipEdgeToOutline(*(pEdge->roofFace->p1Edge.neighborEdge));
			clipEdgeToOutline(*(pEdge->roofFace->p2Edge.neighborEdge));
			clipEdges();
		}

		pEdge= getMinEdge();
	}
	
	numPoints = roofFaces.size();

	for(int i=0; i<numPoints; i++)
	{
		roofFaces[i].generateRoofFace();
	}

	int a=0;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
RoofFaceEdge::RoofFaceEdge(unsigned int _edgeIndex, std::vector<RoofFace2>& roofFaces, unsigned int _leftFaceIndex, unsigned int _rightFaceIndex):
edgeIndex(_edgeIndex),
leftFaceIndex(_leftFaceIndex),
rightFaceIndex(_rightFaceIndex),
bActive(true)
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

		osg::Vec2 testDir = vStart - roofFaces[leftFaceIndex].rightEdge[0];
		osg::Vec2 testNormal(-testDir.y(),testDir.x());

		if((vDir*testNormal) < 0.0f)
			vDir *= -1.0;
	}

	length = defaultLength;
}

osg::Vec2 RoofFaceEdge::getEndPoint()
{
	return vStart + vDir*length;
}

void RoofFaceEdge::clipEdgeToOutline(std::vector<RoofFace2>& roofFaces)
{
	int numFaces = roofFaces.size();
	for(int i=0;i<numFaces;i++)
	{
		if(i==leftFaceIndex || i==rightFaceIndex)
			continue;
		RoofFace2& currentFace = roofFaces[i];
		float t;
		osg::Vec2 intersect;
		if(test2DSegmentSegment(vStart, vDir, length, currentFace.p1, currentFace.edgeDir, currentFace.edgeLength, t, intersect))
		{
			//reduce length to max possible between these 2 edges
			//it's the point on the edge, with the same projection on the 2 face edgeNormals
			osg::Vec2 n1 = currentFace.edgeNormal;
			osg::Vec2 n2 = roofFaces[leftFaceIndex].edgeNormal;
			t = (vStart * n1 - currentFace.p2 * n1) / (vDir *(n2-n1));

			length = t;

			evt.evtType = SkeletonEvent::SPLIT;
			evt.intersection = intersect;
			evt.time = currentFace.edgeNormal * (getEndPoint() - currentFace.p1);
			evt.faceIndex = i;
		}
	}
}

void RoofFaceEdge::clipToEdge(std::vector<RoofFace2>& roofFaces, RoofFaceEdge& edge)
{
	float t;
	osg::Vec2 intersect;
	if(test2DSegmentSegment(vStart, vDir, length, edge.vStart, edge.vDir, edge.length, t, intersect))
	{
		length = t*length;
		edge.length = (intersect-edge.vStart).length();

		evt.evtType = SkeletonEvent::EDGE;
		edge.evt.evtType = SkeletonEvent::EDGE;

		evt.intersection = intersect;
		edge.evt.intersection = intersect;

		evt.time = roofFaces[leftFaceIndex].edgeNormal * (getEndPoint() - roofFaces[leftFaceIndex].p1);
		edge.evt.time = evt.time;

		evt.edgeIndex = edge.edgeIndex;
		edge.evt.edgeIndex = edgeIndex;


	}
}

RoofFace2::RoofFace2(unsigned int index, const osg::Vec2& _p1, const osg::Vec2& _p2):
faceIndex(index),
p1(_p1),
p2(_p2)
{
	leftEdge.push_back(p1);
	rightEdge.push_back(p2);
	initEdgeData();
}

void RoofFace2::initEdgeData()
{
	edgeNormal.x() = -(p2.y()-p1.y());
	edgeNormal.y() =  (p2.x()-p1.x());
	edgeNormal.normalize();

	edgeDir = p2-p1;
	edgeLength = edgeDir.normalize();
}

RoofFace2& RoofFace2::splitFace(std::vector<RoofFace2>& roofFaces, osg::Vec2 splitPoint)
{
	roofFaces.push_back(RoofFace2(roofFaces.size(),splitPoint, p2));
	RoofFace2& newFace = roofFaces[roofFaces.size()-1];

	newFace.rightEdge.clear();
	newFace.rightEdge.insert(newFace.rightEdge.begin(), rightEdge.begin(), rightEdge.end());
	
	rightEdge.clear();
	rightEdge.push_back(splitPoint);
	p2 = splitPoint;
	initEdgeData();

	return newFace;
}

osg::Vec2 RoofFace2::getLeftEndPoint()
{
	return leftEdge[leftEdge.size()-1];
}

osg::Vec2 RoofFace2::getRightEndPoint()
{
	return rightEdge[rightEdge.size()-1];
}

RoofBuilder2DNew::RoofBuilder2DNew(std::vector<osg::Vec3> outline, float roofAngle)
{
	int numPoints = outline.size();
	std::vector<osg::Vec2> outline2D;
	for(int i=0;i<numPoints;i++)
	{
		osg::Vec2 p(outline[i].x(), outline[i].y());
		outline2D.push_back(p);
	}

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
		roofFaces.push_back(RoofFace2(i,outline2D[i], outline2D[iNext]));

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
	}
}

void RoofBuilder2DNew::clipEdgesToOutline()
{
	int numEdges = faceEdges.size();
	for(int i=0;i<numEdges;i++)
	{
		if(!faceEdges[i].bActive)
			continue;

		faceEdges[i].clipEdgeToOutline(roofFaces);
	}
}

void RoofBuilder2DNew::clipEdgesToEdges()
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

int RoofBuilder2DNew::getMinEdgeIndex()
{
	float minTime = FLT_MAX;
	int numEdges = faceEdges.size();
	int minEdgeIndex = -1;
	for(int i=0; i<numEdges; i++)
	{
		if(!faceEdges[i].bActive)
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

void RoofBuilder2DNew::edgeEvent(RoofFaceEdge& edge)
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
	if((endPoint-leftEndPoint).length2() > EPSILON && (endPoint-rightEndPoint).length2() > EPSILON  )
		roofFaces[commonFaceIndex].leftEdge.push_back(endPoint);
	
	
	rightEndPoint = roofFaces[leftNeighborFace].getRightEndPoint();
	if((endPoint-rightEndPoint).length2() > EPSILON)
		roofFaces[leftNeighborFace].rightEdge.push_back(endPoint);

	leftEndPoint = roofFaces[rightNeighborFace].getLeftEndPoint();
	if((endPoint-leftEndPoint).length2() > EPSILON )
		roofFaces[rightNeighborFace].leftEdge.push_back(endPoint);

	// create new edge with left and right neighbor faces
	faceEdges.push_back(RoofFaceEdge(faceEdges.size(),roofFaces,leftNeighborFace,rightNeighborFace));

	clipEdgesToOutline();
	clipEdgesToEdges();
}

void RoofBuilder2DNew::splitEvent(RoofFaceEdge& edge)
{
	edge.Disable();

	osg::Vec2 intersectPoint = edge.evt.intersection;
	osg::Vec2 splitPoint = edge.getEndPoint();

	roofFaces[edge.leftFaceIndex].rightEdge.push_back(splitPoint);
	roofFaces[edge.rightFaceIndex].leftEdge.push_back(splitPoint);
	
	RoofFace2& faceToSplit = roofFaces[edge.evt.faceIndex];
	RoofFace2& splitFace = faceToSplit.splitFace(roofFaces, intersectPoint);
	splitFace.leftEdge.push_back(splitPoint);
	faceToSplit.rightEdge.push_back(splitPoint);

	// create new split edges
	faceEdges.push_back(RoofFaceEdge(faceEdges.size(), roofFaces, edge.leftFaceIndex, splitFace.faceIndex));
	faceEdges.push_back(RoofFaceEdge(faceEdges.size(), roofFaces, faceToSplit.faceIndex, edge.rightFaceIndex));

	clipEdgesToOutline();
	clipEdgesToEdges();
}