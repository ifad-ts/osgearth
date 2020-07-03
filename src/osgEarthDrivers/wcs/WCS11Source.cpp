/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "WCS11Source.h"
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osg/Notify>
#include <osgDB/Registry>
#include <iostream>
#include <stdlib.h>

#define LC "[osgEarth::WCS11] "

using namespace osgEarth;


WCS11Source::WCS11Source( const TileSourceOptions& options ) :
TileSource( options ),
_options  ( options )
{
    _covFormat = _options.format().value();
    
    if ( _covFormat.empty() )
        _covFormat = "image/GeoTIFF";

    _osgFormat = "tif";
}



osgEarth::Status WCS11Source::initialize(const osgDB::Options* dbOptions)
{        
    osg::ref_ptr<const Profile> profile;
    
    _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );

    std::string capUrl;

    if (_options.url().isSet())
    {
        char sep = _options.url()->full().find_first_of('?') == std::string::npos ? '?' : '&';

        capUrl =
            _options.url()->full() +
            sep +
            "SERVICE=WCS&VERSION=1.1.0&REQUEST=GetCapabilities";
    }

    _capabilities = WCSCapabilitiesReader::read(capUrl, _dbOptions.get());
    if (!_capabilities.valid())
    {
        OE_WARN << LC << "Unable to read WCS GetCapabilities." << std::endl;
        //return;
    }
    else
    {
        OE_INFO << LC << "Got capabilities from " << capUrl << std::endl;
    }

    _identifier = _options.identifier().value();

    // Next, try to glean the extents from the coverage list
    if (_capabilities.valid())
    {
        WCSCoverage* coverage;
        if (_options.title().isSet())
        {
            if (_options.identifier().isSet())
            {
                OE_NOTICE << LC << "Both identifier and title specified - using title \"" << _options.title().value() << "\"" << std::endl;
            }
            
            // try to get the coverage by title instead of by identifier
            coverage = _capabilities->getCoverageByTitle(_options.title().value());
            if (coverage)
            {
                _identifier = coverage->getIdentifier();
                OE_INFO << LC << "Matched title \"" << _options.title().value() << "\" to coverage \"" << coverage->getIdentifier() << "\"" << std::endl;
            }
            else
            {
                OE_WARN << LC << "Could not find coverage with title \"" << _options.title().value() << "\"" << std::endl;
            }
        }
        else
        {
            coverage = _capabilities->getCoverageByIdentifier(_options.identifier().value());
        }
        if (!coverage)
        {
            OE_WARN << LC << "Using global extents since no coverage with the following identifier could be found: " << _identifier << std::endl;
        }
        else
        {
			profile = createProfileFromCapabilities(coverage);
        }
    }

    // Last resort: create a global extent profile (only valid for global maps)
    if (!profile.valid())
    {
        profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    }
    setProfile(profile);

    return STATUS_OK;
}

const Profile *WCS11Source::createProfileFromCapabilities(WCSCoverage* coverage)
{
	const SpatialReference* srs = coverage->getExtent().getSRS();
	double xmin, ymin, xmax, ymax;
	coverage->getExtent().getBounds(xmin, ymin, xmax, ymax);
	const Profile *profile = Profile::create(srs, xmin, ymin, xmax, ymax);

	unsigned int maxDataLevel = 25;
	if (_options.maxDataLevelOverride().isSet())
	{
		maxDataLevel = _options.maxDataLevelOverride().value();
		OE_INFO << LC << _options.url().value().full() << " using override max data level " << maxDataLevel << std::endl;
	}
	else if (_options.maxResolution().isSet())
	{
		double maxResolution = _options.maxResolution().value(); //get this from capabilities later (and remove maxResolution option)
		for (unsigned int i = 0; i < 30; ++i)
		{
			maxDataLevel = i;
			double w, h;
			profile->getTileDimensions(i, w, h);
			double resX = (w / (double)getTileSize());
			double resY = (h / (double)getTileSize());

			if (resX < maxResolution || resY < maxResolution)
			{
				break;
			}
		}

		OE_INFO << LC << _options.url().value().full() << " max Data Level: " << maxDataLevel << std::endl;
	}
	GeoExtent extents = GeoExtent(srs, xmin, ymin, xmax, ymax);
	getDataExtents().push_back(DataExtent(extents, 0, maxDataLevel));

	return profile;
}

std::string
WCS11Source::getExtension() const
{
    return "tif";
}


osg::Image*
WCS11Source::createImage(const TileKey&        key,
                         ProgressCallback*     progress)
{
    HTTPRequest request = createRequest( key );

    double lon0,lat0,lon1,lat1;
    key.getExtent().getBounds( lon0, lat0, lon1, lat1 );

    ReadResult out_response = URI(request.getURL()).readImage(_dbOptions.get(), progress);
    if (!out_response.failed())
    {
        return out_response.releaseImage();
    }
    else
    {
        OSG_INFO << LC << "Unable to create image: " << out_response.errorDetail() << ". Set OSGEARTH_HTTP_DEBUG=1 for details." << std::endl;
        return NULL;
    }
}


osg::HeightField*
WCS11Source::createHeightField(const TileKey&        key,
                               ProgressCallback*     progress)
{
    osg::HeightField* field = NULL;

    osg::ref_ptr<osg::Image> image = createImage( key, progress );
    if ( image.valid() )
    {        
        ImageToHeightFieldConverter conv;
        conv.setRemoveNoDataValues( true );
        field = conv.convert( image.get() );
    }

    return field;
}

/*
http://server/ArcGIS/services/WorldElevation/MapServer/WCSServer
    ?SERVICE=WCS
    &VERSION=1.1.0
    &REQUEST=GetCoverage
    &IDENTIFIER=1
    &FORMAT=image/GeoTIFF
    &BOUNDINGBOX=-180,-90,0,90,urn:ogc:def:crs:EPSG::4326  // (sic - coord ordering bug in ESRI)
    &RangeSubset=Field_1:bilinear[Band[1]]
    &GridBaseCRS=urn:ogc:def:crs:EPSG::4326
    &GridCS=urn:ogc:def:crs:EPSG::4326
    &GridType=urn:ogc:def:method:WCS:1.1:2dGridIn2dCrs
    &GridOrigin=-180,90
    &GridOffsets=6,-6
*/


HTTPRequest
WCS11Source::createRequest( const TileKey& key ) const
{
    std::stringstream buf;

    double lon_min, lat_min, lon_max, lat_max;
    key.getExtent().getBounds( lon_min, lat_min, lon_max, lat_max );

    int lon_samples = getPixelsPerTile();
    int lat_samples = getPixelsPerTile();
    double lon_interval = (lon_max-lon_min)/(double)(lon_samples-1);
    double lat_interval = (lat_max-lat_min)/(double)(lat_samples-1);

    HTTPRequest req( _options.url()->full() );

    req.addParameter( "SERVICE",    "WCS" );
    req.addParameter( "VERSION",    "1.1.0" );
    req.addParameter( "REQUEST",    "GetCoverage" );
    req.addParameter( "IDENTIFIER", _identifier );
    req.addParameter( "FORMAT",     _covFormat );

    req.addParameter( "GridBaseCRS", "urn:ogc:def:crs:EPSG::4326" );
    req.addParameter( "GridCS",      "urn:ogc:def:crs:EPSG::4326" );
    req.addParameter( "GridType",    "urn:ogc:def:method:WCS:1.1:2dGridIn2dCrs" );

    // IMPORTANT NOTE:
    //   For WCS1.1+, the BOUNDINGBOX for geographic CRS's (like WGS84) are expressed
    //   at minlat,minlon,maxlat,maxlon instead of the usual minx,miny,maxx,maxy.
    //   So we will somehow need to figure out whether the CRS is geographic.
    //
    // MORE IMPORTANT NOTE:
    //   ESRI's ArcGIS WCS Server doesn't obey the above rule. Their server expects
    //   minx,miny,maxx,maxy no matter what ...

    // Hack to guess whether it's an ArcGIS Server:
    buf.str("");

    //bool use_legacy_geog_bbox_encoding = _url.find( "/MapServer/WCSServer" ) != std::string::npos;
    //if ( use_legacy_geog_bbox_encoding )
    //    buf << lon_min << "," << lat_min << "," << lon_max << "," << lat_max;
    //else
    //    buf << lat_min << "," << lon_min << "," << lat_max << "," << lon_max;
    //buf << ",urn:ogc:def:crs:EPSG::4326";

    // there used to be code here to shift the bounding box out by half a pixel in all directions to make sure that neighboring tiles
    // would have the same elevation values. WCS 1.1, however, samples at the edges of the bounding box, so shifting the bounding box
    // will produce values that don't match up.
    buf << std::setprecision(15) << lon_min << "," << lat_min << "," << lon_max << "," << lat_max << ",EPSG:4326";
	std::string bufStr;
	bufStr = buf.str();
    req.addParameter( "BOUNDINGBOX", bufStr );

    double originX = lon_min;
    double originY = lat_max;

    buf.str("");
    buf << std::setprecision(15) << originX << "," << originY;
	bufStr = buf.str();
    req.addParameter( "GridOrigin", bufStr );
    
    buf.str("");
    buf << std::setprecision(15) << lon_interval << "," << lat_interval;   // note: top-down
    //buf << lon_interval << "," << lat_interval;
	bufStr = buf.str();
    req.addParameter( "GridOffsets", bufStr );

    if ( !_options.rangeSubset()->empty() )
        req.addParameter( "RangeSubset", _options.rangeSubset().value() );

    return req;
}
