/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include "WCS10Source.h"
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osg/Notify>
#include <osgDB/Registry>
#include <iostream>
#include <stdlib.h>

using namespace osgEarth;


WCS10Source::WCS10Source( const TileSourceOptions& options ) :
TileSource( options ),
_options  ( options )
{
    _covFormat = _options.format().value();
    
    if ( _covFormat.empty() )
        _covFormat = "image/GeoTIFF";

    _osgFormat = "tif";
}



osgEarth::TileSource::Status WCS10Source::initialize(const osgDB::Options* dbOptions)
{        
    //TODO: fetch GetCapabilities and set profile from there.
    setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
    _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );    

    return STATUS_OK;
}


std::string
WCS10Source::getExtension() const
{
    return "tif";
}


osg::Image*
WCS10Source::createImage(const TileKey&        key,
                         ProgressCallback*     progress)
{
    HTTPRequest request = createRequest(key);

    OE_INFO << "[osgEarth::WCS1.1] Key=" << key.str() << " URL = " << request.getURL() << std::endl;

    double lon0,lat0,lon1,lat1;
    key.getExtent().getBounds( lon0, lat0, lon1, lat1 );

    // download the data. It's a multipart-mime stream, so we have to use HTTP directly.
    HTTPResponse response = HTTPClient::get( request, _dbOptions.get(), progress );
    if ( !response.isOK() )
    {
        OE_WARN << "[osgEarth::WCS1.1] WARNING: HTTP request failed" << std::endl;
        return NULL;
    }

    //TODO:  Make WCS driver use progress callback
    unsigned int part_num = response.getNumParts() > 1? 1 : 0;
    std::istream& input_stream = response.getPartStream( part_num );

    //TODO: un-hard-code TIFFs
    osgDB::ReaderWriter* reader = osgDB::Registry::instance()->getReaderWriterForExtension( "tiff" );

    if ( !reader )
    {
        OE_NOTICE << "[osgEarth::WCS1.1] WARNING: no reader for \"tiff\"" << std::endl;
        return NULL;
    }

    osgDB::ReaderWriter::ReadResult result = reader->readImage( input_stream ); //, getOptions() );
    //osgDB::ReaderWriter::ReadResult result = reader->readImage("D:/srv/spatialData/derived/elevation/DTM10-DK-MASKED-2015-03-11/data/test.tif");
    //osgDB::ReaderWriter::ReadResult result = reader->readImage("C:/Users/Michael/Downloads/wcs.tif");
    if ( !result.success() )
    {
        OE_NOTICE << "[osgEarth::WCS1.1] WARNING: readImage() failed for Reader " << reader->getName() << std::endl;
        return NULL;
    }

    osg::Image* image = result.getImage();
    //OE_NOTICE << "Returned grid is " << image->s() << "x" << image->t() << std::endl;
    if ( image ) image->ref();
    return image;
}

osg::HeightField*
WCS10Source::createHeightField(const TileKey&        key,
                               ProgressCallback*     progress)
{
    osg::HeightField* field = NULL;

    osg::ref_ptr<osg::Image> image = createImage( key, progress );
    if ( image.valid() )
    {        
        ImageToHeightFieldConverter conv;
        conv.setRemoveNoDataValues( true, NO_DATA_VALUE );
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


osgEarth::HTTPRequest WCS10Source::createRequest(const TileKey& key) const
{
    std::stringstream buf;

    double lon_min, lat_min, lon_max, lat_max;
    key.getExtent().getBounds(lon_min, lat_min, lon_max, lat_max);

    int lon_samples = _options.tileSize().value();
    int lat_samples = _options.tileSize().value();
    double lon_interval = (lon_max - lon_min) / (double)(lon_samples - 1);
    double lat_interval = (lat_max - lat_min) / (double)(lat_samples - 1);

    HTTPRequest req(_options.url()->full());

    req.addParameter("SERVICE", "WCS");
    req.addParameter("VERSION", "1.0.0");
    req.addParameter("REQUEST", "GetCoverage");
    req.addParameter("COVERAGE", _options.identifier().value());
    req.addParameter("FORMAT", _covFormat);

    req.addParameter("CRS", "urn:ogc:def:crs:EPSG::4326");

    buf.str("");
    buf << lon_min << "," << lat_min << "," << lon_max << "," << lat_max;
    req.addParameter("BBOX", buf.str());

    buf.str("");
    buf << lon_samples;
    req.addParameter("WIDTH", buf.str());
    buf.str("");
    buf << lat_samples;
    req.addParameter("HEIGHT", buf.str());

    // Not supported in WCS 1.0.0...
    //if (!_options.rangeSubset()->empty())
    //    req.addParameter("RangeSubset", _options.rangeSubset().value());

    return req;
}