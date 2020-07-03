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

#include <osgEarthUtil/WCS>
#include <osgEarth/XmlUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace std;

namespace
{
    void removeElementNamespace(XmlElement* e)
    {
        for (XmlNodeList::iterator it = e->getChildren().begin(); it != e->getChildren().end(); ++it)
        {
            XmlNode* child = *it;
            if (child->isElement())
            {
                removeElementNamespace(static_cast<XmlElement*>(child));
            }
        }

        std::string::size_type i = e->getName().find(':');
        if (i == std::string::npos)
        {
            return;
        }
        else
        {
            e->setName(e->getName().substr(i + 1, e->getName().length() - 1));
        }
    }
}

WCSCapabilities::WCSCapabilities()
{
}

WCSCoverage*
WCSCapabilities::getCoverageByIdentifier(const std::string& identifier)
{
    for (CoverageList::iterator itr = _coverages.begin(); itr != _coverages.end(); ++itr)
    {
        if (osgDB::equalCaseInsensitive(itr->get()->getIdentifier(),identifier)) return itr->get();
    }
    return NULL;
}

osgEarth::Util::WCSCoverage* osgEarth::Util::WCSCapabilities::getCoverageByTitle(const std::string& title)
{
    for (CoverageList::iterator itr = _coverages.begin(); itr != _coverages.end(); ++itr)
    {
        if (osgDB::equalCaseInsensitive(itr->get()->getTitle(), title)) return itr->get();
    }
    return NULL;
}

#define ATTR_VERSION "version"
#define ELEM_CAPABILITY "capability"
#define ELEM_SERVICE "serviceidentification"
#define ELEM_ABSTRACT "abstract"
#define ELEM_FORMAT "format"
#define ELEM_TITLE "title"
#define ELEM_IDENTIFIER "identifier"
#define ELEM_SRS "srs"
#define ELEM_CONTENTS "contents"
#define ELEM_COVERAGESUMMARY "coveragesummary"
#define ELEM_WGS84BOUNDINGBOX_FUSION "wgs84boudingbox" // Required to support Luciad Fusion (2015.0).
#define ELEM_WGS84BOUNDINGBOX        "wgs84boundingbox"
#define ELEM_LOWERCORNER      "lowercorner"
#define ELEM_UPPERCORNER      "uppercorner"

/**************************************************************************************/
WCSCoverage::WCSCoverage()
{    
}
/**************************************************************************************/

WCSCapabilities* 
WCSCapabilitiesReader::read( const URI& location, const osgDB::Options* dbOptions )
{
    // read the data into a string buffer and parse it from there
    std::string buffer = location.readString(dbOptions).getString();
    if ( !buffer.empty() )
    {
        std::stringstream buf(buffer);
        return read(buf);
    }
    else return 0L;
}

WCSCapabilities*
WCSCapabilitiesReader::read(std::istream &in)
{
    osg::ref_ptr<WCSCapabilities> capabilities = new WCSCapabilities;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
    if (!doc.valid() || doc->getChildren().empty())
    {
        OE_NOTICE << "Failed to load Capabilities " << std::endl;
        return 0;
    }

    //Get the Capabilities version
    osg::ref_ptr<XmlElement> e_root = static_cast<XmlElement*>(doc->getChildren()[0].get());
    capabilities->setVersion( e_root->getAttr(ATTR_VERSION ) );

    removeElementNamespace(e_root);

    osg::ref_ptr<XmlElement> e_service = e_root->getSubElement( ELEM_SERVICE );
    if (!e_service.valid())
    {
        OE_NOTICE << "Could not find Service element" << std::endl;
        return 0;
    }
    

    //Read the parameters from the ServiceIdentification block
    capabilities->setAbstract( e_service->getSubElementText( ELEM_ABSTRACT ) );
    capabilities->setTitle( e_service->getSubElementText(ELEM_TITLE) );

    //Read all the coverages    
    osg::ref_ptr<XmlElement> e_contents = e_root->getSubElement( ELEM_CONTENTS );
    if (e_contents.valid())
    {
        XmlNodeList coverages = e_contents->getSubElements( ELEM_COVERAGESUMMARY );
        for( XmlNodeList::const_iterator itr = coverages.begin(); itr != coverages.end(); itr++ )
        {
            XmlElement* e_coverage = static_cast<XmlElement*>( itr->get() );
            WCSCoverage* coverage = new WCSCoverage();
            coverage->setTitle(e_coverage->getSubElementText(ELEM_TITLE));
            coverage->setIdentifier(e_coverage->getSubElementText(ELEM_IDENTIFIER));
            coverage->setAbstract(e_coverage->getSubElementText(ELEM_ABSTRACT));

            osg::ref_ptr<XmlElement> e_bb = e_coverage->getSubElement( ELEM_WGS84BOUNDINGBOX );
            if (!e_bb.valid())
            {
                // fix for misspelled bounding box element in Luciad Fusion 2015.0
                e_bb = e_coverage->getSubElement(ELEM_WGS84BOUNDINGBOX_FUSION);
            }
            if (e_bb.valid())
            {
                double minX, minY, maxX, maxY;
                std::vector<std::string> lowerCornerStrings;
                std::vector<std::string> upperCornerStrings;
                StringTokenizer(e_bb->getSubElementText(ELEM_LOWERCORNER), lowerCornerStrings, " ", "", false, true);
                StringTokenizer(e_bb->getSubElementText(ELEM_UPPERCORNER), upperCornerStrings, " ", "", false, true);
                minX = as<double>(lowerCornerStrings[0], 0);
                minY = as<double>(lowerCornerStrings[1], 0);
                maxX = as<double>(upperCornerStrings[0], 0);
                maxY = as<double>(upperCornerStrings[1], 0);
                coverage->setExtent( GeoExtent( osgEarth::SpatialReference::create("wgs84"), minX, minY, maxX, maxY) );
            }                       

            capabilities->getCoverages().push_back( coverage );
        }        
    }


    return capabilities.release();
}
