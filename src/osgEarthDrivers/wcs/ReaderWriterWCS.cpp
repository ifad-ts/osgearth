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

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include "WCS10Source.h"
#include "WCS11Source.h"
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

class WCSSourceFactory : public TileSourceDriver
{
public:
    WCSSourceFactory() {}

    virtual const char* className() const
    {
        return "WCS 1.x.0 Reader";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "osgearth_wcs" );
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
    {
        std::string ext = osgDB::getFileExtension( file_name );
        if ( !acceptsExtension( ext ) )
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
        
        WCSOptions wcsOpt(getTileSourceOptions(opt));
        if (wcsOpt.version().value() == std::string("1.0") ||
            wcsOpt.version().value() == std::string("1.0.0"))
        {
            return new WCS10Source(wcsOpt);
        }
        else if (wcsOpt.version().value() == std::string("1.1") ||
                 wcsOpt.version().value() == std::string("1.1.0"))
        {
            return new WCS11Source(wcsOpt);
        }
        else
        {
            OE_NOTICE << "[osgEarth::WCS] Unknown or unspecified WCS version. Reverting to WCS version 1.1.0" << std::endl;
            return new WCS11Source(wcsOpt);
        }
    }
};

REGISTER_OSGPLUGIN(osgearth_wcs, WCSSourceFactory)
