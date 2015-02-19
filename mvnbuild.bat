set NAME=OSGEARTH

CALL :RESOLVE "target\deps\OpenSceneGraph" OSGDIR
CALL :RESOLVE "target\deps\3rdparty" THIRDPARTY
CALL :RESOLVE "target\deps\geos" GEOS

set CURRENT_DIR=%CD%
CALL :RESOLVE "target\mvninstall\%1" INSTALLDIR
CALL :MKANDPUSHD target\mvnbuild

cmake -G "Visual Studio 12 Win64" ^
-D GDAL_INCLUDE_DIR=%THIRDPARTY%/include ^
-D GDAL_LIBRARY=%THIRDPARTY%/lib/gdal_i.lib ^
-D GEOS_INCLUDE_DIR=%GEOS%/include ^
-D GEOS_LIBRARY=%GEOS%/lib/geos_i.lib ^
-D GEOS_LIBRARY_DEBUG=%GEOS%/lib/geos_i_d.lib ^
-D ZLIB_INCLUDE_DIR=%THIRDPARTY%/include ^
-D ZLIB_LIBRARY=%THIRDPARTY%/lib/zlib.lib ^
-D CURL_INCLUDE_DIR=%THIRDPARTY%/include ^
-D CURL_LIBRARY=%THIRDPARTY%/lib/libcurl.lib ^
-D CURL_LIBRARY_DEBUG=%THIRDPARTY%/lib/libcurld.lib ^
-D OSGEARTH_USE_QT=OFF ^
-D CMAKE_INSTALL_PREFIX=%INSTALLDIR% ^
-D WIN32_USE_MP=ON ^
-D ENABLE_FASTDXT=ON ^
%CURRENT_DIR% || exit 

devenv.com %NAME%.sln /build %1 /Project INSTALL || exit 
GOTO :EOF


:RESOLVE
SET TEMPRESOLVE=%~f1
SET "%2=%TEMPRESOLVE:\=/%"
GOTO :EOF

:MKANDPUSHD
IF NOT EXIST %1 ( mkdir %1 )
pushd %1 || exit 
GOTO :EOF
