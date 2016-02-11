set NAME=OSGEARTH

CALL :RESOLVE "packages\openscenegraph-temp" OSGDIR
CALL :RESOLVE "packages\osgvisual-3rdparty-temp\x64" THIRDPARTY
CALL :RESOLVE "packages\geos-temp" GEOS

set CURRENT_DIR=%CD%
CALL :RESOLVE "target\install" INSTALLDIR

rmdir /s /q target\tempinstall
mkdir target\install
move target\install target\tempinstall || exit 

CALL :MKANDPUSHD target\nugetbuild

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
-D OSG_VERSION_EXE=%OSGDIR%/bin/release/osgversion.exe ^
-D WIN32_USE_MP=ON ^
-D ENABLE_FASTDXT=ON ^
-D CMAKE_CXX_FLAGS_RELEASE:STRING="/MD /O2 /Ob2 /D NDEBUG /Zi /Oy-" ^
-D CMAKE_SHARED_LINKER_FLAGS_RELEASE:STRING="/DEBUG /OPT:REF /OPT:ICF /INCREMENTAL:NO" ^
-D CMAKE_EXE_LINKER_FLAGS_RELEASE:STRING="/DEBUG /OPT:REF /OPT:ICF /INCREMENTAL:NO" ^
-D CMAKE_MODULE_LINKER_FLAGS_RELEASE:STRING="/DEBUG /OPT:REF /OPT:ICF /INCREMENTAL:NO" ^
%CURRENT_DIR% || exit 

devenv.com %NAME%.sln /build %1 /Project INSTALL || exit 
popd
for /r %INSTALLDIR% %%p in (*.dll) do (
	for /f "delims=" %%i in ('dir target\nugetbuild\lib\%%~np.pdb /b /s') do ( xcopy /y %%~dpnxi  %%~dpp )
	for /f "delims=" %%i in ('dir target\nugetbuild\bin\%%~np.pdb /b /s') do ( xcopy /y %%~dpnxi  %%~dpp )
)

rmdir /s /q target\nugetinstall\%1
mkdir target\nugetinstall
move target\install target\nugetinstall\%1 || exit 
move target\tempinstall target\install || exit 
xcopy target\nugetinstall\%1 target\install /e /y || exit 
GOTO :EOF

:RESOLVE
SET TEMPRESOLVE=%~f1
SET "%2=%TEMPRESOLVE:\=/%"
GOTO :EOF

:MKANDPUSHD
IF NOT EXIST %1 ( mkdir %1 )
pushd %1 || exit 
GOTO :EOF
