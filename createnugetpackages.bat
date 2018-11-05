rem Build native osgEarth NuGet package(s).
rem Requires nuget.exe, cmake.exe in the path and the Visual Studio x64 setup (vcvarsall.bat x86_amd64)
rem Also requires a NuGet feed containing the dependent packages (most easily setup from within Visual Studio
rem as that setup is picked up by the command-line nuget.exe)

set OSGVISUAL_VERSION=10.0.2
set OSG_VERSION=3.5.10.1

rem get dependencies
nuget install -OutputDirectory packages osgvisual-3rdparty-full -version %OSGVISUAL_VERSION%
nuget install -OutputDirectory packages geos -version 3.4.2
nuget install -OutputDirectory packages OpenSceneGraphIFAD -version %OSG_VERSION%

call convert-nugetinstall-to-cmake-friendly-layout.bat packages\osgvisual-3rdparty-full.%OSGVISUAL_VERSION% packages\osgvisual-3rdparty-temp\x64
call convert-nugetinstall-to-cmake-friendly-layout.bat packages\OpenSceneGraphIFAD.%OSG_VERSION% packages\openscenegraph-temp
call convert-nugetinstall-to-cmake-friendly-layout.bat packages\geos.3.4.2 packages\geos-temp

rem build osgEarth debug and release
call nugetbuild.bat debug
call nugetbuild.bat release

rem build the nuget packages
PowerShell -Command "& {Write-NugetPackage osgearth-ifad.autopkg}"
