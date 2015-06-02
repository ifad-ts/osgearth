%global osg_ver 3.2.1.2

Name:           osgearth-ifad
Version:        2.6.7
Release:        1%{?dist}
Summary:        Dynamic map generation toolkit for OpenSceneGraph (IFAD version)
Obsoletes:      osgearth

License:        LGPLv3 with exceptions
URL:            http://osgearth.org/
Source0:        https://github.com/ifad-ts/osgearth/archive/%{name}-%{version}.tar.gz

BuildRequires:  cmake
BuildRequires:  gdal-devel
BuildRequires:  geos-devel
BuildRequires:  libcurl-devel
BuildRequires:  OpenSceneGraph-ifad = %{osg_ver}
BuildRequires:  OpenSceneGraph-ifad-devel
BuildRequires:  OpenSceneGraph-ifad-qt-devel
BuildRequires:  python-sphinx
BuildRequires:  qt4-devel
BuildRequires:  tinyxml-devel

Requires:       OpenSceneGraph-ifad = %{osg_ver}

%description
osgEarth is a C++ terrain rendering SDK. Just create a simple XML file, point
it at your imagery, elevation, and vector data, load it into your favorite
OpenSceneGraph application, and go! osgEarth supports all kinds of data and
comes with lots of examples to help you get up and running quickly and easily.


%package        devel
Summary:        Development files for %{name}
Requires:       %{name}%{?_isa} = %{version}-%{release}
Obsoletes:      osgearth-devel

%description    devel
The %{name}-devel package contains libraries and header files for
developing applications that use %{name}.


%package        tools
Summary:        %{name} viewers and tools
Requires:       %{name}%{?_isa} = %{version}-%{release}
Obsoletes:      osgearth-tools

%description    tools
The %{name}-tools contains viewers and data manipulation tools for %{name}.


%package        examples
Summary:        %{name} example applications
Requires:       %{name}%{?_isa} = %{version}-%{release}
Requires:       %{name}-examples-data = %{version}-%{release}
Obsoletes:      osgearth-examples

%description    examples
The %{name}-examples contains %{name} example applications.


%package        examples-data
Summary:        Data for %{name} example applications
BuildArch:      noarch
Requires:       %{name}-examples = %{version}-%{release}
Obsoletes:      osgearth-examples-data

%description    examples-data
The %{name}-examples-data contains data for the %{name} example
applications.


%package doc
Summary:        Documentation files for %{name}
Provides:       bundled(jquery)
BuildArch:      noarch
Obsoletes:      osgearth-doc

%description doc
The %{name}-doc package contains documentation files for developing
applications that use %{name}.


%prep
%setup -q -n osgearth-%{name}-%{version}

# Fix spurious-executable-perm and script-without-shebang
chmod -x data/boxman.osg
chmod -x src/osgEarth/TextureCompositor.cpp
chmod -x src/osgEarthAnnotation/AnnotationUtils.cpp
chmod -x src/osgEarthUtil/Controls.cpp
chmod -x src/osgEarth/OverlayDecorator.cpp
chmod -x src/osgEarth/ShaderFactory.cpp

# Remove bundled sources just to be sure
rm -f osgEarth/tiny*

# Remove non-free content
rm -rf data/loopix


%build
mkdir build
(
cd build
LDFLAGS="-Wl,--as-needed" %cmake -DWITH_EXTERNAL_TINYXML=True ..
make %{?_smp_mflags}
)
make -C docs html %{?_smp_mflags}


%post -p /sbin/ldconfig

%postun -p /sbin/ldconfig


%install
%make_install -C build
install -Dd %{buildroot}%{_datadir}/%{name}
cp -a data %{buildroot}%{_datadir}/%{name}/data
cp -a tests %{buildroot}%{_datadir}/%{name}/tests

# Remove unnecessary files
rm -f docs/build/html/.buildinfo


%files
%license LICENSE.txt
%{_libdir}/libosgEarth*.so.*
%{_libdir}/osgdb_*.so

%files devel
%{_includedir}/osgEarth*/
%{_libdir}/libosgEarth*.so

%files tools
%{_bindir}/osgearth_atlas
%{_bindir}/osgearth_backfill
%{_bindir}/osgearth_boundarygen
%{_bindir}/osgearth_cache
%{_bindir}/osgearth_conv
%{_bindir}/osgearth_featureinfo
%{_bindir}/osgearth_package
%{_bindir}/osgearth_package_qt
%{_bindir}/osgearth_qt
%{_bindir}/osgearth_shadergen
%{_bindir}/osgearth_tfs
%{_bindir}/osgearth_tileindex
%{_bindir}/osgearth_version
%{_bindir}/osgearth_viewer
%{_bindir}/osgearth_cache_test
%{_bindir}/osgearth_createtile

%files examples
%{_bindir}/osgearth_annotation
%{_bindir}/osgearth_city
%{_bindir}/osgearth_clamp
%{_bindir}/osgearth_clipplane
%{_bindir}/osgearth_colorfilter
%{_bindir}/osgearth_controls
%{_bindir}/osgearth_demo
%{_bindir}/osgearth_elevation
%{_bindir}/osgearth_featurefilter
%{_bindir}/osgearth_featuremanip
%{_bindir}/osgearth_featurequery
%{_bindir}/osgearth_features
%{_bindir}/osgearth_fog
%{_bindir}/osgearth_graticule
%{_bindir}/osgearth_imageoverlay
%{_bindir}/osgearth_los
%{_bindir}/osgearth_manip
%{_bindir}/osgearth_map
%{_bindir}/osgearth_measure
%{_bindir}/osgearth_minimap
%{_bindir}/osgearth_mrt
%{_bindir}/osgearth_occlusionculling
%{_bindir}/osgearth_overlayviewer
%{_bindir}/osgearth_qt_simple
%{_bindir}/osgearth_qt_windows
%{_bindir}/osgearth_sequencecontrol
%{_bindir}/osgearth_shadercomp
%{_bindir}/osgearth_sharedlayer
%{_bindir}/osgearth_terraineffects
%{_bindir}/osgearth_terrainprofile
%{_bindir}/osgearth_tilesource
%{_bindir}/osgearth_toc
%{_bindir}/osgearth_tracks
%{_bindir}/osgearth_transform

%files examples-data
%{_datadir}/%{name}

%files doc
%license LICENSE.txt
%doc docs/build/html


%changelog
* Tue Jun 02 2015 Michael Bach <michael@ifad.dk> - 2.6.7-1
- Use identifier instead of title when setting up bounds for WCS coverages.

* Fri May 22 2015 Michael Bach <michael@ifad.dk> - 2.6.6-1
- Support for disabling cancellation of tile construction.

* Wed May 20 2015 Michael Bach <michael@ifad.dk> - 2.6.5-2
- Changed OpenSceneGraph dependency to OpenSceneGraph-ifad

* Wed May 13 2015 Michael Bach <michael@ifad.dk> - 2.6.5-1
- Fix: Terrain becoming slightly transparent when you get very close to the ground.

* Fri May 08 2015 Michael Bach <michael@ifad.dk> - 2.6.4-1
- Supports namespaced replies from GetCapabilities.
- No longer queries outside the boundaries of a WCS coverage by adding a call to GetCapabilities.
- Uses new image download techniques to avoid false reports on download errors (due to actively aborted downloads).

* Tue Apr 07 2015 Michael Bach <michael@ifad.dk> - 2.6.3-1
- First rpm build of our custom osgEarth source
- Contains WCS 1.0 support by type <version>1.0</version> in an elevation tag that has specified the wcs driver

* Fri Dec 12 2014 Sandro Mani <manisandro@gmail.com> - 2.6-3
- Parallel build for docs
- Noarch data subpackage

* Fri Dec 12 2014 Sandro Mani <manisandro@gmail.com> - 2.6-2
- Add explicit Requires: OpenSceneGraph = %%{osg_ver}
- Add Provides: bundled(jquery) to -doc
- USe %%license for license
- Use system tinyxml, remove bundled sources
- Remove non-free loopix data
- Remove html/.buildinfo
- Add -Wl,--as-needed
- Improve descriptions
- Rename package data -> examples, put example binaries in that package

* Thu Nov 20 2014 Sandro Mani <manisandro@gmail.com> - 2.6-1
- Initial package