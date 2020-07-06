from conans import ConanFile, CMake, tools
import os
import subprocess

from conans.tools import os_info, SystemPackageTool


class OsgEarthConan(ConanFile):
    name = "osgearth-ifad"
    version = "2.9"
    license = "https://github.com/gwaldron/osgearth/blob/master/LICENSE.txt"
    url = "https://github.com/ifad-ts/osgearth"
    description = "IFAD version of osgEarth. osgEarth is a C++ geospatial SDK and terrain engine. Just create a simple XML file, point it at your map data, and go! osgEarth supports all kinds of data and comes with lots of examples to help you get up and running quickly and easily."
    settings = "os", "compiler", "build_type", "arch"
    # do all options had to be wrapped to cmake?
    options = {"shared": [True, False]}
    default_options = "shared=True", "geos:shared=False"
    generators = "cmake"
    copy_source_to_build_dir = False
    build_policy = "missing"  # "always" #
    short_paths = True  # for win<10 naming
    requires = "openscenegraph-ifad/3.6.3.1@ifad/stable", "geos/3.5.1@ifad/stable", "osgvisual/11_full@ifad/stable"
    exports_sources = "CMakeModules/*", "data/*", "docs/*", "src/*", "tests/*", "*.txt"

    # Manually-specified variables were not used by the project:
    # CONAN_COMPILER
    # CONAN_COMPILER_VERSION
    # CONAN_CXX_FLAGS
    # CONAN_C_FLAGS
    # CONAN_EXPORTED
    # CONAN_LIBCXX
    # CONAN_SHARED_LINKER_FLAGS

    def system_requirements(self):
        self.output.warn("system_requirements: ")
        pack_name = None
        if os_info.linux_distro == "ubuntu":
            self.run('sudo apt-get build-dep openscenegraph', True)
            # gstreamer seams missing after build-dep
            pack_name = "libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libasio-dev libcollada-dom2.4-dp-dev libdcmtk-dev libfltk1.3-dev libnvtt-dev libboost-filesystem-dev"
        elif os_info.linux_distro == "fedora" or os_info.linux_distro == "centos":
            pack_name = "TODOpackage_names_in_fedora_and_centos"
        elif os_info.is_macos:
            pack_name = "TODOpackage_names_in_macos"
        elif os_info.is_freebsd:
            pack_name = "TODOpackage_names_in_freebsd"
        elif os_info.is_solaris:
            pack_name = "TODOpackage_names_in_solaris"

        if pack_name:
            installer = SystemPackageTool()
            installer.install(
                pack_name)  # Install the package, will update the package database if pack_name isn't already installed

    def _cmake_configure(self):
        self.output.warn(self.deps_cpp_info['geos'].include_paths[0])
        self.output.warn(os.path.join(self.deps_cpp_info['geos'].lib_paths[0], self.deps_cpp_info['geos'].libs[0]))
        cmake = CMake(self, set_cmake_flags=True)
        cmake.definitions['BUILD_SHARED_LIBS'] = "ON" if self.options.shared else "OFF"
        cmake.definitions['BUILD_OSGEXAMPLES'] = 'OFF'
        cmake.definitions['BUILD_DOCUMENTATION'] = 'OFF'
        cmake.definitions['BUILD_OSGAPPLICATIONS'] = 'ON'

        if self.settings.compiler == "Visual Studio":
            cmake.definitions['BUILD_WITH_STATIC_CRT'] = "ON" if "MT" in str(self.settings.compiler.runtime) else "OFF"
            self._set_windows_definitions(cmake)

        cmake.configure(source_dir=self.source_folder)
        return cmake

    def _set_windows_definitions(self, cmake):
        thirdparty_folder = os.path.join(self.deps_cpp_info["osgvisual"].rootpath, 'x64')
        thirdparty_include = os.path.join(thirdparty_folder, "include")
        thirdparty_lib = os.path.join(thirdparty_folder, "lib")
        cmake.definitions['GDAL_INCLUDE_DIR'] = thirdparty_include
        cmake.definitions['GDAL_LIBRARY'] = os.path.join(thirdparty_lib, "gdal_i.lib")
        print('GDAL_LIBRARY = ' + cmake.definitions['GDAL_LIBRARY'])
        cmake.definitions['GEOS_INCLUDE_DIR'] = self.deps_cpp_info['geos'].include_paths[0]
        geos_library = os.path.join(self.deps_cpp_info['geos'].lib_paths[0], self.deps_cpp_info['geos'].libs[0])
        cmake.definitions['GEOS_LIBRARY'] = geos_library
        cmake.definitions['GEOS_LIBRARY_DEBUG'] = geos_library
        cmake.definitions['ZLIB_INCLUDE_DIR'] = thirdparty_include
        cmake.definitions['ZLIB_LIBRARY'] = os.path.join(thirdparty_lib, "zlib.lib")
        cmake.definitions['CURL_INCLUDE_DIR'] = thirdparty_include
        cmake.definitions['CURL_LIBRARY'] = os.path.join(thirdparty_lib, "libcurl_imp.lib")
        cmake.definitions['CURL_LIBRARY_DEBUG'] = os.path.join(thirdparty_lib, "libcurl_impd.lib")
        cmake.definitions['OSGEARTH_USE_QT'] = "OFF"
        # cmake.definitions['OSG_VERSION_EXE']="%OSGDIR%/bin/release/osgversion.exe"
        cmake.definitions['WIN32_USE_MP']="ON"
        cmake.definitions['ENABLE_FASTDXT'] = "ON"
        cmake.definitions['CMAKE_CXX_FLAGS_RELEASE'] = "/MD /O2 /Ob2 /D NDEBUG /Zi /Oy-"
        cmake.definitions['CMAKE_SHARED_LINKER_FLAGS_RELEASE'] = "/DEBUG /OPT:REF /OPT:ICF /INCREMENTAL:NO"
        cmake.definitions['CMAKE_EXE_LINKER_FLAGS_RELEASE'] = "/DEBUG /OPT:REF /OPT:ICF /INCREMENTAL:NO"
        cmake.definitions['CMAKE_MODULE_LINKER_FLAGS_RELEASE'] = "/DEBUG /OPT:REF /OPT:ICF /INCREMENTAL:NO"

    def build(self):
        cmake = self._cmake_configure()
        cmake.build()

    def package(self):
        cmake = self._cmake_configure()
        cmake.install()

    def package_info(self):
        if self.settings.os != "Windows":
            self.env_info.LD_LIBRARY_PATH.append(os.path.join(self.package_folder, "lib"))
        self.env_info.PATH.append(os.path.join(self.package_folder, "bin"))
        if self.settings.os == "Windows" and self.settings.compiler == "Visual Studio":
            self.env_info.PATH.append(os.path.join(self.package_folder, "3rdParty/bin"))
