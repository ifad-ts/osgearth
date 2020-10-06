from conans import ConanFile, CMake, tools
from pathlib import Path
import os, shutil


class OsgearthConan(ConanFile):
    name = "osgearth-ifad"
    version = "2.9.3"
    license = "LGPL-3"
    url = "https://github.com/ifad-ts/osgearth"
    description = "IFAD version of osgEarth. osgEarth is a C++ geospatial SDK and terrain engine. Just create a simple XML file, point it at your map data, and go! osgEarth supports all kinds of data and comes with lots of examples to help you get up and running quickly and easily."
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "geos": [True, False]
    }
    default_options = {
        "shared": True,
        "fPIC": True,
        "geos": True,
        "gdal:shared": True,
        "libcurl:shared": True
    }
    build_requires = "cmake_installer/3.15.4@conan/stable"
    requires = (
        "openscenegraph-ifad/3.6.3.3@ifad/stable",
        # "libcurl/7.61.1@bincrafters/stable",
        "libcurl/7.72.0",
        # "protobuf/3.6.1@bincrafters/stable",
        # "poco/1.8.1",
        "gdal/3.1.2"
    )
    generators = "cmake"
    exports_sources = "CMakeModules/*", "data/*", "docs/*", "src/*", "tests/*", "*.txt"
    short_paths = True

    def config_options(self):
        if self.settings.os == 'Windows':
            del self.options.fPIC

    def requirements(self):
        pass
        # self.requires("zlib/1.2.11@conan/stable", override=True)
        # self.requires("libjpeg/9c@bincrafters/stable", override=True)
        # self.requires("libpng/1.6.37@bincrafters/stable", override=True)
        # self.requires("libtiff/4.0.9@bincrafters/stable", override=True)
        # self.requires("openssl/1.1.1g@conan/stable", override=True)
        # self.requires("bzip2/1.0.8@conan/stable", override=True)
        # self.requires("freetype/2.10.0@bincrafters/stable", override=True)
        # self.requires("sqlite3/3.29.0@bincrafters/stable", override=True)
        # self.requires("libtiff/4.1.0", override=True)

        # if self.options.geos:
        #     self.requires("geos/3.7.3@insanefactory/stable")

    def _cmake_configure(self):
        osgDir = str(Path(self.deps_cpp_info["openscenegraph-ifad"].include_paths[0]).parent)
        gdalDir = str(Path(self.deps_cpp_info["gdal"].include_paths[0]).parent)
        cmake = CMake(self)
        cmake.definitions["DYNAMIC_OSGEARTH"] = self.options.shared
        # cmake.definitions["PROTOBUF_USE_DLLS"] = self.options["protobuf"].shared
        cmake.definitions["BUILD_OSGEARTH_EXAMPLES"] = False
        cmake.definitions["BUILD_APPLICATIONS"] = False
        cmake.definitions["BUILD_TESTS"] = False
        cmake.definitions["CURL_INCLUDE_DIR"] = self.deps_cpp_info["libcurl"].include_paths[0]
        cmake.definitions["CURL_LIBRARY"] = os.path.join(self.deps_cpp_info["libcurl"].lib_paths[0],
                                                         self.deps_cpp_info["libcurl"].libs[0]) + '.lib'
        print("CURL_LIBRARY = " + cmake.definitions["CURL_LIBRARY"])
        cmake.definitions["OSG_DIR"] = osgDir
        cmake.definitions["GDAL_DIR"] = gdalDir
        cmake.configure()
        return cmake

    def build(self):
        cmake = self._cmake_configure()
        cmake.build()

    def package(self):
        cmake = self._cmake_configure()
        cmake.install()
        if os.path.exists(os.path.join(self.package_folder, "lib64")):
            os.rename(os.path.join(self.package_folder, "lib64"), os.path.join(self.package_folder, "lib"))

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
        self.env_info.PATH.append(os.path.join(self.package_folder, "bin"))
