from conans import ConanFile, CMake
from conans.tools import os_info
import configparser


class Eth3devaluationConan(ConanFile):

    lib_version = "1.0.0"
    revision = "3"
    name = "eth3d-evaluation"
    version = "{}-{}".format(lib_version, revision)
    license = "https://github.com/ETH3D/multi-view-evaluation/blob/master/LICENSE.txt"
    url = "https://github.com/Pix4D/multi-view-evaluation.git"
    description = "ETH3D multiview benchmark evaluation app"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "lockfile": "ANY"}
    default_options = {"shared": False, "lockfile": None}
    generators = "cmake"
    short_paths = True
    export_sources = ["cmake/*", "CMakeLists.txt", "src/*"]

    def requirements(self):
        self.requires("Eigen3/[3.4.0-2]@pix4d/stable")
        self.requires("boost/[1.73.0-0]@pix4d/stable")
        self.requires("nanoflann/[1.4.3-3]@pix4d/stable")
        self.requires("tinyply/[2.3.3-0]@pix4d/stable")

    def imports(self):
        if os_info.is_windows:
            destination_folders = "conanLibs", "bin"
            for lib_dir in destination_folders:
                self.copy("*.dll", src="lib", dst=lib_dir)
                self.copy("*.dll", src="bin", dst=lib_dir)

        if os_info.is_linux:
            self.copy("*.so*", src="lib", dst="conanLibs")

        if os_info.is_macos:
            lib_dir = "Frameworks"
            self.copy("*.dylib*", src="lib", dst=lib_dir)
            self.copy("*.framework/*", src="lib", dst=lib_dir)

    def build(self):
        cmake = CMake(self, generator="Ninja")
        cmake_args = {
            "BUILD_SHARED_LIBS": self.options.shared,
        }
        if self.settings.os == "Macos":
            cmake_args["CMAKE_MACOSX_RPATH"] = "ON"

        cmake.configure(source_dir=self.source_folder, defs=cmake_args)
        cmake.build(target="install")

    def package(self):
        if self.settings.os == "Windows":
            self.copy(
                "ETH3DMultiViewEvaluation.exe",
                dst="bin",
                src="Release",
                keep_path=False,
            )
        else:
            self.copy("ETH3DMultiViewEvaluation", dst="bin", keep_path=False)
