from conan import ConanFile


class Pathfinder(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    requires = [
        "eigen/3.4.0",
        "fmt/11.0.2",
    ]
    generators = "CMakeDeps", "CMakeToolchain"
