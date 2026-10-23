#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "cpptest.hpp" 


namespace py = pybind11;


/**
 * @brief bindings for instance of template function test_sum
 *   Since C++ templates require the types to be determined at compile time, we must manually bind for each 
 *   desired type.
 */
template <typename T>
void bind_test_sum(py::module& m, const char* name) {
    m.def(name, &test_sum<T>, "Sum elements of a vector", py::arg("vec"));
}


/**
 * @brief bindings for the class Color<int>
 *   due to there is instance of template class Color<int> in cpptest.cpp, binding that concrete class
 */
void bind_color_int(const py::module &m) {
    py::class_<Color<int>>(m, "ColorInt", "RGB Color class with integer components")
        .def(py::init<>(), "Default constructor (black)")
        .def(py::init<int, int, int>(), "Constructor with r, g, b values", 
             py::arg("r"), py::arg("g"), py::arg("b"))
        .def("set", &Color<int>::set, "Set RGB values", 
             py::arg("r"), py::arg("g"), py::arg("b"))
        .def("print", &Color<int>::print, "Print RGB values to stdout")
        .def("components", &Color<int>::components, "Return RGB as a tuple (r, g, b)");
}


PYBIND11_MODULE(fcpp_python, m) { 
    m.doc() = "Python bindings for fcpp test utilities (Person, Color, Eigen, Zlib)";

    // binding for general functions 
    m.def("test_hello", &test_hello, "Prints 'CPP Compiler is ready!'");
    m.def("test_eigen", &test_eigen, "Tests Eigen matrix operations and prints result");
    m.def("test_cpp_zlib", &test_cpp_zlib, "Tests zlib compression/decompression");

    // bindings for template functions
    bind_test_sum<int>(m, "test_sum_int");
    bind_test_sum<float>(m, "test_sum_float");
    bind_test_sum<double>(m, "test_sum_double");

    // bindings for general class (Person)
    py::class_<Person>(m, "Person", "A simple person class")
        .def(py::init<std::string, int>(), "Construct a person with name and age", 
             py::arg("name"), py::arg("age"))
        .def("greet", &Person::greet, "Return a greeting string")
        // bindings for public members 
        .def_readwrite("name", &Person::name, "Person's name")
        .def_readwrite("age", &Person::age, "Person's age");

    // bindings for template class
    bind_color_int(m);

    // Optional: the API version
    m.attr("__version__") = "1.0.0";
}
