// Conan::ImportStart
#include <array>
#include <iostream>
#include <string>
#ifndef __ARM_EABI__
#include <zlib.h>
#include <Eigen/Dense>
#endif
#include <cpptest.hpp>
// Conan::ImportEnd



/**
 * @brief test function in cpp
 * @exporter
 */
void test_hello() { std::cout << "CPP Compiler is ready!" << '\n'; };



#ifndef __ARM_EABI__
/**
 * @brief test eigen in cpp
 * @exporter
 */
void test_eigen() {
    Eigen::Matrix3d A;
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "matrix A:\n" << A << "; Eigen Matrix test done!" << '\n';
}



/**
 * @brief test Person construction
 * @param n name
 * @param a age
 * @attacher
 */
Person::Person(std::string n, int a) : name(std::move(n)), age(a) {};



/**
 * @brief test Person meth 2
 * @return the hello message
 * @attacher
 */
std::string Person::greet() const { return "Hello, I'm " + name; };



/**
 * @brief zlib requirement test in CPP compiler
 * @exporter
 */
void test_cpp_zlib() {
    const char in[] = "Hello, zlib in CPP!";
    std::array<unsigned char, 128> out{};
    std::array<unsigned char, 128> rec{};
    auto len_out = static_cast<uLong>(out.size());
    auto len_rec = static_cast<uLong>(rec.size());
    if (auto compress_result =
            compress(out.data(), &len_out, reinterpret_cast<const Bytef*>(in), static_cast<uLong>(sizeof(in)));
        compress_result != Z_OK) {
        std::cerr << "Compression failed with error code: " << compress_result << '\n';
        return;
    }
    if (auto uncompress_result = uncompress(rec.data(), &len_rec, reinterpret_cast<const Bytef*>(out.data()), len_out);
        uncompress_result != Z_OK) {
        std::cerr << "Decompression failed with error code: " << uncompress_result << '\n';
        return;
    }
    std::cout << "Original: " << in << "; Decompressed: " << reinterpret_cast<char*>(rec.data())
              << "; zlib in C++ test done!" << '\n';
}
#endif /* __ARM_EABI__ */
