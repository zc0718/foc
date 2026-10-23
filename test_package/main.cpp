#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include <foc/core/types.hpp>
#include <foc/version.h>
#include <foc/core/version.hpp>



namespace {

/**
 * @brief [en] Demo: print the packaged resource file, proving RESOURCES_PATH plumbing works.
 *        [zh] 演示：打印打包的资源文件，验证 RESOURCES_PATH 通路可用。
 */
int print_example_text() {
    const std::string filepath = "./resources/example.txt";
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << filepath << std::endl;
        return 1;
    }
    std::cout << "=== Content of " << filepath << " ===" << std::endl;
    std::string line;
    while (std::getline(file, line)) {
        std::cout << line << std::endl;
    }
    return 0;
}

} // namespace



int main() {
    int failures = 0;

    // C path
    const char* c_version = foc_version_string();
    if (c_version == nullptr || std::strcmp(c_version, "0.1.0") != 0) {
        std::cerr << "FAIL: foc_version_string() mismatch" << std::endl;
        ++failures;
    } else {
        std::cout << "foc version (C): " << c_version << std::endl;
    }

    // C++ path
    const char* cpp_version = foc::core::version_string();
    if (cpp_version == nullptr || std::strcmp(cpp_version, c_version) != 0) {
        std::cerr << "FAIL: foc::core::version_string() mismatch" << std::endl;
        ++failures;
    } else {
        std::cout << "foc version (C++): " << cpp_version << std::endl;
    }

    // POD types: default construction must be zero and layout must stay aggregate
    const foc::core::Dq<float> dq{};
    const foc::core::Ab<float> ab{};
    const foc::core::Uvw<float> uvw{};
    if (dq.d != 0.0F || dq.q != 0.0F || ab.alpha != 0.0F || ab.beta != 0.0F || uvw.u != 0.0F || uvw.v != 0.0F ||
        uvw.w != 0.0F) {
        std::cerr << "FAIL: POD default construction is not zero" << std::endl;
        ++failures;
    }

    failures += print_example_text();

    if (failures != 0) {
        std::cerr << "CPP Compiler is ready with " << failures << " failure(s)!" << std::endl;
        return 1;
    }
    std::cout << "CPP Compiler is ready!" << std::endl;
    return 0;
}
