// Conan::ImportStart
#pragma once
#include <vector>
#include <iostream>
#include <tuple>
#include <string>
// Conan::ImportEnd



void test_hello();



void test_eigen();



void test_cpp_zlib();



/**
 * @brief test class declaration
 * @exporter
 */
class Person {
  public:
    Person(std::string n, int a);
    std::string greet() const;
    std::string name;
    int age;
};



/**
 * @brief test template func in headers
 * @tparam T vector like
 * @param vec iterable thing of numbers
 * @return a numeric
 * @exporter
 */
template <typename T> auto test_sum(const std::vector<T>& vec) {
    T sum = T();
    for (const T& elem : vec) {
        sum += elem;
    }
    return sum;
};



/**
 * @brief simple RGB color template
 * @tparam T type trait
 * @exporter
 */
template <typename T> class Color {
  public:
    Color() = default;
    Color(T red_, T green_, T blue_) : r(red_), g(green_), b(blue_) {};
    void set(T red_, T green_, T blue_) {
        this->r = red_;
        this->g = green_;
        this->b = blue_;
    };
    void print() const { std::cout << "RGB(" << r << ", " << g << ", " << b << ")\n"; };
    auto components() const { return std::make_tuple(r, g, b); };

  private:
    T r{};
    T g{};
    T b{};
};



/**
 * @brief simple RGB int color template
 * @exporter
 */
template class Color<int>;