// Conan::ImportStart
#include "doctest.hpp"
#include <iostream>
// Conan::ImportEnd



auto version_test_func() { std::cout << "the version_test_func function" << '\n'; };



void stage_a() {
    // Intentionally empty: staged demo placeholder, see the call graph in stage_d.
}



void stage_b() {
    // Intentionally empty: staged demo placeholder, see the call graph in stage_d.
}



void stage_c() { stage_b(); }



/**
 * @brief call relation demo
 * @note the call relationship can also be automatically calculated.
 * @ingroup demo
 */
void stage_d() {
    stage_a();
    stage_b();
    stage_c();
}
