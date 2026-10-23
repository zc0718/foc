// Conan::ImportStart
#include <etl/algorithm.h>
#include <etl/functional.h>
#include "etl.hpp"
// Conan::ImportEnd

void fcpp_vec_add_f32(const float* a, const float* b, float* y, uint32_t n) {
    etl::transform(a, a + n, b, y, etl::plus<float>());
}

void fcpp_vec_sub_f32(const float* a, const float* b, float* y, uint32_t n) {
    etl::transform(a, a + n, b, y, etl::minus<float>());
}
