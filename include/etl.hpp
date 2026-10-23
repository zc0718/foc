// Conan::ImportStart
#pragma once
#include <cstdint>
// Conan::ImportEnd

/**
 * @brief Element-wise addition of two float arrays.
 *        Implemented internally with etl::transform + etl::plus.
 * @param a   Input array A
 * @param b   Input array B
 * @param y   Output array Y = A + B
 * @param n   Number of elements
 */
void fcpp_vec_add_f32(const float* a, const float* b, float* y, uint32_t n);

/**
 * @brief Element-wise subtraction of two float arrays.
 *        Implemented internally with etl::transform + etl::minus.
 * @param a   Input array A
 * @param b   Input array B
 * @param y   Output array Y = A - B
 * @param n   Number of elements
 */
void fcpp_vec_sub_f32(const float* a, const float* b, float* y, uint32_t n);
