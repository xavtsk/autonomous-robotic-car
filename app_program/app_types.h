/** @file app_types.h
 * @brief Name the application's existing floating-point representations.
 * @details These aliases retain the compiler's float and double types. They do
 * not quantize values, change precision, or introduce conversion operations.
 */
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <float.h>
#include <limits.h>
#include <stdint.h>
#include "build_assert.h"

/** @brief The existing binary32 computation and storage type. */
typedef float float32_t;

/** @brief The existing binary64 odometry and computation type. */
typedef double float64_t;

/** @brief Access a binary32 value or its unchanged bits for explicit packing.
 */
typedef union
{
    float32_t value;
    uint32_t  bits;
} app_types_binary32_view_t;

/** @brief Access a binary64 value or its unchanged bits for explicit packing.
 */
typedef union
{
    float64_t value;
    uint64_t  bits;
} app_types_binary64_view_t;

BUILD_ASSERT((CHAR_BIT == 8), app_types_byte_t);
BUILD_ASSERT((sizeof(float32_t) == 4U) && (FLT_RADIX == 2) &&
                 (FLT_MANT_DIG == 24) && (FLT_MAX_EXP == 128),
             app_types_binary_single_t);
BUILD_ASSERT((sizeof(float64_t) == 8U) && (FLT_RADIX == 2) &&
                 (DBL_MANT_DIG == 53) && (DBL_MAX_EXP == 1024),
             app_types_binary_double_t);

#endif

/* End of file. */

