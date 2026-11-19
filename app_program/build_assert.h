/** @file build_assert.h
 * @brief Check integer constant expressions without C11 language features.
 * @details These declarations allocate no objects and generate no runtime work.
 */
#ifndef BUILD_ASSERT_H
#define BUILD_ASSERT_H

#include <stdbool.h>

/** @brief Reject a false or nonconstant condition during C99 compilation.
 * @param condition Integer constant expression that must evaluate as true.
 * @param assertion_type Unique lowercase typedef name ending in _t.
 * @note A bit-field width must be an integer constant expression. A negative
 * width rejects false conditions; an ordinary function cannot perform this
 * compile-time declaration. The macro evaluates neither argument at runtime.
 */
#define BUILD_ASSERT(condition, assertion_type)                                \
    typedef struct                                                             \
    {                                                                          \
        bool b_is_valid : ((condition) ? 1 : -1);                              \
    } assertion_type

#endif

/* End of file. */

