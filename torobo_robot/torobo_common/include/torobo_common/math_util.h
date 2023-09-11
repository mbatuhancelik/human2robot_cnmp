/**
 * @file  math_util.h
 * @brief math util
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_COMMON_MATH_UTIL_H
#define TOROBO_COMMON_MATH_UTIL_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <iostream>
#include <cmath>
#include <float.h>


namespace math
{
    double square(double x);

    // ref: ROS ecl_math (https://github.com/stonier/ecl_core) and FMath
    bool nearly_equal(const double x, const double y, const double epsilon = DBL_EPSILON);
    bool nearly_equal_or_less_than(const double x, const double y, const double epsilon = DBL_EPSILON);
    bool nearly_equal_or_in_range(const double lower, const double upper, const double x, const double epsilon = DBL_EPSILON);
}


#endif /* TOROBO_COMMON_MATH_UTIL_H */
