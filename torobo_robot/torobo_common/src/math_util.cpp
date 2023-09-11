/**
 * @file  math_utli.cpp
 * @brief math util
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "torobo_common/math_util.h"


using namespace std;

namespace math
{

double square(double x)
{
    return x * x;
}

bool nearly_equal(const double x, const double y, const double epsilon)
{
    return (fabs(x - y) <= epsilon * fmax(1.0f, fmax(fabs(x), fabs(y))));
}

bool nearly_equal_or_less_than(const double x, const double y, const double epsilon)
{
    return (x <= y) || nearly_equal(x, y, epsilon);
}

bool nearly_equal_or_in_range(const double lower, const double upper, const double x, const double epsilon)
{
    if (nearly_equal_or_less_than(lower, x, epsilon) && nearly_equal_or_less_than(x, upper, epsilon))
    {
        return true;
    }
    return false;
}

} /* namespace math */
