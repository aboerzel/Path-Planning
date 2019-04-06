#include "LaneDConverter.h"

int LaneDConverter::d_to_lane(const double d)
{
    // Check which lane the d-value comes from
    // Left is 0, middle is 1, right is 2

    if (d < 4)
    {
        return 0;
    }

    if (d < 8)
    {
        return 1;
    }

    return 2;
}

double LaneDConverter::lane_to_d(const int lane)
{
    return (lane * 4) + 2;
}
