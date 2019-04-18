#pragma once

/**
 * Converts between Frenet lateral position and lane id.
 */
class LaneConverter
{
public:

    /**
     * Converts Frenet lateral position to lane id.
     * @param d Frenet lateral position
     * @return lane id
     */
    static int d_to_lane(double d);

    /**
     * Converts lane id to Frenet lateral position.
     * @param lane lane id
     * @return Frenet lateral position
     */
    static double lane_to_d(int lane);
};

