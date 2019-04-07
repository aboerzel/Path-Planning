#pragma once
/**
 * Converts between d position in frenet coordinates and lane id.
 */
class LaneConverter
{
public:

    /**
     * Converts d position in frenet coordinates to lane id.
     * @param d d position in frenet coordinates
     * @return lane id
     */
    static int d_to_lane(double d);

    /**
     * Converts lane id to d position in frenet coordinates.
     * @param lane lane id
     * @return d position in frenet coordinates
     */
    static double lane_to_d(int lane);
};
