#pragma once

/**
 * Conversion of speeds between different speed units
 */
class SpeedConverter
{
public:
    /**
     * Converts a speed from MPS to m/s
     * @param speed Speed in MPS
     * @return Speed in m/s
     */
    static double miles_per_hour_to_km_per_sec(double speed)
    {
        return speed * 0.44704; // convert speed from MPS to m/s
    }

    /**
     * Converts a speed from m/s to MPS
     * @param speed Speed in m/s
     * @return Speed in MPS
     */
    static double km_per_sec_to_miles_per_hour(double speed)
    {
        return speed / 0.44704; // convert speed from  m/s to MPS
    }
};
