#pragma once

/**
 * Conversion of speeds between different speed units
 */
class SpeedConverter
{
    static const constexpr double factor = 0.44704; // conversion factor between m/s and MPS

public:
    /**
     * Converts a speed from MPS to m/s
     * @param speed Speed in MPS
     * @return Speed in m/s
     */
    static double miles_per_hour_to_km_per_sec(const double speed)
    {
        return speed * factor; // convert speed from MPS to m/s
    }

    /**
     * Converts a speed from m/s to MPS
     * @param speed Speed in m/s
     * @return Speed in MPS
     */
    static double km_per_sec_to_miles_per_hour(const double speed)
    {
        return speed / factor; // convert speed from  m/s to MPS
    }
};
