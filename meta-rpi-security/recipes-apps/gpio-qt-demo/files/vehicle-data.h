#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <cstdint>

/*
 * Common vehicle data structure.
 *
 * This structure is shared conceptually between:
 * Keep this structure POD/plain-data.
 * Do NOT use QString, std::string, pointers, etc.
 */

struct VehicleData
{
    // ========================================================
    // Speed / engine
    // ========================================================

    float speed;
    float rpm;

    // ========================================================
    // Battery
    // ========================================================

    float batteryPercent;
    float batteryVoltage;
    float batteryCurrent;

    // ========================================================
    // Distance
    // ========================================================

    float tripA;
    float tripB;
    float odometer;

    // ========================================================
    // Statistics
    // ========================================================

    float averageSpeed;
    float maxSpeed;
    float range;

    // ========================================================
    // Temperature
    // ========================================================

    float temperature;

    // ========================================================
    // Gear
    // ========================================================

    int32_t gear;

    /*
     * 0 = ECO
     * 1 = NORMAL
     * 2 = SPORT
     */
    int32_t driveMode;

    // ========================================================
    // Indicators / status
    // ========================================================

    uint8_t leftIndicator;
    uint8_t rightIndicator;

    uint8_t highBeam;
    uint8_t abs;
    uint8_t bluetooth;
};

#endif
