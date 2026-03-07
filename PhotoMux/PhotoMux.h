#ifndef PHOTOSENSORSMUX_H
#define PHOTOSENSORSMUX_H

#include <Arduino.h>

// - Sides around the robot
enum Side { FRONT, LEFT, RIGHT, BACK };

class PhotoMux {
public:

    // - Sensor: mux index + analog channel
    struct Sensor {
        uint8_t muxIndex;   // - mux number
        uint8_t channel;    // - analog pin (A0..)
    };

    // - Constructor: store pin mappings
    PhotoMux(const uint8_t selectPins[3], const uint8_t muxPins[4]);

    // - init pins
    void begin();

    // - configure sensors for a side
    void configureSide(Side side, const Sensor* sensors, uint8_t count);
    // - set detection threshold
    void setThreshold(Side side, int threshold);

    // - detection helpers
    bool isLineDetected(Side side); // - compare avg to threshold
    float getAverage(Side side);    // - averaged reading
    float getRawAverage(Side side); // - raw averaged reading

private:
    uint8_t _selectPins[3];     // - mux select pins
    uint8_t _muxPins[4];        // - analog input pins

    int thresholds[4];          // - per-side thresholds

    // - sensor arrays per side
    Sensor* frontSensors;
    uint8_t frontCount;

    Sensor* leftSensors;
    uint8_t leftCount;

    Sensor* rightSensors;
    uint8_t rightCount;

    Sensor* backSensors;
    uint8_t backCount;

    // - low-level helpers
    void selectChannel(uint8_t channel); // - set mux selects
    int readSensor(uint8_t muxIndex, uint8_t channel); // - read one sensor
    float readAverage(const Sensor* sensors, uint8_t size); // - avg on array

    void readWithoutMux(const Sensor* sensors, uint8_t size, float& average, float& rawAverage); // - direct read

    /* How to use readWithoutMux:
    Sensors on A0, A1
    Sensor frontSensors[] = {
    {0, A0},
    {0, A1}
    };
    float frontAvg, frontRaw;
    readWithoutMux(frontSensors, 2, frontAvg, frontRaw);
    frontAvg now holds the average reading of the front sensors
    LEFT 
    Sensors on A2, A3
    Sensor leftSensors[] = {
    {0, A2},
    {0, A3}
    };
    float leftAvg, leftRaw;
    readWithoutMux(leftSensors, 2, leftAvg, leftRaw);
    */

#endif