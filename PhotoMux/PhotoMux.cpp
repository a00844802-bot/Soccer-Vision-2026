#include "PhotoMux.h"
#include <Arduino.h>

// - Constructor: copy pin arrays and init defaults
PhotoMux::PhotoMux(const uint8_t selectPins[3], const uint8_t muxPins[4]) {
    memcpy(_selectPins, selectPins, 3);
    memcpy(_muxPins, muxPins, 4);

    thresholds[FRONT] = 2100; // - sample values
    thresholds[LEFT] = 1500;
    thresholds[RIGHT] = 2200;
    thresholds[BACK] = 3045;

    frontSensors = nullptr;
    leftSensors = nullptr;
    rightSensors = nullptr;
    backSensors = nullptr;

    frontCount = leftCount = rightCount = backCount = 0;
}

// - Setup mux select pins as outputs
void PhotoMux::begin() {
    for (int i = 0; i < 3; i++) {
        pinMode(_selectPins[i], OUTPUT);
    }
}

// - Store a copy of sensor array for a side
void PhotoMux::configureSide(Side side, const Sensor* sensors, uint8_t count) {
    Sensor* ptr = new Sensor[count];
    memcpy(ptr, sensors, count * sizeof(Sensor));

    switch (side) {
        case FRONT: frontSensors = ptr; frontCount = count; break;
        case LEFT:  leftSensors = ptr;  leftCount = count;  break;
        case RIGHT: rightSensors = ptr; rightCount = count; break;
        case BACK:  backSensors = ptr;  backCount = count;  break;
    }
}

// - Adjust threshold for side
void PhotoMux::setThreshold(Side side, int threshold) {
    thresholds[side] = threshold;
}

// - Set mux select lines for desired channel
void PhotoMux::selectChannel(uint8_t channel) {
    digitalWrite(_selectPins[0], channel & 0x01);
    digitalWrite(_selectPins[1], (channel >> 1) & 0x01);
    digitalWrite(_selectPins[2], (channel >> 2) & 0x01);
    delayMicroseconds(5); 
}

// - Read one analog sensor via mux
int PhotoMux::readSensor(uint8_t muxIndex, uint8_t channel) {
    selectChannel(channel);
    return analogRead(_muxPins[muxIndex]);
}

// - Average N sensor readings
float PhotoMux::readAverage(const Sensor* sensors, uint8_t size) {
    int sum = 0;
    for (uint8_t i = 0; i < size; i++) {
        sum += readSensor(sensors[i].muxIndex, sensors[i].channel);
    }
    return sum / float(size);
}

// - Get averaged value for a side
float PhotoMux::getAverage(Side side) {
    switch (side) {
        case FRONT: return readAverage(frontSensors, frontCount);
        case LEFT:  return readAverage(leftSensors, leftCount);
        case RIGHT: return readAverage(rightSensors, rightCount);
        case BACK:  return readAverage(backSensors, backCount);
        default:    return 0;
    }
}

// - Raw average (currently same as getAverage)
float PhotoMux::getRawAverage(Side side) {
    return getAverage(side);
}

// - Line detection: compare average to threshold
bool PhotoMux::isLineDetected(Side side) {
    return getAverage(side) > thresholds[side];
}

// - Read sensors directly (no mux) and compute average
void PhotoMux::readWithoutMux(const Sensor* sensors, uint8_t size, float& average, float& rawAverage){
    int sum = 0;
    for (uint8_t i = 0; i < size; i++) {
        sum += analogRead(sensors[i].channel); 
    }
    average = sum / float(size);
    rawAverage = average; 
}