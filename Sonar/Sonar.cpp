#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar(int pin) {
    this.newPing = NewPing(pin, pin);
}

bool Sonar::hitDetected(int position) {
    bool hitDetected = false;
    usigned long median = this.newPing.ping_median();
    
    if(abs(memory[position] - median) > EPSILON) {
        hitDetected = true;
    }
    memory[position] = median;
    return hitDetected;
}

void setNormalRange(int position) {
    unsigned long median = this.newPing.ping_median();
    memory[position] = median;
}

void reset() {
    for(int i = 0; i < NUM_POSITIONS; ++i) {
        memory[i] = 0;
    }
}

int numPositions() {
    return NUM_POSITIONS;
}