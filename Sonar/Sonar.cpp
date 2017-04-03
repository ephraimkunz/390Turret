#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar(int pinNum) {
    ping = new NewPing(pinNum, pinNum);
}

Sonar::~Sonar(){
    delete ping;
}

bool Sonar::hitDetected(int position) {
    bool hitDetected = false;
    unsigned long median = ping->ping_median(20);

    if(abs(ping->convert_in(memory[position]) - ping->convert_in(median)) > EPSILON) {
        hitDetected = true;
    }
    memory[position] = median;
    return hitDetected;
}

void Sonar::setNormalRange(int position) {
    unsigned long median = ping->ping_median(20);
    memory[position] = median;
}

void Sonar::reset() {
    for(int i = 0; i < NUM_POSITIONS; ++i) {
        memory[i] = 0;
    }
}

unsigned long Sonar::getDistanceAt(int position){
    return ping->convert_in(memory[position]);
}

int Sonar::numPositions() {
    return NUM_POSITIONS;
}
