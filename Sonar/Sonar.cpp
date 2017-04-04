#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar(int pinNum) {
    ping = new NewPing(pinNum, pinNum);
}

Sonar::~Sonar(){
    delete ping;
}

bool Sonar::hitDetected(int position, int* oldDist, int* newDist) {
    bool hitDetected = false;
    unsigned long median = ping->ping_median(20);
	unsigned oldMedian = ping->convert_in(memory[position]);
	unsigned newMedian = ping->convert_in(median);
	*oldDist = oldMedian;
	*newDist = newMedian;
	//Don't use abs here, because we are dealing with unsigned types.
    if((max(oldMedian, newMedian) - min(oldMedian, newMedian)) > EPSILON) {
        hitDetected = true;
    }
    memory[position] = median;
    return hitDetected;
}

void Sonar::setNormalRange(int position, int* dist) {
    unsigned long median = ping->ping_median(20);
	*dist =   ping->convert_in(median);
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
