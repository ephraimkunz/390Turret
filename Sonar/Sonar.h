#include "Arduino.h"
#include "NewPing.h"
#define NUM_POSITIONS 12 //30 degrees for each move
#define EPSILON 3 //3 inches is the max wiggle room before detection

class Sonar {
    public:
        Sonar(int pin);
        bool hitDetected(int position);
        void setNormalRange(int position);
        void reset();
        int numPositions();
    private:
        unsigned long [NUM_POSITIONS] memory;
        NewPing newPing;
}