#include "Arduino.h"
#include "NewPing.h"
#define NUM_POSITIONS 12 //30 degrees for each move
#define EPSILON 10 //3 inches is the max wiggle room before detection

class Sonar {
public:
    /* Init a new Sonar instance. Use like this: Sonar instance(pinNum);
     * pin: We use the same pin for trigger and echo
     */
    Sonar(int pin);

    /* Destructor cleans up at the end
     */
    ~Sonar();

    /* Returns bool indicating if a hit is detected at position, after doing a read and comparing previous
     * read for this position
     */
    bool hitDetected(int position, int* oldDist, int* newDist);

    /* The first time through the rotation, we need to set a normal range to compare against later
     * to detect a hit
     */
    void setNormalRange(int position, int* dist);

    /* Sets all positions to 0 distance.
     * Call this after creating a Sonar object, or when reset is pressed
     */
    void reset();

    /* Returns the number of positions possible. Changed with a #define above
     */
    int numPositions();

    /* Gets the last measured distance at position. For debugging
     */
    unsigned long getDistanceAt(int position);
private:
    unsigned long memory [NUM_POSITIONS];
    NewPing *ping;
};
