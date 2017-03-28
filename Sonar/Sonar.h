#include "Arduino.h"
#include "NewPing.h"
#define NUM_POSITIONS 12 //30 degrees for each move
#define EPSILON 3 //3 inches is the max wiggle room before detection

/*
You might need to adjust the sensitivity of the hit detection. To do this, adjust EPSILON to a higher value. Alternativly,
adjust the number of hits for median up from 5 or the max distance in the constructor to be less (if we have issues at
higher distances).
*/

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
    bool hitDetected(int position);

    /* The first time through the rotation, we need to set a normal range to compare against later
     * to detect a hit
     */
    void setNormalRange(int position);

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
