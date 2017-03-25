#include <Sonar.h>

#define DATA_PIN 12
Sonar sonar(DATA_PIN);

void setup() {
  Serial.begin(115200);
  sonar.reset();
}

void loop() {
  Serial.print("Num positions: ");
  Serial.println(sonar.numPositions());
  sonar.setNormalRange(0);
  Serial.println(sonar.hitDetected(0));
 }

