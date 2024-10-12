#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  Serial.println("Board has started!");
}

void loop()
{
  Serial.printf("Millis %lu\r\n");
  delay(1000);
}
