#include "Arduino.h"
#include "mongoose_startup.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(50);

  ethernet_init();
  mongoose_init();
}

void loop() {
  mongoose_poll();
}
