#include "include/tca_selector.h"
#include <Wire.h>

#define TCAADDR 0x70

void tcaselect(short i) {
    if (i > 7) return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}
