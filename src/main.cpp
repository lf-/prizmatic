#include <prizmatic.hpp>

PRIZMatic prizm{};

void setup() {
    delay(500);
    prizm.dump_eeprom_steps();
    prizm.PrizmBegin();
}

void loop() {}