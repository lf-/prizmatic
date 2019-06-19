#include <prizmatic.hpp>

PRIZMatic prizm{};

void setup() {
#pragma region setup
    Serial.begin(9600);
    delay(200);
    prizm.dump_eeprom_steps();
    prizm.PrizmBegin();
#pragma endregion

    // write your code here
}

void loop() {}