// Example usage for DFROBOTPH library by DFROBOT.

#include "DFROBOTPH.h"

// Initialize objects from the lib
DFROBOTPH dFROBOTPH;

void setup() {
    // Call functions on initialized library objects that require hardware
    dFROBOTPH.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    dFROBOTPH.process();
}
