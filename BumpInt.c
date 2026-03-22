// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"

//pointer to store the task
static void (*BumpTask)(uint8_t) = 0;

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)

void BumpInt_Init(void(*task)(uint8_t)){
    // Store the task function pointer in the global variable
    BumpTask = task;

    //configure pins as inputs with pull-ups
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;
    P4->DIR  &= ~0xED;   // Set bump pins as inputs
    P4->REN  |=  0xED;   // Enable resistors
    P4->OUT  |=  0xED;   // pull-UP resistors

    //interrupt for falling edge
    P4->IES |= 0xED;      //trigger interrupt on falling edge
    P4->IFG &= ~0xED;     // Clear any old interrupt flags
    P4->IE  |= 0xED;      // Enable interrupts on these pins

    NVIC_EnableIRQ(PORT4_IRQn);
}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    uint8_t portValue = P4->IN;
    uint8_t result = 0;

    if ((~portValue & 0x01) != 0) {
        result |= 0x01;  // set bit 0
    }

    if ((~portValue & 0x04) != 0) {  //bit 2
        result |= 0x02;  // set bit 1
    }

    if ((~portValue & 0x08) != 0) {  //bit 3
        result |= 0x04;  // set bit 2
    }

    if ((~portValue & 0x20) != 0) {  // bit 5
        result |= 0x08;  // set bit 3
    }

    if ((~portValue & 0x40) != 0) {  // bit 6
        result |= 0x10;  // set bit 4
    }

    if ((~portValue & 0x80) != 0) {  // bit 7
        result |= 0x20;  // set bit 5
    }

    return result;
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    uint8_t status = P4->IFG & 0xED;  // Read which pins triggered
    P4->IFG &= ~status;                  // Clear interrupt flags

    // Read the bump sensor state
    uint8_t bumpData = Bump_Read();

    //use stored function pointer to call task
    if(BumpTask != 0) {
        BumpTask(bumpData);
    }
}

