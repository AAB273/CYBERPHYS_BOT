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
    uint8_t Result;
    Result = ~(P4->IN);
    Result = ((Result >> 2) & 0x38) | ((Result >> 1) & 0x06) | ((Result) & 0x01); //Shifts all the bits to 5,4,3,2,1,0 position (6 LSB)
    return Result;
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

