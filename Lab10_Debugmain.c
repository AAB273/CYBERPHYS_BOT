
// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"

volatile uint8_t LineData  = 0;
volatile uint8_t BumpData  = 0;
volatile uint8_t DataReady = 0;


void Debug_Init(void){
  // write this as part of Lab 10
}
void Debug_Dump(uint8_t x, uint8_t y){
  // write this as part of Lab 10
}
void Debug_FlashInit(void){ 
  // write this as part of Lab 10
}
void Debug_FlashRecord(uint16_t *pt){
  // write this as part of Lab 10
}
void SysTick_Handler(void){ // every 1ms
    static uint8_t tickCount = 0;  // counts 0..9 then wraps

    if(tickCount == 0){
        Reflectance_Start();            // turn on LEDs, pulse high, switch to input
    }

    else if(tickCount == 1){
        // get sensor data
        LineData  = Reflectance_End();  // read sensors
        BumpData  = Bump_Read();        // read bump switches (positive logic)
        DataReady = 1;

    }

    tickCount = (tickCount + 1) % 10;
}


int main(void){
    Clock_Init48MHz();
    LaunchPad_Init();       // P1, P2 LEDs and switches

    Reflectance_Init();
    Bump_Init();

    SysTick_Init(48000,5);

    EnableInterrupts();

    while(1){
        //-------------------------
        // Add main stuff here
        //------------------------
    }

}

