// Lab07_FSMmain.c
// Runs on MSP432
// Student version of FSM lab, FSM with 2 inputs and 2 outputs.
// Rather than real sensors and motors, it uses LaunchPad I/O
// Daniel and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Clock.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Bump.h"
#include "..\inc\Motor.h"

/*(Left,Right) Motors, call LaunchPad_Output (positive logic)
3   1,1     both motors, yellow means go straight
2   1,0     left motor,  green  means turns right
1   0,1     right motor, red    means turn left
0   0,0     both off,    dark   means stop
(Left,Right) Sensors, call LaunchPad_Input (positive logic)
3   1,1     both buttons pushed means on line,
2   1,0     SW2 pushed          means off to right
1   0,1     SW1 pushed          means off to left
0   0,0     neither button      means lost
 */

// Linked data structure
struct State {
  uint16_t left_duty;                // left duty cycle
  uint16_t right_duty;               //right duty cycle
  const struct State *next[11];      // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define center              &fsm[0]
#define slight_left         &fsm[1]
#define mid_left            &fsm[2]
#define mid_hard_left       &fsm[3]
#define hard_left           &fsm[4]
#define slight_right        &fsm[5]
#define mid_right           &fsm[6]
#define mid_hard_right      &fsm[7]
#define hard_right          &fsm[8] //9 total states for each pairing or edge case of light sensors
// student starter code

State_t fsm[9]={
//left right
  {2500, 2500, { hard_right, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, center}},  // Center
//left turns
  {2750, 2250, { hard_left, hard_right, mid_hard_right, mid_right, slight_right, center, center, mid_left, mid_hard_left, hard_left, slight_left}},  // slight left
  {3250, 1750, { hard_left, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, center, mid_hard_left, hard_left, mid_left}},   // mid left
  {3500, 1000, { hard_left, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, center, hard_left, mid_hard_left}}, //mid_hard left
  {4000, 500, { hard_left, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, center, hard_left}}, //hard_left
//right turns
  {2250, 2750, { hard_right, hard_right, mid_hard_right, mid_right, center, center, slight_left, mid_left, mid_hard_left, hard_left, slight_right}}, //slight_right
  {1750, 3250, { hard_right, hard_right, mid_hard_right, center, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, mid_right}}, //mid_right
  {1000, 3500, { hard_right, hard_right, center, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, mid_hard_right}}, //mid_hard_right
  {500, 4000, { hard_right, center, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, hard_right}}  //hard_right
};

uint8_t encode(uint8_t sensors) {
    switch(sensors) {
        // --- Core clean readings ---
        case 0x00: return 0;  // lost
        case 0x01: return 1;  // hard left edge
        case 0x03: return 2;  // mid_hard left
        case 0x06: return 3;  // mid left
        case 0x0C: return 4;  // slight left
        case 0x18: return 5;  // centered
        case 0x30: return 6;  // slight right
        case 0x60: return 7;  // mid right
        case 0xC0: return 8;  // mid_hard right
        case 0x80: return 9;  // hard right edge

        // --- 3-sensor transitions (line crossing between two clean positions) ---
        case 0x07: return 2;  // 0000 0111 -> mid_hard left  (between hard and mid left)
        case 0x0E: return 3;  // 0000 1110 -> mid left       (between mid_hard and slight left)
        case 0x1C: return 4;  // 0001 1100 -> slight left    (between mid left and center)
        case 0x38: return 5;  // 0011 1000 -> centered       (wide center read)
        case 0x70: return 6;  // 0111 0000 -> slight right   (between center and mid right)
        case 0xE0: return 7;  // 1110 0000 -> mid right      (between slight and mid_hard right)

        // --- Single sensor isolated reads (noise or sharp edge) ---
        case 0x02: return 2;  // single bit near left  -> mid_hard left
        case 0x04: return 3;  // single bit mid-left   -> mid left
        case 0x08: return 4;  // single bit center-left -> slight left
        case 0x10: return 5;  // single bit center     -> centered
        case 0x20: return 6;  // single bit center-right -> slight right
        case 0x40: return 7;  // single bit mid-right  -> mid right

        // --- Wide/saturated reads (robot very straight, wide line or glare) ---
        case 0x3C: return 5;  // 0011 1100 -> centered (4 sensors, well centered)
        case 0x7E: return 5;  // 0111 1110 -> centered (6 sensors, fully on line)
        case 0xFF: return 5;  // all sensors -> centered (intersection or very wide line)

        // --- Partial wide left reads ---
        case 0x0F: return 3;  // 0000 1111 -> mid left
        case 0x1F: return 4;  // 0001 1111 -> slight left
        case 0x3F: return 5;  // 0011 1111 -> slight left bias, treat as center

        // --- Partial wide right reads ---
        case 0xF0: return 7;  // 1111 0000 -> mid right
        case 0xF8: return 8;  // 1111 1000 -> mid_hard right
        case 0xFC: return 5;  // 1111 1100 -> slight right bias, treat as center

        default:   return 10; // truly unrecognized, hold current state
    }
}

State_t *Spt;  // pointer to the current state
uint32_t reflectance_result = 0;
uint32_t Input;
uint32_t Output;

volatile uint8_t LineData  = 0;
volatile uint8_t BumpData  = 0;
volatile uint8_t DataReady = 0;

/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */

//insert handler here
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
    Motor_Init();
    LaunchPad_Init();       // P1, P2 LEDs and switches
    Reflectance_Init();
    Bump_Init();
    SysTick_Init(48000,5);
    EnableInterrupts();
    Spt = center;
    while(1){
        if(DataReady == 1){
            DataReady = 0;
            uint16_t leftD = Spt->left_duty;
            uint16_t rightD = Spt->right_duty;
            Motor_Forward(leftD, rightD); //output to motors
            uint8_t input = encode(LineData);
            if(input != 10){
                Spt = Spt->next[input];} //ignore noisey/invalid input
        }
    }
}
