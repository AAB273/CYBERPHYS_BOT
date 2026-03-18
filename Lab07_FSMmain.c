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
#include "msp.h"
#include "../inc/clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Texas.h"

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
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[10]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center              &fsm[0]
#define slight_left         &fsm[1]
#define mid_left            &fsm[2]
#define mid_hard_left       &fsm[3]
#define hard_left           &fsm[4]
#define slight_right        &fsm[5]
#define mid_right           &fsm[6]
#define mid_hard_right      &fsm[7]
#define hard_right          &fsm[8] //9 total states for each pairing or edge case of light sensors
#define stop                &fsm[9] //in fsm example, we could change this to be continuing current state until path is found
// student starter code

State_t fsm[9]={
  {0x03, 500, { stop, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, center}},  // Center
  {0x02, 500, { stop, hard_right, mid_hard_right, mid_right, slight_right, center, center, mid_left, mid_hard_left, hard_left, slight_left}},  // slight left
  {0x01, 500, { stop, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, center, mid_hard_left, hard_left, mid_left}},   // mid left
  {0x00, 500, { stop, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, center, hard_left, mid_hard_left}}, //mid_hard left
  {0x00, 500, { stop, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, center, hard_left}}, //hard_left
  {0x00, 500, { stop, hard_right, mid_hard_right, mid_right, center, center, slight_left, mid_left, mid_hard_left, hard_left, slight_right}}, //slight_right
  {0x00, 500, { stop, hard_right, mid_hard_right, center, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, mid_right}}, //mid_right
  {0x00, 500, { stop, hard_right, center, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, mid_hard_right}}, //mid_hard_right
  {0x00, 500, { stop, center, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, hard_right}}, //hard_right
  {0x00, 500, { stop, hard_right, mid_hard_right, mid_right, slight_right, center, slight_left, mid_left, mid_hard_left, hard_left, stop}} //stop
};

uint8_t encode(uint8_t sensors) {
    switch(sensors) {
        case 0x00: return 0; // lost // could use this to stay in same state.
        case 0x01: return 1; //edge, off far left
        case 0x03: return 2; //mid_hard left
        case 0x06: return 3; //mid off left
        case 0x0C: return 4; //slight off left
        case 0x18: return 5; //centered
        case 0x30: return 6; //slight off right
        case 0x60: return 7; //mid off right
        case 0xC0: return 8; //mid hard off right
        case 0x80: return 9; //hard off right
        default:   return 10; // edge/error //continue with current state
    }
}

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
int main(void){ uint32_t heart=0;
  Clock_Init48MHz();
  LaunchPad_Init();
  TExaS_Init(LOGICANALYZER);  // optional
  Spt = Center;
  while(1){
    Output = Spt->out;            // set output from FSM
    LaunchPad_Output(Output);     // do output to two motors
    TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer
    Clock_Delay1ms(Spt->delay);   // wait
    Input = LaunchPad_Input();    // read sensors
    Spt = Spt->next[Input];       // next depends on input and state
    heart = heart^1;
    LaunchPad_LED(heart);         // optional, debugging heartbeat
  }
}

// Color    LED(s) Port2
// dark     ---    0
// red      R--    0x01
// blue     --B    0x04
// green    -G-    0x02
// yellow   RG-    0x03
// sky blue -GB    0x06
// white    RGB    0x07
// pink     R-B    0x05
