// Lab10_Debugmain.c
// Runs on MSP432
// Student version to Debug lab
// Daniel and Jonathan Valvano
// September 4, 2017
// Interrupt interface for QTRX reflectance sensor array
// Pololu part number 3672.
// Debugging dump, and Flash black box recorder

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


// red R-- 0x01
// blue --B 0x04
// green -G- 0x02
// yellow RG- 0x03
// sky blue -GB 0x06
// white RGB 0x07
// pink R-B 0x05


#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"

volatile int semaphore = 0, systick_count = 0;
volatile uint8_t bump_values, sensors;

void Debug_Init(void){
    Clock_Init48MHz();      // running on crystal
    SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
    LaunchPad_Init();       // P1.0 is red LED on LaunchPad
    Reflectance_Init();
    Bump_Init();
}

void SysTick_Handler(void){ // every 1ms
//    if (systick_count == 10){
//        Reflectance_Start();
//        systick_count++;
//    }
//    else if (systick_count == 11){
//        uint8_t i;
//        sensors = Reflectance_End();
          bump_values = Bump_Read();
          semaphore = 1;
//        for (i = 0; i <= 7; i++){
//            if(sensors == (1 << i)) {
//                P2->OUT = (P2->OUT & 0xF8) | i;  // Keeps bits 8-3 then it only alters the LED's output bits 0-3.
//            } //This allows for the sensors to be tied to each consecutive LED state, with RED being 0x01 to White being 0x07
//            if(sensors == 0x01){
//                P1->OUT |= 0x01;
//            }
//            else{
//                P1->OUT &= ~0x01;
//            }
//        }
            if(bump_values == 1) {P2->OUT = 0x01;}
            else if(bump_values == 2) {P2->OUT = 0x02;}
            else if(bump_values == 4) {P2->OUT = 0x03;}
            else if(bump_values == 8) {P2->OUT = 0x04;}
            else if(bump_values == 16) {P2->OUT = 0x05;}
            else if(bump_values == 32) {P2->OUT = 0x06;}
            else {P2->OUT = 0x00;
//     systick_count = 0;
//    }
//    else{
          semaphore = 0;
//        systick_count++;
//    }
    // The commented out code is the RGB LED mapping for the sensor.
    }
}

int main(void){
    Debug_Init();
    EnableInterrupts();
    WaitForInterrupt();
  while(1){
      uint32_t i, j, temp, length;
              uint32_t a[100]={5000,5308,5614,5918,6219,6514,
              6804,7086,7361,7626,7880,8123,8354,8572,8776,8964,9137,
              9294,9434,9556,9660,9746,9813,9861,9890,9900,9890,9861,
              9813,9746,9660,9556,9434,9294,9137,8964,8776,8572,8354,
              8123,7880,7626, 7361,7086,6804,6514,6219,5918,5614,
              5308,5000,4692,4386,4082,3781,3486,3196,2914,2639,2374,
              2120,1877,1646,1428,1224,1036,863,706,566,444,340,254,
              187,139,110,100,110,139,187,254,340,444,566,706,863,
              1036, 1224, 1428, 1646,1877,2120,2374,2639,2914,
              3196,3486,3781,4082,4386,4692};

              length = 100;

              for (i = 0; i < length; i++){
                  for (j = 0; j < length - i - 1; j++){

                     if (a[j + 1] < a[j]){
                          temp = a[j];
                          a[j] = a[j + 1];
                          a[j + 1] = temp;
                     }
                  }
              }

          }
}
