#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Clock.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Bump.h"
#include "../inc/BumpInt.h"
#include "..\inc\Motor.h"

volatile uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}


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
// left   right   [ 0=lost          1=h_l          2=mh_l          3=m_l        4=sl_l        5=ctr    6=sl_r         7=m_r        8=mh_r           9=h_r        10=err        ]
  {2250,  2250,   { center,         hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  center        }}, // center
  {2175,  2475,   { slight_left,    hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  slight_left   }}, // slight_left
  {1575,  2925,   { mid_left,       hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  mid_left      }}, // mid_left
  {900,   3150,   { mid_hard_left,  hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  mid_hard_left }}, // mid_hard_left
  {1575,  3150,   { hard_left,      hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  hard_left     }}, // hard_left
  {2475,  2175,   { slight_right,   hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  slight_right  }}, // slight_right
  {2925,  1575,   { mid_right,      hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  mid_right     }}, // mid_right
  {3150,  900,    { mid_hard_right, hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  mid_hard_right}}, // mid_hard_right
  {3150,  1575,   { hard_right,     hard_left,      mid_hard_left,  mid_left,    slight_left,  center,  slight_right,  mid_right,   mid_hard_right,  hard_right,  hard_right    }}  // hard_right
}; //1.5x speed

// PWM 3000,3000 90 Degree turn!

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
        case 0x38: return 6;  // 0011 1000 -> centered       (wide center read)
        case 0x70: return 7;  // 0111 0000 -> slight right   (between center and mid right)
        case 0xE0: return 8;  // 1110 0000 -> mid right      (between slight and mid_hard right)

        // --- Single sensor isolated reads (noise or sharp edge) ---
        case 0x02: return 2;  // single bit near left  -> mid_hard left
        case 0x04: return 3;  // single bit mid-left   -> mid left
        case 0x08: return 4;  // single bit center-left -> slight left
        case 0x10: return 6;  // single bit center     -> centered
        case 0x20: return 7;  // single bit center-right -> slight right
        case 0x40: return 8;  // single bit mid-right  -> mid right

        // --- Wide/saturated reads (robot very straight, wide line or glare) ---
        case 0x3C: return 5;  // 0011 1100 -> centered (4 sensors, well centered)
        case 0x7E: return 5;  // 0111 1110 -> centered (6 sensors, fully on line)
        case 0xFF: return 5;  // all sensors -> centered (intersection or very wide line)

        // --- Partial wide left reads ---
        case 0x0F: return 1;  // 0000 1111 -> mid left
        case 0x1F: return 1;  // 0001 1111 -> slight left
        case 0x3F: return 1;  // 0011 1111 -> slight left bias, treat as center

        // --- Partial wide right reads ---
        case 0xF0: return 9;  // 1111 0000 -> hard right
        case 0xF8: return 9;  // 1111 1000 -> mid_hard right
        case 0xFC: return 9;  // 1111 1100 -> slight right bias, treat as center

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
    tickCount = (tickCount + 1) % 7;
}

void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

int main(void){
    Clock_Init48MHz();
    CollisionFlag = 0;
    Motor_Init();
    LaunchPad_Init();       // P1, P2 LEDs and switches
    Reflectance_Init();
    BumpInt_Init(&HandleCollision);
    SysTick_Init(48000,5);
    EnableInterrupts();
    Spt = center;
    Motor_Stop();
    Pause();
    while(1){
        //WaitForInterrupt();
//        if(CollisionFlag){
//            Motor_Stop();
//            Clock_Delay1ms(10);          // 10ms pause
//            Motor_Backward(2000, 2000);
//            Clock_Delay1ms(10);          // 10ms backup
//            Motor_Stop();
//
//            switch(CollisionData){
//                case 0x01:  // Bump0 (P4.0 - right side)
//                    Clock_Delay1ms(10);
//                    Motor_Forward(2250, 0);
//                    Clock_Delay1ms(10);
//                    Motor_Stop();
//                    break;
//
//                case 0x02:  // Bump1 (P4.2)
//                    Clock_Delay1ms(10);
//                    Motor_Forward(5000, 0);
//                    Clock_Delay1ms(10);
//                    Motor_Stop();
//                    break;
//
//                case 0x04:  // Bump2 (P4.3)
//                    Clock_Delay1ms(10);
//                    Motor_Right(4000, 4000);
//                    Clock_Delay1ms(10);
//                    Motor_Stop();
//                    break;
//
//                case 0x08:  // Bump3 (P4.5)
//                    Clock_Delay1ms(10);
//                    Motor_Left(6000, 6000);
//                    Clock_Delay1ms(10);
//                    Motor_Stop();
//                    break;
//
//                case 0x10:  // Bump4 (P4.6)
//                    Clock_Delay1ms(10);
//                    Motor_Forward(0, 5000);
//                    Clock_Delay1ms(10);
//                    Motor_Stop();
//                    break;
//
//                case 0x20:  // Bump5 (P4.7 - left side)
//                    Clock_Delay1ms(10);
//                    Motor_Forward(0, 2250);
//                    Clock_Delay1ms(10);
//                    Motor_Stop();
//                    break;
//
//                default:
//                    Clock_Delay1ms(10);
//                    Motor_Backward(2000, 2000);
//                    Clock_Delay1ms(10);
//                    Motor_Stop();
//                    break;
//            }
//
//            CollisionFlag = 0;
//            Spt = center;
//        }
        if(DataReady){
            DataReady = 0;
            uint16_t leftD = Spt->left_duty;
            uint16_t rightD = Spt->right_duty;
            if (Spt == hard_left || Spt == mid_hard_left) {Motor_Left(leftD,rightD);}
            else if (Spt == hard_right || Spt == mid_hard_right) {Motor_Right(leftD,rightD);}
            else{Motor_Forward(leftD, rightD);} //output to motors
            uint8_t input = encode(LineData);
            if(input != 10){
                Spt = Spt->next[input];} //ignore noisy/invalid input
        }
    }
}
