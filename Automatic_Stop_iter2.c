// main.c
// Runs on MSP432
// Robot drives forward continuously, stopping if any OPT3101
// channel detects an object within 200mm threshold.
// Uses busy-wait polling for distance sensor.

#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/BumpInt.h"
#include "../inc/UART0.h"

// -------------------------
// Constants
// -------------------------
#define DISTANCE_THRESHOLD_MM   150
#define DRIVE_DUTY              3750    // ~25% of 14998 max — tune as needed
#define INVALID_DISTANCE_MIN    65530   // sentinel values from OPT3101 driver

typedef enum {
    STATE_FIND_WALL,
    STATE_FOLLOW_WALL,
    STATE_EMERGENCY
} RobotState;


// -------------------------
// Globals
// -------------------------
uint32_t Distances[3];       // raw distances per channel (mm)
uint32_t Amplitudes[3];      // signal amplitudes per channel
uint32_t TxChannel;          // most recently updated channel (0, 1, or 2)



#define MAXPWM 7000
#define MINPWM 1000
#define EMERGENCY  100                   // minimum distance from wall without emergency handling
#define DESIREDRANGE 150                       //maximimum distance from wall without wall searching
uint16_t ActualL;                        // actual rotations per minute
uint16_t ActualR;                        // actual rotations per minute
uint16_t LeftDuty = 3000;                // duty cycle of left wheel (0 to 14,998)
uint16_t RightDuty = 3000;               // duty cycle of right wheel (0 to 14,998)

int32_t Error_L;    //proportional error based on distance and desired
int32_t Error_R;
int32_t Kp=10;  //proportional gain
int32_t t;
int32_t UR, UL;  // PWM duty bounded between 2000 and 7000

//averager if needed
uint16_t avg(uint16_t *array, int length)
{
  int i;
  uint32_t sum = 0;

  for(i=0; i<length; i=i+1)
  {
    sum = sum + array[i];
  }
  return (sum/length);
}

// Returns 0 if left wall is closer/valid, 2 if right wall is closer/valid, 1 if neither
uint32_t getActiveWallChannel(void) {
    uint32_t dL = Distances[0];
    uint32_t dR = Distances[2];

    bool leftValid  = (dL > 0 && dL < INVALID_DISTANCE_MIN && dL > 100 && dL <= 300);
    bool rightValid = (dR > 0 && dR < INVALID_DISTANCE_MIN && dR > 100 && dR <= 300);

    if (leftValid && rightValid)
        return (dL <= dR) ? 0 : 2;   // follow whichever is closer
    if (leftValid)  return 0;
    if (rightValid) return 2;
    return 1;   // sentinel: no valid wall
}

volatile uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}

// -------------------------
// Helper: poll sensor, update Distances[] for whichever channel fired
// Returns true if a new measurement was ready
// -------------------------
bool pollDistanceSensor(uint32_t *channel)
{
    if (OPT3101_CheckDistanceSensor())
    {
        *channel = OPT3101_GetMeasurement(Distances, Amplitudes);
        return true;
    }
    return false;
}

// -------------------------
// Helper: returns true if ANY channel reads a valid distance
// that is at or below the threshold
// -------------------------
bool objectWithinThreshold(void)
{
        uint32_t d = Distances[1];

        // Ignore uninitialized (0) and OPT3101 sentinel error values
        if (d == 0 || d >= INVALID_DISTANCE_MIN)
            return false;

        if (d <= DISTANCE_THRESHOLD_MM)
            return true;
        else{return false;}
}

//function to notify if we are in wall following range
bool WithinRange(void){
    uint32_t ch;
    uint32_t d;
    for(ch = 0 ; ch < 3; ch++){
        if(ch == 1)
            continue;
        d = Distances[ch];
        if((d < 300) && (d > 100)){
            return true;
        }
    }
    return false;
}

//function telling us if we are too close to walls
bool EmergencyRange(void) {
    uint32_t ch;
    for (ch = 0; ch < 3; ch++) {
        uint32_t d = Distances[ch];
        if (d > 0 && d < INVALID_DISTANCE_MIN && d < EMERGENCY)
            return true;
    }
    return false;
}

// -------------------------
// Main
// -------------------------
void main(void)
{
    uint32_t channel = 0;   // start on channel 0
    //bool driving = false;

    Clock_Init48MHz();

    // SysTick free-running for timing reference
//    SysTick->LOAD = 0x00FFFFFF;
//    SysTick->CTRL = 0x00000005;

    // Peripherals
    I2CB1_Init(30);          // 12 MHz / 30 = 400 kHz I2C
    UART0_Init();
    Motor_Init();
    BumpInt_Init(&HandleCollision);

    //Start motors with nominal values
    UR = RightDuty;
    UL = LeftDuty;

    // Initialise all distance slots to a safe "no reading" value
    Distances[0] = Distances[1] = Distances[2] = 65535;
    Amplitudes[0] = Amplitudes[1] = Amplitudes[2] = 0;

    // OPT3101 startup sequence
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();

    // Kick off first measurement
    OPT3101_StartMeasurementChannel(channel);

    UART0_OutString("\n\rObstacle-avoidance robot starting...\n\r");
    RobotState state = STATE_FIND_WALL;


    while (1) {
        Motor_Stop();
        CollisionFlag = 0;

        while (!CollisionFlag)
        {
            // ---- Poll sensor every iteration ----
            if (pollDistanceSensor(&channel)) {
                uint32_t nextChannel = (channel + 1) % 3;
                OPT3101_StartMeasurementChannel(nextChannel);

                UART0_OutString("Ch="); UART0_OutUDec(channel);
                UART0_OutString(" D=");  UART0_OutUDec(Distances[channel]);
                UART0_OutString("mm\n\r");
            }

            // ---- State transitions ----
            if (EmergencyRange()) {
                state = STATE_EMERGENCY;
            } else if (getActiveWallChannel() != 1) {
                state = STATE_FOLLOW_WALL;
            } else {
                state = STATE_FIND_WALL;
            }

            // ---- State actions ----
            switch (state) {

                case STATE_FIND_WALL:
                    // No wall in range: drive forward at nominal duty
                    Motor_Forward(LeftDuty, RightDuty);
                    break;

                case STATE_FOLLOW_WALL: {
                    uint32_t wallCh  = getActiveWallChannel();
                    int32_t measured = (int32_t)Distances[wallCh];
                    int32_t desired  = 300;

                    // Error: positive = too far, negative = too close
                    int32_t error = measured - desired;

                    if (wallCh == 0) {
                        // Left wall: too far  turn left (slow left, fast right)
                        //            too close  turn right
                        UL = LeftDuty  - Kp * error;
                        UR = RightDuty + Kp * error;
                    } else {
                        // Right wall: too far  turn right (fast left, slow right)
                        //             too close  turn left
                        UL = LeftDuty  + Kp * error;
                        UR = RightDuty - Kp * error;
                    }

                    // Clamp
                    if (UL > MAXPWM) UL = MAXPWM;
                    if (UL < MINPWM) UL = MINPWM;
                    if (UR > MAXPWM) UR = MAXPWM;
                    if (UR < MINPWM) UR = MINPWM;

                    Motor_Forward((uint16_t)UL, (uint16_t)UR);
                    break;
                }

                case STATE_EMERGENCY: {

                    // Back away from whichever side is closest
                    if (Distances[0] < Distances[2]) {
                        // Left is closer  back up curving right
                        Motor_Backward(MINPWM, MAXPWM);

                    } else {
                        // Right is closer  back up curving left
                        Motor_Backward(MAXPWM, MINPWM);
                    }
                    Clock_Delay1ms(175);
                    state = STATE_FIND_WALL;  // re-evaluate next iteration
                    break;
                }
            }
        }

    }

}
