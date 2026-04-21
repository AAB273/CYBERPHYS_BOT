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
#include "../inc/Bump.h"
#include "../inc/UART0.h"

// -------------------------
// Constants
// -------------------------
#define DISTANCE_THRESHOLD_MM   150
#define DRIVE_DUTY              3750    // ~25% of 14998 max — tune as needed
#define INVALID_DISTANCE_MIN    65530   // sentinel values from OPT3101 driver

// -------------------------
// Globals
// -------------------------
uint32_t Distances[3];       // raw distances per channel (mm)
uint32_t Amplitudes[3];      // signal amplitudes per channel
uint32_t TxChannel;          // most recently updated channel (0, 1, or 2)

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

#define MAXPWM 7000
#define MIN PWM 2000
#define DESIREDMIN  200                   // minimum distance from wall without emergency handling
#define DESIREMAX 600                       //maximimum distance from wall without wall searching
uint16_t ActualL;                        // actual rotations per minute
uint16_t ActualR;                        // actual rotations per minute
uint16_t LeftDuty = 5000;                // duty cycle of left wheel (0 to 14,998)
uint16_t RightDuty = 5000;               // duty cycle of right wheel (0 to 14,998)

int32_t Error_L;    //proportional error based on distance and desired
int32_t Error_R;
int32_t Kp=8;  //proportional gain
int32_t t;
int32_t UR, UL;  // PWM duty bounded between 2000 and 7000
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
        if((d < 600) && (d > 200)){
            return true;
        }
    }
    return false;
}

//function telling us if we are too close to walls
bool EmergencyRange(void){
        uint32_t ch;
        uint32_t d;
        for(ch = 0 ; ch < 3; ch++){
            d = Distances[ch];
            if(d < 150){
                return true;
            }
        }
        return false;
}

// -------------------------
// Main
// -------------------------
void main(void)
{
    uint32_t channel = 0;   // start on channel 0
    bool driving = false;

    Clock_Init48MHz();

    // SysTick free-running for timing reference
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;

    // Peripherals
    I2CB1_Init(30);          // 12 MHz / 30 = 400 kHz I2C
    UART0_Init();
    Motor_Init();
    Bump_Init();

    //Start motors with nominal values
    UR = RightDuty;
    UL = LeftDuty;

    // Initialise all distance slots to a safe "no reading" value
    Distances[0] = Distances[1] = Distances[2] = 0;
    Amplitudes[0] = Amplitudes[1] = Amplitudes[2] = 0;

    // OPT3101 startup sequence
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();

    // Kick off first measurement
    OPT3101_StartMeasurementChannel(channel);

    UART0_OutString("\n\rObstacle-avoidance robot starting...\n\r");
    while(1){
        Motor_Stop();

        while (Bump_Read() == 0) //handle bumps outside of main loop
        {
            //pseudocode:
            //based on polling sensors execute different behavior:
            //if outside desired range from wall either sensors, continue driving forward at set speed
            //if wall found (200mm - 600mm) enter wall following protocol
            //if too close (<200mm) execute emergency stop turns.

            // ----  Poll distance sensor ----
            if (pollDistanceSensor(&channel))
            {
                // Debug output over UART
                UART0_OutString("Ch=");
                UART0_OutUDec(channel);
                UART0_OutString(" Dist=");
                UART0_OutUDec(Distances[channel]);
                UART0_OutString("mm\n\r");

                // Advance to next channel for the following measurement
                uint32_t nextChannel = (channel + 1) % 3;
                OPT3101_StartMeasurementChannel(nextChannel);
            }

            //if in too close of distance-> execute this loop
            if(EmergencyRange()){

            }














//
//            // ---- 3. Motor decision: forward unless too close ---- //now instantiated only to be channel 1 at 150mm
//            if (objectWithinThreshold())
//            {
//                if (driving)
//                {
//                    Motor_Stop();
//                   //stop motor only if we are at a dead end (comparatively to middle channel)
//                    driving = false;
//                    UART0_OutString("Object < 150mm — STOPPED\n\r");
//                }
//            }
//            else
//            {
//                if (!driving)
//                {
//                    Motor_Forward(DRIVE_DUTY, DRIVE_DUTY);
//                    driving = true;
//                    UART0_OutString("Path clear — FORWARD\n\r");
//                }
//            }
        }
    }
}
