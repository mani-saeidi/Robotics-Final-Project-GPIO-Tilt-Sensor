/**
 * @file Final_Project_main.c
 *
 * @brief Main source code for the Final Project.
 *
 * @author Mani Saeidi & Matthew Gordon
 *
 */


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/Timer_A0_PWM.h"

#include "../inc/Motor.h"
// Constant definitions for the built-in red LED
const uint8_t RED_LED_OFF           =   0x00;
const uint8_t RED_LED_ON            =   0x01;

// Constant definitions for the RGB LED colors
const uint8_t RGB_LED_OFF           =   0x00;
const uint8_t RGB_LED_RED           =   0x01;
const uint8_t RGB_LED_GREEN         =   0x02;
const uint8_t RGB_LED_YELLOW        =   0x03;
const uint8_t RGB_LED_BLUE          =   0x04;
const uint8_t RGB_LED_PINK          =   0x05;
const uint8_t RGB_LED_SKY_BLUE      =   0x06;
const uint8_t RGB_LED_WHITE         =   0x07;

// Constant definitions for the PMOD 8LD module
const uint8_t PMOD_8LD_ALL_OFF      =   0x00;
const uint8_t PMOD_8LD_ALL_ON       =   0xFF;
//const uint8_t PMOD_8LD_0_3_ON       =   0x0F;
//Lab task1 test case 0
const uint8_t PMOD_8LD_0_3_ON       =   0x55;
//const uint8_t PMOD_8LD_4_7_ON       =   0xF0;
//Lab task1 test case 1
const uint8_t PMOD_8LD_4_7_ON       =   0xAA;

//FOR LEDS 0,2,4,6   0x01 + 0x04 + 0x10 + 0x40  = 0x55
//FOR LEDS 1,3,5,7   0x02 + 0x08 + 0x20 + 0x80  = 0xAA

//MATTI GAEIDI  VARIABLES LED PATTERN 5
const uint8_t PMOD_8LD_0_ON       =   0x01;
const uint8_t PMOD_8LD_1_ON       =   0x02;
const uint8_t PMOD_8LD_2_ON       =   0x04;
const uint8_t PMOD_8LD_3_ON       =   0x08;
const uint8_t PMOD_8LD_4_ON       =   0x10;
const uint8_t PMOD_8LD_5_ON       =   0x20;
const uint8_t PMOD_8LD_6_ON       =   0x40;
const uint8_t PMOD_8LD_7_ON       =   0x80;




/**
 * @brief The LED1_Init function initializes the built-in red LED (P1.0).
 *
 * This function initializes the built-in red LED located at pin P1.0
 * and configures it as a GPIO pin. It sets the direction of the pin as output.
 *
 * @param None
 *
 * @return None
 */
void LED1_Init()
{
    P1->SEL0 &= ~0x01;
    P1->SEL1 &= ~0x01;
    P1->DIR |= 0x01;
}

/**
 * @brief The LED1_Output function sets the output of the built-in red LED and returns the status.
 *
 * This function sets the output of the built-in red LED based on the value of the input, led_value.
 * A bitwise AND operation (& 0xFE) is performed to mask the first bit (LSB) of the output register
 * to preserve the state of other pins connected to Port 1 while keeping the LED pin unaffected.
 * Then, a bitwise OR operation is performed with led_value to set the LED pin to the desired state
 * specified by led_value.
 *
 * @param led_value An 8-bit unsigned integer that determines the output of the built-in red LED. To turn off
 *                  the LED, set led_value to 0. Otherwise, setting led_value to 1 turns on the LED.
 *
 * @return Indicates the status of the LED.
 *         - 0: LED Off
 *         - 1: LED On
 */
uint8_t LED1_Output(uint8_t led_value)
{
    P1->OUT = (P1->OUT & 0xFE) | led_value;
    return ((P1->OUT != 0) ? 1 : 0);
}

/**
 * @brief The LED2_Init function initializes the RGB LED (P2.0 - P2.2).
 *
 * This function initializes the following RGB LED, configures the pins as GPIO pins with high drive strength,
 * and sets the direction of the pins as output. The RGB LED is off by default upon initialization.
 *  - RGBLED_RED      (P2.0)
 *  - RGBLED_GREEN    (P2.1)
 *  - RGBLED_BLUE     (P2.2)
 *
 * @param None
 *
 * @return None
 */
void LED2_Init()
{
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DS |= 0x07;
    P2->DIR |= 0x07;
    P2->OUT &= ~0x07;
}

/**
 * @brief The LED2_Output function sets the output of the RGB LED and returns the status.
 *
 * This function sets the output of the RGB LED based on the value of the input, led_value.
 * A bitwise AND operation (& 0xF8) is performed to mask the lower three bits of the output register
 * to preserve the state of other pins connected to Port 2 while keeping the RGB LED pin unaffected.
 * Then, a bitwise OR operation is performed with led_value to set the RGB LED pin to the desired state
 * specified by led_value.
 *
 * @param led_value An 8-bit unsigned integer that determines the output of the RGB LED. To turn off
 *                  the RGB LED, set led_value to 0. The following values determine the color of the RGB LED:
 *
 *  Color       LED(s)   led_value
 *  Off         ---         0x00
 *  Red         R--         0x01
 *  Green       -G-         0x02
 *  Yellow      RG-         0x03
 *  Blue        --B         0x04
 *  Pink        R-B         0x05
 *  Sky Blue    -GB         0x06
 *  White       RGB         0x07
 *
 * @return Indicates the status of the RGB LED.
 *          - 0: RGB LED Off
 *          - 1: RGB LED On
 */
uint8_t LED2_Output(uint8_t led_value)
{
    P2->OUT = (P2->OUT & 0xF8) | led_value;
    return ((P2->OUT != 0) ? 1 : 0);
}

/**
 * @brief The Tilt_Init() function initializes the Tilt Sensors (P4.0, P4.2, P4.4 & P4.5)
 *
 * This function initializes the Tilt Sensors, configures the pins as GPIO pins with low drive strength,
 * and sets the direction of the pins as input.
 *
 * @param None
 *
 * @return None
 */
void Tilt_Init()
{
    P4->SEL0 &= ~0x35;
    P4->SEL1 &= ~0x35;
    P4->REN |= 0x35;
    P4->DIR &= ~0x35;
    P4->OUT &= ~0x35;
}

/**
 * @brief The Tilt_Status function gets the current pins that are active low.
 *
 * 
 *
 * @param None
 *
 * @return Pins of Tilt Sensors experiencing Tilt (Active Low)
 */
uint8_t TILT_Status()
{
    uint8_t incerjeet = P4->IN & 0x35;
    return incerjeet;
}

int main(void)
{
    // Initialize the 48 MHz Clock and Motor
    Clock_Init48MHz();
    Motor_Init();

    // Initialize the built-in red LED and the RGB LEDs
    LED1_Init();
    LED2_Init();

    // Initialize the PMOD 8LD module
    Tilt_Init();

    while(1)
    {
        uint8_t incerjeet = TILT_Status();
        printf("%d\n",incerjeet);
        Clock_Delay1ms(1000);
        switch(incerjeet){
            case 0x00:
                // All LEDS on - Negative Logic
                LED2_Output(RGB_LED_SKY_BLUE);
                Motor_Forward(1000,1000);
                break;
            case 0x04:
                // LED 2 OFF - Warning
                LED2_Output(RGB_LED_YELLOW);
                break;
            case 0x05:
                // LED 1 & 2 ARE OFF and LED 3 & 4 ARE ON
                LED2_Output(RGB_LED_RED);
                Motor_Backward(1000,1000);
                break;
            case 0x20:
                // LED 4 is OFF
                LED2_Output(RGB_LED_YELLOW);
                Motor_Forward(1000,1000);
                break;
            case 0x30:
                // LED 3 and 4 are off
                LED2_Output(RGB_LED_RED);
                Motor_Backward(1000,1000);
                Clock_Delay1ms(3000);
                Motor_Stop();
                Clock_Delay1ms(2500);
                break;
            default:
                LED2_Output(RGB_LED_OFF);
                Motor_Stop();
                break;
        }
    }
}






