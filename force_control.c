#include "msp430fr5739.h"
#include <stdlib.h>
// Define the data packet start byte
#define START_BYTE 255
// ADC channels for GPIO analog inputs (P1.2 -> A2, P1.3 -> A3)
#define ADC_PIN1_CHANNEL ADC10INCH_4
#define ADC_PIN2_CHANNEL ADC10INCH_3
#define ADC_PIN3_CHANNEL ADC10INCH_2
#define ADC_PIN4_CHANNEL ADC10INCH_5
// GPIO binary input (P3.0)
#define BINARY_INPUT_PORT P3IN
#define BINARY_INPUT_PIN BIT0
unsigned int adc_value1, adc_value2; // Store raw 10-bit ADC results
unsigned char forcevalue, currentsensorvalue = 0; // Store 8-bit ADC results
volatile unsigned char transmit = 0; // Flag to trigger data transmission
volatile unsigned char current_pin = 0; // Track which pin is currently
volatile unsigned int force_input_voltage;
volatile unsigned int input_current;
volatile unsigned int curr_digital;
volatile unsigned int amplified_curr;
// PI control variables
volatile signed int error; // Proportional error
volatile signed int integral_error = 0; // Accumulated integral error
const float Kp = 10; // Proportional gain
const float Ki = 1.0; // Integral gain
const unsigned int max_integral = 1000; // Maximum integral value to prevent
windup
// Define Saturation as a scaling factor to map from 0–255 to 0–2000
const float Saturation = 2000.0f / 255.0f;
#define VoltagetoCurrentGain 1
#define MaxCurr 1
// Function Prototypes
void configure_ADC10();
void configure_analog_pins();
void configure_binary_input();
void configure_UART();
void configure_timer_b0();
void configure_timer_b();
void configure_clocks();
void configure_gpio_3_6_and_3_7();
unsigned char sampleADC(unsigned int channel);
void control_dc_motor(unsigned int duty_cycle, unsigned char direction);
void transmit_uart(unsigned int data);
// Function to configure binary input pin (P3.0 as input)
void configure_binary_input() {
 P3DIR &= ~BINARY_INPUT_PIN; // Set P3.0 as input
 P3REN |= BINARY_INPUT_PIN; // Enable pull-up/down resistor
 P3OUT |= BINARY_INPUT_PIN; // Configure pull-up resistor
}
// Function to configure ADC for sampling GPIO analog inputs
void configure_ADC10() {
 ADC10CTL0 = ADC10SHT_2 | ADC10ON; // Sample and hold time = 16 ADC10CLK cycles,
ADC on
 ADC10CTL1 = ADC10SHP | ADC10SSEL_3; // Use sampling timer, SMCLK source
 ADC10CTL2 = ADC10RES; // 10-bit resolution
}
// Function to configure GPIO pins as analog inputs
void configure_analog_pins()
{ P1SEL1 |= BIT2 | BIT3 | BIT4 | BIT5; // Configure P1.4 and P1.3, P1.5 as analog
inputs
P1SEL0 |= BIT2 | BIT3 | BIT4 | BIT5; }
// Function to sample ADC for a given channel
unsigned char sampleADC(unsigned int channel) {
 ADC10CTL0 &= ~ADC10ENC; // Disable ADC before changing channels
 ADC10MCTL0 = channel; // Select the channel (e.g., A2 for P1.2, A3
for P1.3)
 ADC10CTL0 |= ADC10ENC | ADC10SC; // Enable and start conversion
 while (ADC10CTL1 & ADC10BUSY); // Wait until conversion completes
 return ADC10MEM0 >> 2; // Return 8-bit result (shifted 10-bit)
}
// UART Configuration
void configure_UART() {
 // Select SMCLK for UART and configure UART pins
 P2SEL0 &= ~(BIT5 | BIT6);
 P2SEL1 |= (BIT5 | BIT6);
 UCA1CTLW0 |= UCSWRST; // Put UART in reset mode
 UCA1CTLW0 |= UCSSEL__SMCLK; // Use SMCLK (1 MHz after division)
 UCA1BRW = 104; // Set baud rate for 9600 (SMCLK 1 MHz)
 UCA1MCTLW = 0xD600; // Set modulation UCBRSx=0xD6, UCOS16=1
 UCA1CTLW0 &= ~UCSWRST; // Release UART from reset
}
// Configure Timer B0 to periodically trigger the encoder count transmission
void configure_timer_b0() {
 TB0CCTL0 = CCIE; // Enable interrupt for CCR0
 TB0CCR0 = 40000 - 1; // Set interval (e.g., 8000 for 10 ms with
SMCLK = 1 MHz)
 TB0CTL = TBSSEL_2 | MC_1 | TBCLR; // SMCLK, up mode, clear timer
}
// Timer B Configuration for DC Motor PWM (TB2.1 -> P2.1)
void configure_timer_b() {
 P2DIR |= BIT1; // Set P2.1 as output for Timer B (TB2.1)
 P2SEL1 &= ~BIT1; // Clear P2SEL1 to select Timer B functionality
 P2SEL0 |= BIT1; // Set P2SEL0 to select TB2.1 (PWM output)
 // Stop Timer B2 during setup
 TB2CTL = TBSSEL_2 | MC_0 | TBCLR; // SMCLK as clock source, stop mode, clear
timer
 // Configure Timer B2 for PWM (500 Hz)
 TB2CCR0 = 2000 - 1; // Set the PWM period for 500 Hz (SMCLK 1 MHz / 2000)
 TB2CCTL1 = OUTMOD_7; // Set/reset output mode (PWM mode)
 TB2CCR1 = 1000; // Set initial duty cycle to 50% (1000/2000)
 // Start Timer B2 in up mode
 TB2CTL = TBSSEL_2 | MC_1 | TBCLR; // SMCLK as clock source, up mode
}
// Clock Configuration (8 MHz DCO, SMCLK 1 MHz)
void configure_clocks() {
 CSCTL0 = CSKEY;
 CSCTL1 |= DCOFSEL_3; // DCO frequency select: 8 MHz
 CSCTL2 |= SELS__DCOCLK; // DCO as the source for SMCLK
 CSCTL3 |= DIVS__2; // SMCLK divider 1
}
// GPIO Configuration for H-Bridge Direction Control (P3.6 and P3.7)
void configure_gpio_3_6_and_3_7() {
 P3DIR |= BIT6 | BIT7; // Set P3.6 and P3.7 as outputs
 P3OUT &= ~BIT6; // Set P3.6 high
 P3OUT |= BIT7; // Set P3.7 low
}
// UART Transmission Function
void transmit_uart(unsigned int data) {
 while (!(UCTXIFG)); // Wait until the transmit buffer is ready
 UCA1TXBUF = data; // Transmit data
}
// Control DC Motor using received duty cycle and direction
void control_dc_motor(unsigned int duty_cycle, unsigned char direction) {
 TB2CCR1 = duty_cycle; // Set PWM duty cycle for DC motor speed
 if (direction == 1) {
 P3OUT &= ~BIT6; // Set P3.6 high
 P3OUT |= BIT7; // Set P3.7 low (forward direction)
 } else {
 P3OUT |= BIT6; // Set P3.6 low
 P3OUT &= ~BIT7; // Set P3.7 high (reverse direction)
 }
}
// Timer B0 Interrupt ISR for periodic UART transmission
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void) {
 forcevalue = sampleADC(ADC_PIN1_CHANNEL); // Sample P1.2 (A2)
 currentsensorvalue = sampleADC(ADC_PIN2_CHANNEL); // Sample P1.3 (A3)
 TB0CCTL0 &= ~CCIFG; // Clear interrupt flag
}
// Main program
int main(void) {
 WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
 configure_clocks(); // Configure clock settings
 configure_UART(); // Configure UART
 configure_timer_b0(); // Configure Timer B for periodic transmission
 configure_timer_b(); // Configure Timer B for PWM (DC motor control)
 configure_gpio_3_6_and_3_7(); // Configure GPIO for H-Bridge (motor direction
control)
 configure_analog_pins(); // Configure P1.2 and P1.3 as analog inputs
 configure_ADC10(); // Set up ADC for GPIO inputs
 configure_binary_input(); // Configure P3.0 as binary input
 __bis_SR_register(GIE); // Enable global interrupts
 while (1) {
 if (BINARY_INPUT_PORT & BINARY_INPUT_PIN) {
 // Binary input HIGH
 force_input_voltage = forcevalue * 3.22 / 256;
 input_current = force_input_voltage * VoltagetoCurrentGain;
 error = input_current - currentsensorvalue * VoltagetoCurrentGain;
 integral_error += error;
 if (integral_error > max_integral) {
 integral_error = max_integral;
 } else if (integral_error < -max_integral) {
 integral_error = -max_integral;
 }
 // Calculate control output (PI controller)
 amplified_curr = (error * Kp) + (integral_error * Ki);
 // Scale amplified_curr to duty cycle
 duty_cycle = (unsigned int)abs(amplified_curr * Saturation);
 // Cap the duty cycle to 2000 if it exceeds the limit
 if (duty_cycle > 1999) {
 duty_cycle = 1999;
 }
 // Set escape_byte based on the sign of the error
 escape_byte = (error > 0) ? 1 : 0;
 // Control motor speed and direction
 control_dc_motor(duty_cycle, escape_byte);
 } else {
 // Binary input LOW: Reset force value and integral term
 forcevalue = 0;
 integral_error = 0; // Reset integral error
 }
 }
}
