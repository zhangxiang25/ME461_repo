//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
// ZHX EX4 We are going to play a long song in our code.
//#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t updown=1;
int16_t dancount=0;
float dancount2=0;
float dancount3=0;
int16_t notecount=0;
//ZHX EX2 predefinition for functions
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
//ZHX EX3 predefinition for functions
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);
//ZHX EX4 
#define C4NOTE ((uint16_t)(((50000000/2)/2)/261.63))
#define D4NOTE ((uint16_t)(((50000000/2)/2)/293.66))
#define E4NOTE ((uint16_t)(((50000000/2)/2)/329.63))
#define F4NOTE ((uint16_t)(((50000000/2)/2)/349.23))
#define G4NOTE ((uint16_t)(((50000000/2)/2)/392.00))
#define A4NOTE ((uint16_t)(((50000000/2)/2)/440.00))
#define B4NOTE ((uint16_t)(((50000000/2)/2)/493.88))
#define C5NOTE ((uint16_t)(((50000000/2)/2)/523.25))
#define D5NOTE ((uint16_t)(((50000000/2)/2)/587.33))
#define E5NOTE ((uint16_t)(((50000000/2)/2)/659.25))
#define F5NOTE ((uint16_t)(((50000000/2)/2)/698.46))
#define G5NOTE ((uint16_t)(((50000000/2)/2)/783.99))
#define A5NOTE ((uint16_t)(((50000000/2)/2)/880.00))
#define B5NOTE ((uint16_t)(((50000000/2)/2)/987.77))
#define E6NOTE ((uint16_t)(((50000000/2)/2)/1318.51))
#define F4SHARPNOTE ((uint16_t)(((50000000/2)/2)/369.99))
#define A4FLATNOTE ((uint16_t)(((50000000/2)/2)/415.3))
#define C5SHARPNOTE ((uint16_t)(((50000000/2)/2)/554.37))
#define D5SHARPNOTE ((uint16_t)(((50000000/2)/2)/622.25))
#define A5FLATNOTE ((uint16_t)(((50000000/2)/2)/830.61))
#define OFFNOTE 0
#define SONG_LENGTH 255
uint16_t songarray[SONG_LENGTH] = {
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
B4NOTE,
B4NOTE,
D5NOTE,
D5NOTE,
C5NOTE,
C5NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
OFFNOTE,//
C4NOTE,
C4NOTE,
E4NOTE,
E4NOTE,
A4NOTE,
A4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
OFFNOTE,
E4NOTE,
E4NOTE,
F4SHARPNOTE,
F4SHARPNOTE,
B4NOTE,
B4NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
E4NOTE,
E4NOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
B4NOTE,
B4NOTE,
D5NOTE,
D5NOTE,
C5NOTE,
C5NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
OFFNOTE,
C4NOTE,
C4NOTE,
E4NOTE,
E4NOTE,
A4NOTE,
A4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
OFFNOTE,
D4NOTE,
D4NOTE,
C5NOTE,
C5NOTE,
B4NOTE,
B4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
OFFNOTE, // PART A
B4NOTE,
B4NOTE,
C5NOTE,
C5NOTE,
D5NOTE,
D5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
G4NOTE,
G4NOTE,
F5NOTE,
F5NOTE,
E5NOTE,
E5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
F4NOTE,
F4NOTE,
E5NOTE,
E5NOTE,
D5NOTE,
D5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
E4NOTE,
E4NOTE,
D5NOTE,
D5NOTE,
C5NOTE,
C5NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
OFFNOTE, // PART B
E4NOTE,
E4NOTE,
E5NOTE,
E5NOTE,
E4NOTE,
E4NOTE,
E5NOTE,
E5NOTE,
OFFNOTE,
E5NOTE,
E5NOTE,
E6NOTE,
E6NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
B4NOTE,
B4NOTE,
D5NOTE,
D5NOTE,
C5NOTE,
C5NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
OFFNOTE,
C4NOTE,
C4NOTE,
E4NOTE,
E4NOTE,
A4NOTE,
A4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
OFFNOTE,
E4NOTE,
E4NOTE,
F4SHARPNOTE,
F4SHARPNOTE,
B4NOTE,
B4NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
OFFNOTE,
E4NOTE,
E4NOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
D5SHARPNOTE,
D5SHARPNOTE,
E5NOTE,
E5NOTE,
B4NOTE,
B4NOTE,
D5NOTE,
D5NOTE,
C5NOTE,
C5NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
OFFNOTE,
C4NOTE,
C4NOTE,
E4NOTE,
E4NOTE,
A4NOTE,
A4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
B4NOTE,
OFFNOTE,
D4NOTE,
D4NOTE,
C5NOTE,
C5NOTE,
B4NOTE,
B4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
};


void main(void)
{

    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
	//ZHX EX4 timer1 is called every 125 milliseconds
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    // ZHX EX1 set up the EPWM registers for EPWM12A
	// ZHX EX1 for the definition of TBCTL and AQCTLA,we can type EPwm12Regs then selecting"Open declaration" 
	EPwm12Regs.TBCTL.bit.CLKDIV=0; // set the CLKDIV be 1. CLKDIV takes 3 bits in TBCTL rigister the smallest number we could set is 0 and the largest number is 7(111 in decimal)
	EPwm12Regs.TBCTL.bit.PHSEN=0; // disable the phase loading which means do not load the TBCTR from the TBPHS(time base phase register)
	EPwm12Regs.TBCTL.bit.CTRMODE=0; // count up mode. CTRMODE takes 2 bits in TBCTL,down count mode is 1;up-down count mode is 2; freeze counter operation is 3
	EPwm12Regs.TBCTL.bit.FREE_SOFT=2; // free run so that the PWM continues when you set a break point in your code. FREE_SOFT takes 2 bits in TBCTL. 
	// 0 means stop after the next time -base counter increment or decrement.1 meansStop then counter completes a whole cycle
	EPwm12Regs.TBCTR=0; // time base counter register is 0. TBCTR is a 16-bit register
	EPwm12Regs.TBPRD=2500; // time base period register is 2500. 
	// we know the clock source has a frequency of 50MHz and we need the period of PWM signal be 20kHz. 20k*TBPRD=50M. TBPRD=2500

    EPwm12Regs.CMPA.bit.CMPA=0; // the duty cycle at beginning is 0%.CMPA(counter compare a register) determine the duty cycle

    EPwm12Regs.AQCTLA.bit.CAU=1; // when TBCTR=CMPA clear the signal pin. CAU takes up 2 bits of the AQCTLA register,4 values can be assigned to it.
// 0 is do nothing; 2 is set EPWMxA to high; 3 is toggle EPWNxA output
    EPwm12Regs.AQCTLA.bit.ZRO=2; // when TBCTR=0, make the pin be set, force output high. ZRO takes up 2 bits,4 values can be assigned to it.
// 0 is do nothing; 1 is clear(force EPWMxA output low; 3 is toogle EPWMxA output.

    EPwm12Regs.TBPHS.bit.TBPHS=0; // set the phase to 0. TBPHS(time base phase high) has two parts:TBPHS and TBPHSHR, each part had 16-bit.
// ZHX EX1 TBCTR counter with a 50MHz,CLKDIV is 5. After the divide, 50/(2^5)=50/32,so the period is 32/50M. TBPRD=39062, the period of PWm signal is 32/50M*39062
	
// ZHX EX1 EPWM2A and 2B drive the robot's DC motors.EPWM2A controls roght motor and EPWM2B controls left motor
    EPwm2Regs.TBCTL.bit.CLKDIV=0;
    EPwm2Regs.TBCTL.bit.PHSEN=0;
    EPwm2Regs.TBCTL.bit.CTRMODE=0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT=2;

    EPwm2Regs.TBCTR=0;

    EPwm2Regs.TBPRD=2500;

    EPwm2Regs.CMPA.bit.CMPA=0;
    EPwm2Regs.CMPB.bit.CMPB=0;

    EPwm2Regs.AQCTLA.bit.CAU=1;
    EPwm2Regs.AQCTLA.bit.ZRO=2;
// ZHX EX1 different with EPWM12A, EPWM2B had additional AQCTLB and CMPB.
    EPwm2Regs.AQCTLB.bit.CBU=1;
    EPwm2Regs.AQCTLB.bit.ZRO=2;

    EPwm2Regs.TBPHS.bit.TBPHS=0;

	// ZHX EX1 EPWM8A and 8B controls 2 rc servos. 
    EPwm8Regs.TBCTL.bit.CLKDIV=4; // ZHX EX3 the frequency of EPWM8 is 50MHz 50000000/2^4=3125000
    EPwm8Regs.TBCTL.bit.PHSEN=0;
    EPwm8Regs.TBCTL.bit.CTRMODE=0;
    EPwm8Regs.TBCTL.bit.FREE_SOFT=2;

    EPwm8Regs.TBCTR=0;
	// ZHX EX3 we want the servo carrier frequency be 50Hz 50*TBPRD=3125000,so TBPRD=62500
    EPwm8Regs.TBPRD=62500; // ZHX EX3 TBPRD is a 16-bit register,the largest number we can set is 2^16-1=65535

    EPwm8Regs.CMPA.bit.CMPA=5000;// ZHX EX3 the intialvalue of CMPA and CMPB is commanding the servo to 8% duty cycle 0.08*62500(TBPRD)=5000
    EPwm8Regs.CMPB.bit.CMPB=5000;

    EPwm8Regs.AQCTLA.bit.CAU=1;
    EPwm8Regs.AQCTLA.bit.ZRO=2;
    EPwm8Regs.AQCTLB.bit.CBU=1;
    EPwm8Regs.AQCTLB.bit.ZRO=2;

    EPwm8Regs.TBPHS.bit.TBPHS=0;

	// ZHX EX1 EPWM9A drives the buzzer

    EPwm9Regs.TBCTL.bit.CLKDIV=1;
    EPwm9Regs.TBCTL.bit.PHSEN=0;
    EPwm9Regs.TBCTL.bit.CTRMODE=0;
    EPwm9Regs.TBCTL.bit.FREE_SOFT=2;

    EPwm9Regs.TBCTR=0;

    EPwm9Regs.TBPRD=0;
	// ZHX EX4 in order to pruduce varied frequency signal,we comment out the intialization of CMPA rigister
    // EPwm9Regs.CMPA.bit.CMPA=0;

    EPwm9Regs.AQCTLA.bit.CAU=0; // ZHX EX4 when CMPA is reached, no action is needed
    EPwm9Regs.AQCTLA.bit.ZRO=3; // ZHX EX4 when TBCTR=0, set to 3 to toggle the LOW or HIGH output of the PMW. This is to show on the oscilliscope the operation signal

    EPwm9Regs.TBPHS.bit.TBPHS=0;

	//ZHX EX1 use the GPIO_SetupPinMux() function to change the pin output by using PinMux table
    GPIO_SetupPinMux(2,GPIO_MUX_CPU1,1); // EPWM2A is GPIO2
    GPIO_SetupPinMux(3,GPIO_MUX_CPU1,1); // EPWM2B is GPIO3
    GPIO_SetupPinMux(22,GPIO_MUX_CPU1,5);// EPWM12A is used  instead of GPIO22
    GPIO_SetupPinMux(14,GPIO_MUX_CPU1,1);// EPWM8A is GPIO14
    GPIO_SetupPinMux(15,GPIO_MUX_CPU1,1);// EPWM8B is GPIO15
    GPIO_SetupPinMux(16,GPIO_MUX_CPU1,5);// EPWM9A is GPIO16

    EALLOW; // Below are pretected register
// ZHX EX1 disable the pull-up resistor when an I/O pin is set as a PWM output pin
    GpioCtrlRegs.GPAPUD.bit.GPIO2=1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3=1;
    GpioCtrlRegs.GPAPUD.bit.GPIO14=1;
    GpioCtrlRegs.GPAPUD.bit.GPIO15=1;
    GpioCtrlRegs.GPAPUD.bit.GPIO16=1;
    GpioCtrlRegs.GPAPUD.bit.GPIO22=1;
    EDIS;


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
	init_serialSCIC(&SerialC,115200);
	init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%25) == 0) {
        // displayLEDletter(LEDdisplaynum);
	// ZHX EX1 comment out this code in order to see EPWM12A signal drives LED1 with 0% duty cycle(led should be off)
	// ZHX EX1 For chaging the CMPA register, in CCS View-Register-find the register you want->(expand).
	// ZHX EX! CMPA=TBPRD--100% duty cycle
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
// ZHX EX4 set TBPRD to the value currently stored in songarray(we are in the beginning of the song)
	EPwm9Regs.TBPRD=songarray[notecount];
	if (notecount<SONG_LENGTH){
	    notecount++; //ZHX EX4 increasing notecount to keep track where are we in the song
	}

	if (notecount==SONG_LENGTH){
	    GPIO_SetupPinMux(16,GPIO_MUX_CPU1,0);// ZHX EX4 when the song ended,change the pin from EPWM9A to GPIO16
	    GpioDataRegs.GPACLEAR.bit.GPIO16=1; // ZHX EX4 Set GPIO16 to low so the buzzer does not make any noise
	}
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{

	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
// ZHX EX1 when the updown is equal to 1,count up by 1. 2500 is the TBPRD, when dancount reach that value,switch counting and decrease the value by 1.
// In this way we can change the duty cycle from 0 to 100 then from 100 to 0.
    /*if(updown==1){ 
        dancount++; 
        if(dancount>=2500){ 
            updown=0; 
        }
        EPwm12Regs.CMPA.bit.CMPA=dancount;
    }
    else{
        dancount--;
        if(dancount<=0){
            updown=1;
        }
        EPwm12Regs.CMPA.bit.CMPA=dancount;
    }
// ZHX EX2 we create a global variable dancount2. when the updown is equal to 1,count up. 
// we gradually increase the dancount2 to 10 with step of 0.01 then switch to decreasing dancount2 to -10 then repeat.
    if (updown==1){
        dancount2 = dancount2+0.01;
	    if (dancount2>10) {
	        updown=0;
	    }
	    }
    else {
        dancount2= dancount2-0.01;
	    if (dancount2<-10) {
	        updown= 1;
	        }
	    }
	setEPWM2A(dancount2);
	setEPWM2B(dancount2);

	*/
	//ZHX EX3 dancount3 variable is for Servo motor,gradually change this value so the servo is driven back and forth
    if (updown==1){
            dancount3 = dancount3+0.05;
            if (dancount3>90) {
                updown=0;
            }
            }
        else {
            dancount3= dancount3-0.05;
            if (dancount3<-90) {
                updown= 1;
                }
            }

    setEPWM8A_RCServo(dancount3);
    setEPWM8B_RCServo(dancount3);

    CpuTimer2.InterruptCount++;
	
	if ((CpuTimer2.InterruptCount % 100) == 0) {
		UARTPrint = 1;
	}
}
// ZHX EX2 following 2 functions are going to saturate controleffort.If the value is greater thn 10, set it to 10;whe the value is lower than -10, set it to -10
// ZHX EX2 this function set EPWM2A to a duty cycle value related to the passed controleffort value
void setEPWM2A(float controleffort){
	if (controleffort>10){
		controleffort=10;
	}
	if (controleffort<-10){
		controleffort=10;
	}
	//ZHX EX2 when the control effort is -10, duty cycle is 0%;0 is 50% and 10 is 100%. CMPA&TBPRD are 16 bit integer and controleffort is a float, there is a type conversion.
	//ZHX EX2 duty cycle greater than 50% will cause the motor spin in postive direction, duty cycle less than 50% cause the motor spin in negative direction.
	EPwm2Regs.CMPA.bit.CMPA = (int16_t)((controleffort + 10)/20*((float)EPwm2Regs.TBPRD));
}
// ZHX EX2 similar to void setEPWM2A(float controleffort)
void setEPWM2B(float controleffort){
	if (controleffort>10){
		controleffort=10;
	}
	if (controleffort<-10){
		controleffort=10;
	}
	EPwm2Regs.CMPB.bit.CMPB = (int16_t)((controleffort + 10)/20*((float)EPwm2Regs.TBPRD));
}
// ZHX EX3 following two functions will firstly saturate angle between -90 to 90 in case the value outside this range, then find relationship between angle and CMPA.
void setEPWM8A_RCServo(float angle){
	if(angle > 90) {
		angle = 90;
	}
	if(angle < -90) {
		angle = -90;
	}
	// ZHX EX3 angle is a variable between -90 to 90.-90 equals 4% duty cycle, 0 equals 8%, 90 equals 12%
	EPwm8Regs.CMPA.bit.CMPA =(int16_t)((angle+180.0)/180.0*0.08*(float)(EPwm8Regs.TBPRD));
		}
//ZHX EX3 similar to void setEPWM8A_RCServo()
void setEPWM8B_RCServo(float angle){
	if(angle > 90) {
		angle = 90;
	}
	if(angle < -90) {
		angle = -90;
	}
	EPwm8Regs.CMPB.bit.CMPB =(int16_t)((angle+180.0)/180.0*0.08*(float)(EPwm8Regs.TBPRD));
		}
