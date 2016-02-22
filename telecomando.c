// PROGRAM: Telecomando
// WRITTEN BY: Massimo Clementi
// DATA: 22/02/2016
// VERSION: 1.0
// FILE SAVED AS: telecomando.c
// FOR PIC: 18F4480
// CLOCK FREQUENCY: 16 MHz
// PROGRAM FUNCTION: misura, impacchettamento ed invio dei dati del joystick
// attraverso USART. Viene eseguita prima la misura di entrambi gli assi e
// e memorizzato il dato ottenuto in un array di signed char (-128;+127),
// successivamente inviato con questo ordine attraverso seriale.
// [Il codice commentato servirà in futuro ma ora non è utile]
//
//  Inputs and outputs: RA0 = acceleration axle (joystick)
//                      RA1 = steering axle (joystick)
//                      RC6/7 = USART

#define USE_AND_MASKS
#define _XTAL_FREQ 16000000
//#include <string.h>
//#include <stdio.h>
#include <xc.h>
#include "telecomando.h"
#include <usart.h>
#include "delay.h"
#include "delay.c"

//#define ACCELERATION_AXLE 0
//#define STEERING_AXLE 1
#define HIGH 1
#define LOW 0

//Subroutines used:
void board_initialization(void);
void Joystick_Polling(void);

//////////////////
//  Variables   //
//////////////////

//Serial variables
//signed char usartTx [2] = 0;
//unsigned char usartRx[11] = 0;

//Program variables
volatile unsigned char i = 0;
volatile signed char JoystickValues[2] = {0x00, 0x00};

//__interrupt(high_priority) void ISR_bassa(void) {
//}

//////////
// MAIN //
//////////

void main(void) {
    board_initialization();
    //aggiungere controllo stato centraline
    while (1) {
        Joystick_Polling();
        for (i = 0; i < 2; i++) {
            WriteUSART(JoystickValues[i]);
            while (BusyUSART());
        }
        delay_ms(10); //delay per evitare di saturare il sistema di messaggi
    }
}

///////////////////
//  SUBROUTINES  //
///////////////////

void Joystick_Polling(void) {
    for (i = 0; i < 2; i++) {
        ADCON0bits.GO = HIGH;
        while (ADCON0bits.GO);
        JoystickValues[i] = ADRESH - 128;
        ADCON0bits.CHS0 = ~ADCON0bits.CHS0;
    }
}

void board_initialization(void) {//[PER ORA DISABILITATI GLI INTERRUPT PER RX]
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b00000011;
    LATB = 0x00;
    TRISB = 0bFF;
    LATC = 0x00;
    TRISC = 0bFF;
    LATD = 0x00;
    TRISD = 0x11000000; //USART Tx e Rx
    LATE = 0x00;
    TRISE = 0xFF;
    
    CloseUSART();

    //Interrupt Flags
    //    PIR1bits.RCIF = LOW;

    //Interrupts Priority
    //    RCONbits.IPEN = HIGH; //abilita priorità interrupt
    //    IPR1bits.RCIP = HIGH; //interrupt alta priorità ricezione seriale

    //Open Serial Module (asyncronous mode - 8 bit - 9600 baud)
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_BRGH_HIGH & USART_CONT_RX, 103);
    RCSTAbits.SPEN = HIGH; //abilita la periferica

    //Configurazione ADC
    ADCON1 = 0b00001101;//RA0 and RA1 analogic, AVdd and AVss voltage reference (?)
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;
    ADCON2bits.ACQT2 = 1;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 0;
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
    ADCON2bits.ADFM = 0; //Left Justified
    ADCON0bits.ADON = HIGH;

    //Interrupts Enables
    //    PIE1bits.RCIE = HIGH;
    //    INTCONbits.GIEH = HIGH;
    //    INTCONbits.GIEL = HIGH;

    delay_ms(2);
}