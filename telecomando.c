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
// successivamente convertito ed inviato in questo ordine attraverso seriale.
// [Il codice commentato servirà in futuro ma ora non è utile]
//
//  Inputs and outputs: RA0 = X-Axis (steering)
//                      RA1 = Y-Axis (speed)
//                      RA2 - RA3 = Pulsante a tre posizioni
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

#define X_AXIS 0 ///Steering
#define Y_AXIS 1//Speed
#define HIGH_POS 0
#define MID_POS 1
#define LOW_POS 2
#define HIGH 1
#define LOW 0

//Subroutines used:
void board_initialization(void);
void Joystick_Polling(void);
void USART_Send(void);

//////////////////
//  Variables   //
//////////////////

//Serial variables
unsigned char USART_Tx [7] = 0;
//unsigned char usartRx[11] = 0;

//Program variables
volatile bit dir = LOW;
volatile unsigned char i = 0;
volatile unsigned char switch_position = 0;
volatile unsigned char set_steering = 0;
volatile unsigned int set_speed = 0;
volatile unsigned char set_speed_pk1 = 0;
volatile unsigned char set_speed_pk0 = 0;
volatile unsigned char analogic_brake = 0;
volatile signed char JoystickValues[2] = 0; //steering - speed
volatile signed float JoystickConstants[2] = 0;

//__interrupt(high_priority) void ISR_bassa(void) {
//}

//////////
// MAIN //
//////////

void main(void) {
    board_initialization();
    
    //Inizializzazione Arrays
    USART_Tx[0] = 0xAA;
    USART_Tx[6] = 0xAA;
    JoystickConstants[0] = 0.703;
    JoystickConstants[1] = 34;
    
    //[AGGIUNGERE CONTROLLO STATO CENTRALINE]
    
    while (1) {
        Joystick_Polling();

        //Gestione switch tre posizioni
        if (PORTAbits.RA2 == HIGH) {
            switch_position = HIGH_POS;
        } else {
            if (PORTAbits.RA3 == LOW) {
                switch_position = MID_POS;
                dir = HIGH;
            } else {
                switch_position = LOW_POS;
                dir = LOW;
            }
        }

        set_steering = (128 + JoystickValues[X_AXIS])*(JoystickConstants[X_AXIS]);
        if (JoystickValues[Y_AXIS] > 0) {
            set_speed = (JoystickValues[Y_AXIS])*(JoystickConstants[Y_AXIS]);
            analogic_brake = 0;
        } else {
            set_speed = 0;
            analogic_brake = -((2 * JoystickValues[Y_AXIS]) + 1);
        }
        USART_Send();
        delay_ms(10);
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

void USART_Send(void) {
    set_speed_pk1 = set_speed >> 8;
    set_speed_pk0 = set_speed;
    USART_Tx[1] = dir;
    USART_Tx[2] = set_speed_pk1;
    USART_Tx[3] = set_speed_pk0;
    USART_Tx[4] = set_steering;
    USART_Tx[5] = analogic_brake;
    //    USART_Tx = {0xAA, dir, set_speed_pk1, set_speed_pk0, set_steering, analogic_brake, 0xAA};
    for (i = 0; i < 7; i++) {
        WriteUSART(USART_Tx[i]);
        while (BusyUSART());
    }
}

void board_initialization(void) {//[PER ORA DISABILITATI GLI INTERRUPT PER RX]
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b00001111;
    LATB = 0x00;
    TRISB = 0xFF;
    LATC = 0x00;
    TRISC = 0xFF;
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
    ADCON1 = 0b00001101; //RA0 and RA1 analogic, AVdd and AVss voltage reference (?)
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