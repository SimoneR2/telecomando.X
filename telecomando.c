#define USE_AND_MASKS
/*==============================================================================
 *PROGRAM: Can to Serial converter
 *WRITTEN BY: Ignoto
 *DATA: 07/02/2016
 *VERSION: 1.0
 *FILE SAVED AS: CanToSerial.c
 *FOR PIC: 18F4480
 *CLOCK FREQUENCY: 16 MHz
 *PROGRAM FUNCTION: 
 *DA AGGIUNGERE---------
 */
#include <string.h>
#include <stdio.h>
#define _XTAL_FREQ 16000000
#include <xc.h>
#include "telecomando.h"
#include "CANlib.h"
#include <usart.h>
#include "delay.h"
#include "delay.c"
#include <xc.h>

unsigned char usartTx [11] = 0;
unsigned char usartRx[11] = 0;

__interrupt(low_priority) void ISR_bassa(void){
    if (PIR1bits.RCIF == 1) {
        getsUSART((char*) usartRx, 11);
    }
}
void main(void) {
    inizializzazione();
}

void inizializzazione(void) { //da completare
    CloseUSART(); //disabilita periferica per poterla impostare

    RCONbits.IPEN = 1; //abilita priorità interrupt
    
    PIR1bits.RCIF = 0; //azzera flag interrupt ricezione seriale
    
    IPR1bits.RCIP = 0; //interrupt bassa priorità ricezione seriale
    
    PIE1bits.RCIE = 1; //abilita interrupt ricezione seriale

    INTCONbits.GIEH = 1; //abilita interrupt 
    INTCONbits.GIEL = 1; //abilita interrupt periferiche
    
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_BRGH_HIGH & USART_CONT_RX, 103); //Seriale asincrona, 8bit, 9600baud
    RCSTAbits.SPEN = 1; //abilita la periferica
     LATA = 0x00;
    TRISA = 0b01111101;

    LATB = 0x00;
    TRISB = 0b11111011;

    LATC = 0x00;
    TRISC = 0b10111111;

    LATD = 0x00;
    TRISD = 0x00;

    LATE = 0x00;
    TRISE = 0xFF;
}