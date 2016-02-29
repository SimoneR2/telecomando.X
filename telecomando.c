// PROGRAM: Telecomando
// WRITTEN BY: Massimo Clementi
// DATA: 22/02/2016
// VERSION: 1.0
// FILE SAVED AS: telecomando.c
// FOR PIC: 18F4480 (18F4685)
// CLOCK FREQUENCY: 16 MHz
// PROGRAM FUNCTION: misura, impacchettamento ed invio dei dati del joystick
// attraverso USART. Viene eseguita prima la misura di entrambi gli assi e
// e memorizzato il dato ottenuto in un array di signed char (-128;+127),
// successivamente convertito ed inviato in questo ordine attraverso seriale.
// [AGGIUNGERE INFORMAZIONI]
//
// ============================== I/O ==========================================
//  Inputs: RA0 = X-Axis (steering)
//          RA1 = Y-Axis (speed)
//          RA2 - RA3 = Pulsante a tre posizioni
//          RB3 = ON/OFF
//                      
//  Outputs: RC6/7 = USART
//           RA6 - RC0/1/2/3 - RD0/1 = LCD (vedi librerie per ulteriori info)
//           RD7 = Backlight ON/OFF
// =============================================================================

#define USE_AND_MASKS
#define LCD_DEFAULT
#define _XTAL_FREQ 16000000
//#include <string.h>
//#include <stdio.h>
#include <xc.h>
#include "telecomando.h"
#include <usart.h>
#include "delay.h"
#include "delay.c"
#include "LCD_44780.h" 
#include "LCD_44780.c"
#include <stdio.h>
#include <math.h>

#define X_AXIS 0 ///Steering
#define Y_AXIS 1//Speed
#define HIGH_POS 0
#define MID_POS 1
#define LOW_POS 2
#define HIGH 1
#define LOW 0
#define FWD 1
#define BKWD 0

//Subroutines used:
void board_initialization(void);
void Joystick_Polling(void);
void USART_Send(void);
void LCD_Handler(void);

//////////////////
//  Variables   //
//////////////////

//Serial variables
unsigned char USART_Tx [7] = 0;
unsigned char USART_Rx[7] = 0;
volatile bit actual_dir = LOW;
volatile unsigned int actual_speed = 0;
volatile unsigned char actual_speed_pk1 = 0;
volatile unsigned char actual_speed_pk0 = 0;
volatile unsigned char ECU_Check = 0;
volatile unsigned char Battery_Check = 0;

//Program variables
volatile bit dir = LOW;
volatile bit power_switch = LOW;
volatile bit wait_low = LOW;
volatile unsigned char i = 0;
volatile unsigned char switch_position = 0;
volatile unsigned char set_steering = 0;
volatile unsigned int set_speed = 0;
volatile unsigned char set_speed_pk1 = 0;
volatile unsigned char set_speed_pk0 = 0;
volatile unsigned char analogic_brake = 0;
volatile signed char JoystickValues[2] = 0; //steering - speed
volatile signed float JoystickConstants[2] = 0;

//TMR3 variables
volatile unsigned long time_counter = 0;
volatile unsigned long pr_time_1 = 0;
volatile unsigned long pr_time_2 = 0;
volatile unsigned long pr_time_3 = 0;

//LCD variables
unsigned char str [12] = 0;
signed float actual_speed_kmh = 0;

__interrupt(high_priority) void ISR_alta(void) {
    if (PIR1bits.RCIF == HIGH) {
        getsUSART((char*) USART_Rx, 7);
        if ((USART_Rx[0] == 0xAA) && (USART_Rx[6] == 0xAA)) {
            actual_dir = USART_Rx[1];
            actual_speed_pk1 = USART_Rx[2];
            actual_speed_pk0 = USART_Rx[3];
            ECU_Check = USART_Rx[4];
            Battery_Check = USART_Rx[5];
            actual_speed = (actual_speed_pk1 << 8) || (actual_speed_pk0);
        }
        PIR1bits.RCIF = LOW;
    }
}

__interrupt(low_priority) void ISR_bassa(void) {
    if (PIR2bits.TMR3IF) { //10ms
        time_counter++;
        TMR3H = 0x63;
        TMR3L = 0xC0;
        PIR2bits.TMR3IF = 0;
    }
}

//////////
// MAIN //
//////////

void main(void) {
    board_initialization();
    PORTDbits.RD7 = LOW; //Turn off ON/OFF switch backlight

    //Inizializzazione Arrays
    USART_Tx[0] = 0xAA;
    USART_Tx[6] = 0xAA;
    JoystickConstants[0] = 0.703;
    JoystickConstants[1] = 34;

    //[AGGIUNGERE CONTROLLO STATO CENTRALINE]

    while (1) {
        if ((PORTBbits.RB3 == LOW) || (wait_low == LOW)) {
            wait_low = LOW;
            if (PORTBbits.RB3 == HIGH) {
                power_switch = ~power_switch;
                wait_low = HIGH;
            }
        }

        if (power_switch == LOW) {
            dir = FWD;
            set_speed = 0;
            set_steering = 90;
            analogic_brake = 0;
            USART_Send();
            while (power_switch == LOW) {
                LCD_clear();
                LCD_goto_line(1);
                LCD_write_message("====================");
                LCD_goto_line(2);
                LCD_write_message("==> VEHICLE  OFF <==");
                LCD_goto_line(3);
                LCD_write_message("Turn the switch ON! ");
                LCD_goto_line(4);
                LCD_write_message("====================");
                if ((time_counter - pr_time_1) >= 700) {
                    pr_time_1 = time_counter;
                    PORTDbits.RD7 = ~PORTDbits.RD7;
                }
                delay_ms(300); //[!!]Verificare
            }
            PORTDbits.RD7 = LOW; //Turn off ON/OFF switch backlight
        }

        Joystick_Polling();

        //Gestione switch tre posizioni
        if (PORTAbits.RA2 == HIGH) {
            switch_position = HIGH_POS;
        } else {
            if (PORTAbits.RA3 == LOW) {
                switch_position = MID_POS;
                dir = FWD;
            } else {
                switch_position = LOW_POS;
                dir = BKWD;
            }
        }

        if (switch_position != HIGH_POS) {
            set_steering = (128 + JoystickValues[X_AXIS])*(JoystickConstants[X_AXIS]);
            if (JoystickValues[Y_AXIS] > 0) {
                set_speed = (JoystickValues[Y_AXIS])*(JoystickConstants[Y_AXIS]);
                analogic_brake = 0;
            } else {
                set_speed = 0;
                analogic_brake = -((2 * JoystickValues[Y_AXIS]) + 1);
            }
        } else {
            set_speed = 0;
            analogic_brake = 255;
        }

        if ((time_counter - pr_time_2) >= 1) {
            pr_time_2 = time_counter;
            USART_Send();
        }

        if ((time_counter - pr_time_3) >= 30) {
            pr_time_3 = time_counter;
            LCD_Handler();
        }
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
    putsUSART((char *) USART_Tx);
}

void LCD_Handler(void) {
    actual_speed_kmh = actual_speed / 278;

    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("=== VEHICLE DATA ===");

    LCD_goto_line(2);
    LCD_write_message("Direction: ");
    if (switch_position != HIGH_POS) {
        if (dir == FWD) {
            LCD_write_message("FWD");
        } else {
            LCD_write_message("BKWD");
        }
    } else {
        LCD_write_message("P");
    }

    LCD_goto_line(3);
    sprintf(str, "Speed: %.3f", actual_speed_kmh);
    str[11] = '\0';
    LCD_write_string(str);
    LCD_write_message("Km/h");

    LCD_goto_line(4);
    LCD_write_message("====================");
}

void board_initialization(void) {
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b00001111; // X-Axis / Y-Axis / 3P Switch
    LATB = 0x00;
    TRISB = 0xFF; // ON/OFF Switch
    LATC = 0x00;
    TRISC = 0b11110000; //USART Tx and Rx / LCD
    LATD = 0x00;
    TRISD = 0x01111100; //LCD / Backlight ON/OFF
    LATE = 0x00;
    TRISE = 0xFF;

    //LCD Initialize
    LCD_initialize(16);
    LCD_backlight(0);
    LCD_clear();
    LCD_goto_line(1);
    LCD_write_message("Wait...");

    //Interrupt Flags
    PIR1bits.RCIF = LOW;
    PIR2bits.TMR3IF = LOW;

    //Interrupts Priority
    RCONbits.IPEN = HIGH; //abilita priorità interrupt
    IPR1bits.RCIP = HIGH; //interrupt alta priorità ricezione seriale
    IPR2bits.TMR3IP = LOW; // interrupt bassa priorità TMR3

    //USART Configuration (asyncronous mode - 8 bit - 9600 baud)
    CloseUSART();
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE
              & USART_EIGHT_BIT & USART_BRGH_HIGH & USART_CONT_RX, 103);

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

    //Configurations
    TMR3H = 0x63;
    TMR3L = 0xC0;

    //Interrupts Enables
    PIE1bits.RCIE = HIGH;
    PIE2bits.TMR3IE = HIGH;
    INTCONbits.GIEH = HIGH;
    INTCONbits.GIEL = HIGH;

    RCSTAbits.SPEN = HIGH; //USART Enable
    delay_ms(2);
    T3CON = 0x01; //Timer Enable
    LCD_clear();
}