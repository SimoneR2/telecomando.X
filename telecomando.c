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
#include <usart.h> //not used in Rx
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
#define FWD 2
#define BKWD 1

//Subroutines used:
void board_initialization(void);
void PWR_Button_Polling(void);
void Joystick_Polling(void);
void USART_Send(void);
void LCD_Handler(void);
void USART_RX(void);

//////////////////
//  Variables   //
//////////////////

//Serial variables
unsigned char USART_Tx [8] = 0;
unsigned char USART_Tx_Old [8] = 0;
unsigned char USART_Rx[8] = 0;
volatile bit actual_dir = LOW;
volatile char spam_counter = 0;
volatile unsigned int actual_speed = 0;
volatile unsigned char actual_speed_pk1 = 0;
volatile unsigned char actual_speed_pk0 = 0;
volatile unsigned char ECU_Check = 0;
volatile unsigned char Battery_Check = 0;

//Program variables
volatile bit power_switch = LOW;
volatile bit wait_low = LOW;
volatile char dir = 0;
volatile unsigned char i = 0;
volatile unsigned char switch_position = 0;
volatile unsigned char set_steering = 0;
volatile unsigned int set_speed = 0;
volatile unsigned char set_speed_pk1 = 0;
volatile unsigned char set_speed_pk0 = 0;
volatile unsigned char analogic_brake = 0;
volatile unsigned char JoystickValues[2] = 0; //steering - speed
volatile signed float JoystickConstants[2] = 0;
//volatile unsigned char JoystickConstants[2] = 0;

//TMR3 variables
volatile unsigned long time_counter = 0;
volatile unsigned long pr_time_1 = 0;
volatile unsigned long pr_time_2 = 0;
volatile unsigned long pr_time_3 = 0;

//LCD variables
unsigned char str [12] = 0;
signed float actual_speed_kmh = 0;

__interrupt(high_priority) void ISR_alta(void) {
    USART_RX();
   
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
    //Turn off ON/OFF switch backlight
    PORTDbits.RD7 = LOW;

    //Inizializzazione Arrays
    USART_Tx[0] = 0xAA;
    USART_Tx[1] = 0x01;
    USART_Tx[2] = 0x01;
    USART_Tx[3] = 0x01;
    USART_Tx[4] = 0x01;
    USART_Tx[5] = 0x01;
    USART_Tx[6] = 0xAA;
    USART_Tx[7] = '\0';
    JoystickConstants[X_AXIS] = 0.703;
    JoystickConstants[Y_AXIS] = 35; //35

    while (1) {
        //[Check BLUETOOTH CONNECTION]
        //[CHECK ECU]

        USART_RX();
        PWR_Button_Polling();

        if (power_switch == LOW) {
            dir = FWD;
            set_speed = 0;
            set_steering = 90;
            analogic_brake = 0;
            while (BusyUSART() == HIGH) {
            };
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
                if ((time_counter - pr_time_1) >= 50) {
                    pr_time_1 = time_counter;
                    PORTDbits.RD7 = ~PORTDbits.RD7;
                }
                PWR_Button_Polling();
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

        set_steering = (JoystickValues[X_AXIS])*(JoystickConstants[X_AXIS]);
        if (switch_position != HIGH_POS) {
            if (JoystickValues[Y_AXIS] > 132) {
                set_speed = (JoystickValues[Y_AXIS] - 130)*(JoystickConstants[Y_AXIS]); //guardare
                analogic_brake = 0;
            } else {
                set_speed = 0;
                analogic_brake = ((130 - JoystickValues[Y_AXIS]))*(1.9);
            }
        }

        if ((time_counter - pr_time_2) >= 2) {
            pr_time_2 = time_counter;
            USART_Send();
        }

        if ((time_counter - pr_time_3) >= 50) {
            pr_time_3 = time_counter;
            LCD_Handler();
        }
    }
}

///////////////////
//  SUBROUTINES  //
///////////////////

void PWR_Button_Polling(void) {
    if ((PORTBbits.RB3 == LOW) || (wait_low == LOW)) {
        wait_low = LOW;
        if (PORTBbits.RB3 == HIGH) {
            power_switch = ~power_switch;
            wait_low = HIGH;
        }
    }
}

void Joystick_Polling(void) {
    for (i = 0; i < 2; i++) {
        ADCON0bits.GO = HIGH;
        while (ADCON0bits.GO);
        JoystickValues[i] = ADRESH;
        ADCON0bits.CHS0 = ~ADCON0bits.CHS0;
    }
}

void USART_Send(void) {
    if (switch_position == LOW_POS) {
        set_speed = set_speed / 4;
    }
    set_speed_pk1 = set_speed >> 8;
    if (set_speed_pk1 == 0x00) {
        set_speed_pk1 = 0b10000000;
    }
    set_speed_pk0 = set_speed;
    USART_Tx[1] = dir;
    USART_Tx[2] = set_speed_pk1;
    USART_Tx[3] = set_speed_pk0;

    if ((switch_position == HIGH_POS) || ((JoystickValues[Y_AXIS] >= 128)&&(JoystickValues[Y_AXIS] <= 132))) {
        USART_Tx[2] = 0b10000000;
        USART_Tx[3] = 0b00000001;
    }
    USART_Tx[4] = set_steering;
    USART_Tx[5] = analogic_brake;

    //Set to 1 the null bytes to avoid serial send interrupts
    for (char i = 0; i < 6; i++) {
        if (USART_Tx[i] == 0) {
            USART_Tx[i] = 1; //debug
        }

    }

    //Checks if the data serial send is identical to the prevoius and blocks it
    spam_counter = 0;
    for (char i = 1; i < 6; i++) {
        if (USART_Tx[i] == USART_Tx_Old[i]) {
            spam_counter++;
        }
    }
    if (((BusyUSART() != HIGH)&&(spam_counter != 5)) || (switch_position == HIGH_POS)) {

        if (switch_position == HIGH_POS) {
            delay_ms(100);
        }
        INTCONbits.GIEH = 0; //debug
        INTCONbits.GIEL = 0; //debug
        putsUSART(USART_Tx);
        
        INTCONbits.GIEH = 1; //debug
        INTCONbits.GIEL = 1; //debug
        for (char i = 0; i < 6; i++) {

            USART_Tx_Old[i] = USART_Tx[i];
        }
    }
}

void USART_RX(void) {
    if (PIR1bits.RCIF == HIGH) {
        INTCONbits.GIEH = 0;
        INTCONbits.GIEL = 0;
        PIE2bits.TMR3IE = 0;
        //  getsUSART(USART_Rx, 8);
        PIE1bits.RCIE = 0; //disabilita interrupt ricezione seriale
        if (RCREG == 0xAA) {
             
            for (unsigned char i = 0; i < 7; i++) {
               PORTDbits.RD7 = 1;//debug
                while (PIR1bits.RCIF != 1) {
                }
             
                USART_Rx[i] = RCREG;
                PIR1bits.RCIF = 0;
            }
           // PORTDbits.RD7 = 0;//debug
        }
        if (USART_Rx[5] == 0xAA) {
            actual_dir = USART_Rx[1];
            actual_speed_pk1 = USART_Rx[2];
            if (actual_speed_pk1 == 0b10000000) {

                actual_speed_pk1 = 0x00;
            }
            actual_speed_pk0 = USART_Rx[3];
            ECU_Check = USART_Rx[4];
            Battery_Check = USART_Rx[5];
            actual_speed = (actual_speed_pk1 << 8) | (actual_speed_pk0);
        }
        PIR1bits.RCIF = LOW;
        PIE1bits.RCIE = HIGH;
        PIE2bits.TMR3IE = 1;
    }
    INTCONbits.GIEH = 1; //debug
    INTCONbits.GIEL = 1; //debug    
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
    //    LCD_write_message("Valore: ");
    //    LCD_write_integer(JoystickValues[Y_AXIS], 3, ZERO_CLEANING_ON);

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
    TRISC = 0b10110000; //USART Tx and Rx / LCD
    LATD = 0x00;
    TRISD = 0x00; //LCD / Backlight ON/OFF
    LATE = 0x00;
    TRISE = 0x00;

    //Interrupt Flags
    PIR1bits.RCIF = LOW;
    PIR2bits.TMR3IF = LOW;

    //Interrupts Priority
    RCONbits.IPEN = HIGH; //abilita priorità interrupt
    IPR1bits.RCIP = HIGH; //interrupt alta priorità ricezione seriale
    IPR2bits.TMR3IP = LOW; // interrupt bassa priorità TMR3

    //USART Configuration (asyncronous mode - 8 bit - 9600 baud)
    CloseUSART();
    delay_ms(10);
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE
            & USART_EIGHT_BIT & USART_BRGH_HIGH & USART_CONT_RX, 25);

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

    PORTDbits.RD2 = 1;
    delay_ms(10);
    PORTDbits.RD3 = 1;
    delay_s(1);
    //LCD Initialize
    LCD_initialize(16);
    LCD_backlight(0);
    LCD_clear();
    LCD_goto_line(1);

    LCD_write_message("Wait...");
    delay_ms(300);

    PORTDbits.RD2 = 0;
    PORTDbits.RD3 = 0;

    //Configurations
    TMR3H = 0x63;
    TMR3L = 0xC0;

    //Interrupts Enables
    PIE1bits.RCIE = HIGH;
    PIE2bits.TMR3IE = HIGH;
    INTCONbits.GIEH = HIGH;
    INTCONbits.GIEL = HIGH;

    RCSTAbits.SPEN = HIGH; //USART Enable
    T3CON = 0x01; //Timer Enable
    LCD_clear();
    WriteUSART(0xaa); //inizializza CantoSerial
}