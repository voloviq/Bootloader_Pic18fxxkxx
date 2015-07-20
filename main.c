/*
  Author: Michal Wolowik
  Corporation: VOLOVIQUE
  Date: Warsaw 20-VII-2015
  Destination: 
    Bootloader for Pic18fxxkxx devices

  File name:
    main.c

  File destination:
    Main destiny of current code is update flash
    conten using UART, USB, CAN medium and 
    Universal_Bootloader application on PC. More
    detail regarding to PC application refer to http:TBD

  Dependecies:
    Uart (UART 1 used with 9600br/8bits/no_parity/1stopbits/Asynchronous)
    CAN if available
    USB if available
    Software heartbeat located on PB1 Led

  Compiler:
    XC8 - version: v1.33(A) - in free mode

  GUI:
    MPLAB X IDE v3.05

  Documentation:
    PIC18F66K80FAMILY DS39977F Revision F Februrary 2012.

  Responsible:
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    Lesser General Public License for more details.
		
  Notes:
    Application was tested on AVT-5275v2 development board.
 */



/*
 Language Standard Library:
 */
#include <stdio.h>
#include <stdlib.h>




/*
 Specific Device Library:
 */
#include <p18cxxx.h>




/*
 Configurations Bits Definitions:
 */
#include "Config_Bits.h"




/*
 Clock Definition:
 */
#define MCU_FREQ 64000000




/*
 Port Definitions:
 */
#define LED LATBbits.LATB0





/*
 Bootloader specific
 */
#define VERSION "1.0.a"
#define PAGE_SIZE 64



/*
 Global variables
 */
unsigned char gbuffer[PAGE_SIZE];



/*
Function name   :   delay_ms
Description     :   Function give simple delay functionality.
Input 1         :   None
Output          :   None
Function return :   None
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
/*static void delay_ms(unsigned int duration);*/




/*
Function name   :   UART_init
Description     :   Function initalize UART 1 (9600, 8bit, 1stp, no parity)
Input 1         :   None
Output          :   None
Function return :   None
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
static void UART_init(void);




/*
Function name   :   UART_putchar
Description     :   Function send character via UART
Input 1         :   unsigned char: byte to send
Output          :   None
Function return :   None
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
static void UART_putchar(unsigned char outchar);




/*
Function name   :   UART_getchar
Description     :   Function receive character acquired from UART
Input 1         :   None
Output          :   None
Function return :   Received byte
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
static unsigned char UART_getchar(void);




/*
Function name   :   send_boot_version
Description     :   Function send via UART bootloader version
Input 1         :   unsigned char *: string with version terminated with NULL
Output          :   None
Function return :   None
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
static void send_boot_version(unsigned char *buf);




/*
Function name   :   bootloader_state_machine
Description     :   Main state machine of bootloader - refer to algorithm
Input 1         :   unsigned char : received byte
Output          :   None
Function return :   None
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
static void bootloader_state_machine(unsigned char state);




int main(int argc, char** argv) {

    unsigned long int hbeat = 0;
    
    /* Main Oscillator Configuration */
    
    /* Set Internal Oscillator Frequency to 16[MHz] */
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;

    /* Set frequency stable bit */
    OSCCONbits.HFIOFS = 1;

    /* Enable Internal PLL x 4 - MCLK 64 [MHz] */
    OSCTUNEbits.PLLEN = 1;
    
    /* GPIO Ports Configuration */
    
    /* Heartbeat Led diode port configuration */
    TRISBbits.TRISB0 = 0;
    
    /* Initialize UART 1 - 9600, 8bits, 1stb, no parity */
    UART_init();
    
    while(1){
        
        if(hbeat == 0x03FF){
			LED = 1;
            hbeat++;
		}
		else if( hbeat == 0x2FFF ){
			LED = 0;
            hbeat++;
		}
		else if (hbeat == 0x1FFFF){
			hbeat = 0x00;
        }
        else{			
            hbeat++;
        }
        
        bootloader_state_machine(UART_getchar());
    }
    
    return (EXIT_SUCCESS);
}



/*
static void delay_ms(unsigned int duration) {
    unsigned int i;
    unsigned int j;
    for (i = duration; i != 0; i--) {
        for (j = 0; j <= 1000; j++) {
            asm("nop");
            asm("nop");
            asm("nop");
        }
        asm("nop");
        asm("nop");
    }
}*/




static void UART_init(void)
{
    
    TRISCbits.TRISC6 = 0; /* TX pin set as output */
    TRISCbits.TRISC7 = 1; /* RX pin set as input */
 
    /* Don't calculate Baudrate just take value from user manual
       page 339. */
    
    /* Sync = 0, BRGH = 0, BRG16 = 0 FOSC = 64MHz */
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 0;
    BAUDCON1bits.BRG16 = 0;
    SPBRG = 103;
    
    /* Serial Port Enable Transmission - ON */
    TXSTAbits.TXEN = 1;
    
    /* Serial Port Enable bit - ON */
    RCSTAbits.SPEN = 1;
    
    /* Continous Receive Enable bit - ON */
    RCSTAbits.CREN = 1;
}




static void UART_putchar(unsigned char outchar)
{

    while(!TXSTA1bits.TRMT);

    TXREG1 = outchar;

    while(!TXSTA1bits.TRMT);
}




static unsigned char UART_getchar(void)
{   
    if(RCIF){
        while(!RCIF);
        if( (RCSTAbits.FERR == 1) || (RCSTAbits.OERR == 1) ){
            RCSTAbits.CREN = 0; /* Clear an error */
            RCSTAbits.CREN = 1; /* Start continous receiving again */
        }
        return((unsigned char)RCREG1);
    }
    else{
        return(0);
    }
}




static void send_boot_version(unsigned char *buf)
{
    unsigned char i = 0x00;
	
	do{
        if(buf[i] != '\0'){
            UART_putchar(buf[i]);
        }
	}while(buf[i++] != '\0');
}




static void bootloader_state_machine(unsigned char state)
{
    static unsigned char byte_in_record = 0x00;
    static unsigned char temp = 0x00;
    static unsigned int crc = 0x00;
    static unsigned int record_address = 0x00;
    static unsigned int record_type = 0x00;
    static unsigned char end_of_record = 0x00;
    static unsigned int byte_index = 0x00;
    static unsigned int page_index = 0x00;
    static unsigned long timeout = 0x00;
    static unsigned char i = 0;
    
    switch (state)
    {
        case 0xC1:{
			send_boot_version((unsigned char*)VERSION);	
			timeout = 0;
        break;
        }
                
        case 0xAA:{
            UART_putchar(0x55);
            state = 0x00;
        break;
        }

        case 0x31:{
            if(UART_getchar() == 'Q')
                if(UART_getchar() == 'W')
                    if(UART_getchar() == 'E')
                        if(UART_getchar() == 'R')
                            if(UART_getchar() == 'T'){
                                //xxxeraseFlash();
                                UART_putchar(0xEE);
                            }
            state = 0x00;
        break;
        }

        case 0x32:{	
            if(UART_getchar() == 'Q')
                if(UART_getchar() == 'W')
                    if(UART_getchar() == 'E')
                        if(UART_getchar() == 'R')
                            if(UART_getchar() == 'T'){
                                /* Processing data which came from UART, byte after byte */
                                do{
                                    if(UART_getchar() == ':'){
                                        /* First byte is byte quantity in current record */
                                        byte_in_record = UART_getchar();
                                        crc += byte_in_record; // Add to crc

                                        /* Get higher nibble of record address */
                                        temp = UART_getchar();
                                        crc += temp; // Add to crc
                                        record_address  = temp<<8;

                                        /* Get lower nibble of record address */
                                        temp = UART_getchar();
                                        crc += temp; // Add to crc
                                        record_address |= temp;

                                        /* Get record type */
                                        temp = UART_getchar();
                                        crc += temp; // Add to crc
                                        record_type = temp;


                                        /* Check what kind of record came */
                                        if(record_type == 0){
                                            /* One separate record analysis - data buffering */
                                            for(i=0; i<=byte_in_record; i++)
                                            {
                                                gbuffer[byte_index] = UART_getchar();
                                                crc += gbuffer[byte_index];
                                                byte_index++;
                                            }
                                            byte_index--;
                                            if((crc&0x00FF) == 0){
                                                /* If collected page size byte, save page into flash */
                                                if(byte_index >= PAGE_SIZE){
                                                    //xxxboot_program_page(page_index, gBuffer);
                                                    //xxxClear_Buffer(gBuffer,SPM_PAGESIZE);
                                                    byte_index = 0x00;
                                                    page_index +=256;
                                                }
                                                UART_putchar(0xCC);
                                                crc = 0x00;
                                                timeout = 0x00;
                                            }
                                            else{	
                                                UART_putchar(0xCE);
                                                crc = 0x00;
                                            }
                                            crc = 0;
                                            record_type = 1;
                                        }
                                        /* The 16-bit Extended segment address record */
                                        else if(record_type == 2){
                                            /* According to specification the address should be calculated
                                            as follow: */
                                            for(i=0; i<=byte_in_record; i++){
                                                crc += UART_getchar();
                                            }
                                            if((crc&0x00FF) == 0){
                                                UART_putchar(0xCC);
                                                timeout = 0x00;
                                            }
                                            else{
                                                UART_putchar(0xCE);
                                            }
                                            crc = 0;
                                        }
                                        else if(record_type == 1){
                                            /* If all page filled save into flash */
                                            //xxxboot_program_page(page_index, gBuffer);
                                            //xxxClear_Buffer(gBuffer,SPM_PAGESIZE);
                                            end_of_record = 1; /* Last record detected */
                                            UART_putchar(0xF0);
                                        }
                                    }
                                    else{	
                                        end_of_record = 1;
                                    }
                                    //xxxtimeout++;
                                }while((end_of_record != 1)||(timeout==10)); /* Leave loop when processing finish or detected wrong crc */

                                //xxxboot_program_page(1, gBuffer);
                                }
            state = 0x00;
            page_index =0x00;
            byte_index = 0x00;
            record_address = 0x00;
            byte_in_record = 0x00;
            record_type = 0x00;
            timeout = 0x00;
            end_of_record = 0x00;
            crc = 0x00;
        break;
        }

        case 0x33:
        {
            UART_putchar(0x76);
            //xxxjump_to_app(); /* Jump to user application code */

        break;
        }

        default:
            ;
        break;
    }
}