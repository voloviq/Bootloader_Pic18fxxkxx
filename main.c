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
    MCU Clock set to 64MHz
    Software heartbeat located on PB0 Led

  Compiler:
    XC8 - version: v1.34 - in free mode

  GUI:
    MPLAB X IDE v3.06

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
 Standard ANSI C Library:
 */
#include <stdio.h>
#include <stdlib.h>




/*
 Specific Device Library:
 */
#include <p18cxxx.h>
#include <xc.h>
#include <plib.h>




/*
 Configurations Bits Definitions:
 */
#include "config_bits.h"




/*
 Allowable XC8 types definitions
 */
#include "typedef.h"




/*
 Port Definitions:
 */
/* Heartbeat LED port definition */
#define LED LATBbits.LATB0




/*
 Bootloader specific
 */
/* Bootloader version definition */
#define VERSION "1.0.a"
/* One separately page size */
#define PAGE_SIZE 64




/*
 * Flash necessary defines definition
 */
/* Define where user application start */
#define APP_ENTRY 0x1100
/* Maximum amount of Flash */
#define FLASH_SIZE 0x10000
/* Quantity 64bytes flash block available for user application */
#define COUNTS_TO_ERASE_APP (FLASH_SIZE-APP_ENTRY) / FLASH_ERASE_BLOCK




/*
 * Intel Hexadecimal Object File Format Specification defines
 */
#define DATA_RECORD 0x00
#define END_OF_FILE_RECORD 0x01
#define EXTENDED_SEGMENT_ADDRESS_RECORD 0x02
#define START_SEGMENT_ADDRESS_RECORD 0x03
#define EXTENDED_LINEAR_ADDRESS_RECORD 0x04
#define START_LINEAR_ADDRESS_RECORD 0x05

#define RECORD_MARK ':'



/*
 Global variables
 */
/* Buffer which is intended to contain bytes to be written in flash */
u8 gbuffer[PAGE_SIZE]; 



/*
Function name   :   delay_ms
Description     :   Function give simple delay functionality.
Input 1         :   u16: Expected delay value
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
/* static void delay_ms(u16 duration); */




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
Input 1         :   u8: byte to send
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
static void UART_putchar(u8 outchar);




/*
Function name   :   UART_getchar
Description     :   Function receive character acquired from UART if available
Input 1         :   None
Output          :   None
Function return :   u8: Return recived byte from UART
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
static u8 UART_getchar(void);




/*
Function name   :   UART_wait_for_char
Description     :   Function wait for character from UART and return in value
Input 1         :   None
Output          :   None
Function return :   u8: Return recived byte from UART
Testing result  :   Not tested yet
Test conditions :   AVT-5275v2 - MCU Clock 64MHz
Optymalization  :   For XC8 used - 'free'
Function valid  :   Function invalid - not tested yet
Function rev.   :   V1_0
Approved by     :   not approved yet
Issue date      :   20.VII.2015
Problem report  :   None
*/
static u8 UART_wait_for_char(void);




/*
Function name   :   send_boot_version
Description     :   Function send via UART bootloader version
Input 1         :   u8 *: string with version terminated with NULL
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
static void send_boot_version(u8 *buf);




/*
Function name   :   bootloader_state_machine
Description     :   Main state machine of bootloader - refer to algorithm
Input 1         :   u8: received byte
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
static void bootloader_state_machine(u8 state);




/*
Function name   :   clear_buffer
Description     :   Function clear buffer
Input 1         :   u8 *: buffer to be clear
Input 2         :   u16 : number of elements to be clear
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
static void clear_buffer(u8 *buf, u16 idx);




/*
Function name   :   jump_to_app
Description     :   Function only jump to user application
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
static void jump_to_app(void);




/*
Function name   :   erase_flash
Description     :   Function erase 64bit blocks
Input 1         :   u32: block address
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
static void erase_flash(u32 address);




/*
Function name   :   write_flash
Description     :   Function write data to flash
Input 1         :   u32: block address
Input 2         :   u8*: buffer with data to be written
Input 3         :   u8: quantity of data to be written
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
static void write_flash(u32 address, u8 *buffer, u8 length);




int main(int argc, char** argv)
{
    u32 hbeat = 0;

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
    
    /* Initialize receive buffer to 0xFF */
    for(hbeat=0; hbeat<PAGE_SIZE; hbeat++)
    {
        gbuffer[hbeat] = 0xFF;
    }

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




/*static void delay_ms(u16 duration)
{
  u16 i;
  u16 j;
  for (i = duration; i != 0; i--){
    for (j = 0; j <= 1000; j++){
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




static void UART_putchar(u8 outchar)
{
    while(!TXSTA1bits.TRMT);

    TXREG1 = outchar;

    while(!TXSTA1bits.TRMT);
}




static u8 UART_getchar(void)
{   
    if(RCIF){
        while(!RCIF);
        if( (RCSTAbits.FERR == 1) || (RCSTAbits.OERR == 1) ){
            RCSTAbits.CREN = 0; /* Clear an error */
            RCSTAbits.CREN = 1; /* Start continous receiving again */
        }
        return((u8)RCREG1);
    }
    else{
        return(0);
    }
}




static u8 UART_wait_for_char(void)
{   
    while(!RCIF);
    if( (RCSTAbits.FERR == 1) || (RCSTAbits.OERR == 1) ){
        RCSTAbits.CREN = 0; /* Clear an error */
        RCSTAbits.CREN = 1; /* Start continous receiving again */
    }
    return((u8)RCREG1);
}




static void send_boot_version(u8 *buf)
{
    u8 i = 0x00;

    do{
        if(buf[i] != '\0'){
            UART_putchar(buf[i]);
        }
    }while(buf[i++] != '\0');
}




static void bootloader_state_machine(u8 state)
{
    static u8 byte_in_record = 0x00;
    static u8 temp = 0x00;
    static u8 crc = 0x00;
    static u32 address = 0x00000000;
    static u16 offset = 0x00;
    static u16 record_type = 0x00;
    static u8 end_of_record = 0x00;
    static u16 byte_index = 0x00;
    static u16 page_index = 0x00;
    static u32 timeout = 0x00;
    static u8 i = 0;

    switch (state)
    {
        case 0xC1:{
            send_boot_version((u8 *)VERSION);	
            timeout = 0;
        break;
        }
        case 0xAA:{
            UART_putchar(0x55);
            state = 0x00;
        break;
        }
        case 0x31:{
            if(UART_wait_for_char() == 'Q'){
                if(UART_wait_for_char() == 'W'){
                    if(UART_wait_for_char() == 'E'){
                        if(UART_wait_for_char() == 'R'){
                            if(UART_wait_for_char() == 'T'){
                                for (u32 adr = APP_ENTRY; adr < FLASH_SIZE; adr += FLASH_ERASE_BLOCK){
                                    erase_flash(adr);
                                }
                                UART_putchar(0xEE);
                            }
                        }
                    }
                }
            }
            state = 0x00;
        break;
        }
        case 0x32:{	
            if(UART_wait_for_char() == 'Q'){
                if(UART_wait_for_char() == 'W'){
                    if(UART_wait_for_char() == 'E'){
                        if(UART_wait_for_char() == 'R'){
                            if(UART_wait_for_char() == 'T'){
                                /* Processing data which came from UART, byte after byte */
                                do{
                                    if(UART_wait_for_char() == RECORD_MARK){
                                        /* First byte is byte quantity in current record */
                                        byte_in_record = UART_wait_for_char();
                                        crc += byte_in_record; // Add to crc

                                        /* Get higher nibble of record address */
                                        temp = UART_wait_for_char();
                                        crc += temp; // Add to crc
                                        offset  = temp<<8;

                                        /* Get lower nibble of record address */
                                        temp = UART_wait_for_char();
                                        crc += temp; // Add to crc
                                        offset |= temp;

                                        /* Get record type */
                                        temp = UART_wait_for_char();
                                        crc += temp; // Add to crc
                                        record_type = temp;
                                        
                                        /* Check what kind of record came */
                                        switch (record_type){
                                            case DATA_RECORD:
                                                if((u32)address == (u32)0x00){
                                                    /* First data record detected concatenate offset and byte_in_record */
                                                    //address &= 0xFFFF0000;
                                                    address = (((u32)offset)+byte_in_record);
                                                }
                                                else if( (address) != (u32)offset){
                                                    
                                                    /* Perform separate write */
                                                    write_flash(address-byte_in_record, gbuffer, PAGE_SIZE);
                                                    clear_buffer(gbuffer,PAGE_SIZE);
                                                    
                                                    if(offset == 0xFF90)
                                                    {
                                                        byte_index = 1; 
                                                    }
                                                    if(offset == 0xFFA0)
                                                    {
                                                        byte_index = 1; 
                                                    }
                                                    if(offset == 0xFFB0)
                                                    {
                                                        byte_index = 1; 
                                                    }
                                                    if(offset == 0xFFC0)
                                                    {
                                                        byte_index = 1; 
                                                    }
                                                    if(offset == 0xFFD0)
                                                    {
                                                        byte_index = 1; 
                                                    }
                                                    if(offset == 0xFFE0)
                                                    {
                                                        byte_index = 1; 
                                                    }
                                                    if(offset == 0xFFF0)
                                                    {
                                                        byte_index = 1; 
                                                    }
                                                    
                                                    /* Concatenate offset with address */
                                                    /* Assign new address base on offset */
                                                    // address &= 0xFFFF0000;
                                                    address = (((u32)offset)+byte_in_record);
                                                    
                                                    page_index = address-byte_in_record;
                                                    
                                                    /* Set again buffer filing to zero */
                                                    byte_index = 0; 
                                                }
                                                else{                                            
                                                    address += (u32)byte_in_record;
                                                }
                                                /* One separate record analysis - data buffering */
                                                for(i=0; i<=byte_in_record; i++){
                                                    gbuffer[byte_index] = UART_wait_for_char();
                                                    crc += gbuffer[byte_index];
                                                    byte_index++;
                                                }
                                                byte_index--;
                                                if((crc&0xFF) == 0){
                                                    /* If collected page size byte, save page into flash */
                                                    if(byte_index >= PAGE_SIZE){
                                                        write_flash((page_index), gbuffer, PAGE_SIZE);
                                                        clear_buffer(gbuffer,PAGE_SIZE);
                                                        byte_index = 0x00;
                                                        page_index += PAGE_SIZE;
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
                                                break;
                                            case END_OF_FILE_RECORD:
                                                /* If all page filled save into flash */
                                                write_flash((address), gbuffer, PAGE_SIZE);
                                                /* Perform separate write */
                                              //  write_flash(address, gbuffer, PAGE_SIZE);
                                                clear_buffer(gbuffer,PAGE_SIZE);
                                                end_of_record = 1; /* Last record detected */
                                                UART_putchar(0xF0);
                                                break;
                                            case EXTENDED_SEGMENT_ADDRESS_RECORD:
                                                /* According to specification the address should 
                                                be calculated as follow: */

                                                /* Get higher nibble of record address */
                                                temp = UART_wait_for_char();
                                                crc += temp; // Add to crc
                                                offset  = temp<<8;

                                                /* Get lower nibble of record address */
                                                temp = UART_wait_for_char();
                                                crc += temp; // Add to crc
                                                offset |= temp;

                                                if((crc&0xFF) == 0){
                                                    UART_putchar(0xCC);
                                                    timeout = 0x00;
                                                }
                                                else{
                                                    UART_putchar(0xCE);
                                                }
                                                crc = 0;
                                                break;
                                            case START_SEGMENT_ADDRESS_RECORD:
                                                break;
                                            case EXTENDED_LINEAR_ADDRESS_RECORD:
                                                /* According to specification the address should 
                                                be calculated as follow: */
                                                
                                                /* Perform separate write */
                                                write_flash(page_index, gbuffer, PAGE_SIZE);
                                                clear_buffer(gbuffer,PAGE_SIZE);
                                                
                                                /* Get higher nibble of record address */
                                                temp = UART_wait_for_char();
                                                crc += temp; // Add to crc
                                                address  = (u32)temp<<24;

                                                /* Get lower nibble of record address */
                                                temp = UART_wait_for_char();
                                                crc += temp; // Add to crc
                                                address |= (u32)temp<<16;
                                                
                                                if((crc&0xFF) == 0){
                                                    UART_putchar(0xCC);
                                                    timeout = 0x00;
                                                }
                                                else{
                                                    UART_putchar(0xCE);
                                                }
                                                crc = 0;
                                                break;
                                            case START_LINEAR_ADDRESS_RECORD:
                                                /* TBD */
                                                break;
                                            default: /* Unsupported record type */
                                                break;
                                        }
                                    }
                                    timeout++;
                                    /* Leave loop when processing finish or detected wrong crc */
                                }while((end_of_record != 1)||(timeout==100));
                            }
                            state = 0x00;
                            page_index =0x00;
                            byte_index = 0x00;
                            offset = 0x00;
                            byte_in_record = 0x00;
                            record_type = 0x00;
                            timeout = 0x00;
                            end_of_record = 0x00;
                            crc = 0x00;
                        }
                    }
                }
            }
        break;
        }
        case 0x33:
        {
            UART_putchar(0x76);
            jump_to_app(); /* Jump to user application code */
        break;
        }
        default:
        {
            ;
        break;
        }
    }
}




static void clear_buffer(u8 *buf, u16 idx)
{
    u16 i = 0x00;
    for(i=0; i<idx; i++)
    {
        buf[i]=0xFF;
    }
}




static void jump_to_app(void)
{
    RCON |= 0x93; /* Set all flags as just after reset */
    asm("goto " ___mkstr(APP_ENTRY));
}




static void erase_flash(u32 address)
{
    PIR4bits.EEIF = 0;

    TBLPTRL = (address) & 0xFF;
    TBLPTRH = (address >> 8) & 0xFF;
    TBLPTRU = (address >> 16) & 0xFF;

    /*
     * bit 7, EEPGD = 1, memory is flash (unimplemented on J PIC)
     * bit 6, CFGS  = 0, enable acces to flash (unimplemented on J PIC)
     * bit 5, WPROG = 1, enable single word write (unimplemented on non-J PIC)
     * bit 4, FREE  = 0, enable write operation (1 if erase operation)
     * bit 3, WRERR = 0,
     * bit 2, WREN  = 1, enable write to memory
     * bit 1, WR    = 0,
     * bit 0, RD    = 0, (unimplemented on J PIC)
     */

    EECON1 = 0xA4; /* 0b10100100 */

    EECON1bits.FREE = 1;    /* perform erase operation */
    EECON2 = 0x55;          /* unlock sequence */
    EECON2 = 0xAA;          /* unlock sequence */
    EECON1bits.WR = 1;      /* start write or erase operation */
    EECON1bits.FREE = 0;    /* back to write operation */

    INTCONbits.GIE = 1;

    while (!PIR4bits.EEIF);
    PIR4bits.EEIF = 0;
    EECON1bits.WREN = 0;

    INTCONbits.GIE = 0;
}




static void write_flash(u32 address, u8 *buffer, u8 length)
{
    int counter;

    PIR4bits.EEIF = 0;

    TBLPTRL = (address) & 0xFF;
    TBLPTRH = (address >> 8) & 0xFF;
    TBLPTRU = (address >> 16) & 0xFF;

    /* The programming block is 32 bytes for all chips except x5k50 */
    /* The programming block is 64 bytes for x5k50 */

    /* Load max. 64 holding registers*/
    for (counter = 0; counter < length; counter++) {
      TABLAT = buffer[counter]; /* present data to table latch */
      /* write data in TBLWT holding register */
      /* TBLPTR is incremented after the read/write */
      #asm
          TBLWT*+
      #endasm;
    }

    /* One step back to be inside the 64 bytes range */
    #asm
        TBLRD*-
    #endasm;
    

    /* Start block write */
    /*
     * bit 7, EEPGD = 1, Flash Program or Data EEPROM Memory Select bit (1-Access Flash program memory)
     * bit 6, CFGS  = 0, Flash Program/Data EEPROM or Configure Select bit (0-Access Flash progrm or data EEPROM memory)
     * bit 5, -     = x, Bit unimplemented on PIC18Fxxkxx microcontrollers
     * bit 4, FREE  = 0, Flash Row Erase Enable bit (0-Performs write only)
     * bit 3, WRERR = 0, Flash Program/Data EEPROM Error flag bit (0-The write operation completed)
     * bit 2, WREN  = 1, Flash Program/Data EEPROM Write Enable bit (1-Allows write cycles to Flash program/data EEPROM) 
     * bit 1, WR    = 0, Write Control bit (0-Write cycle to the EEPROM is complete)
     * bit 0, RD    = 0, Read Control bit (0-Does not initiate an EEPROM read)
     */

    EECON1 = 0x84; /* 0b10000100 */

    INTCONbits.GIE = 0;

    EECON2 = 0x55;
    EECON2 = 0xAA;

    EECON1bits.WR = 1;  /* WR = 1; start write or erase operation */
                        /* WR cannot be cleared, only set, in software */
                        /* It is cleared in hardware at the completion */
                        /* of the write or erase operation */
                        /* CPU stall here for 2ms */

    INTCONbits.GIE = 1;

    while (!PIR4bits.EEIF);
    PIR4bits.EEIF = 0;
    EECON1bits.WREN = 0;
}