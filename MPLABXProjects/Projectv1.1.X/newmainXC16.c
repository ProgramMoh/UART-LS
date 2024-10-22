/*
 * File:   main.c
 * Author: STUDENT NAMES HERE
 *
 * Created on November 2, 2023, 9:26 AM
 */



// PIC24F16KA101 Configuration Bit Settings

/*
 * I recommend adding the FWDTEN setting to OFF pragma, as this stops your microcontroller periodically resetting itself when nothing happens
 */


#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))

// FOSCSEL
#pragma config FNOSC = FRCDIV           // Oscillator Select (8 MHz FRC oscillator with divide-by-N (FRCDIV))
#pragma config IESO = ON                // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-Speed Start-up enabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = OFF           // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
/*
 * Adding the FCKSM = CSECMD enables clock switching
 */
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)
/*
 * This pragma enables using the stepping debugger/setting breakpoints on the microcontroller
 */
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (PGC2/PGD2 are used for programming and debugging the device)

#include "clkChange.h"
#include "uart.h"
#include "math.h"
#include "xc.h"
#include "string.h"

#define BUF_SIZE 10
extern uint8_t RXFlag;

char receive_buffer[BUF_SIZE];

//every single variable has a detailed description of how and when theyre used in the project documentation. Please refer to that for everything.

uint16_t Flag = 0;
uint16_t passchange1 = 0;
uint16_t passchange2 = 0;
uint16_t passchange3 = 0;
uint16_t potcheck = 0;
uint16_t transition = 0;

uint16_t pb3gameinit = 0;

char passarray[3] = {'s', 's', 's'};

uint16_t pot = 0;
uint16_t waitbit = 0;

uint16_t Flag1 = 0;
uint16_t Flag2 = 0;
uint16_t Flag3 = 0;
uint16_t incorrect = 0;
uint16_t unlocked = 0;
uint16_t gamestart = 0;
uint16_t hard = 0;
uint16_t lockout = 0;
uint16_t hardmode = 1;
uint16_t easymode = 0;
uint16_t error = 0;
uint16_t set = 0;
uint16_t flag = 0;
uint16_t success = 0;
uint16_t clkset = 0;
uint8_t finished = 0;
uint8_t pb1_pressed = 0;
uint8_t pb2_pressed = 0;
uint8_t pb3_pressed = 0;
uint16_t do_ADC(void)
{
    uint16_t ADCvalue ; // 16 bit register used to hold ADC converted digital output ADC1BUF0
/* ------------- ADC INITIALIZATION ------------------*/
 // Configure ADC by setting bits in AD1CON1 re
    //AD1CON1bits.ADON = 1;
    AD1CON1bits.FORM = 0b00;
    AD1CON1bits.SSRC = 0b111;
    AD1CON1bits.DONE = 1;
    AD1CON1bits.ASAM = 0;
    //AD1CON1bits.SAMP = 1;
    AD1CON2bits.VCFG = 0b000; // Selects AVDD, AVSS (supply voltage to PIC) as Vref
    AD1CON2bits.CSCNA = 0;
    AD1CON2bits.SMPI = 0000;
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.ALTS = 0;
    AD1CON3bits.ADRC = 0;
    AD1CON3bits.SAMC = 11111;
    AD1CHSbits.CH0NA = 0;
    AD1CHSbits.CH0SA = 0b0101;
    //AD1PCFG = 0;
    AD1CSSL = 0;
    
    //AD1CON3bits.ADCS = 11111; FILLER NUMBER
//    AD1CHSbits.CH0NB = 0;
//    AD1CHSbits.CH0SB = 0101;
//    AD1PCFGbits.PCFG12 = 0; //upper and lower bits
//    AD1PCFGbits.PCFG11 = 0;
//    AD1PCFGbits.PCFG10 = 0;
//    AD1PCFGbits.PCFG5 = 0;
//    AD1PCFGbits.PCFG4 = 0;
//    AD1PCFGbits.PCFG3 = 0;
//    AD1PCFGbits.PCFG2 = 0;
//    AD1PCFGbits.PCFG1= 0;
//    AD1PCFGbits.PCFG0 = 0;
//    AD1CSSLbits.CSSL12 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL11 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL10 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL5 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL4 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL3 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL2 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL1 = 0; //upper and lower bits
//    AD1CSSLbits.CSSL0 = 0; //upper and lower bits
    AD1CSSL = 0;
    
 // Configure ADC by setting bits in AD1CON2
   // AD1CON3bits.ADRC = 0; // Use system clock
 //Configure the ADC?s sample time by setting bits in AD1CON3
 // Ensure sample time is 1/10th of signal being sampled
 // Select and configure ADC input
/* ------------- ADC SAMPLING AND CONVERSION ------------------*/
    AD1CON1bits.ADON = 1; // turn on ADC module
    AD1CON1bits.SAMP=1; //Start Sampling, Conversion starts automatically after SSRC and SAMC settings
    while(AD1CON1bits.DONE==0)
    {}
    ADCvalue = ADC1BUF0; // ADC output is stored in ADC1BUF0 as this point
    AD1CON1bits.SAMP=0; //Stop sampling
    AD1CON1bits.ADON=0; //Turn off ADC, ADC value stored in ADC1BUF0;
    //IPC3bits.AD1IP = 3;
    //IEC0bits.AD1IE = 1;
    return (ADCvalue); //returns 10 bit ADC output stored in ADC1BIF0 to calling function
}


void newClk(unsigned int clkval) {
    char COSCNOSC;
    switch(clkval) {
        case 8: // 8 MHz
            COSCNOSC = 0x00;
            break;
        case 500: // 500 kHz
            COSCNOSC = 0x66;
            break;
        case 32: // 32 kHz
            COSCNOSC = 0x55;
            break;
        default:
            COSCNOSC = 0x55;
    }
    
        if(easymode){
    U2BRG = 12; //gives a baud rate of 4807 Baud with 500kHz clock; Set Baud to 4800 on realterm
    
    }else if(hardmode){
    U2BRG = 207; //gives a baud rate of 4807 Baud with 8MHz clock; Set Baud to 4800 on realterm
    }
    
    SRbits.IPL = 7;
    CLKDIVbits.RCDIV = 0;
    __builtin_write_OSCCONH(COSCNOSC);
    __builtin_write_OSCCONL(0x01);
//    OSCCONbits.OSWEN = 1;
    while(OSCCONbits.OSWEN==1) {}
    SRbits.IPL = 0;
}

void BarGenerator(uint64_t ADCResult){
    uint64_t BarWidth = ADCResult * 9 / 1024;
    potcheck = 1;
    //Disp2Dec(pb2_pressed);
    if(BarWidth==4&&!PORTBbits.RB4){
        pot = 1;
                    Disp2String("Device Unlocked, Press PB1 to lock the system.");
                            XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        Disp2String("Press PB2 to enter a new password.");
                                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        Disp2String("Press PB3 to enter special mode.");
                                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        potcheck = 0;
    }else if(BarWidth!=4&&!PORTBbits.RB4){
        pot = 1;
                    Disp2String("Incorrect, Press PB2 to reset...");
                            XmitUART2('\n' , 1);
                            incorrect = 1;
        potcheck = 0;
    }

    
    for (int i = 0; i < BarWidth; i++){
        if(!pot&&!incorrect){
        XmitUART2('=' , 1);
        }
    }  
    if(!pot&&!incorrect){
        Disp2Dec(BarWidth , 1);
        XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
    }

}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{
    IFS0bits.AD1IF = 0; // Clear the ADC1 Interrupt Flag
}


/*
 * File:   uart.c
 * Author: psbta
 *
 * Created on October 10, 2023, 4:10 PM
 */




uint8_t received_char = 0;
uint8_t RXFlag = 0;
//uint16_t Flag = 0;

// extern uint16_t CNflag; // uncomment if CNflag is implemented to break out of the busy wait for new input

void InitUART2(void) 
{
	// configures UART2 module on pins RB0 (Tx) and RB1 (Rx) on PIC24F16KA101 
	// Enables UART2 
    
    // TARGET: 4800 baud @ 500 kHz  FOSC
    
	U2MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U2MODEbits.IREN = 0;	// Bit12 No IR translation
	U2MODEbits.RTSMD = 0;	// Bit11 Flow Control Mode Mode
	U2MODEbits.UEN = 00;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U2MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
	U2MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U2MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U2MODEbits.RXINV = 0;	// Bit4 Idle state is '1'
	U2MODEbits.BRGH = 1;	// Bit3 16 clocks per bit period
	U2MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
	U2MODEbits.STSEL = 0;	// Bit0 One Stop Bit
    
  //  if (OSCCONbits.COSC == 0b110)
	//{
	//	U2BRG = 12;	// gives a baud rate of 4807.7 Baud with 500kHz clock; Set Baud to 4800 on realterm
	//}
	//else if (OSCCONbits.COSC == 0b101)
	//{
//		U2BRG = 12;	// gives a baud rate of 300 Baud with 32kHz clock; set Baud to 300 on realterm
//	}
//	else if (OSCCONbits.COSC == 0b000)
//	{
//		U2BRG=103;	// gives a baud rate of 9600 with 8MHz clock; set Baud to 9600 on real term
//	}
 

    U2STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
    U2STAbits.UTXISEL0 = 0;	//Generate interrupt with last character shifted out of U2TXREG buffer
	U2STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U2STAbits.UTXBRK = 0;	//Bit11 Disabled
	U2STAbits.UTXEN = 1;	//Bit10 TX pins controlled by periph
//	U2STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
//	U2STAbits.TRMT = 0;		//Bit8 *Read Only bit*
	U2STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U2STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
//	U2STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
//	U2STAbits.PERR = 0;		//Bit3 *Read Only Bit*
//	U2STAbits.FERR = 0;		//Bit2 *Read Only Bit*
//	U2STAbits.OERR = 0;		//Bit1 *Read Only Bit*
//	U2STAbits.URXDA = 0;	//Bit0 *Read Only Bit*

	
    IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
    IPC7bits.U2TXIP = 3; // UART2 TX interrupt has interrupt priority 3-4th highest priority
    
	IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
	IPC7bits.U2RXIP = 7;    //UART2 Rx interrupt has 2nd highest priority
    IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts

	U2MODEbits.UARTEN = 1;	// And turn the peripheral on
    
//	U2STAbits.UTXEN = 1;
}



void IOinit() {
    //NewClk(500);
       
    //Input port initialization
    TRISAbits.TRISA4 = 1; // RA4
    TRISBbits.TRISB4 = 1; // RB4
    TRISAbits.TRISA2 = 1;  // RA2

    //Pull-up configuration
    CNPU2bits.CN30PUE = 1;  // RA2/CN30 in pull up config
    CNPU1bits.CN0PUE = 1; // RA4/CN0 in pull up config
    CNPU1bits.CN1PUE = 1; // RB4/CN1 in pull up config

    //I/O change notifications enables
    CNEN1bits.CN0IE = 1; // RA2/CN30 change notification enable
    CNEN1bits.CN1IE = 1; // RB4/CN30 change notification enable
    CNEN2bits.CN30IE = 1; // RA4/CN30 change notification enable

    IFS1bits.CNIF = 0; //Clear flag
    IEC1bits.CNIE = 1; //Enable CN interrupts
    IPC4bits.CNIP = 6; //Priority of interrupt

///    AD1PCFG = 0xFFFF; // Turn off analog input on all pins

    TRISBbits.TRISB8 = 0; // RB8 is an output (LED)
    
    
}

void Disp2String(char *str) //Displays String of characters
{
    unsigned int i;
    for (i=0; i<= strlen(str); i++)
    {
        XmitUART2(str[i],1);
    }

    return;
}

void Disp2Char(char *str) //Displays String of characters
{
    
        XmitUART2(str,1);
    

    return;
}

void XmitUART2(char CharNum, unsigned int repeatNo)
{	
	
	U2STAbits.UTXEN = 1;
	while(repeatNo!=0) 
	{
		while(U2STAbits.UTXBF==1)	//Just loop here till the FIFO buffers have room for one more entry
		{
			// Idle();  //commented to try out serialplot app
		}	
		U2TXREG=CharNum;	//Move Data to be displayed in UART FIFO buffer
		repeatNo--;
	}
	while(U2STAbits.TRMT==0)	//Turn off UART2 upon transmission of last character; also can be Verified in interrupt subroutine U2TXInterrupt()
	{
	}

    U2STAbits.UTXEN = 0;
}

/************************************************************************
 * Receive a buf_size number of characters over UART
 * Description: This function allows you to receive buf_size number of characters from UART,
 * provided the UART module has been initialized correctly. The function currently returns
 * if the "enter" key (ASCII 0x0D) is received. The function does not handle receiving
 * the DELETE or BACKSPACE keys meaningfully. 
 * 
 * Note: there is commented-out skeleton code that could be (re)implemented to allow the function
 * return early without the ENTER key given an interrupt-set global flag. 
 ************************************************************************/
void RecvUart(char* input, uint8_t buf_size)
{	
    uint16_t i = 0;
    char last_char;
    // wait for enter key
    while (last_char != 0x0D) {
        if (RXFlag == 1) {
            // only store alphanumeric characters
            if (received_char >= 32 && received_char <= 126) {
                if (i > buf_size-2) {
                    Disp2String("\ntoo long\n\r");
                    RXFlag = 0;
                    return;
                }
                input[i] = received_char;
                i++;
                XmitUART2(received_char,1); // loop back display
                U2STAbits.OERR = 0;
            }
            last_char = received_char;
            RXFlag = 0;
        }
        // wait for next character
        
        // if (CNflag == 1) { // this allows breaking out of the busy wait if CN interrupts are enabled...
        //     add logic here
        // }
    }
}

/************************************************************************
 * Receive a single (alphanumeric) character over UART
 * Description: This function allows you to receive a single character, which will only be 
 * "accepted" (returned) after receiving the ENTER key (0x0D). 
 * While receiving characters, the program is designed to send back the received character.
 * To display this, it sends a BACKSPACE (0x08) to clear the previous character from the 
 * receiving terminal, before sending the new current character. 
 * 
 * Note: there is commented-out skeleton code that could be (re)implemented to allow the function
 * return early without the ENTER key given an interrupt-set global flag. 
 ************************************************************************/
char RecvUartChar()
{	
    char last_char;
    XmitUART2(' ',1);
    // wait for enter key
    while (last_char != 0x0D) {
        if (RXFlag == 1) {
            
            // return the last character received if you see ENTER
            if (received_char == 0x0D) {
                RXFlag = 0;
                return last_char;
            }
            
            // only store alphanumeric characters
            if (received_char >= 32 && received_char <= 126) {
                XmitUART2(0x08,1); // send backspace
                last_char = received_char;
                XmitUART2(received_char,1); // loop back display
            }
           
            U2STAbits.OERR = 0;
            RXFlag = 0;
        }
        
        // if (CNflag == 1) { // this allows breaking out of the busy wait if CN interrupts are enabled...
        //     add logic here
        // }
    }
}

void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void) {

	IFS1bits.U2RXIF = 0;
    transition = 1;
    received_char = U2RXREG;
    //if(IFS0bits.T2IF == 0){
        
    
               // Disp2String("Test");
                //Disp2Char(received_char);
    
    if(pb3gameinit){
            if(received_char=='h'&&!set){ //hard mode declaration
                set = 1;
        Disp2String("\nHard mode selected \n\r");
        hardmode = 1;
        easymode = 0;
        error = 0;
        
    }else if(received_char=='e'&&!set){ //easy mode declaration
        set = 1;
        Disp2String("\nEasy mode selected \n\r");
        easymode = 1;
        hardmode = 0;
        error = 0;
    }else{
        if(set){
        Disp2String("\n Incorrect format \n\r");
        error = 1;
        }else{
        //Disp2String("\n Difficulty already set \n\r");
 
        }
    }
    }

    
    if(passchange3){
        Disp2String("Third bit set"); 
                                XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
                Disp2String("Password Change Successful");
                                        XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);


        waitbit = 1;
        passarray[0] = received_char;
        passchange3 = 0;
    }
    
    if(passchange2){
        Disp2String("Second bit set");
                                XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        passarray[1] = received_char;
        passchange2 = 0;
        passchange3 = 1;
    }
    
    if(passchange1){  
        Disp2String("First bit set"); //this checks the recieved_char and sets it to the new password array, it zeros its change bit and waits to be recalled
                                XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        passarray[2] = received_char;
        passchange1 = 0;
        passchange2 = 1;
    }
    
    
    

    if(!incorrect){
    if(Flag1&&Flag2&&!Flag3){
        if(received_char==passarray[0]){ //this checks the passarray to verify the password
            Flag3 = 1;
            //incorrect = 0
            Disp2String("Correct 3rd character. Enter Unlock Number: ");
                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        }else{
            //Disp2String("Incorrect 3");
            incorrect = 1;
            Flag1 = 0;
            Flag2 = 0;
            Flag3 = 0;
        }
    }
    }
     
    if(!incorrect){
    if(Flag1&&!Flag2&&!Flag3){
        //Disp2String("Test2");
        if(received_char==passarray[1]){ //this checks the passarray to verify the password
            Flag2 = 1;
            //incorrect = 0;
            Disp2String("Correct 2nd character. Enter 3rd character: ");
                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);

        }else{
            //Disp2String("Incorrect 2");
            incorrect = 1;
            Flag1 = 0;
            Flag2 = 0;
        }
    }
    }
    
    if(!incorrect){
    if(!Flag1&&!Flag2&&!Flag3){
                     //   Disp2String("Test2");
        if(received_char==passarray[2]){ //this checks the passarray to verify the password
            Flag1 = 1;
            //incorrect = 0;
            Disp2String("Correct 1st character. Enter 2nd character: ");
                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        }else{
            incorrect = 1;

            //Disp2String("Incorrect 1");

            Flag1 = 0;
        }
    }
    }
    
    
        
    
    RXFlag = 1;
   // }
}

void __attribute__ ((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
	IFS1bits.U2TXIF = 0;

}

// Displays 16 bit number in Hex form using UART2
void Disp2Hex(unsigned int DispData)
{
    char i;
    char nib = 0x00;
    XmitUART2(' ',1); // Disp Gap
    XmitUART2('0',1); // Disp Hex notation 0x
    XmitUART2('x',1);
    for (i=3; i>=0; i--)
        {
        nib = ((DispData >> (4*i)) & 0x000F);
    if (nib >= 0x0A)
        {
    nib = nib +0x37; //For Hex values A-F
    }
    else    
    {
    nib = nib+0x30; //For hex values 0-9
    }
    XmitUART2(nib,1);
    }
    XmitUART2(' ',1);
    DispData = 0x0000; // Clear DispData
    return;
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{

    
    
    
    if(IFS1bits.CNIF == 1) //check if interrupt flag set bit is high (CN has happened)
    {
        if ((PORTAbits.RA2 == 0) && (PORTAbits.RA4 == 0)) { //check which PB is current active and change its state
            Disp2String("NU UH");
            pb1_pressed = 1;
            pb2_pressed = 1;
            pb3_pressed = 0;           
        } else if (!PORTAbits.RA2) {
                       // Disp2String("PB1");
            //IFS0bits.T2IF = 0;
            if(pb3gameinit){ //if the game has started and is waiting in idle, and pb1 is pressed reset the game and lock, also change the baud
                                       Disp2String("Game Reset \n\r");

                            easymode = 0; 
                hardmode = 0;
                error = 0;
                set = 0;
                gamestart = 0;
                pb3gameinit = 0;
                newClk(500); 
                U2BRG = 12;
            }
                        Flag1 = 0; //when button is pressed reset all flags
            Flag2 = 0;
            Flag3 = 0;
            pot = 0;
            incorrect = 0;
            LATBbits.LATB8 = 1;
            potcheck = 0;
            pb1_pressed = 1;
            pb2_pressed = 0;
            pb3_pressed = 0;
            XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
            Disp2String("Device Locked");
                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
                Disp2String("Welcome! Enter 1st character: ");
        XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
         //           Disp2String("Welcome! Enter 1st character: ");
       // XmitUART2('\n' , 1);
       // XmitUART2('\r' , 1);
            
                    LATBbits.LATB8 = 1;

        //            XmitUART2('\n' , 1);
       // XmitUART2('\r' , 1);
            Flag1 = 0;
            Flag2 = 0;
            Flag3 = 0;
            pot = 0;
        } else if (!PORTAbits.RA4) {
                       // Disp2String("PB3");
                        if(flag&&!PORTAbits.RA4){ //this is the flag refrence so that the game can check if youve won or lost.
                                Disp2String("\n Success!!! \n\r");
                                 Disp2String("Game over now idling, press PB3 to reset or press PB1 to lock device... \n\r");
                success = 1;
        }else if(!flag&&!success&&!PORTAbits.RA4){
                           // Disp2String("\n Failure \n\r");

            success = 0;
        }else{
            success = 0;
        }
                       
            
                       // if(LATBbits.LATB8&&gamestart){
                         //   Disp2String("damn you fast");
                           //         XmitUART2('\n' , 1);
                         //XmitUART2('\r' , 1); 
                         //finished = 1;
                         //gamestart = 0;
                        //}else if(!LATBbits.LATB8&&gamestart){
                        //    Disp2String("youre too slow");
                          //          XmitUART2('\n' , 1);
                         //XmitUART2('\r' , 1); 
                         //finished = 1;
                         //gamestart = 0;
                       // }
                        
                        //if(finished){
                        //    finished = 0;
                      //  }
                        
            pb1_pressed = 0;
            pb2_pressed = 0;
            pb3_pressed = 1;
            //Disp2String("test1");
            if(Flag1&&Flag2&&Flag3&&pot&&!pb3gameinit){ //start game check when everything is unlocked and game hasnt started
              //              Disp2String("test2");
                pb3gameinit = 1;
            }
            if(Flag1&&Flag2&&Flag3&&pot&&pb3gameinit&&finished){ //reset the game
                finished = 0;
            }
    }
         else if (!PORTBbits.RB4) {
                     //   Disp2String("test");

          //  Disp2String("PB2"); //this is the device reset, all flags are reset here
            if(incorrect){      
            incorrect = 0;
            Disp2String("Device Reset");
                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        Disp2String("Welcome! Enter 1st character: ");
                    XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
            Flag1 = 0;
            Flag2 = 0;
            Flag3 = 0;
            pot = 0;
                        LATBbits.LATB8 = 1;
            potcheck = 0;
            incorrect = 0;
            }
                           // Disp2String("test dump");
                            //Disp2Dec(Flag1);
                            //Disp2Dec(Flag2);
                            //Disp2Dec(Flag3);
                            //Disp2Dec(potcheck);
            if(Flag1&&Flag2&&Flag3&&!incorrect&&!potcheck){
                Disp2String("Password Change Initialized");
                        XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
                passchange1 = 1;
            }
            pb1_pressed = 0;
            pb2_pressed = 1;
            pb3_pressed = 0;
        } else {
            pb1_pressed = 0;
            pb2_pressed = 0;
            pb3_pressed = 0;        
        }
    }
    IFS1bits.CNIF = 0; // clear IF flag
    Nop();
} 





 //Displays 16 bit unsigned in in decimal form
void Disp2Dec(uint64_t Dec_num)
{
    uint8_t rem; //remainder in div by 10
    uint16_t quot;
    uint8_t ctr = 0; //counter
    XmitUART2(' ',1); // Disp Gap
    while(ctr<5)
    {
        quot = Dec_num/(pow(10,(4-ctr)));
        rem = (quot%10);
        XmitUART2(rem + 0x30 , 1);
        ctr = ctr + 1;
    }
    XmitUART2(' ',1); // Disp Gap
    return;
}


void delay_ms(uint16_t time_ms){

    T2CONbits.T32 = 0; //operate t2 in 16 bit mode
    T2CONbits.TCKPS = 0b00; //pre-scalar
    T2CONbits.TCS = 0; //use internal clock
    T2CONbits.TSIDL = 0; //operate in idle mode
    IPC1bits.T2IP = 2;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    
    switch(time_ms){
        case 40:
            PR2 = 40000; //these are different PR2 values to change the blinking speed
            T2CONbits.TON = 1; 
            TMR2 = 0;
            break;
        case 30:
            PR2 = 25000;
            T2CONbits.TON = 1; 
            TMR2 = 0;
            break; 
        case 20:
            PR2 = 10000;
            T2CONbits.TON = 1; 
            TMR2 = 0;
            break;   
        case 50:
            PR2 = 50000;
            T2CONbits.TON = 1; 
            TMR2 = 0;
            break;       
        
    }    
    
    Idle(); 
    return;
}

void __attribute__((interrut,no_auto_psv)) _T2Interrupt(void){ //required function T2 ISR, this is just whats in the notes needs to be adjusted (i think?)   
    IFS0bits.T2IF = 0;
    T2CONbits.TON = 0; 
}
//void countdown(){
//            Disp2String("Countdown initialted: ");
//                    XmitUART2('\n' , 1);
//        XmitUART2('\r' , 1);
//
//    
//        for (int i = 0; i < 5; i++){   
//        delay_ms(50);
//        LATBbits.LATB8 ^= 1;
//        }
//        
//    
//        
//        //LATBbits.LATB8 ^= 1;
//        XmitUART2('\n' , 1);
//        XmitUART2('\r' , 1);
//    }
//        


int main(void) {
    
    newClk(500); //sets the clock to initial value
    U2BRG = 12; //appropriate for baud calculation of 4800
    
    InitUART2(); //uart init
    IOinit(); //io init
    //double vr = 3000/1024;
    AD1PCFG = 0xFFFF; // Make sure the UART RX bit is set to digital! //sets the pins to digital
    
    incorrect = 0;
    
//    Disp2String("test\n\r");
    while(1) {
                uint64_t x = do_ADC();

        if(!Flag1&&!Flag2&&!Flag3&&!pot){
        LATBbits.LATB8 = 1; //turns the led on when no flags are set
        }
        if(Flag1&&!Flag2&&!Flag3&&!pot){ //checks flag1 is set and all the others arent
        int i = 0; //flashes the led at a given rate according to delay_ms, will flash 100 times until timeout or a button is pressed
        while (i < 100 && !transition && !IFS1bits.CNIF) {
            if(i<98){
        delay_ms(50);
        LATBbits.LATB8 ^= 1;
        i++;
        //Disp2String("Time Remaining : ");
           //     Disp2Dec(100 - i);
        //XmitUART2('\n' , 1);
        //XmitUART2('\r' , 1);
        }else{
                if(Flag1&&!Flag2&&!Flag3&&!pot){
              incorrect = 1; //timeout bit
        Disp2String("1 Incorrect, Press PB2 to reset..."); //user reset prompt over uart
                XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        uint16_t i = 0;
        while (i < 20) {
  
        delay_ms(40);
        i++;   
        }
        }
        }
        }
        if(transition){
        transition = 0;
        }
        }
                        
        
        if(Flag1&&Flag2&&!Flag3&&!pot){ //checks flag1 and flag2 are set and all the others arent
        int i = 0;
        while (i < 100 && !transition && !IFS1bits.CNIF) {
            if(i<98){
        delay_ms(40);
        LATBbits.LATB8 ^= 1;
        i++;   
//Disp2String("Time Remaining : ");
       //         Disp2Dec(100 - i);
        //XmitUART2('\n' , 1);
        //XmitUART2('\r' , 1);
        }else{
                if(Flag1&&Flag2&&!Flag3&&!pot){
                                incorrect = 1;
        Disp2String("1 Incorrect, Press PB2 to reset...");
        Disp2Dec(Flag1);
        Disp2Dec(Flag2);
        Disp2Dec(Flag3);
        Disp2Dec(pot);
                XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        uint16_t i = 0;
        while (i < 20) {
  
        delay_ms(40);
        i++;   
        }
        }
        }
        }
        if(transition){
        transition = 0;
        }
        }
        
    
        if(Flag1&&Flag2&&Flag3&&!pot){ //checks flag1 flag2 and flag 3 are set and pot is not
        int i = 0;
        while (i < 100 && !pot && !IFS1bits.CNIF) {
            if(i<98){
        delay_ms(20);
        LATBbits.LATB8 ^= 1;
        i++;   
       // if(!incorrect){
        BarGenerator(do_ADC());
       // }
        //       Disp2String("Time Remaining : ");
          //      Disp2Dec(100 - i);
        //XmitUART2('\n' , 1);
        //XmitUART2('\r' , 1);
        }else{     
                                incorrect = 1;
        Disp2String("2 Incorrect, Press PB2 to reset...");
        Disp2Dec(Flag1);
        Disp2Dec(Flag2);
        Disp2Dec(Flag3);
        Disp2Dec(pot);
                XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        uint16_t i = 0;
        while (i < 20) {
  
        delay_ms(40);
        i++;   
        }
        }
        
            //else{
          //  incorrect = 1;
          //              Disp2String("test\n\r");

       // }
        }
        if(transition){
        transition = 0;
        }      
        }
        
        if(Flag1&&Flag2&&Flag3&&pot){ //all flags are set, the device is unlocked, waiting for the next state
          
        int i = 0;
        while (i < 100 && !pb1_pressed&&!pb3gameinit) {
            
        delay_ms(10);
        LATBbits.LATB8 ^= 1;
        i++;
        }
        }
                
               
        
        
        

        if(!Flag1&&!Flag2&&!Flag3&&!incorrect&&!IFS0bits.T2IF&&!pb2_pressed&&!pb1_pressed&&!pb3gameinit&&!finished){ //welcome message when system is booted and in locked mode
        Disp2String("Welcome! Enter 1st character: ");
        XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        //countdown();
        
       

        Idle(); //idle to wait for user cn or uart interrupt
        }
 
        //if(Flag1&&Flag2&&Flag3&&!incorrect){
        //Disp2String("Unlocked");
        //Idle();
        //}
        
        if(incorrect){ //one of many incorrect conditions
 
        Disp2String("3 Incorrect, Press PB2 to reset...");
                XmitUART2('\n' , 1);
        XmitUART2('\r' , 1);
        uint16_t i = 0;
        while (i < 20) {
  
        delay_ms(40);
        i++;   
        }
        //delay_ms(20);
        if(pb2_pressed){
            incorrect = 0;
        }else{
                       // T2CONbits.TON = 0; 

            Idle();
        }
        }
        
        if(waitbit){ //waitbit helps with the 5s transition back to the unlocked state
            waitbit = 0;
                            for (int i = 0; i < 20; i++){
                            delay_ms(50);
                            }
            
                    Disp2String("Returning to main menu");

        }
        
        
if(pb3gameinit){ //this is the heart of the PB3 game function, when pb3 is pressed in the CN interrupt the game can begin
    if(!gamestart&&!finished){ //checks the current state of the game, if were finished or if weve just started. This helps with setting the difficulty and clock timer bits
            gamestart = 1;
            LATBbits.LATB8 = 0;
            Disp2String("Enter 'h' for hard mode, and 'e' for easy mode \n\r");
        }
        if((hardmode||easymode)&&set){ //this waits for the set bit, which only changes after the clock is changed
           // set = 1;
            if(!finished){
            Disp2String("Get ready the game is about to begin! \n\r");
            }
            if(hardmode){ //when hardmode is selected execute this section
                newClk(8);   //change the clock to 8mhz
                clkset = 1;
//                for(int i = 0; i<100; i++){
//                delay_ms(50);
//                delay_ms(50);
//                delay_ms(50);
//                LATBbits.LATB8 ^= 1; //off any idle mode
//                }
            }else if(easymode){
                newClk(500);   //when easymode is selected execute this section, change the clock to 500khz
                clkset = 1;
//                for(int i = 0; i<100; i++){
//                delay_ms(50);
//                LATBbits.LATB8 ^= 1; //off any idle mode
//                }
            }
        }
        
        if(clkset&&!finished){ //when clock is set and the game isnt over
            if(easymode){
            delay_ms(50); //this section needs a bit of clarification, when looping I was having trouble interrupting out of my delays
            delay_ms(50); //to prevent this i simply am calling these function over and over. This is not a very appealing interpretation however
            Disp2String("3... \n\r"); //its nessicary to insure interrupts can occur the second the flag has gone high, and not wait for a LED cycle.
            delay_ms(50);
            delay_ms(50);
            Disp2String("2... \n\r");
            delay_ms(50);
            delay_ms(50);
            Disp2String("1... \n\r");
            delay_ms(50);
            delay_ms(50);
            Disp2String("GO!!! \n\r");
            clkset = 0;
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50); //flag is default off
            flag = 1; //check flag for PB3 refrence via CN interrupt
            LATBbits.LATB8 = 1; //led on
            //flag = 1;
            
            delay_ms(50);
            delay_ms(50);
            flag = 0; //turn flag off
            LATBbits.LATB8 = 0; //led off
            flag = 0;
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            finished = 1;
            }
            else if(hardmode){
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            Disp2String("3... \n\r");
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
                       delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            Disp2String("2... \n\r");
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
                       delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            Disp2String("1... \n\r");
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
                       delay_ms(50);
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            Disp2String("GO!!! \n\r");
            clkset = 0;
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
                       delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            flag = 1;
            LATBbits.LATB8 = 1; //off any idle mode
            //flag = 1;
            
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            flag = 0;
            LATBbits.LATB8 = 0; //off any idle mode
            flag = 0;
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);           delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
                       delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            delay_ms(50);
            finished = 1;
            }
            //Idle();
        }
        if(finished){ //when finished determine if successful
           // uint16_t stopper = 0;
            if(!success){ //if non successful then display failure
                                                            Disp2String("Failure... \n\r");
                                                             Disp2String("Game over now idling, press PB3 to reset or press PB1 to lock device... \n\r");
            }
            //if(!stopper){
            //Disp2String("Game over now idling, press PB3 to reset or press PB1 to lock device... \n\r");
           // stopper = 1;
            //}
            //finished = 0;
                
            Idle(); //wait in idle for the PB3 button press to occur and reset the program or for the PB1 to occur and relock the device

        }
            
    //return 0;
}
        
    }
    
    
    return 0;
}


