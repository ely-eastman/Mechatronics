#include<xc.h>// processor SFR definitions
#include<sys/attribs.h>
#include "spi.h"
#include<stdio.h>// __ISR macro
#include<math.h>

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

void readUART1(char * string, int maxLength);
void writeUART1(const char * string);

// initialize SPI1
void initSPI() {
    // Pin B14 has to be SCK1
    // Turn of analog pins
    ANSELA = 0;
    // Make an output pin for CS
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 1;
    // Set SDO1
    RPA1Rbits.RPA1R = 0b0011;
    // Set SDI1
    SDI1Rbits.SDI1R = 0b0001;

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1000; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
}


// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

unsigned short set_wave_value(unsigned char c, unsigned short v){
    unsigned short p = c<<15;
    p = p|(0b111<<12);
    p = p|(v<<4);
    return p;
}


int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
    TRISBbits.TRISB4 = 1;
    
    U1RXRbits.U1RXR = 0b0001; //U1RX on RPB6
    RPB7Rbits.RPB7R = 0b0001; //U1TX on RPB7
    
    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((48000000 / 230400) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    
    // enable the uart
    U1MODEbits.ON = 1;

    __builtin_enable_interrupts();
    
    initSPI();
    
    int counter = 0;
   
    unsigned short vsine[256];
    unsigned short vtri[256];
    unsigned short set;
    unsigned short test = 132;
    int triindex = 0;
    int i;
    
    for(i=0;i<256;i = i+1){
        vsine[i] = (unsigned short) (127 * sin( i /40.6) + 128);
        if(i <= 127){
            vtri[i]= (unsigned short) (255.0/127.0 * i);
        }
        else{
            vtri[i]= (unsigned short) (-255.0/127.0 * ((double) (i-127)) + 255);
        }
    }
    
    while (1) {
        _CP0_SET_COUNT(0);
        //set = set_wave_value(0, test);
        //LATAbits.LATA0 = 0;
        //spi_io(set>>8);
        //spi_io(set);
        //LATAbits.LATA0 = 1;
        while(_CP0_GET_COUNT() < (12000000/255)){}
            set = set_wave_value(0, vsine[counter]);
            LATAbits.LATA0 = 0;
            spi_io(set>>8);
            spi_io(set);
            LATAbits.LATA0 = 1;
            if(counter % 2 == 0){
                set = set_wave_value(1, vtri[triindex]);
                LATAbits.LATA0 = 0;
                spi_io(set>>8);
                spi_io(set);
                LATAbits.LATA0 = 1;
                triindex++;
                if (triindex > 254){
                    triindex =0;
                }
            }
            counter++;
            if(counter > 255){
                counter = 0;
            }
        }
    }
//}