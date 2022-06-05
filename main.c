#include <msp430.h> 
#include <sd_card_raw_library.h>
#include <i2c_prot.h>

//___________________________________________________________________GLOBALS____________________________________________________________//
// GPS VARS
char gpsReceiveMessage[100];
unsigned int gpsCnt = 0;
int gpsDone = 0;
char gpsSunnySide[100];

// SD Card VARS
unsigned char sd_buffer[512];                                                                                                           // buffer for SD card

// I2C Sensor VARS
char ADT7410Addr = 0x4B;
char LM92Addr = 0x48;
char MS5607Addr = 0x77;

char sensorData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 0: ADT7410 MSB, 1: ADT7410 LSB, 2: LM92 MSB, 3: LM92 LSB, 4: MS5607 Pressure MSB, 5: -, 6: MS5607 Pressure LSB
// 7: MS5607 Temp MSB, 8: -, 9: MS5607 Temp LSB

//___________________________________________________________________SUB_PROC_PROT______________________________________________________//
void sendByteGPS(char byte);
void sendGPSMessOff(char* message, int length);
void descrambleGPSData(void);
void readSensors(void);
void sendDataToSDCard(unsigned long addr);

//___________________________________________________________________MAIN_______________________________________________________________//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                                                                                                           // stop watchdog timer

    //-------UART--------
    UCA0CTLW0 |= UCSWRST;                                                                                                               //put A0 in software reset

    UCA0CTLW0 |= UCSSEL__ACLK;                                                                                                          //Set clock

    //Clock dividing for 9600 baud rate on UART
    UCA0BRW = 3;
    UCA0MCTLW |= 0x9200;

    //Set the TX and RX channels to UART
    P1SEL1 &= ~BIT6;
    P1SEL0 |= BIT6;
    P1SEL1 &= ~BIT7;
    P1SEL0 |= BIT7;

    //for UART
    UCA0CTLW0 &= ~UCSWRST;                                                                                                              //Take A0 out of software reset

    //for I2C
    i2c_init_proc();

    //for SD CARD
    SPIInit();                                                                                                                          // Initialize SPI Ports
    unsigned long sdAddr = 0;

    PM5CTL0 &= ~LOCKLPM5;                                                                                                               // Turn on GPIO

    P1DIR |= BIT0 | BIT1;
    P1OUT |= BIT0;                                                                                                                      // Power on led
    P1OUT &=~BIT1;                                                                                                                      // Writing to SD CARD

    // Turn off messages
    char rmcOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X04, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X03, 0X3F};
    char vtgOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X05, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X04, 0X46};
    char gsaOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X02, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X01, 0X31};
    char gsvOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X03, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X02, 0X38};
    char gllOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X01, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X2A};
    char ggaOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XFF, 0X23};
    //char ggaOn[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X00, 0X00, 0X01, 0X01, 0X00, 0X00, 0X00, 0X01, 0X2C};       //gga on test
    char zdaOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X08, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X07, 0X5B};
    char polReq[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x19, 0xE5};

    int j;
    for(j=0; j<5000; j++){}

    // GPS Configuration
    sendGPSMessOff(rmcOff, sizeof(rmcOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(vtgOff, sizeof(vtgOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(gsaOff, sizeof(gsaOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(gsvOff, sizeof(gsvOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(gllOff, sizeof(gllOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(ggaOff, sizeof(ggaOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(zdaOff, sizeof(zdaOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(polReq, sizeof(polReq));
    for(j=0; j<2000; j++){}

    // SD Card Initialization
    sdCardInit();
    unsigned char dataIn[6];
    sendCommand(0x50, 0x200, 0xFF, dataIn);                                                                                                 // CMD 16 : Set Block Length

    for(j=0; j<sizeof(sd_buffer); j++)
    {
        sd_buffer[j] = 0;
    }


    // Interrupts Initialization
    UCA0IE &= ~UCRXIE;                                                                                                                      // disable receiving
    __enable_interrupt();

    //Reset Pressure Sensor
    i2c_send_address_proc(MS5607Addr, I2C_WRITE);
    i2c_send_byte_proc(0x00);
    i2c_receive_ack_or_nack_proc();
    i2c_send_byte_proc(0x00);
    i2c_send_nack_proc();
    i2c_stop_proc();

    __delay_cycles(100000);



//___________________________________________________________________MAIN_LOOP__________________________________________________________//
    while(1)
    {
        gpsCnt = 0;
        gpsDone = 0;                                                                                                                        // Reset GPS Flag
        UCA0IE |= UCRXIE;                                                                                                                   // Enable GPS
        while(gpsDone != 1){}                                                                                                               // Wait for GPS to finish
        UCA0IE &= ~UCRXIE;                                                                                                                  // disable UART message receiving
        descrambleGPSData();

        // READ TEMP AND SUCH
        readSensors();

        P1OUT |= BIT1;
        sendDataToSDCard(sdAddr++);                                                                                                         // increase address after writing to the sd card
        P1OUT &=~BIT1;


        __delay_cycles(10000);
    }

    //return 0;                                                                                                                             // ignore as it will never be reached
}



//___________________________________________________________________SUB_PROC___________________________________________________________//

// Send a byte to the GPS
void sendByteGPS(char byte)
{
    int j;
    UCA0TXBUF = (byte);
    for(j=0; j<100;j++){}
}

// Send a message to the GPS
void sendGPSMessOff(char* message, int length)
{
    int i;
    for(i=0; i<length; i++){
        char temp = message[i];
        sendByteGPS(temp);
    }
}

// Descramble the GPS data to make it always the same
void descrambleGPSData(void)
{
    int i;
    int firstBestIndex = 0;
    for(i=0; i<sizeof(gpsSunnySide); i++)
    {
        if(gpsReceiveMessage[i] == 0xB5)
        {
            if(gpsReceiveMessage[i+1] == 0x62)
            {
                if(gpsReceiveMessage[i+2] == 0x01)
                {
                    if(gpsReceiveMessage[i+3] == 0x07)
                    {
                        if(gpsReceiveMessage[i+4] == 0x5C)
                        {
                            firstBestIndex = i;
                        }
                    }
                }
            }
        }
    }
    for(i=0; i<sizeof(gpsSunnySide); i++)
    {
        gpsSunnySide[i] = gpsReceiveMessage[(i+firstBestIndex)%sizeof(gpsReceiveMessage)];
    }
    return;
}

// Read I2C Data
void readSensors(void)
{
    //Reset Pressure Sensor
    i2c_send_address_proc(MS5607Addr, I2C_WRITE);
    i2c_send_byte_proc(0x00);
    i2c_receive_ack_or_nack_proc();
    i2c_send_byte_proc(0x00);
    i2c_send_nack_proc();
    i2c_stop_proc();

    // Read ADT7410
    i2c_send_address_proc(ADT7410Addr, I2C_WRITE);
    i2c_send_byte_proc(0x00);
    i2c_send_address_proc(ADT7410Addr, I2C_READ);
    sensorData[0] = i2c_receive_byte_proc();
    i2c_send_ack_proc();
    sensorData[1] = i2c_receive_byte_proc();
    i2c_send_nack_proc();
    i2c_stop_proc();
    __delay_cycles(10000);

    // Read LM92
    i2c_send_address_proc(LM92Addr, I2C_WRITE);
    i2c_send_byte_proc(0x00);
    i2c_send_address_proc(LM92Addr, I2C_READ);
    sensorData[2] = i2c_receive_byte_proc();
    i2c_send_ack_proc();
    sensorData[3] = i2c_receive_byte_proc();
    i2c_send_nack_proc();
    i2c_stop_proc();
    __delay_cycles(10000);

    // Read MS5607
    i2c_send_address_proc(MS5607Addr, I2C_WRITE);
    i2c_send_byte_proc(0x48);                                                                                                               // Send command to begin converting Press
    i2c_stop_proc();
    __delay_cycles(10000);                                                                                                                  // Wait for it to convert

    i2c_send_address_proc(MS5607Addr, I2C_WRITE);
    i2c_send_byte_proc(0x00);
    i2c_send_address_proc(MS5607Addr, I2C_READ);
    sensorData[4] = i2c_receive_byte_proc();
    i2c_send_ack_proc();
    sensorData[5] = i2c_receive_byte_proc();
    i2c_send_ack_proc();
    sensorData[6] = i2c_receive_byte_proc();
    i2c_send_nack_proc();
    i2c_stop_proc();
    __delay_cycles(10000);

    i2c_send_address_proc(MS5607Addr, I2C_WRITE);
    i2c_send_byte_proc(0x58);                                                                                                               // Send command to begin converting Temp
    i2c_stop_proc();
    __delay_cycles(10000);                                                                                                                  // Wait for it to convert

    i2c_send_address_proc(MS5607Addr, I2C_WRITE);
    i2c_send_byte_proc(0x00);
    i2c_send_address_proc(MS5607Addr, I2C_READ);
    sensorData[7] = i2c_receive_byte_proc();
    i2c_send_ack_proc();
    sensorData[8] = i2c_receive_byte_proc();
    i2c_send_ack_proc();
    sensorData[9] = i2c_receive_byte_proc();
    i2c_send_nack_proc();
    i2c_stop_proc();
    __delay_cycles(10000);

}

// Put all the data read into the SD Card Buffer then onto the SD Card
void sendDataToSDCard(unsigned long addr)
{
    int i;
    unsigned char dataIn[6];
    for(i=0; i<sizeof(sensorData); i++)
    {
        sd_buffer[i] = sensorData[i];
    }
    for(i=0; i<sizeof(gpsSunnySide); i++)
    {
        sd_buffer[i+sizeof(sensorData)] = gpsSunnySide[i];
    }
    sendData(addr, sd_buffer);
    __delay_cycles(10000);     // may need to increase (50000 originally)
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    __delay_cycles(10000);    // this one too
    return;
}

//____________________________________________________________________ISR's_____________________________________________________________//
#pragma vector=EUSCI_A0_VECTOR
__interrupt void EUSCI_A0_RX_ISR(void)
{
    gpsReceiveMessage[gpsCnt] = UCA0RXBUF;
    if(gpsCnt == sizeof(gpsReceiveMessage)-1)
    {
        gpsCnt = 0;
        gpsDone = 1;
    }
    else
    {
        gpsCnt++;
    }
}
