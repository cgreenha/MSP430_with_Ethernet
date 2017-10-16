//------------------------------------------------------------------------------
// Name: easyweb.c
// Func: implements a dynamic HTTP-server by using
//       the easyWEB-API
// Ver.: 1.1
// Date: January 2004
// Auth: Andreas Dannenberg
//       MSP430 Applications
//       Texas Instruments Inc.
// Rem.: - For detailed software/hardware information refer to MSP430
//         application note SLAA137 (www.ti.com/msp430)
//       - Build with IAR EW V2.21B
//       - The compiler option "Formatted write" should be set to "Medium"
//         (to ensure proper operation of the sprintf() function)
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <webside.h>                             // webside for our HTTP server (HTML)

#include "msp430x14x.h"
//#include "support.h"
#include "easyweb.h"
#include "tcpip.h"                               // easyWEB TCP/IP stack


static const unsigned char GetResponse[] =       // 1st thing server sends to a client
{
  "HTTP/1.0 200 OK\r\n"                          // protocol ver 1.0, code 200, reason OK
  "Content-Type: text/html\r\n"                  // type of data we want to send
  "\r\n"                                         // indicate end of HTTP-header
};

static unsigned char *PWebSide;                  // pointer to webside
static unsigned int HTTPBytesToSend;             // bytes left to send
static unsigned char HTTPStatus;                 // status byte

//------------------------------------------------------------------------------
// ADC12 Module Temperature Table
//
// These values assume nominal values for the temp sensor and
// temperature coefficient. They also assume the 2.5V internal
// reference is used for the conversion. The transfer function out
// of the MSP430x1xx Family User's Guide has been used.
//------------------------------------------------------------------------------
static const unsigned int Temp_Tab[] =
{
  0x064F,                                        // 0C
  0x0655,
  0x065B,
  0x0660,
  0x0666,
  0x066C,                                        // 5C
  0x0672,
  0x0678,
  0x067D,
  0x0683,
  0x0689,                                        // 10C
  0x068F,
  0x0695,
  0x069B,
  0x06A0,
  0x06A6,                                        // 15C
  0x06AC,
  0x06B2,
  0x06B8,
  0x06BD,
  0x06C3,                                        // 20C
  0x06C9,
  0x06CF,
  0x06D5,
  0x06DB,
  0x06E0,                                        // 25C
  0x06E6,
  0x06EC,
  0x06F2,
  0x06F8,
  0x06FD,                                        // 30C
  0x0703,
  0x0709,
  0x070F,
  0x0715,
  0x071B,                                        // 35C
  0x0720,
  0x0726,
  0x072C,
  0x0732,
  0x0738,                                        // 40C
  0x073D,
  0x0743,
  0x0749,
  0x074F,
  0x0755,                                        // 45C
  0x0FFF                                         // Too high
};
//------------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------------
static void InitOsc(void);
static void InitPorts(void);
static void InitADC12(void);
static void HTTPServer(void);
static void InsertDynamicValues(void);
static unsigned int GetAD7Val(void);
static unsigned int GetTempVal(void);
//------------------------------------------------------------------------------
void main(void)
{
  InitOsc();
  InitPorts();
  InitADC12();

  TCPLowLevelInit();

  //CHASE
  //__enable_interrupt();                          // enable interrupts

/*
  *(unsigned char *)RemoteIP = 24;               // uncomment those lines to get the
  *((unsigned char *)RemoteIP + 1) = 8;          // quote of the day from a real
  *((unsigned char *)RemoteIP + 2) = 69;         // internet server! (gateway must be
  *((unsigned char *)RemoteIP + 3) = 7;          // set to your LAN-router)

  TCPLocalPort = 2025;
  TCPRemotePort = TCP_PORT_QOTD;

  TCPActiveOpen();

  while (SocketStatus & SOCK_ACTIVE)             // read the quote from memory
  {                                              // by using the hardware-debugger
    DoNetworkStuff();
  }
*/

  HTTPStatus = 0;                                // clear HTTP-server's flag register

  TCPLocalPort = TCP_PORT_HTTP;                  // set port we want to listen to
  
  while (1)                                      // repeat forever
  {
    if (!(SocketStatus & SOCK_ACTIVE)) TCPPassiveOpen();   // listen for incoming TCP-connection
    DoNetworkStuff();                                      // handle network and easyWEB-stack
                                                           // events
    HTTPServer();
  }
}
//------------------------------------------------------------------------------
// This function implements a very simple dynamic HTTP-server.
// It waits until connected, then sends a HTTP-header and the
// HTML-code stored in memory. Before sending, it replaces
// some special strings with dynamic values.
// NOTE: For strings crossing page boundaries, replacing will
// not work. In this case, simply add some extra lines
// (e.g. CR and LFs) to the HTML-code.
//------------------------------------------------------------------------------
static void HTTPServer(void)
{
  if (SocketStatus & SOCK_CONNECTED)             // check if somebody has connected to our TCP
  {
    if (SocketStatus & SOCK_DATA_AVAILABLE)      // check if remote TCP sent data
      TCPReleaseRxBuffer();                      // and throw it away

    if (SocketStatus & SOCK_TX_BUF_RELEASED)     // check if buffer is free for TX
    {
      if (!(HTTPStatus & HTTP_SEND_PAGE))        // init byte-counter and pointer to webside
      {                                          // if called the 1st time
        HTTPBytesToSend = sizeof(WebSide) - 1;   // get HTML length, ignore trailing zero
        PWebSide = (unsigned char *)WebSide;     // pointer to HTML-code
      }

      if (HTTPBytesToSend > MAX_TCP_TX_DATA_SIZE)     // transmit a segment of MAX_SIZE
      {
        if (!(HTTPStatus & HTTP_SEND_PAGE))           // 1st time, include HTTP-header
        {
          memcpy(TCP_TX_BUF, GetResponse, sizeof(GetResponse) - 1);
          memcpy(TCP_TX_BUF + sizeof(GetResponse) - 1, PWebSide,
            MAX_TCP_TX_DATA_SIZE - sizeof(GetResponse) + 1);
          HTTPBytesToSend -= MAX_TCP_TX_DATA_SIZE - sizeof(GetResponse) + 1;
          PWebSide += MAX_TCP_TX_DATA_SIZE - sizeof(GetResponse) + 1;
        }
        else
        {
          memcpy(TCP_TX_BUF, PWebSide, MAX_TCP_TX_DATA_SIZE);
          HTTPBytesToSend -= MAX_TCP_TX_DATA_SIZE;
          PWebSide += MAX_TCP_TX_DATA_SIZE;
        }
          
        TCPTxDataCount = MAX_TCP_TX_DATA_SIZE;   // bytes to xfer
        InsertDynamicValues();                   // exchange some strings...
        TCPTransmitTxBuffer();                   // xfer buffer
      }
      else if (HTTPBytesToSend)                  // transmit leftover bytes
      {
        memcpy(TCP_TX_BUF, PWebSide, HTTPBytesToSend);
        TCPTxDataCount = HTTPBytesToSend;        // bytes to xfer
        InsertDynamicValues();                   // exchange some strings...
        TCPTransmitTxBuffer();                   // send last segment
        TCPClose();                              // and close connection
        HTTPBytesToSend = 0;                     // all data sent
      }

      HTTPStatus |= HTTP_SEND_PAGE;              // ok, 1st loop executed
    }
  }
  else
    HTTPStatus &= ~HTTP_SEND_PAGE;               // reset help-flag if not connected
}
//------------------------------------------------------------------------------
// samples and returns the AD-converter value of channel 7
// (associated with Port P6.7)
//------------------------------------------------------------------------------
static unsigned int GetAD7Val(void)
{
  ADC12MCTL0 = SREF_1 + INCH_7;                  // Select channel 7, Vref+
  
  ADC12CTL0 |= ENC;                              // enable conversion
  ADC12CTL0 |= ADC12SC;                          // sample & convert
  
  while (ADC12CTL0 & ADC12SC);                   // wait until conversion is complete
  
  ADC12CTL0 &= ~ENC;                             // disable conversion

  return ADC12MEM0 >> 5;                         // Scale value
}
//------------------------------------------------------------------------------
// samples and returns AD-converter value of channel 10
// (MSP430 internal temperature reference diode)
//------------------------------------------------------------------------------
static unsigned int GetTempVal(void)
{
  unsigned int i;
  unsigned int ADCResult;

  ADC12MCTL0 = SREF_1 + INCH_10;                 // Select channel A10, Vref+

  ADC12CTL0 |= ENC;                              // enable conversion
  ADC12CTL0 |= ADC12SC;                          // sample & convert
  
  while (ADC12CTL0 & ADC12SC);                   // wait until conversion is complete
  
  ADC12CTL0 &= ~ENC;                             // disable conversion

  ADCResult = ADC12MEM0;                         // Read out ADC12 value

  for (i = 0; i < sizeof Temp_Tab; i++)          // Get temperature value out
    if (ADCResult < Temp_Tab[i])                 // of table
      break;

  return i << 1;                                 // Scale value
}
//------------------------------------------------------------------------------
// searches the TX-buffer for special strings and replaces them
// with dynamic values (AD-converter results)
//------------------------------------------------------------------------------
static void InsertDynamicValues(void)
{
  char *Key;
  char NewKey[5];
  int i;
  
  if (TCPTxDataCount < 4) return;                // there can't be any special string
  
  Key = (char *)TCP_TX_BUF;
  
  for (i = 0; i < TCPTxDataCount - 3; i++)
  {
    if (*Key == 'A')
     if (*(Key + 1) == 'D')
       if (*(Key + 3) == '%')
         if (*(Key + 2) == '7')                  // "AD7%"?
         {
           sprintf(NewKey, "%3u", GetAD7Val());  // insert AD converter value
           memcpy(Key, NewKey, 3);               // channel 7 (P6.7)
         }
         else if (*(Key + 2) == 'A')
         {
           sprintf(NewKey, "%3u", GetTempVal()); // insert AD converter value
           memcpy(Key, NewKey, 3);               // channel 10 (temp.-diode)
         }
    Key++;
  }
}
//------------------------------------------------------------------------------
// enables the 8MHz crystal on XT1 and use
// it as MCLK
//------------------------------------------------------------------------------
static void InitOsc(void)
{
  WDTCTL = WDTPW + WDTHOLD;                      // stop watchdog timer

  BCSCTL1 |= XTS;                                // XT1 as high-frequency
  __bic_SR_register(OSCOFF);                     // turn on XT1 oscillator

  do
  {
    IFG1 &= ~OFIFG;                              // Clear OFIFG
    // CHASE: DelayCycles to __delay_cycles
    __delay_cycles(100);                            // Wait ~130us
  } while (IFG1 & OFIFG);                        // Test oscillator fault flag
  
  BCSCTL2 = SELM_3;                              // set XT1 as MCLK
}  
//------------------------------------------------------------------------------
static void InitPorts(void)
{
  P1OUT = 0;                                     // switch all unused ports to output   
  P1DIR = 0xff;                                  // (Rem.: ports 3 & 5 are set in "cs8900.c")

  P2OUT = 0;
  P2DIR = 0xff;

  P4OUT = 0;
  P4DIR = 0xff;

  P6SEL = 0x80;                                  // use P6.7 for the ADC module
  P6OUT = 0;
  P6DIR = 0x7f;                                  // all output except P6.7
}
//------------------------------------------------------------------------------
static void InitADC12(void)
{
  // CHASE
  //ADC12CTL0 = ADC12ON + REFON + REF2_5V + SHT0_6;// Turn on ADC12, 2.5Vref, set SHT0
  //ADC12CTL1 = SHS_0 + SHP + CONSEQ_0;            // Use sampling timer, ADC12SC
                                                 // triggers conversion,
                                                 // single channel single conversion,
}


