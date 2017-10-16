//------------------------------------------------------------------------------
// Name: cs8900.c
// Func: ethernet driver for use with LAN controller CS8900A
// Ver.: 1.1
// Date: January 2004
// Auth: Andreas Dannenberg
//       MSP430 Applications
//       Texas Instruments Inc.
// Rem.: -
//------------------------------------------------------------------------------

#include "msp430x14x.h"
//#include "support.h"
#include "cs8900.h"

//------------------------------------------------------------------------------
const unsigned int MyMAC[] =                     // "M1-M2-M3-M4-M5-M6"
{

  MYMAC_1 + (unsigned int)(MYMAC_2 << 8),
  MYMAC_3 + (unsigned int)(MYMAC_4 << 8),
  MYMAC_5 + (unsigned int)(MYMAC_6 << 8)
};

static const TInitSeq InitSeq[] =
{
  PP_IA, MYMAC_1 + (MYMAC_2 << 8),               // set our MAC as Individual Address
  PP_IA + 2, MYMAC_3 + (MYMAC_4 << 8),
  PP_IA + 4, MYMAC_5 + (MYMAC_6 << 8),
  PP_LineCTL, SERIAL_RX_ON | SERIAL_TX_ON,       // configure the Physical Interface
  PP_RxCTL, RX_OK_ACCEPT | RX_IA_ACCEPT | RX_BROADCAST_ACCEPT
};

//------------------------------------------------------------------------------
// configure port-pins for use with LAN-controller,
// issue reset and send the configuration-sequence (InitSeq[])
//------------------------------------------------------------------------------
void Init8900(void)
{
  unsigned int i;

  P3OUT = IOR | IOW;                             // reset outputs, control lines high
  P3DIR = 0xff;                                  // switch control port to output

  P5OUT = 0;                                     // reset outputs
  P5DIR = 0xff;                                  // switch data port to output

  // CHASE: vv DelayCycles to __delay_cycles
  __delay_cycles(40000);                            // delay 10ms @ 8MHz MCLK to allow
  __delay_cycles(40000);                            // time for CS8900 POR
  // CHASE: ^^ DelayCycles to __delay_cycles

  Write8900(ADD_PORT, PP_SelfCTL);               // set register
  Write8900(DATA_PORT, POWER_ON_RESET);          // reset the Ethernet-Controller

  do
    Write8900(ADD_PORT, PP_SelfST);              // set register
  while (!(Read8900(DATA_PORT) & INIT_DONE));    // wait until chip-reset is done
  
  for (i = 0; i < sizeof InitSeq / sizeof (TInitSeq); i++) // configure the CS8900
  {
    Write8900(ADD_PORT, InitSeq[i].Addr);
    Write8900(DATA_PORT, InitSeq[i].Data);
  }
}
//------------------------------------------------------------------------------
// writes a word in little-endian byte order to
// a specified port-address
//------------------------------------------------------------------------------
void Write8900(unsigned char Address, unsigned int Data)
{
  P5DIR = 0xff;                                  // data port to output
  P3OUT = IOR | IOW | Address;                   // put address on bus
  P5OUT = Data;                                  // write low order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT = IOR | IOW | (Address + 1);             // and put next address on bus
  P5OUT = Data >> 8;                             // write high order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT |= IOW;
}
//------------------------------------------------------------------------------
// writes a word in little-endian byte order to TX_FRAME_PORT
//------------------------------------------------------------------------------
void WriteFrame8900(unsigned int Data)
{
  P5DIR = 0xff;                                  // data port to output
  P3OUT = IOR | IOW | TX_FRAME_PORT;             // put address on bus
  P5OUT = Data;                                  // write low order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT = IOR | IOW | (TX_FRAME_PORT + 1);       // and put next address on bus
  P5OUT = Data >> 8;                             // write high order byte to data bus
  P3OUT &= ~IOW;                                 // toggle IOW-signal
  P3OUT |= IOW;
}
//------------------------------------------------------------------------------
// copies bytes from MCU-memory to frame port
// NOTES:     * MCU-memory MUST start at word-boundary
//------------------------------------------------------------------------------
void CopyToFrame8900(void *Source, unsigned int Size)
{
  unsigned int *pSource = Source;
  
  P5DIR = 0xff;                                  // data port to output

  while (Size > 1)
  {
    P3OUT = IOR | IOW | TX_FRAME_PORT;           // put address on bus
    P5OUT = *pSource;                            // write low order byte to data bus
    P3OUT &= ~IOW;                               // toggle IOW-signal
    P3OUT = IOR | IOW | (TX_FRAME_PORT + 1);     // and put next address on bus
    P5OUT = (*pSource++) >> 8;                   // write high order byte to data bus
    P3OUT &= ~IOW;                               // toggle IOW-signal
    P3OUT |= IOW;
    Size -= 2;
  }
  
  if (Size)                                      // if odd num. of bytes...
  {
    P3OUT = IOR | IOW | TX_FRAME_PORT;           // put address on bus
    P5OUT = *pSource;                            // write byte to data bus
    P3OUT &= ~IOW;                               // toggle IOW-signal
    P3OUT |= IOW;
  }
}
//------------------------------------------------------------------------------
// reads a word in little-endian byte order from
// a specified port-address
//------------------------------------------------------------------------------
unsigned int Read8900(unsigned char Address)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | Address;                   // put address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue = P5IN;                            // get low order byte from data bus
  P3OUT = IOR | IOW | (Address + 1);             // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue |= P5IN << 8;                      // get high order byte from data bus
  P3OUT |= IOR;
  P5DIR = 0xff;                                  // data port to output
  
  return ReturnValue;
}
//------------------------------------------------------------------------------
// reads a word in little-endian byte order from RX_FRAME_PORT
//------------------------------------------------------------------------------
unsigned int ReadFrame8900(void)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | RX_FRAME_PORT;             // access to RX_FRAME_PORT
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue = P5IN;                            // get 1st byte from data bus (low-byte)
  P3OUT = IOR | IOW | (RX_FRAME_PORT + 1);       // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue |= P5IN << 8;                      // get 2nd byte from data bus (high-byte)
  P3OUT |= IOR;
  P5DIR = 0xff;                                  // data port to output
  
  return ReturnValue;
}
//------------------------------------------------------------------------------
// reads a word in big-endian byte order from RX_FRAME_PORT
// (useful to avoid permanent byte-swapping while reading
// TCP/IP-data)
//------------------------------------------------------------------------------
unsigned int ReadFrameBE8900(void)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | RX_FRAME_PORT;             // access to RX_FRAME_PORT
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue = P5IN << 8;                       // get 1st byte from data bus (high-byte)
  P3OUT = IOR | IOW | (RX_FRAME_PORT + 1);       // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue |= P5IN;                           // get 2nd byte from data bus (low-byte)
  P3OUT |= IOR;
  P5DIR = 0xff;                                  // data port to output
  
  return ReturnValue;
}
//------------------------------------------------------------------------------
// reads a word in little-endian byte order from
// a specified port-address
// NOTE: this func. xfers the high-byte 1st, must be used to
//       access some special registers (e.g. RxStatus)
//------------------------------------------------------------------------------
unsigned int ReadHB1ST8900(unsigned char Address)
{
  unsigned int ReturnValue;

  P5DIR = 0x00;                                  // data port to input
  P3OUT = IOR | IOW | (Address + 1);             // put address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue = P5IN << 8;                       // get high order byte from data bus
  P3OUT = IOR | IOW | Address;                   // IOR high and put next address on bus
  P3OUT &= ~IOR;                                 // IOR-signal low
  ReturnValue |= P5IN;                           // get low order byte from data bus
  P3OUT |= IOR;
  P5DIR = 0xff;                                  // data port to output
  
  return ReturnValue;
}
//------------------------------------------------------------------------------
// copies bytes from frame port to MCU-memory
// NOTES:     * MCU-memory MUST start at word-boundary
//------------------------------------------------------------------------------
void CopyFromFrame8900(void *Dest, unsigned int Size)
{
  unsigned int *pDest = Dest;

  P5DIR = 0x00;                                  // data port to input

  while (Size > 1)
  {
    P3OUT = IOR | IOW | RX_FRAME_PORT;           // access to RX_FRAME_PORT
    P3OUT &= ~IOR;                               // IOR-signal low
    *pDest = P5IN;                               // get 1st byte from data bus (low-byte)
    P3OUT = IOR | IOW | (RX_FRAME_PORT + 1);     // IOR high and put next address on bus
    P3OUT &= ~IOR;                               // IOR-signal low
    *pDest++ |= P5IN << 8;                       // get 2nd byte from data bus (high-byte)
    P3OUT |= IOR;
    Size -= 2;
  }
  
  if (Size)                                      // check for leftover byte...
  {
    P3OUT = IOR | IOW | RX_FRAME_PORT;           // access to RX_FRAME_PORT
    P3OUT &= ~IOR;                               // IOR-signal low
    *(unsigned char *)pDest = P5IN;              // get byte from data bus
    P3OUT |= IOR;                                // IOR high
  }

  P5DIR = 0xff;                                  // data port to output
}
//------------------------------------------------------------------------------
// does a dummy read on the CS8900A frame-I/O-port
//------------------------------------------------------------------------------
void DummyReadFrame8900(unsigned int Size)
{
  P5DIR = 0x00;                                  // data port to input

  while (Size--)
  {
    P3OUT = IOR | IOW | RX_FRAME_PORT;           // access to RX_FRAME_PORT
    P3OUT &= ~IOR;                               // IOR-signal low
  }
  
  P3OUT |= IOR;                                  // IOR high
  P5DIR = 0xff;                                  // data port to output
}
//------------------------------------------------------------------------------
// requests space in CS8900 on-chip memory for
// storing an outgoing frame
//------------------------------------------------------------------------------
void RequestSend(unsigned int FrameSize)
{
  Write8900(TX_CMD_PORT, TX_START_ALL_BYTES);
  Write8900(TX_LEN_PORT, FrameSize);
}
//------------------------------------------------------------------------------
// check if CS8900 is ready to accept the
// frame we want to send
//------------------------------------------------------------------------------
unsigned int Rdy4Tx(void)
{
  Write8900(ADD_PORT, PP_BusST);
  return Read8900(DATA_PORT) & READY_FOR_TX_NOW;
}
