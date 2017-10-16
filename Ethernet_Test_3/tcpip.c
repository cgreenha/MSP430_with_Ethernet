//------------------------------------------------------------------------------
// Name: tcpip.c
// Func: implements the TCP/IP-stack and provides an API
// Ver.: 1.1
// Date: January 2004
// Auth: Andreas Dannenberg
//       MSP430 Applications
//       Texas Instruments Inc.
// Rem.: -
//------------------------------------------------------------------------------

//#include <support.h>
#include "msp430x14x.h"
#include "cs8900.h"
#include "tcpip.h"

// constants
const unsigned int MyIP[] =                      // "MYIP1.MYIP2.MYIP3.MYIP4"
{
  MYIP_1 + (unsigned int)(MYIP_2 << 8),
  MYIP_3 + (unsigned int)(MYIP_4 << 8)
};

const unsigned int SubnetMask[] =                // "SUBMASK1.SUBMASK2.SUBMASK3.SUBMASK4"
{
  SUBMASK_1 + (unsigned int)(SUBMASK_2 << 8),
  SUBMASK_3 + (unsigned int)(SUBMASK_4 << 8)
};

const unsigned int GatewayIP[] =                 // "GWIP1.GWIP2.GWIP3.GWIP4"
{
  GWIP_1 + (unsigned int)(GWIP_2 << 8),
  GWIP_3 + (unsigned int)(GWIP_4 << 8)
};

// variables
static TTCPStateMachine TCPStateMachine;         // perhaps the most important var at all ;-)
static TLastFrameSent LastFrameSent;             // retransmission type

static unsigned int ISNGenHigh;                  // upper word of our Initial Sequence Number
static unsigned long TCPSeqNr;                   // next sequence number to send
static unsigned long TCPUNASeqNr;                // last unaknowledged sequence number
                                                 // incremented AFTER sending data
static unsigned long TCPAckNr;                   // next seq to receive and ack to send
                                                 // incremented AFTER receiving data
static unsigned char TCPTimer;                   // inc'd each 262ms
static unsigned char RetryCounter;               // nr. of retransmissions

static unsigned int TxFrame1Size;                // bytes to send in TxFrame1
static unsigned char TxFrame2Size;               // bytes to send in TxFrame2
static unsigned char TransmitControl;
static unsigned char TCPFlags;
unsigned int TCPRxDataCount;                     // nr. of bytes rec'd
unsigned int TCPTxDataCount;                     // nr. of bytes to send
unsigned int TCPLocalPort;                       // TCP ports
unsigned int TCPRemotePort;
unsigned int RemoteMAC[3];                       // MAC and IP of current TCP-session
unsigned int RemoteIP[2];
unsigned char SocketStatus;

// properties of the just received frame
static unsigned int RecdFrameLength;             // CS8900 reported frame length
static unsigned int RecdFrameMAC[3];             // 48 bit MAC
static unsigned int RecdFrameIP[2];              // 32 bit IP
static unsigned int RecdIPFrameLength;           // 16 bit IP packet length

// the next 3 buffers must be word-aligned!
unsigned int TxFrame1Mem[(ETH_HEADER_SIZE + IP_HEADER_SIZE + TCP_HEADER_SIZE +
                          MAX_TCP_TX_DATA_SIZE + 1) >> 1];
static unsigned int TxFrame2Mem[(ETH_HEADER_SIZE + MAX_ETH_TX_DATA_SIZE + 1) >> 1];
unsigned int RxTCPBufferMem[(MAX_TCP_RX_DATA_SIZE + 1) >> 1];  // space for incoming TCP-data
//------------------------------------------------------------------------------
// CHASE: added asembly function writeDwbe
void WriteDWBE(unsigned char *Add, unsigned long Data);

// Handlers for incoming frames
static void ProcessEthBroadcastFrame(void);
static void ProcessEthIAFrame(void);
static void ProcessICMPFrame(void);
static void ProcessTCPFrame(void);

// fill TX-buffers
static void PrepareARP_REQUEST(void);
static void PrepareARP_ANSWER(void);
static void PrepareICMP_ECHO_REPLY(void);

static void PrepareTCP_FRAME(unsigned long seqnr, unsigned long acknr,
  unsigned int TCPCode);
static void PrepareTCP_DATA_FRAME(void);

// general help functions
static void TCPStartRetryTimer(void);
static void TCPStartFinTimer(void);
static void TCPRestartTimer(void);
static void TCPStopTimer(void);
static void TCPHandleRetransmission(void);
static void TCPHandleTimeout(void);
static unsigned int CalcChecksum(void *Start, unsigned int Count,
  unsigned char IsTCP);
//------------------------------------------------------------------------------
// easyWEB-API function
// initalizes the LAN-controller, reset flags, starts timer-ISR
//------------------------------------------------------------------------------
void TCPLowLevelInit(void)
{
  BCSCTL1 &= ~DIVA0;                             // ACLK = XT1 / 4 = 2 MHz
  BCSCTL1 |= DIVA1;
  TACTL = ID_3 + TASSEL_1 + MC_2 + TAIE;         // stop timer, use ACLK / 8 = 250 kHz, gen. int.
                                                 // start timer in continuous up-mode
  Init8900();
  TransmitControl = 0;
  TCPFlags = 0;
  TCPStateMachine = CLOSED;
  SocketStatus = 0;
}
//------------------------------------------------------------------------------
// easyWEB-API function
// does a passive open (listen on 'MyIP:TCPLocalPort' for an incoming
// connection)
//------------------------------------------------------------------------------
void TCPPassiveOpen(void)
{
  if (TCPStateMachine == CLOSED)
  {
    TCPFlags &= ~TCP_ACTIVE_OPEN;                // let's do a passive open!
    TCPStateMachine = LISTENING;
    SocketStatus = SOCK_ACTIVE;                  // reset, socket now active
  }
}
//------------------------------------------------------------------------------
// easyWEB-API function
// does an active open (tries to establish a connection between
// 'MyIP:TCPLocalPort' and 'RemoteIP:TCPRemotePort')
//------------------------------------------------------------------------------
void TCPActiveOpen(void)
{
  if ((TCPStateMachine == CLOSED) || (TCPStateMachine == LISTENING))
  {
    TCPFlags |= TCP_ACTIVE_OPEN;                 // let's do an active open!
    TCPFlags &= ~IP_ADDR_RESOLVED;               // we haven't opponents MAC yet
  
    PrepareARP_REQUEST();                        // ask for MAC by sending a broadcast
    LastFrameSent = ARP_REQUEST;
    TCPStartRetryTimer();
    SocketStatus = SOCK_ACTIVE;                  // reset, socket now active    
  }
}
//------------------------------------------------------------------------------
// easyWEB-API function
// closes an open connection
//------------------------------------------------------------------------------
void TCPClose(void)
{
  switch (TCPStateMachine)
  {
    case LISTENING :
    case SYN_SENT :
      TCPStateMachine = CLOSED;
      TCPFlags = 0;
      SocketStatus = 0;
      break;
    case SYN_RECD :
    case ESTABLISHED :
      TCPFlags |= TCP_CLOSE_REQUESTED;
      break;
  }
}
//------------------------------------------------------------------------------
// easyWEB-API function
// releases the receive-buffer and allows easyWEB to store new data
// NOTE: rx-buffer MUST be released periodically, else the other TCP
//       get no ACKs for the data it sent
//------------------------------------------------------------------------------
void TCPReleaseRxBuffer(void)
{
  SocketStatus &= ~SOCK_DATA_AVAILABLE;
}
//------------------------------------------------------------------------------
// easyWEB-API function
// transmitts data stored in 'TCP_TX_BUF'
// NOTE: * number of bytes to transmit must have been written to 'TCPTxDataCount'
//       * data-count MUST NOT exceed 'MAX_TCP_TX_DATA_SIZE'
//------------------------------------------------------------------------------
void TCPTransmitTxBuffer(void)
{
  if ((TCPStateMachine == ESTABLISHED) || (TCPStateMachine == CLOSE_WAIT))
    if (SocketStatus & SOCK_TX_BUF_RELEASED)
    {
      SocketStatus &= ~SOCK_TX_BUF_RELEASED;               // occupy tx-buffer
      TCPUNASeqNr += TCPTxDataCount;                       // advance UNA
      
      TxFrame1Size = ETH_HEADER_SIZE + IP_HEADER_SIZE + TCP_HEADER_SIZE + TCPTxDataCount;
      TransmitControl |= SEND_FRAME1;
      
      LastFrameSent = TCP_DATA_FRAME;
      TCPStartRetryTimer();
    }
}
//------------------------------------------------------------------------------
// easyWEB's 'main()'-function
// must be called from user program periodically (the often - the better)
// handles network, TCP/IP-stack and user events
//------------------------------------------------------------------------------
void DoNetworkStuff(void)
{
  unsigned int ActRxEvent;                       // copy of cs8900's RxEvent-Register

  Write8900(ADD_PORT, PP_RxEvent);               // point to RxEvent
  ActRxEvent = Read8900(DATA_PORT);              // read, implied skip of last frame

  if (ActRxEvent & RX_OK)
  {
    if (ActRxEvent & RX_IA) ProcessEthIAFrame();
    if (ActRxEvent & RX_BROADCAST) ProcessEthBroadcastFrame();
  }

  if (TCPFlags & TCP_TIMER_RUNNING)
  {
    if (TCPFlags & TIMER_TYPE_RETRY)
    {
      if (TCPTimer > RETRY_TIMEOUT)
      {
        TCPRestartTimer();                       // set a new timeout

        if (RetryCounter)
        {
          TCPHandleRetransmission();             // resend last frame
          RetryCounter--;
        }
        else
        {
          TCPStopTimer();
          TCPHandleTimeout();
        }
      }
    }
    else if (TCPTimer > FIN_TIMEOUT)
    {
      TCPStateMachine = CLOSED;
      TCPFlags = 0;                              // reset all flags, stop retransmission...
      SocketStatus &= SOCK_DATA_AVAILABLE;       // clear all flags but data available
    }
  }
  switch (TCPStateMachine)
  {
    case CLOSED :
    case LISTENING :
      if (TCPFlags & TCP_ACTIVE_OPEN)            // stack has to open a connection?
        if (TCPFlags & IP_ADDR_RESOLVED)         // IP resolved?
          if (!(TransmitControl & SEND_FRAME2))  // buffer free?
          {
            TCPSeqNr = ((unsigned long)ISNGenHigh << 16) | TAR; // set local ISN
            TCPUNASeqNr = TCPSeqNr;
            TCPAckNr = 0;                                       // we don't know what to ACK!
            TCPUNASeqNr++;                                      // count SYN as a byte
            PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_SYN); // send SYN frame
            LastFrameSent = TCP_SYN_FRAME;
            TCPStartRetryTimer();                               // we NEED a retry-timeout
            TCPStateMachine = SYN_SENT;
          }
      break;
    case SYN_RECD :
    case ESTABLISHED :
      if (TCPFlags & TCP_CLOSE_REQUESTED)                  // user has user initated a close?
        if (!(TransmitControl & (SEND_FRAME2 | SEND_FRAME1)))   // buffers free?
          if (TCPSeqNr == TCPUNASeqNr)                          // all data ACKed?
          {
            TCPUNASeqNr++;
            PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_FIN | TCP_CODE_ACK);
            LastFrameSent = TCP_FIN_FRAME;
            TCPStartRetryTimer();
            TCPStateMachine = FIN_WAIT_1;
          }
      break;
    case CLOSE_WAIT :
      if (!(TransmitControl & (SEND_FRAME2 | SEND_FRAME1)))     // buffers free?
        if (TCPSeqNr == TCPUNASeqNr)                            // all data ACKed?
        {
          TCPUNASeqNr++;                                        // count FIN as a byte
          PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_FIN | TCP_CODE_ACK);  // we NEED a retry-timeout
          LastFrameSent = TCP_FIN_FRAME;                        // time to say goodbye...
          TCPStartRetryTimer();
          TCPStateMachine = LAST_ACK;
        }
      break;
  }

  if (TransmitControl & SEND_FRAME2)
  {
    RequestSend(TxFrame2Size);

    if (Rdy4Tx())                                // NOTE: when using a very fast MCU,
    {                                            // maybe the CS8900 isn't ready yet
      CopyToFrame8900((unsigned char *)TxFrame2Mem, TxFrame2Size);
    }
    else
    {
      TCPStateMachine = CLOSED;
      SocketStatus = SOCK_ERR_ETHERNET;          // indicate an error to user
      TCPFlags = 0;                              // clear all flags, stop timers etc.
    }

    TransmitControl &= ~SEND_FRAME2;             // clear tx-flag
  }

  if (TransmitControl & SEND_FRAME1)
  {
    PrepareTCP_DATA_FRAME();                     // build frame w/ actual SEQ, ACK....
    RequestSend(TxFrame1Size);
  
    if (Rdy4Tx())                                // CS8900 ready to accept our frame?
    {                                            // (see note above)
      CopyToFrame8900((unsigned char *)TxFrame1Mem, TxFrame1Size);
    }
    else
    {
      TCPStateMachine = CLOSED;
      SocketStatus = SOCK_ERR_ETHERNET;          // indicate an error to user
      TCPFlags = 0;                              // clear all flags, stop timers etc.
    }

    TransmitControl &= ~SEND_FRAME1;             // clear tx-flag
  }
}
//------------------------------------------------------------------------------
// easyWEB internal function
// handles an incoming broadcast frame
//------------------------------------------------------------------------------
static void ProcessEthBroadcastFrame(void)
{
  unsigned int TargetIP[2];

  // next two words MUST be read with High-Byte 1st (CS8900 AN181 Page 2)
  ReadHB1ST8900(RX_FRAME_PORT);                  // ignore RxStatus Word
  RecdFrameLength = ReadHB1ST8900(RX_FRAME_PORT);// get real length of frame 
    
  DummyReadFrame8900(6);                         // ignore DA (FF-FF-FF-FF-FF-FF)
  CopyFromFrame8900(&RecdFrameMAC, 6);           // store SA (for our answer)

  if (ReadFrameBE8900() == FRAME_ARP)            // get frame type, check for ARP
    if (ReadFrameBE8900() == HARDW_ETH10)        // Ethernet frame
      if (ReadFrameBE8900() == FRAME_IP)         // check protocol
        if (ReadFrameBE8900() == IP_HLEN_PLEN)   // check HLEN, PLEN
          if (ReadFrameBE8900() == OP_ARP_REQUEST)
          {
            DummyReadFrame8900(6);               // ignore sender's hardware address
            CopyFromFrame8900(&RecdFrameIP, 4);  // read sender's protocol address
            DummyReadFrame8900(6);               // ignore target's hardware address
            CopyFromFrame8900(&TargetIP, 4);     // read target's protocol address
            if ((MyIP[0] == TargetIP[0]) && (MyIP[1] == TargetIP[1]))  // is it for us?
              PrepareARP_ANSWER();               // yes->create ARP_ANSWER frame
          }
}
//------------------------------------------------------------------------------
// easyWEB internal function
// handles an incoming frame that passed CS8900's address filter
// (individual addressed = IA)
//------------------------------------------------------------------------------
static void ProcessEthIAFrame(void)
{
  unsigned int TargetIP[2];
  unsigned char ProtocolType;

  // next two words MUST be read with High-Byte 1st (CS8900 AN181 Page 2)
  ReadHB1ST8900(RX_FRAME_PORT);                  // ignore RxStatus Word
  RecdFrameLength = ReadHB1ST8900(RX_FRAME_PORT);// get real length of frame 
  
  DummyReadFrame8900(6);                         // ignore DA
  CopyFromFrame8900(&RecdFrameMAC, 6);           // store SA (for our answer)

  switch (ReadFrameBE8900())                     // get frame type
  {
    case FRAME_ARP :                             // check for ARP
      if ((TCPFlags & (TCP_ACTIVE_OPEN | IP_ADDR_RESOLVED)) == TCP_ACTIVE_OPEN)
        if (ReadFrameBE8900() == HARDW_ETH10)         // check for the right prot. etc.
          if (ReadFrameBE8900() == FRAME_IP)
            if (ReadFrameBE8900() == IP_HLEN_PLEN)
              if (ReadFrameBE8900() == OP_ARP_ANSWER)
              {
                TCPStopTimer();                       // OK, now we've the MAC we wanted ;-)
                CopyFromFrame8900(&RemoteMAC, 6);     // extract opponents MAC
                TCPFlags |= IP_ADDR_RESOLVED;
              }
      break;
    case FRAME_IP :                                        // check for IP-type
      if ((ReadFrameBE8900() & 0xff00 ) == IP_VER_IHL)     // IPv4, IHL=5 (20 Bytes Header)
      {                                                    // ignore Type Of Service
        RecdIPFrameLength = ReadFrameBE8900();             // get IP frame's length
        ReadFrameBE8900();                                 // ignore identification

        if (!(ReadFrameBE8900() & (IP_FLAG_MOREFRAG | IP_FRAGOFS_MASK)))  // only unfragm. frames
        {
          ProtocolType = ReadFrameBE8900();                // get protocol, ignore TTL
          ReadFrameBE8900();                               // ignore checksum
          RecdFrameIP[0] = ReadFrame8900();                // get source IP
          RecdFrameIP[1] = ReadFrame8900();
          TargetIP[0] = ReadFrame8900();                   // get destination IP
          TargetIP[1] = ReadFrame8900();

          if ((MyIP[0] == TargetIP[0]) && (MyIP[1] == TargetIP[1]))  // is it for us?
            switch (ProtocolType)
            {
              case PROT_ICMP :
                ProcessICMPFrame();
                break;
              case PROT_TCP :
                ProcessTCPFrame();
                break;
              case PROT_UDP :                              // not implemented!
                break;
            }
        }      
      break;
    }
  }
}
//------------------------------------------------------------------------------
// easyWEB internal function
// we've just rec'd an ICMP-frame (Internet Control Message Protocol)
// check what to do and branch to the appropriate sub-function
//------------------------------------------------------------------------------
static void ProcessICMPFrame(void)
{
  unsigned int ICMPTypeAndCode;

  ICMPTypeAndCode = ReadFrameBE8900();           // get Message Type and Code
  ReadFrameBE8900();                             // ignore ICMP checksum

  switch (ICMPTypeAndCode >> 8)                  // check type
  {
    case ICMP_ECHO :                             // is echo request?
      PrepareICMP_ECHO_REPLY();                  // echo as much as we can...
      break;
  }
}
//------------------------------------------------------------------------------
// easyWEB internal function
// we've just rec'd an TCP-frame (Transmission Control Protocol)
// this function mainly implements the TCP state machine according to RFC793
//------------------------------------------------------------------------------
static void ProcessTCPFrame(void)
{
  unsigned int TCPSegSourcePort;                 // segment's source port
  unsigned int TCPSegDestPort;                   // segment's destination port
  unsigned long TCPSegSeq;                       // segment's sequence number
  unsigned long TCPSegAck;                       // segment's acknowledge number
  unsigned int TCPCode;                          // TCP code and header length
  unsigned char TCPHeaderSize;                   // real TCP header length
  unsigned int NrOfDataBytes;                    // real number of data
    
  TCPSegSourcePort = ReadFrameBE8900();                    // get ports
  TCPSegDestPort = ReadFrameBE8900();

  if (TCPSegDestPort != TCPLocalPort) return;              // drop segment if port doesn't match

  TCPSegSeq = (unsigned long)ReadFrameBE8900() << 16;      // get segment sequence nr.
  TCPSegSeq |= ReadFrameBE8900();

  TCPSegAck = (unsigned long)ReadFrameBE8900() << 16;      // get segment acknowledge nr.
  TCPSegAck |= ReadFrameBE8900();

  TCPCode = ReadFrameBE8900();                             // get control bits, header length...

  TCPHeaderSize = (TCPCode & DATA_OFS_MASK) >> 10;         // header length in bytes
  NrOfDataBytes = RecdIPFrameLength - IP_HEADER_SIZE - TCPHeaderSize;     // seg. text length

  if (NrOfDataBytes > MAX_TCP_RX_DATA_SIZE) return;        // drop, packet too large for us :'(

  if (TCPHeaderSize > TCP_HEADER_SIZE)                     // ignore options if any
    DummyReadFrame8900(TCPHeaderSize - TCP_HEADER_SIZE);

  switch (TCPStateMachine)                                 // implement the TCP state machine
  {                                                        // RFC793
    case CLOSED :
      if (!(TCPCode & TCP_CODE_RST))
      {
        TCPRemotePort = TCPSegSourcePort;                  // get remote TCP port
        
        RemoteMAC[0] = RecdFrameMAC[0];                    // save opponents MAC and IP
        RemoteMAC[1] = RecdFrameMAC[1];                    // for later use
        RemoteMAC[2] = RecdFrameMAC[2];
        RemoteIP[0] = RecdFrameIP[0];
        RemoteIP[1] = RecdFrameIP[1];
        
        if (TCPCode & TCP_CODE_ACK)                        // make the reset sequence
        {                                                  // acceptable to the other
          PrepareTCP_FRAME(TCPSegAck, 0, TCP_CODE_RST);    // TCP
        }
        else
        {
          TCPAckNr = TCPSegSeq + NrOfDataBytes;
          if (TCPCode & (TCP_CODE_SYN | TCP_CODE_FIN)) TCPAckNr++;
          PrepareTCP_FRAME(0, TCPAckNr, TCP_CODE_RST | TCP_CODE_ACK);
        }
      }
      break;
    case LISTENING :
      if (!(TCPCode & TCP_CODE_RST))                       // ignore segment containing RST
      {
        TCPRemotePort = TCPSegSourcePort;                  // get remote TCP port

        RemoteMAC[0] = RecdFrameMAC[0];                    // save opponents MAC and IP
        RemoteMAC[1] = RecdFrameMAC[1];                    // for later use
        RemoteMAC[2] = RecdFrameMAC[2];
        RemoteIP[0] = RecdFrameIP[0];
        RemoteIP[1] = RecdFrameIP[1];

        if (TCPCode & TCP_CODE_ACK)                        // reset a bad
        {                                                  // acknowledgement
          PrepareTCP_FRAME(TCPSegAck, 0, TCP_CODE_RST);
        }
        else if (TCPCode & TCP_CODE_SYN)
        {
          // initialize global connection variables
          TCPAckNr = TCPSegSeq + 1;                           // get remote ISN, next byte we expect
          TCPSeqNr = ((unsigned long)ISNGenHigh << 16) | TAR; // set local ISN
          TCPUNASeqNr = TCPSeqNr + 1;                         // one byte out -> increase by one
          PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_SYN | TCP_CODE_ACK); // acknowledge connection request
          LastFrameSent = TCP_SYN_ACK_FRAME;
          TCPStartRetryTimer();
          TCPStateMachine = SYN_RECD;
        }
      }
      break;
    case SYN_SENT :
      // drop segment if its IP doesn't belong to current session
      if ((RemoteIP[0] != RecdFrameIP[0]) || (RemoteIP[1] != RecdFrameIP[1]))
        break;      

      // drop segment if port doesn't match
      if (TCPSegSourcePort != TCPRemotePort)
        break;   
      
      if (TCPCode & TCP_CODE_ACK)                // ACK field significant?
        if (TCPSegAck != TCPUNASeqNr)            // is our ISN ACKed?
        {
          if (!(TCPCode & TCP_CODE_RST))
          {
            PrepareTCP_FRAME(TCPSegAck, 0, TCP_CODE_RST);
          }
          break;                                 // drop segment
        }

      if (TCPCode & TCP_CODE_RST)                // RST??
      {
        if (TCPCode & TCP_CODE_ACK)              // if ACK was acceptable, reset
        {                                        // connection
          TCPStateMachine = CLOSED;
          TCPFlags = 0;                          // reset all flags, stop retransmission...
          SocketStatus = SOCK_ERR_CONN_RESET;
        }
        break;                                   // drop segment
      }
        
      if (TCPCode & TCP_CODE_SYN)                // SYN??
      {
        TCPAckNr = TCPSegSeq;                    // get opponents ISN
        TCPAckNr++;                              // inc. by one...

        if (TCPCode & TCP_CODE_ACK)
        {
          TCPStopTimer();                        // stop retransmission, other TCP got our SYN
          TCPSeqNr = TCPUNASeqNr;                // advance our sequence number

          PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_ACK);        // ACK this ISN
          TCPStateMachine = ESTABLISHED;
          SocketStatus |= SOCK_CONNECTED;
          SocketStatus |= SOCK_TX_BUF_RELEASED;  // user may send data now :-)
        }
        else
        {
          TCPStopTimer();
          PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_SYN | TCP_CODE_ACK);   // our SYN isn't ACKed yet,
          LastFrameSent = TCP_SYN_ACK_FRAME;               // now continue with sending
          TCPStartRetryTimer();                            // SYN_ACK frames
          TCPStateMachine = SYN_RECD;
        }
      }
      break;
    default :
      // drop segment if IP doesn't belong to current session
      if ((RemoteIP[0] != RecdFrameIP[0]) || (RemoteIP[1] != RecdFrameIP[1]))
        break;      

      // drop segment if port doesn't match        
      if (TCPSegSourcePort != TCPRemotePort)
        break;     

      // drop segment if it doesn't fall into the receive window
      if ((TCPSegSeq < TCPAckNr) || (TCPSegSeq >= TCPAckNr + MAX_TCP_RX_DATA_SIZE))
        break;
            
      if (TCPCode & TCP_CODE_RST)                // RST??
      {
        TCPStateMachine = CLOSED;                // close the state machine
        TCPFlags = 0;                            // reset all flags, stop retransmission...
        SocketStatus = SOCK_ERR_CONN_RESET;      // indicate an error to user
        break;
      }

      if (TCPCode & TCP_CODE_SYN)                // SYN??
      {
        PrepareTCP_FRAME(TCPSegAck, 0, TCP_CODE_RST);  // is NOT allowed here! send a reset,
        TCPStateMachine = CLOSED;                // close connection...
        TCPFlags = 0;                            // reset all flags, stop retransmission...
        SocketStatus = SOCK_ERR_REMOTE;          // fatal error!
        break;                                   // ...and drop the frame
      }

      if (TCPSegSeq != TCPAckNr)                 // drop if it's not the segment we expect,
      {                                          // and send an ACK
        PrepareTCP_FRAME(TCPUNASeqNr, TCPAckNr, TCP_CODE_ACK);
        break;
      }

      if (!(TCPCode & TCP_CODE_ACK)) break;      // drop segment if the ACK bit is off

      if (TCPSegAck == TCPUNASeqNr)              // is our last data sent ACKed?
      {
        TCPStopTimer();                          // stop retransmission
        TCPSeqNr = TCPUNASeqNr;                  // advance our sequence number

        switch (TCPStateMachine)                 // change state if necessary
        {
          case SYN_RECD :                        // ACK of our SYN?
            TCPStateMachine = ESTABLISHED;       // user may send data now :-)
            SocketStatus |= SOCK_CONNECTED;
            break;
          case ESTABLISHED :
            SocketStatus |= SOCK_TX_BUF_RELEASED;  // give TX buffer back
            break;
          case FIN_WAIT_1 :                      // ACK of our FIN?
            TCPStateMachine = FIN_WAIT_2;

            /*
              The following code lines are not according to RFC793, but prevent from
              being stuck in the FIN_WAIT_2 state when no FIN packet within a certain
              timeframe will be received (RFC793 says there are two ways to leave this
              state: 1. FIN is received or 2. user closes/resets the connection)
            */

            TCPStartFinTimer();                  // start TIME_WAIT timeout
            break;
          case CLOSING :                         // ACK of our FIN?
            TCPStateMachine = TIME_WAIT;
            TCPStartFinTimer();                  // start TIME_WAIT timeout
            break;
          case LAST_ACK :                        // ACK of our FIN?
            TCPStateMachine = CLOSED;
            TCPFlags = 0;                        // reset all flags, stop retransmission...
            SocketStatus &= SOCK_DATA_AVAILABLE; // clear all flags but data available
            break;
          case TIME_WAIT :
            // ACK a retransmission of remote FIN
            PrepareTCP_FRAME(TCPSegAck, TCPAckNr, TCP_CODE_ACK);  
            TCPRestartTimer();                   // restart TIME_WAIT timeout
            break;
        }
      }

      if ((TCPStateMachine == ESTABLISHED) ||
          (TCPStateMachine == FIN_WAIT_1) ||
          (TCPStateMachine == FIN_WAIT_2))
      {
        if (NrOfDataBytes)                                 // data available?
        {
          if (!(SocketStatus & SOCK_DATA_AVAILABLE))       // rx data-buffer empty?
          {
            DummyReadFrame8900(6);                         // ignore window, checksum, urgent pointer
            CopyFromFrame8900(RxTCPBufferMem, NrOfDataBytes);  // fetch data and
            TCPRxDataCount = NrOfDataBytes;                // ...tell the user...
            SocketStatus |= SOCK_DATA_AVAILABLE;           // indicate the new data to user
            TCPAckNr += NrOfDataBytes;
            PrepareTCP_FRAME(TCPSegAck, TCPAckNr, TCP_CODE_ACK);  // ACK rec'd data
          }
          else
            break;                               // stop processing here, we cannot send an
                                                 // acknowledge packet as the received data
                                                 // could not be passed to the app layer
        }
      }
      if (TCPCode & TCP_CODE_FIN)                // FIN??
      {
        switch (TCPStateMachine)
        {
          case SYN_RECD :
          case ESTABLISHED :
            TCPStateMachine = CLOSE_WAIT;
            break;
          case FIN_WAIT_1 :                      
            TCPStateMachine = CLOSING;           // enter FIN_WAIT_2 (look above) and therefore
            SocketStatus &= ~SOCK_CONNECTED;     // TIME_WAIT
            break;
          case FIN_WAIT_2 :
            TCPStateMachine = TIME_WAIT;
            SocketStatus &= ~SOCK_CONNECTED;            
            TCPStartFinTimer();                  // start TIME_WAIT timeout
            break;
          case TIME_WAIT :
            TCPRestartTimer();                   // restart TIME_WAIT timeout
            break;
        }
        TCPAckNr++;                              // ACK remote's FIN flag
        
        PrepareTCP_FRAME(TCPSegAck, TCPAckNr, TCP_CODE_ACK);
      }
  }
}
//------------------------------------------------------------------------------
// easyWEB internal function
// prepares the TxFrame2-buffer to send an ARP-request
//------------------------------------------------------------------------------
static void PrepareARP_REQUEST(void)
{
  // Ethernet
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS) = 0xffff;  // we don't know opposites MAC!
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 2) = 0xffff;
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 4) = 0xffff;
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS) = MyMAC[0];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 2) = MyMAC[1];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 4) = MyMAC[2];
  ACCESS_UINT(TxFrame2Mem, ETH_TYPE_OFS) = SWAPB(FRAME_ARP);

  // ARP
  ACCESS_UINT(TxFrame2Mem, ARP_HARDW_OFS) = SWAPB(HARDW_ETH10);
  ACCESS_UINT(TxFrame2Mem, ARP_PROT_OFS) = SWAPB(FRAME_IP); 
  ACCESS_UINT(TxFrame2Mem, ARP_HLEN_PLEN_OFS) = SWAPB(IP_HLEN_PLEN);
  ACCESS_UINT(TxFrame2Mem, ARP_OPCODE_OFS) = SWAPB(OP_ARP_REQUEST);
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_HA_OFS) = MyMAC[0];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_HA_OFS + 2) = MyMAC[1];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_HA_OFS + 4) = MyMAC[2];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_IP_OFS) = MyIP[0];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_IP_OFS + 2) = MyIP[1];
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_HA_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_HA_OFS + 2) = 0;
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_HA_OFS + 4) = 0;
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS + 2) = 0;

  if (((RemoteIP[0] ^ MyIP[0]) & SubnetMask[0]) ||
      ((RemoteIP[1] ^ MyIP[1]) & SubnetMask[1]))
  {
    ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS) = GatewayIP[0];
    ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS + 2) = GatewayIP[1];
  }
  else
  {
    ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS) = RemoteIP[0];
    ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS + 2) = RemoteIP[1];
  }

  TxFrame2Size = ETH_HEADER_SIZE + ARP_FRAME_SIZE;
  TransmitControl |= SEND_FRAME2;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// prepares the TxFrame2-buffer to send an ARP-answer (reply)
//------------------------------------------------------------------------------
static void PrepareARP_ANSWER(void)
{
  // Ethernet
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS) = RecdFrameMAC[0];
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 2) = RecdFrameMAC[1];
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 4) = RecdFrameMAC[2];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS) = MyMAC[0];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 2) = MyMAC[1];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 4) = MyMAC[2];
  ACCESS_UINT(TxFrame2Mem, ETH_TYPE_OFS) = SWAPB(FRAME_ARP);

  // ARP
  ACCESS_UINT(TxFrame2Mem, ARP_HARDW_OFS) = SWAPB(HARDW_ETH10);
  ACCESS_UINT(TxFrame2Mem, ARP_PROT_OFS) = SWAPB(FRAME_IP); 
  ACCESS_UINT(TxFrame2Mem, ARP_HLEN_PLEN_OFS) = SWAPB(IP_HLEN_PLEN);
  ACCESS_UINT(TxFrame2Mem, ARP_OPCODE_OFS) = SWAPB(OP_ARP_ANSWER);
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_HA_OFS) = MyMAC[0];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_HA_OFS + 2) = MyMAC[1];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_HA_OFS + 4) = MyMAC[2];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_IP_OFS) = MyIP[0];
  ACCESS_UINT(TxFrame2Mem, ARP_SENDER_IP_OFS + 2) = MyIP[1];
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_HA_OFS) = RecdFrameMAC[0];
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_HA_OFS + 2) = RecdFrameMAC[1];
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_HA_OFS + 4) = RecdFrameMAC[2];
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS) = RecdFrameIP[0];
  ACCESS_UINT(TxFrame2Mem, ARP_TARGET_IP_OFS + 2) = RecdFrameIP[1];

  TxFrame2Size = ETH_HEADER_SIZE + ARP_FRAME_SIZE;
  TransmitControl |= SEND_FRAME2;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// prepares the TxFrame2-buffer to send an ICMP-echo-reply
//------------------------------------------------------------------------------
static void PrepareICMP_ECHO_REPLY(void)
{
  unsigned int ICMPDataCount;

  if (RecdIPFrameLength > MAX_ETH_TX_DATA_SIZE)  // don't overload TX-buffer
    ICMPDataCount = MAX_ETH_TX_DATA_SIZE - IP_HEADER_SIZE - ICMP_HEADER_SIZE;
  else
    ICMPDataCount = RecdIPFrameLength - IP_HEADER_SIZE - ICMP_HEADER_SIZE;

  // Ethernet
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS) = RecdFrameMAC[0];
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 2) = RecdFrameMAC[1];
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 4) = RecdFrameMAC[2];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS) = MyMAC[0];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 2) = MyMAC[1];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 4) = MyMAC[2];
  ACCESS_UINT(TxFrame2Mem, ETH_TYPE_OFS) = SWAPB(FRAME_IP);
  
  // IP   
  ACCESS_UINT(TxFrame2Mem, IP_VER_IHL_TOS_OFS) = SWAPB(IP_VER_IHL);
  ACCESS_UINT(TxFrame2Mem, IP_TOTAL_LENGTH_OFS) =
    __swap_bytes(IP_HEADER_SIZE + ICMP_HEADER_SIZE + ICMPDataCount);
  ACCESS_UINT(TxFrame2Mem, IP_IDENT_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, IP_FLAGS_FRAG_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, IP_TTL_PROT_OFS) = SWAPB((DEFAULT_TTL << 8) | PROT_ICMP);
  ACCESS_UINT(TxFrame2Mem, IP_HEAD_CHKSUM_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, IP_SOURCE_OFS) = MyIP[0];
  ACCESS_UINT(TxFrame2Mem, IP_SOURCE_OFS + 2) = MyIP[1];
  ACCESS_UINT(TxFrame2Mem, IP_DESTINATION_OFS) = RecdFrameIP[0];
  ACCESS_UINT(TxFrame2Mem, IP_DESTINATION_OFS + 2) = RecdFrameIP[1];
  ACCESS_UINT(TxFrame2Mem, IP_HEAD_CHKSUM_OFS) = 
    CalcChecksum((unsigned char *)TxFrame2Mem + IP_VER_IHL_TOS_OFS, IP_HEADER_SIZE, 0);

  // ICMP
  ACCESS_UINT(TxFrame2Mem, ICMP_TYPE_CODE_OFS) = SWAPB(ICMP_ECHO_REPLY << 8);
  ACCESS_UINT(TxFrame2Mem, ICMP_CHKSUM_OFS) = 0;                // initialize checksum field
  CopyFromFrame8900((unsigned char *)TxFrame2Mem + ICMP_DATA_OFS, ICMPDataCount);  // get data to echo...
  ACCESS_UINT(TxFrame2Mem, ICMP_CHKSUM_OFS) = 
    CalcChecksum((unsigned char *)TxFrame2Mem + IP_DATA_OFS, ICMPDataCount +
      ICMP_HEADER_SIZE, 0);

  TxFrame2Size = ETH_HEADER_SIZE + IP_HEADER_SIZE + ICMP_HEADER_SIZE + ICMPDataCount;
  TransmitControl |= SEND_FRAME2;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// prepares the TxFrame2-buffer to send a general TCP frame
// the TCPCode-field is passed as an argument
//------------------------------------------------------------------------------
static void PrepareTCP_FRAME(unsigned long seqnr, unsigned long acknr,
  unsigned int TCPCode)
{
  // Ethernet
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS) = RemoteMAC[0];
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 2) = RemoteMAC[1];
  ACCESS_UINT(TxFrame2Mem, ETH_DA_OFS + 4) = RemoteMAC[2];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS) = MyMAC[0];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 2) = MyMAC[1];
  ACCESS_UINT(TxFrame2Mem, ETH_SA_OFS + 4) = MyMAC[2];
  ACCESS_UINT(TxFrame2Mem, ETH_TYPE_OFS) = SWAPB(FRAME_IP);
  
  // IP
  ACCESS_UINT(TxFrame2Mem, IP_VER_IHL_TOS_OFS) = SWAPB(IP_VER_IHL);

  if (TCPCode & TCP_CODE_SYN)                    // if SYN, we want to use the MSS option
    ACCESS_UINT(TxFrame2Mem, IP_TOTAL_LENGTH_OFS) =
      SWAPB(IP_HEADER_SIZE + TCP_HEADER_SIZE + TCP_OPT_MSS_SIZE);
  else
    ACCESS_UINT(TxFrame2Mem, IP_TOTAL_LENGTH_OFS) =
      SWAPB(IP_HEADER_SIZE + TCP_HEADER_SIZE);

  ACCESS_UINT(TxFrame2Mem, IP_IDENT_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, IP_FLAGS_FRAG_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, IP_TTL_PROT_OFS) = SWAPB((DEFAULT_TTL << 8) | PROT_TCP);
  ACCESS_UINT(TxFrame2Mem, IP_HEAD_CHKSUM_OFS) = 0;
  ACCESS_UINT(TxFrame2Mem, IP_SOURCE_OFS) = MyIP[0];
  ACCESS_UINT(TxFrame2Mem, IP_SOURCE_OFS + 2) = MyIP[1];
  ACCESS_UINT(TxFrame2Mem, IP_DESTINATION_OFS) = RemoteIP[0];
  ACCESS_UINT(TxFrame2Mem, IP_DESTINATION_OFS + 2) = RemoteIP[1];
  ACCESS_UINT(TxFrame2Mem, IP_HEAD_CHKSUM_OFS) = 
    CalcChecksum((unsigned char *)TxFrame2Mem + IP_VER_IHL_TOS_OFS,
      IP_HEADER_SIZE, 0);
  
  // TCP
  ACCESS_UINT(TxFrame2Mem, TCP_SRCPORT_OFS) = __swap_bytes(TCPLocalPort);
  ACCESS_UINT(TxFrame2Mem, TCP_DESTPORT_OFS) = __swap_bytes(TCPRemotePort);

  WriteDWBE((unsigned char *)TxFrame2Mem + TCP_SEQNR_OFS, seqnr);
  WriteDWBE((unsigned char *)TxFrame2Mem + TCP_ACKNR_OFS, acknr);

  ACCESS_UINT(TxFrame2Mem, TCP_WINDOW_OFS) = SWAPB(MAX_TCP_RX_DATA_SIZE);    // data bytes to accept
  ACCESS_UINT(TxFrame2Mem, TCP_CHKSUM_OFS) = 0;  // initalize checksum
  ACCESS_UINT(TxFrame2Mem, TCP_URGENT_OFS) = 0;

  if (TCPCode & TCP_CODE_SYN)                    // if SYN, we want to use the MSS option
  {
    ACCESS_UINT(TxFrame2Mem, TCP_DATA_CODE_OFS) = SWAPB(0x6000 | TCPCode);   // TCP header length = 24
    ACCESS_UINT(TxFrame2Mem, TCP_DATA_OFS) = SWAPB(TCP_OPT_MSS);             // MSS option
    ACCESS_UINT(TxFrame2Mem, TCP_DATA_OFS + 2) = SWAPB(MAX_TCP_RX_DATA_SIZE);// max. length of TCP-data we accept
    ACCESS_UINT(TxFrame2Mem, TCP_CHKSUM_OFS) =
      CalcChecksum((unsigned char *)TxFrame2Mem + TCP_SRCPORT_OFS,
        TCP_HEADER_SIZE + TCP_OPT_MSS_SIZE, 1);
    TxFrame2Size = ETH_HEADER_SIZE + IP_HEADER_SIZE + TCP_HEADER_SIZE +
      TCP_OPT_MSS_SIZE;
  }
  else
  {
    ACCESS_UINT(TxFrame2Mem, TCP_DATA_CODE_OFS) = SWAPB(0x5000 | TCPCode);   // TCP header length = 20
    ACCESS_UINT(TxFrame2Mem, TCP_CHKSUM_OFS) =
      CalcChecksum((unsigned char *)TxFrame2Mem + TCP_SRCPORT_OFS,
        TCP_HEADER_SIZE, 1);
    TxFrame2Size = ETH_HEADER_SIZE + IP_HEADER_SIZE + TCP_HEADER_SIZE;
  }

  TransmitControl |= SEND_FRAME2;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// prepares the TxFrame1-buffer to send a payload-packet
//------------------------------------------------------------------------------
static void PrepareTCP_DATA_FRAME(void)
{
  // Ethernet
  ACCESS_UINT(TxFrame1Mem, ETH_DA_OFS) = RemoteMAC[0];
  ACCESS_UINT(TxFrame1Mem, ETH_DA_OFS + 2) = RemoteMAC[1];
  ACCESS_UINT(TxFrame1Mem, ETH_DA_OFS + 4) = RemoteMAC[2];
  ACCESS_UINT(TxFrame1Mem, ETH_SA_OFS) = MyMAC[0];
  ACCESS_UINT(TxFrame1Mem, ETH_SA_OFS + 2) = MyMAC[1];
  ACCESS_UINT(TxFrame1Mem, ETH_SA_OFS + 4) = MyMAC[2];
  ACCESS_UINT(TxFrame1Mem, ETH_TYPE_OFS) = SWAPB(FRAME_IP);
  
  // IP   
  ACCESS_UINT(TxFrame1Mem, IP_VER_IHL_TOS_OFS) = SWAPB(IP_VER_IHL);
  ACCESS_UINT(TxFrame1Mem, IP_TOTAL_LENGTH_OFS) =
    __swap_bytes(IP_HEADER_SIZE + TCP_HEADER_SIZE + TCPTxDataCount);
  ACCESS_UINT(TxFrame1Mem, IP_IDENT_OFS) = 0;
  ACCESS_UINT(TxFrame1Mem, IP_FLAGS_FRAG_OFS) = 0;
  ACCESS_UINT(TxFrame1Mem, IP_TTL_PROT_OFS) = SWAPB((DEFAULT_TTL << 8) | PROT_TCP);
  ACCESS_UINT(TxFrame1Mem, IP_HEAD_CHKSUM_OFS) = 0;
  ACCESS_UINT(TxFrame1Mem, IP_SOURCE_OFS) = MyIP[0];
  ACCESS_UINT(TxFrame1Mem, IP_SOURCE_OFS + 2) = MyIP[1];
  ACCESS_UINT(TxFrame1Mem, IP_DESTINATION_OFS) = RemoteIP[0];
  ACCESS_UINT(TxFrame1Mem, IP_DESTINATION_OFS + 2) = RemoteIP[1];
  ACCESS_UINT(TxFrame1Mem, IP_HEAD_CHKSUM_OFS) = 
    CalcChecksum((unsigned char *)TxFrame1Mem + IP_VER_IHL_TOS_OFS,
      IP_HEADER_SIZE, 0);

  // TCP
  ACCESS_UINT(TxFrame1Mem, TCP_SRCPORT_OFS) = __swap_bytes(TCPLocalPort);
  ACCESS_UINT(TxFrame1Mem, TCP_DESTPORT_OFS) = __swap_bytes(TCPRemotePort);

  WriteDWBE((unsigned char *)TxFrame1Mem + TCP_SEQNR_OFS, TCPSeqNr);
  WriteDWBE((unsigned char *)TxFrame1Mem + TCP_ACKNR_OFS, TCPAckNr);
  
  ACCESS_UINT(TxFrame1Mem, TCP_DATA_CODE_OFS) = SWAPB(0x5000 | TCP_CODE_ACK);  // TCP header length = 20
  ACCESS_UINT(TxFrame1Mem, TCP_WINDOW_OFS) = SWAPB(MAX_TCP_RX_DATA_SIZE);  // data bytes to accept
  ACCESS_UINT(TxFrame1Mem, TCP_CHKSUM_OFS) = 0;  // initalize checksum
  ACCESS_UINT(TxFrame1Mem, TCP_URGENT_OFS) = 0;
  ACCESS_UINT(TxFrame1Mem, TCP_CHKSUM_OFS) =
    CalcChecksum((unsigned char *)TxFrame1Mem + TCP_SRCPORT_OFS,
      TCP_HEADER_SIZE + TCPTxDataCount, 1);
}
//------------------------------------------------------------------------------
// easyWEB internal function
// calculates the TCP/IP checksum. if 'IsTCP != 0', the TCP pseudo-header
// will be included.
//------------------------------------------------------------------------------
static unsigned int CalcChecksum(void *Start, unsigned int Count,
  unsigned char IsTCP)
{
  unsigned long Sum = 0;
  unsigned int *pStart = Start;

  if (IsTCP)
  {                                              // if we've a TCP frame...
    Sum += MyIP[0];                              // ...include TCP pseudo-header
    Sum += MyIP[1];
    Sum += RemoteIP[0];
    Sum += RemoteIP[1];
    Sum += __swap_bytes(Count);                  // TCP header length plus data length
    Sum += SWAPB(PROT_TCP);
  }

  while (Count > 1)                              // sum words
  {                            
    Sum += *pStart++;
    Count -= 2;
  }

  if (Count)                                     // add left-over byte, if any
    Sum += *(unsigned char *)pStart;
  
 while (Sum >> 16)                              // fold 32-bit sum to 16 bits
  {
      Sum = (Sum & 0xFFFF) + (Sum >> 16);
  }
  return ~Sum;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// starts the timer as a retry-timer (used for retransmission-timeout)
//------------------------------------------------------------------------------
static void TCPStartRetryTimer(void)
{
  TCPTimer = 0;
  RetryCounter = MAX_RETRYS;
  TCPFlags |= TCP_TIMER_RUNNING;
  TCPFlags |= TIMER_TYPE_RETRY;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// starts the timer as a 'TIME_WAIT'-timer (used to finish a TCP-session)
//------------------------------------------------------------------------------
static void TCPStartFinTimer(void)
{
  TCPTimer = 0;
  TCPFlags |= TCP_TIMER_RUNNING;
  TCPFlags &= ~TIMER_TYPE_RETRY;  
}
//------------------------------------------------------------------------------
// easyWEB internal function
// restarts the timer
//------------------------------------------------------------------------------
static void TCPRestartTimer(void)
{
  TCPTimer = 0;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// stopps the timer
//------------------------------------------------------------------------------
static void TCPStopTimer(void)
{
  TCPFlags &= ~TCP_TIMER_RUNNING;
}
//------------------------------------------------------------------------------
// easyWEB internal function
// if a retransmission-timeout occured, check which packet
// to resend.
//------------------------------------------------------------------------------
static void TCPHandleRetransmission(void)
{
  switch (LastFrameSent)
  {
    case ARP_REQUEST :
      PrepareARP_REQUEST();
      break;
    case TCP_SYN_FRAME :
      PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_SYN);
      break;
    case TCP_SYN_ACK_FRAME :
      PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_SYN | TCP_CODE_ACK);
      break; 
    case TCP_FIN_FRAME :
      PrepareTCP_FRAME(TCPSeqNr, TCPAckNr, TCP_CODE_FIN | TCP_CODE_ACK);
      break; 
    case TCP_DATA_FRAME :
      TransmitControl |= SEND_FRAME1;
      break;
  }
}

// CHASE: added command WriteDWBE
asm("WriteDWBE:");
asm("            swpb    R13                         ; Swap bytes in lower word");
asm("            swpb    R14                         ; Swap bytes in upper word");
asm("            mov.w   R14,0(R12)                  ; Write 1st word to memory");
asm("            mov.w   R13,2(R12)                  ; Write 2nd word to memory");
asm("            swpb    R13                         ; Restore R14");
asm("            swpb    R14                         ; Restore R15");
asm("            ret");
//------------------------------------------------------------------------------
// easyWEB internal function
// if all retransmissions failed, close connection and indicate an error
//------------------------------------------------------------------------------
static void TCPHandleTimeout(void)
{
  TCPStateMachine = CLOSED;

  if ((TCPFlags & (TCP_ACTIVE_OPEN | IP_ADDR_RESOLVED)) == TCP_ACTIVE_OPEN)
    SocketStatus = SOCK_ERR_ARP_TIMEOUT;         // indicate an error to user
  else
    SocketStatus = SOCK_ERR_TCP_TIMEOUT;

  TCPFlags = 0;                                  // clear all flags
}
//------------------------------------------------------------------------------
// easyWEB internal function
// function executed every 0.262s by the MCU. used for the
// inital sequence number generator (ISN) and the TCP-timer
//------------------------------------------------------------------------------
//#pragma vector = TIMERA1_VECTOR
//__interrupt void TCPClockHandler(void)
//{
 // if (TAIV == 10)                                // check for timer overflow, reset int.-flag
  //{
   // ISNGenHigh++;                                // upper 16 bits of initial sequence number
   // TCPTimer++;                                  // timer for retransmissions
 // }
//}
