#include <Arduino.h>
#include "main.h"
#include "ipv6.h"
#include "src/exi/projectExiConnector.h"

/* Todo: implement a retry strategy, to cover the situation that single packets are lost on the way. */

#define NEXT_TCP 0x06  // the next protocol is TCP

#define TCP_FLAG_SYN 0x02
#define TCP_FLAG_PSH 0x08
#define TCP_FLAG_ACK 0x10

uint8_t tcpHeaderLen;
uint8_t tcpPayloadLen;
//uint8_t tcpPayload[TCP_PAYLOAD_LEN];


#define TCP_ACTIVITY_TIMER_START (5*33) /* 5 seconds */
uint16_t tcpActivityTimer;

#define TCP_TRANSMIT_PACKET_LEN 200
uint8_t TcpTransmitPacketLen;
uint8_t TcpTransmitPacket[TCP_TRANSMIT_PACKET_LEN];

#define TCPIP_TRANSMIT_PACKET_LEN 200
uint8_t TcpIpRequestLen;
uint8_t TcpIpRequest[TCPIP_TRANSMIT_PACKET_LEN];

#define TCP_STATE_CLOSED 0
#define TCP_STATE_SYN_ACK 1
#define TCP_STATE_ESTABLISHED 2
#define TCP_RECEIVE_WINDOW 1000 /* number of octets we are able to receive */

uint8_t tcpState = TCP_STATE_CLOSED;
uint32_t TcpSeqNr;
uint32_t TcpAckNr;

#define TCP_RX_DATA_LEN 1000
uint8_t tcp_rxdataLen=0;
uint8_t tcp_rxdata[TCP_RX_DATA_LEN];


void routeDecoderInputData(void) {
    /* connect the data from the TCP to the exiDecoder */
    /* The TCP receive data consists of two parts: 1. The V2GTP header and 2. the EXI stream.
        The decoder wants only the EXI stream, so we skip the V2GTP header.
        In best case, we would check also the consistency of the V2GTP header here.
    */
    global_streamDec.data = &tcp_rxdata[V2GTP_HEADER_SIZE];
    global_streamDec.size = tcp_rxdataLen - V2GTP_HEADER_SIZE;
    
    /* We have something to decode, this is a good sign that the connection is fine.
        Inform the ConnectionManager that everything is fine. */
    //connMgr_ApplOk();
}


void decodeV2GTP(void) {

    uint16_t arrayLen, i;
    uint8_t strNamespace[50];
    uint8_t SchemaID, n;
    uint16_t NamespaceLen;

    routeDecoderInputData();
    projectExiConnector_decode_appHandExiDocument();
    //projectExiConnector_decode_DinExiDocument();      // Decode EXI
    tcp_rxdataLen = 0; /* mark the input data as "consumed" */
    // process data when no errors occured during decoding
    if (g_errn == 0) {
        arrayLen = aphsDoc.supportedAppProtocolReq.AppProtocol.arrayLen;
        Serial.printf("The car supports %u schemas.\n", arrayLen);
    
        // check all schemas for DIN
        for(n=0; n<arrayLen; n++) {
            memset(strNamespace, 0, sizeof(strNamespace));
            NamespaceLen = aphsDoc.supportedAppProtocolReq.AppProtocol.array[n].ProtocolNamespace.charactersLen;
            SchemaID = aphsDoc.supportedAppProtocolReq.AppProtocol.array[n].SchemaID;
            for (i=0; i< NamespaceLen; i++) {
                strNamespace[i] = aphsDoc.supportedAppProtocolReq.AppProtocol.array[n].ProtocolNamespace.characters[i];    
            }
            Serial.printf("strNameSpace %s SchemaID: %u\n", strNamespace, SchemaID);

            if (strstr((const char*)strNamespace, ":din:70121:") != NULL) {
                Serial.printf("Detected DIN\n");
                projectExiConnector_encode_appHandExiDocument(SchemaID); // test
            }
        }
    }

    
    
    
    //aphsDoc.supportedAppProtocolReq.AppProtocol.array;
    //Serial.printf("AppProtocolReq:%s AppProtocolReq_isused:%d", aphsDoc.supportedAppProtocolReq.AppProtocol.array->SchemaID, aphsDoc.supportedAppProtocolReq_isUsed);
    //projectExiConnector_encode_DinExiDocument();
    
}


void tcp_packRequestIntoEthernet(void) {
    //# packs the IP packet into an ethernet packet
    uint16_t i;
    uint16_t length;        
    
    length = TcpIpRequestLen + 6 + 6 + 2; // # Ethernet header needs 14 bytes:
                                                    // #  6 bytes destination MAC
                                                    // #  6 bytes source MAC
                                                    // #  2 bytes EtherType
    //# fill the destination MAC with the MAC of the charger
    setMacAt(pevMac, 0);
    setMacAt(myMac, 6); // bytes 6 to 11 are the source MAC
    txbuffer[12] = 0x86; // # 86dd is IPv6
    txbuffer[13] = 0xdd;
    memcpy(txbuffer+14, TcpIpRequest, length);
    
    Serial.print("[TX] ");
    for(int x=0; x<length; x++) Serial.printf("%02x",txbuffer[x]);
    Serial.printf("\n\n");

    qcaspi_write_burst(txbuffer, length);
}

void tcp_packRequestIntoIp(void) {
    // # embeds the TCP into the lower-layer-protocol: IP, Ethernet
    uint8_t i;
    uint16_t plen;
    TcpIpRequestLen = TcpTransmitPacketLen + 8 + 16 + 16; // # IP6 header needs 40 bytes:
                                                //  #   4 bytes traffic class, flow
                                                //  #   2 bytes destination port
                                                //  #   2 bytes length (incl checksum)
                                                //  #   2 bytes checksum
    TcpIpRequest[0] = 0x60; // traffic class, flow
    TcpIpRequest[1] = 0x00; 
    TcpIpRequest[2] = 0x00;
    TcpIpRequest[3] = 0x00;
    plen = TcpTransmitPacketLen; // length of the payload. Without headers.
    TcpIpRequest[4] = plen >> 8;
    TcpIpRequest[5] = plen & 0xFF;
    TcpIpRequest[6] = NEXT_TCP; // next level protocol, 0x06 = TCP in this case
    TcpIpRequest[7] = 0x40; // hop limit
    //
    // We are the EVSE. So the PevIp is our own link-local IP address.
    for (i=0; i<16; i++) {
        TcpIpRequest[8+i] = SeccIp[i]; // source IP address
    }            
    for (i=0; i<16; i++) {
        TcpIpRequest[24+i] = EvccIp[i]; // destination IP address
    }
    for (i=0; i<TcpTransmitPacketLen; i++) {
        TcpIpRequest[40+i] = TcpTransmitPacket[i];
    }
    //showAsHex(TcpIpRequest, TcpIpRequestLen, "TcpIpRequest");
    tcp_packRequestIntoEthernet();
}



void tcp_prepareTcpHeader(uint8_t tcpFlag) {
    uint8_t i;
    uint16_t checksum;

    // # TCP header needs at least 24 bytes:
    // 2 bytes source port
    // 2 bytes destination port
    // 4 bytes sequence number
    // 4 bytes ack number
    // 4 bytes DO/RES/Flags/Windowsize
    // 2 bytes checksum
    // 2 bytes urgentPointer
    // n*4 bytes options/fill (empty for the ACK frame and payload frames)
    TcpTransmitPacket[0] = (uint8_t)(seccPort >> 8); /* source port */
    TcpTransmitPacket[1] = (uint8_t)(seccPort);
    TcpTransmitPacket[2] = (uint8_t)(evccTcpPort >> 8); /* destination port */
    TcpTransmitPacket[3] = (uint8_t)(evccTcpPort);

    TcpTransmitPacket[4] = (uint8_t)(TcpSeqNr>>24); /* sequence number */
    TcpTransmitPacket[5] = (uint8_t)(TcpSeqNr>>16);
    TcpTransmitPacket[6] = (uint8_t)(TcpSeqNr>>8);
    TcpTransmitPacket[7] = (uint8_t)(TcpSeqNr);

    TcpTransmitPacket[8] = (uint8_t)(TcpAckNr>>24); /* ack number */
    TcpTransmitPacket[9] = (uint8_t)(TcpAckNr>>16);
    TcpTransmitPacket[10] = (uint8_t)(TcpAckNr>>8);
    TcpTransmitPacket[11] = (uint8_t)(TcpAckNr);
    TcpTransmitPacketLen = tcpHeaderLen + tcpPayloadLen; 
    TcpTransmitPacket[12] = (tcpHeaderLen/4) << 4; /* 70 High-nibble: DataOffset in 4-byte-steps. Low-nibble: Reserved=0. */

    TcpTransmitPacket[13] = tcpFlag; 
    TcpTransmitPacket[14] = (uint8_t)(TCP_RECEIVE_WINDOW>>8);
    TcpTransmitPacket[15] = (uint8_t)(TCP_RECEIVE_WINDOW);

    // checksum will be calculated afterwards
    TcpTransmitPacket[16] = 0;
    TcpTransmitPacket[17] = 0;

    TcpTransmitPacket[18] = 0; /* 16 bit urgentPointer. Always zero in our case. */
    TcpTransmitPacket[19] = 0;

//    TcpTransmitPacket[20] = 0x02; // Options
//    TcpTransmitPacket[21] = 0x04;
//    TcpTransmitPacket[22] = 0x05;
//    TcpTransmitPacket[23] = 0xa0;
    

    checksum = calculateUdpAndTcpChecksumForIPv6(TcpTransmitPacket, TcpTransmitPacketLen, SeccIp, EvccIp, NEXT_TCP); 
    TcpTransmitPacket[16] = (uint8_t)(checksum >> 8);
    TcpTransmitPacket[17] = (uint8_t)(checksum);

    Serial.printf("Source:%u Dest:%u Seqnr:%08x Acknr:%08x\n", seccPort, evccTcpPort, TcpSeqNr, TcpAckNr);  
}


void tcp_sendFirstAck(void) {
    Serial.printf("[TCP] sending first ACK\n");
    tcpHeaderLen = 20;
    tcpPayloadLen = 0;
    tcp_prepareTcpHeader(TCP_FLAG_ACK | TCP_FLAG_SYN);	
    tcp_packRequestIntoIp();
}

void tcp_sendAck(void) {
   Serial.printf("[TCP] sending ACK\n");
   tcpHeaderLen = 20; /* 20 bytes normal header, no options */
   tcpPayloadLen = 0;   
   tcp_prepareTcpHeader(TCP_FLAG_ACK);	
   tcp_packRequestIntoIp();
}


void evaluateTcpPacket(void) {
    uint8_t flags;
    uint32_t remoteSeqNr;
    uint32_t remoteAckNr;
    uint16_t SourcePort, DestinationPort, pLen, hdrLen, tmpPayloadLen;
        
    /* todo: check the IP addresses, checksum etc */
    //nTcpPacketsReceived++;
    pLen =  rxbuffer[18]*256 + rxbuffer[19]; /* length of the IP payload */
    hdrLen = (rxbuffer[66]>>4) * 4; /* header length in byte */
    if (pLen >= hdrLen) {
        tmpPayloadLen = pLen - hdrLen;
    } else {
        tmpPayloadLen = 0; /* no TCP payload data */
    } 
    Serial.printf("pLen=%u, hdrLen=%u, Payload=%u\n", pLen, hdrLen, tmpPayloadLen);  
    SourcePort = rxbuffer[54]*256 +  rxbuffer[55];
    DestinationPort = rxbuffer[56]*256 +  rxbuffer[57];
    if (DestinationPort != 15118) {
        Serial.printf("[TCP] wrong port.\n");
        return; /* wrong port */
    }
    //  tcpActivityTimer=TCP_ACTIVITY_TIMER_START;
    remoteSeqNr = 
            (((uint32_t)rxbuffer[58])<<24) +
            (((uint32_t)rxbuffer[59])<<16) +
            (((uint32_t)rxbuffer[60])<<8) +
            (((uint32_t)rxbuffer[61]));
    remoteAckNr = 
            (((uint32_t)rxbuffer[62])<<24) +
            (((uint32_t)rxbuffer[63])<<16) +
            (((uint32_t)rxbuffer[64])<<8) +
            (((uint32_t)rxbuffer[65]));
    Serial.printf("Source:%u Dest:%u Seqnr:%08x Acknr:%08x flags:%02x\n", SourcePort, DestinationPort, remoteSeqNr, remoteAckNr, flags);        
    flags = rxbuffer[67];
    if (flags == TCP_FLAG_SYN) { /* This is the connection setup reqest from the EV. */
        if (tcpState == TCP_STATE_CLOSED) {
            evccTcpPort = SourcePort; // update the evccTcpPort to the new TCP port
            TcpSeqNr = 0x01020304; // We start with a 'random' sequence nr
            TcpAckNr = remoteSeqNr+1; // The ACK number of our next transmit packet is one more than the received seq number.
            tcpState = TCP_STATE_SYN_ACK;
            tcp_sendFirstAck();
        }
        return;
    }    
    if (flags == TCP_FLAG_ACK && tcpState == TCP_STATE_SYN_ACK) {
        if (remoteAckNr == (TcpSeqNr + 1) ) {
            Serial.printf("-------------- TCP connection established ---------------\n\n");
            tcpState = TCP_STATE_ESTABLISHED;
        }
        return;
    }
    /* It is no connection setup. We can have the following situations here: */
    if (tcpState != TCP_STATE_ESTABLISHED) {
        /* received something while the connection is closed. Just ignore it. */
        Serial.printf("[TCP] ignore, not connected.\n");
        return;    
    } 

    // It can be an ACK, or a data package, or a combination of both. We treat the ACK and the data independent from each other,
    // to treat each combination. 
   if ((tmpPayloadLen>0) && (tmpPayloadLen< TCP_RX_DATA_LEN)) {
        /* This is a data transfer packet. */
        // flag bit PSH should also be set.
        tcp_rxdataLen = tmpPayloadLen;
        /* rxbuffer[74] is the first payload byte. */
        memcpy(tcp_rxdata, rxbuffer+74, tcp_rxdataLen);  /* provide the received data to the application */
        //     connMgr_TcpOk();
        decodeV2GTP();
        TcpAckNr = remoteSeqNr + tcp_rxdataLen; /* The ACK number of our next transmit packet is tcp_rxdataLen more than the received seq number. */
        TcpSeqNr = remoteAckNr;
        tcp_sendAck();
        return;
    }

   if (flags & TCP_FLAG_ACK) {
       //nTcpPacketsReceived+=1000;
       TcpSeqNr = remoteAckNr; /* The sequence number of our next transmit packet is given by the received ACK number. */      
   }
}

