/*----------------------------------------------------------------------------
;    
; Project:   EVSE PLC tests
;
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
;-----------------------------------------------------------------------------*/

// Code snippets from Qualcomm's linux driver, open-plc-tools,
// and the majority from uhi22's pyPLC. see https://github.com/uhi22/pyPLC
//
// Information on setting up the SLAC communication can be found in ISO 15118-3:2016
//


#include <Arduino.h>
#include <SPI.h>

#include "main.h"
#include "ipv6.h"


uint8_t txbuffer[3164], rxbuffer[3164];
uint8_t modem_state;
uint8_t myMac[6]; // the MAC of the EVSE (derived from the ESP32's MAC).
uint8_t pevMac[6]; // the MAC of the PEV.
uint8_t myModemMac[6]; // our own modem's MAC (this is different from myMAC !). Unused.
uint8_t pevModemMac[6]; // the MAC of the PEV's modem (obtained with GetSwReq). Could this be used to identify the EV?
uint8_t pevRunId[8]; // pev RunId. Received from the PEV in the CM_SLAC_PARAM.REQ message.
uint16_t AvgACVar[58]; // Average AC Variable Field. (used in CM_ATTEN_PROFILE.IND)
uint8_t NMK[16]; // Network Key. Will be initialized with a random key on each session.
uint8_t NID[] = {1, 2, 3, 4, 5, 6, 7}; // a default network ID. MSB bits 6 and 7 need to be 0.
unsigned long SoundsTimer = 0;
unsigned long ModemSearchTimer = 0;
uint8_t ModemsFound = 0;
uint8_t ReceivedSounds = 0;
uint8_t EVCCID[6];  // Mac address or ID from the PEV, used in V2G communication
uint8_t EVSOC = 0;  // State Of Charge of the EV, obtained from the 'ContractAuthenticationRequest' message


void SPI_InterruptHandler() { // Interrupt handler is currently unused

    volatile uint16_t rx_data;

    // Write zero into the SPI_REG_INTR_ENABLE register
    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(QCA7K_SPI_WRITE | QCA7K_SPI_INTERNAL | SPI_REG_INTR_ENABLE);
    SPI.transfer16(0);                  // write the value to the bus
    digitalWrite(PIN_QCA700X_CS, HIGH);
    // Read the Interrupt Cause register
    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(QCA7K_SPI_READ | QCA7K_SPI_INTERNAL | SPI_REG_INTR_CAUSE);
    rx_data = SPI.transfer16(0x0000);   // read the reason of the interrrupt
    digitalWrite(PIN_QCA700X_CS, HIGH);
    // Write contents back to Interrupt Cause register
    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(QCA7K_SPI_WRITE | QCA7K_SPI_INTERNAL | SPI_REG_INTR_CAUSE);
    SPI.transfer16(rx_data);   
    digitalWrite(PIN_QCA700X_CS, HIGH);

    


    // Re-enable Packet Available interrupt
    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(QCA7K_SPI_WRITE | QCA7K_SPI_INTERNAL | SPI_REG_INTR_ENABLE);
    SPI.transfer16(SPI_INT_PKT_AVLBL);   
    digitalWrite(PIN_QCA700X_CS, HIGH);

}

uint16_t qcaspi_read_register16(uint16_t reg) {
    uint16_t tx_data;
    uint16_t rx_data;

    tx_data = QCA7K_SPI_READ | QCA7K_SPI_INTERNAL | reg;
    
    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(tx_data);                // send the command to read the internal register
    rx_data = SPI.transfer16(0x0000);       // read the data on the bus
    digitalWrite(PIN_QCA700X_CS, HIGH);

    return rx_data;
}

void qcaspi_write_register(uint16_t reg, uint16_t value) {
    uint16_t tx_data;

    tx_data = QCA7K_SPI_WRITE | QCA7K_SPI_INTERNAL | reg;

    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(tx_data);                // send the command to write the internal register
    SPI.transfer16(value);                  // write the value to the bus
    digitalWrite(PIN_QCA700X_CS, HIGH);

}

void qcaspi_write_burst(uint8_t *src, uint32_t len) {
    uint16_t total_len;
    uint8_t buf[10];

    buf[0] = 0xAA;
	  buf[1] = 0xAA;
	  buf[2] = 0xAA;
	  buf[3] = 0xAA;
	  buf[4] = (uint8_t)((len >> 0) & 0xFF);
	  buf[5] = (uint8_t)((len >> 8) & 0xFF);
	  buf[6] = 0;
	  buf[7] = 0;

    total_len = len + 10;
    // Write nr of bytes to write to SPI_REG_BFR_SIZE
    qcaspi_write_register(SPI_REG_BFR_SIZE, total_len);
    //Serial.printf("Write buffer bytes sent: %u\n", total_len);

//    Serial.print("[TX] ");
//    for(int x=0; x< len; x++) Serial.printf("%02x ",src[x]);
//    Serial.printf("\n");
    
    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(QCA7K_SPI_WRITE | QCA7K_SPI_EXTERNAL);      // Write External
    SPI.transfer(buf, 8);     // Header
    SPI.transfer(src, len);   // Data
    SPI.transfer16(0x5555);   // Footer
    digitalWrite(PIN_QCA700X_CS, HIGH);
}

uint32_t qcaspi_read_burst(uint8_t *dst) {
    uint16_t available, rxbytes;

    available = qcaspi_read_register16(SPI_REG_RDBUF_BYTE_AVA);

    if (available && available <= QCA7K_BUFFER_SIZE) {    // prevent buffer overflow
        // Write nr of bytes to read to SPI_REG_BFR_SIZE
        qcaspi_write_register(SPI_REG_BFR_SIZE, available);
        
        digitalWrite(PIN_QCA700X_CS, LOW);
        SPI.transfer16(QCA7K_SPI_READ | QCA7K_SPI_EXTERNAL);
        SPI.transfer(dst, available);
        digitalWrite(PIN_QCA700X_CS, HIGH);

        return available;   // return nr of bytes in the rxbuffer
    }
    return 0;
}

void randomizeNmk() {
    // randomize the Network Membership Key (NMK)
    for (uint8_t i=0; i<16; i++) NMK[i] = random(256); // NMK 
}

void setNmkAt(uint16_t index) {
    // sets the Network Membership Key (NMK) at a certain position in the transmit buffer
    for (uint8_t i=0; i<16; i++) txbuffer[index+i] = NMK[i]; // NMK 
}

void setNidAt(uint16_t index) {
    // copies the network ID (NID, 7 bytes) into the wished position in the transmit buffer
    for (uint8_t i=0; i<7; i++) txbuffer[index+i] = NID[i];
}

void setMacAt(uint8_t *mac, uint16_t offset) {
    // at offset 0 in the ethernet frame, we have the destination MAC
    // at offset 6 in the ethernet frame, we have the source MAC
    for (uint8_t i=0; i<6; i++) txbuffer[offset+i]=mac[i];
}

void setRunId(uint16_t offset) {
    // at the given offset in the transmit buffer, fill the 8-bytes-RunId.
    for (uint8_t i=0; i<8; i++) txbuffer[offset+i]=pevRunId[i];
}

void setACVarField(uint16_t offset) {
    for (uint8_t i=0; i<58; i++) txbuffer[offset+i]=AvgACVar[i];
}    

uint16_t getManagementMessageType() {
    // calculates the MMTYPE (base value + lower two bits), see Table 11-2 of homeplug spec
    return rxbuffer[16]*256 + rxbuffer[15];
}

uint16_t getFrameType() {
    // returns the Ethernet Frame type
    // 88E1 = HomeplugAV 
    // 86DD = IPv6
    return rxbuffer[12]*256 + rxbuffer[13];
}



void ModemReset() {
    uint16_t reg16;
    Serial.printf("Reset QCA700X Modem. ");
    reg16 = qcaspi_read_register16(SPI_REG_SPI_CONFIG);
    reg16 = reg16 | SPI_INT_CPU_ON;     // Reset QCA700X
    qcaspi_write_register(SPI_REG_SPI_CONFIG, reg16);
}


void composeSetKey() {
    
    memset(txbuffer, 0x00, 60);  // clear buffer
    txbuffer[0]=0x00; // Destination MAC
    txbuffer[1]=0xB0;
    txbuffer[2]=0x52;
    txbuffer[3]=0x00;
    txbuffer[4]=0x00;
    txbuffer[5]=0x01;                
    setMacAt(myMac, 6);  // Source MAC         
    txbuffer[12]=0x88; // Protocol HomeplugAV
    txbuffer[13]=0xE1;
    txbuffer[14]=0x01; // version
    txbuffer[15]=0x08; // CM_SET_KEY.REQ
    txbuffer[16]=0x60; 
    txbuffer[17]=0x00; // frag_index
    txbuffer[18]=0x00; // frag_seqnum
    txbuffer[19]=0x01; // 0 key info type
                     // 20-23 my nonce (0x00 in spec!)
                     // 24-27 your nonce
    txbuffer[28]=0x04; // 9 nw info pid
        
    txbuffer[29]=0x00; // 10 info prn
    txbuffer[30]=0x00; // 11
    txbuffer[31]=0x00; // 12 pmn
    txbuffer[32]=0x00; // 13 CCo capability
    setNidAt(33);    // 14-20 nid  7 bytes from 33 to 39
                     // Network ID to be associated with the key distributed herein.
                     // The 54 LSBs of this field contain the NID (refer to Section 3.4.3.1). The
                     // two MSBs shall be set to 0b00.
    txbuffer[40]=0x01; // NewEKS. Table A.8 01 is NMK.
    setNmkAt(41); 
}

void composeGetSwReq() {
		// GET_SW.REQ request
    memset(txbuffer, 0x00, 60);  // clear buffer
    txbuffer[0]=0xff;  // Destination MAC Broadcast
    txbuffer[1]=0xff;
    txbuffer[2]=0xff;
    txbuffer[3]=0xff;
    txbuffer[4]=0xff;
    txbuffer[5]=0xff;                
    setMacAt(myMac, 6);  // Source MAC         
    txbuffer[12]=0x88; // Protocol HomeplugAV
    txbuffer[13]=0xE1;
    txbuffer[14]=0x00; // version
    txbuffer[15]=0x00; // GET_SW.REQ
    txbuffer[16]=0xA0;  
    txbuffer[17]=0x00; // Vendor OUI
    txbuffer[18]=0xB0;  
    txbuffer[19]=0x52;  
}

void composeSlacParamCnf() {

    memset(txbuffer, 0x00, 60);  // clear txbuffer
    setMacAt(pevMac, 0);  // Destination MAC
    setMacAt(myMac, 6);  // Source MAC
    txbuffer[12]=0x88; // Protocol HomeplugAV
    txbuffer[13]=0xE1;
    txbuffer[14]=0x01; // version
    txbuffer[15]=0x65; // SLAC_PARAM.CNF
    txbuffer[16]=0x60; // 
    txbuffer[17]=0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    txbuffer[18]=0x00; // 
    txbuffer[19]=0xff; // 19-24 sound target
    txbuffer[20]=0xff; 
    txbuffer[21]=0xff; 
    txbuffer[22]=0xff; 
    txbuffer[23]=0xff; 
    txbuffer[24]=0xff; 
    txbuffer[25]=0x0A; // sound count
    txbuffer[26]=0x06; // timeout
    txbuffer[27]=0x01; // resptype
    setMacAt(pevMac, 28);  // forwarding_sta, same as PEV MAC, plus 2 bytes 00 00
    txbuffer[34]=0x00; // 
    txbuffer[35]=0x00; // 
    setRunId(36);  // 36 to 43 runid 8 bytes 
    // rest is 00
}

 void composeAttenCharInd() {
    
    memset(txbuffer, 0x00, 130);  // clear txbuffer
    setMacAt(pevMac, 0);  // Destination MAC
    setMacAt(myMac, 6);  // Source MAC
    txbuffer[12]=0x88; // Protocol HomeplugAV
    txbuffer[13]=0xE1;
    txbuffer[14]=0x01; // version
    txbuffer[15]=0x6E; // ATTEN_CHAR.IND
    txbuffer[16]=0x60;  
    txbuffer[17]=0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    txbuffer[18]=0x00; // 
    txbuffer[19]=0x00; // apptype
    txbuffer[20]=0x00; // security
    setMacAt(pevMac, 21); // Mac address of the EV Host which initiates the SLAC process
    setRunId(27); // RunId 8 bytes 
    txbuffer[35]=0x00; // 35 - 51 source_id, 17 bytes 0x00 (defined in ISO15118-3 table A.4)
        
    txbuffer[52]=0x00; // 52 - 68 response_id, 17 bytes 0x00. (defined in ISO15118-3 table A.4)
    
    txbuffer[69]=ReceivedSounds; // Number of sounds. 10 in normal case. 
    txbuffer[70]=0x3A; // Number of groups = 58. (defined in ISO15118-3 table A.4)
    setACVarField(71); // 71 to 128: The group attenuation for the 58 announced groups.
 }


void composeSlacMatchCnf() {
    
    memset(txbuffer, 0x00, 109);  // clear txbuffer
    setMacAt(pevMac, 0);  // Destination MAC
    setMacAt(myMac, 6);  // Source MAC
    txbuffer[12]=0x88; // Protocol HomeplugAV
    txbuffer[13]=0xE1;
    txbuffer[14]=0x01; // version
    txbuffer[15]=0x7D; // SLAC_MATCH.CNF
    txbuffer[16]=0x60; // 
    txbuffer[17]=0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    txbuffer[18]=0x00; // 
    txbuffer[19]=0x00; // apptype
    txbuffer[20]=0x00; // security
    txbuffer[21]=0x56; // length 2 byte
    txbuffer[22]=0x00;  
                          // 23 - 39: pev_id 17 bytes. All zero.
    setMacAt(pevMac, 40); // Pev Mac address
                          // 46 - 62: evse_id 17 bytes. All zero.
    setMacAt(myMac, 63);  // 63 - 68 evse_mac 
    setRunId(69);         // runid 8 bytes 69-76 run_id.
                          // 77 to 84 reserved 0
    setNidAt(85);         // 85-91 NID. We can nearly freely choose this, but the upper two bits need to be zero
                          // 92 reserved 0                                 
    setNmkAt(93);         // 93 to 108 NMK. We can freely choose this. Normally we should use a random number. 
}        

void composeFactoryDefaults() {

    memset(txbuffer, 0x00, 60);  // clear buffer
    txbuffer[0]=0x00; // Destination MAC
    txbuffer[1]=0xB0;
    txbuffer[2]=0x52;
    txbuffer[3]=0x00;
    txbuffer[4]=0x00;
    txbuffer[5]=0x01;                
    setMacAt(myMac, 6); // Source MAC         
    txbuffer[12]=0x88; // Protocol HomeplugAV
    txbuffer[13]=0xE1;
    txbuffer[14]=0x00; // version
    txbuffer[15]=0x7C; // Load modem Factory Defaults (same as holding GPIO3 low for 15 secs)
    txbuffer[16]=0xA0; 
    txbuffer[17]=0x00; 
    txbuffer[18]=0xB0; 
    txbuffer[19]=0x52; 
}

// Received SLAC messages from the PEV are handled here
void SlacManager(uint16_t rxbytes) {
    uint16_t reg16, mnt, x;

    mnt = getManagementMessageType();
  
  //  Serial.print("[RX] ");
  //  for (x=0; x<rxbytes; x++) Serial.printf("%02x ",rxbuffer[x]);
  //  Serial.printf("\n");

    if (mnt == (CM_SET_KEY + MMTYPE_CNF)) {
        Serial.printf("received SET_KEY.CNF\n");
        if (rxbuffer[19] == 0x01) {
            modem_state = MODEM_CONFIGURED;
            // copy MAC from the EVSE modem to myModemMac. This MAC is not used for communication.
            memcpy(myModemMac, rxbuffer+6, 6);
            Serial.printf("NMK set\n");
        } else Serial.printf("NMK -NOT- set\n");

    } else if (mnt == (CM_SLAC_PARAM + MMTYPE_REQ)) {
        Serial.printf("received CM_SLAC_PARAM.REQ\n");
        // We received a SLAC_PARAM request from the PEV. This is the initiation of a SLAC procedure.
        // We extract the pev MAC from it.
        memcpy(pevMac, rxbuffer+6, 6);
        // extract the RunId from the SlacParamReq, and store it for later use
        memcpy(pevRunId, rxbuffer+21, 8);
        // We are EVSE, we want to answer.
        composeSlacParamCnf();
        qcaspi_write_burst(txbuffer, 60); // Send data to modem
        modem_state = SLAC_PARAM_CNF;
        Serial.printf("transmitting CM_SLAC_PARAM.CNF\n");

    } else if (mnt == (CM_START_ATTEN_CHAR + MMTYPE_IND) && modem_state == SLAC_PARAM_CNF) {
        Serial.printf("received CM_START_ATTEN_CHAR.IND\n");
        SoundsTimer = millis(); // start timer
        memset(AvgACVar, 0x00, 58); // reset averages.
        ReceivedSounds = 0;
        modem_state = MNBC_SOUND;

    } else if (mnt == (CM_MNBC_SOUND + MMTYPE_IND) && modem_state == MNBC_SOUND) { 
        Serial.printf("received CM_MNBC_SOUND.IND\n");
        ReceivedSounds++;

    } else if (mnt == (CM_ATTEN_PROFILE + MMTYPE_IND) && modem_state == MNBC_SOUND) { 
        Serial.printf("received CM_ATTEN_PROFILE.IND\n");
        for (x=0; x<58; x++) AvgACVar[x] += rxbuffer[27+x];
      
        if (ReceivedSounds == 10) {
            Serial.printf("Start Average Calculation\n");
            for (x=0; x<58; x++) AvgACVar[x] = AvgACVar[x] / ReceivedSounds;
        }  

    } else if (mnt == (CM_ATTEN_CHAR + MMTYPE_RSP) && modem_state == ATTEN_CHAR_IND) { 
        Serial.printf("received CM_ATTEN_CHAR.RSP\n");
        // verify pevMac, RunID, and succesful Slac fields
        if (memcmp(pevMac, rxbuffer+21, 6) == 0 && memcmp(pevRunId, rxbuffer+27, 8) == 0 && rxbuffer[69] == 0) {
            Serial.printf("Successful SLAC process\n");
            modem_state = ATTEN_CHAR_RSP;
        } else modem_state = MODEM_CONFIGURED; // probably not correct, should ignore data, and retransmit CM_ATTEN_CHAR.IND

    } else if (mnt == (CM_SLAC_MATCH + MMTYPE_REQ) && modem_state == ATTEN_CHAR_RSP) { 
        Serial.printf("received CM_SLAC_MATCH.REQ\n"); 
        // Verify pevMac, RunID and MVFLength fields
        if (memcmp(pevMac, rxbuffer+40, 6) == 0 && memcmp(pevRunId, rxbuffer+69, 8) == 0 && rxbuffer[21] == 0x3e) {
            composeSlacMatchCnf();
            qcaspi_write_burst(txbuffer, 109); // Send data to modem
            Serial.printf("transmitting CM_SLAC_MATCH.CNF\n");
            modem_state = MODEM_GET_SW_REQ;
        }

    } else if (mnt == (CM_GET_SW + MMTYPE_CNF) && modem_state == MODEM_WAIT_SW) { 
        // Both the local and Pev modem will send their software version.
        // check if the MAC of the modem is the same as our local modem.
        if (memcmp(rxbuffer+6, myModemMac, 6) != 0) { 
            // Store the Pev modem MAC, as long as it is not random, we can use it for identifying the EV (Autocharge / Plug N Charge)
            memcpy(pevModemMac, rxbuffer+6, 6);
        }
        Serial.printf("received GET_SW.CNF\n");
        ModemsFound++;
    }
}




// Task
// 
// called every 20ms
//
void Timer20ms(void * parameter) {

    uint16_t reg16, rxbytes, mnt, x;
    uint16_t FrameType;
    
    while(1)  // infinite loop
    {
        switch(modem_state) {
          
            case MODEM_POWERUP:
                Serial.printf("Searching for local modem.. ");
                reg16 = qcaspi_read_register16(SPI_REG_SIGNATURE);
                if (reg16 == QCASPI_GOOD_SIGNATURE) {
                    Serial.printf("QCA700X modem found\n");
                    modem_state = MODEM_WRITESPACE;
                }    
                break;

            case MODEM_WRITESPACE:
                reg16 = qcaspi_read_register16(SPI_REG_WRBUF_SPC_AVA);
                if (reg16 == QCA7K_BUFFER_SIZE) {
                    Serial.printf("QCA700X write space ok\n"); 
                    modem_state = MODEM_CM_SET_KEY_REQ;
                }  
                break;

            case MODEM_CM_SET_KEY_REQ:
                randomizeNmk();       // randomize Nmk, so we start with a new key.
                composeSetKey();      // set up buffer with CM_SET_KEY.REQ request data
                qcaspi_write_burst(txbuffer, 60);    // write minimal 60 bytes according to an4_rev5.pdf
                Serial.printf("transmitting SET_KEY.REQ, to configure the EVSE modem with random NMK\n"); 
                modem_state = MODEM_CM_SET_KEY_CNF;
                break;

            case MODEM_GET_SW_REQ:
                composeGetSwReq();
                qcaspi_write_burst(txbuffer, 60); // Send data to modem
                Serial.printf("Modem Search..\n");
                ModemsFound = 0; 
                ModemSearchTimer = millis();        // start timer
                modem_state = MODEM_WAIT_SW;
                break;

            default:
                // poll modem for data
                reg16 = qcaspi_read_burst(rxbuffer);

                while (reg16) {
                    // we received data, read the length of the first packet.
                    rxbytes = rxbuffer[8] + (rxbuffer[9] << 8);
                    
                    // check if the header exists and a minimum of 60 bytes are available
                    if (rxbuffer[4] == 0xaa && rxbuffer[5] == 0xaa && rxbuffer[6] == 0xaa && rxbuffer[7] == 0xaa && rxbytes >= 60) {
                        // now remove the header, and footer.
                        memcpy(rxbuffer, rxbuffer+12, reg16-14);
                        //Serial.printf("available: %u rxbuffer bytes: %u\n",reg16, rxbytes);
                    
                        FrameType = getFrameType();
                        if (FrameType == FRAME_HOMEPLUG) SlacManager(rxbytes);
                        else if (FrameType == FRAME_IPV6) IPv6Manager(rxbytes);

                        // there might be more data still in the buffer. Check if there is another packet.
                        if ((int16_t)reg16-rxbytes-14 >= 74) {
                            reg16 = reg16-rxbytes-14;
                            // move data forward.
                            memcpy(rxbuffer, rxbuffer+2+rxbytes, reg16);
                        } else reg16 = 0;
                      
                    } else {
                        Serial.printf("Invalid data!\n");
                        ModemReset();
                        modem_state = MODEM_POWERUP;
                    }  
                }
                break;
        }

        // Did the Sound timer expire?
        if (modem_state == MNBC_SOUND && (SoundsTimer + 600) < millis() ) {
            Serial.printf("SOUND timer expired\n");
            // Send CM_ATTEN_CHAR_IND, even if no Sounds were received.
            composeAttenCharInd();
            qcaspi_write_burst(txbuffer, 129); // Send data to modem
            modem_state = ATTEN_CHAR_IND;
            Serial.printf("transmitting CM_ATTEN_CHAR.IND\n");
        }

        if (modem_state == MODEM_WAIT_SW && (ModemSearchTimer + 1000) < millis() ) {
            Serial.printf("MODEM timer expired. ");
            if (ModemsFound >= 2) {
                Serial.printf("Found %u modems. Private network between EVSE and PEV established\n", ModemsFound); 
                
                Serial.printf("PEV MAC: ");
                for(x=0; x<6 ;x++) Serial.printf("%02x", pevMac[x]);
                Serial.printf(" PEV modem MAC: ");
                for(x=0; x<6 ;x++) Serial.printf("%02x", pevModemMac[x]);
                Serial.printf("\n");

                modem_state = MODEM_LINK_READY;
            } else {
                Serial.printf("(re)transmitting MODEM_GET_SW.REQ\n");
                modem_state = MODEM_GET_SW_REQ;
            } 
        }


        // Pause the task for 20ms
        vTaskDelay(20 / portTICK_PERIOD_MS);

    } // while(1)
}    


void setup() {

    pinMode(PIN_QCA700X_CS, OUTPUT);           // SPI_CS QCA7005 
    pinMode(PIN_QCA700X_INT, INPUT);           // SPI_INT QCA7005 
    pinMode(SPI_SCK, OUTPUT);     
    pinMode(SPI_MISO, INPUT);     
    pinMode(SPI_MOSI, OUTPUT);     

    digitalWrite(PIN_QCA700X_CS, HIGH); 

    // configure SPI connection to QCA modem
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, PIN_QCA700X_CS);
    // SPI mode is MODE3 (Idle = HIGH, clock in on rising edge), we use a 10Mhz SPI clock
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    //attachInterrupt(digitalPinToInterrupt(PIN_QCA700X_INT), SPI_InterruptHandler, RISING);

    Serial.begin();
    Serial.printf("\npowerup\n");

    // Create Task 20ms Timer
    xTaskCreate(
        Timer20ms,      // Function that should be called
        "Timer20ms",    // Name of the task (for debugging)
        3072,           // Stack size (bytes)                              
        NULL,           // Parameter to pass
        1,              // Task priority
        NULL            // Task handle
    );

    
    esp_read_mac(myMac, ESP_MAC_ETH); // select the Ethernet MAC     
    setSeccIp();  // use myMac to create link-local IPv6 address.

    modem_state = MODEM_POWERUP;
   
}

void loop() {

  // Serial.printf("Total heap: %u\n", ESP.getHeapSize());
  //  Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
  //  Serial.printf("Flash Size: %u\n", ESP.getFlashChipSize());
  //  Serial.printf("Total PSRAM: %u\n", ESP.getPsramSize());
  //  Serial.printf("Free PSRAM: %u\n", ESP.getFreePsram());

    delay(1000);
}