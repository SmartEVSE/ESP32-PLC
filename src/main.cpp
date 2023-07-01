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

// Code snippets from Qualcomm's linux driver, open-plc-tools, and uhi22's pyPLC
// see https://github.com/uhi22/pyPLC


#include <Arduino.h>
#include <SPI.h>

#include "main.h"


uint16_t available;
uint8_t buffer[3164], rxbuffer[3164];
uint8_t modem_state;
uint8_t evseMac[6] = {0x55, 0x56, 0x57, 0xAA, 0xAA, 0xAA};  // a default evse MAC. Will be overwritten later.
uint8_t NMK[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}; // a default network key. Will be overwritten later.
uint8_t NMK_EVSE[] = {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}; // In EvseMode, we use this key.
uint8_t NID[] = {1, 2, 3, 4, 5, 6, 7}; // a default network ID




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
    uint16_t cmd, total_len;
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

    for(int x=0; x< len; x++) Serial.printf("%02x ",src[x]);
    Serial.printf("\n");

    cmd = QCA7K_SPI_WRITE | QCA7K_SPI_EXTERNAL;

    digitalWrite(PIN_QCA700X_CS, LOW);
    SPI.transfer16(cmd);      // Write External
    SPI.transfer(buf, 8);     // Header
    SPI.transfer(src, len);   // Data
    SPI.transfer16(0x5555);   // Footer
    digitalWrite(PIN_QCA700X_CS, HIGH);
}

uint32_t qcaspi_read_burst(uint8_t *dst) {
    uint16_t cmd, available, rxbytes;

    available = qcaspi_read_register16(SPI_REG_RDBUF_BYTE_AVA);

    if (available) {
      // Write nr of bytes to read to SPI_REG_BFR_SIZE
      qcaspi_write_register(SPI_REG_BFR_SIZE, available);

      cmd = QCA7K_SPI_READ | QCA7K_SPI_EXTERNAL;

      digitalWrite(PIN_QCA700X_CS, LOW);
      SPI.transfer16(cmd);
      SPI.transfer(dst, available);
      digitalWrite(PIN_QCA700X_CS, HIGH);

      // we received the data, now remove the header, and footer.
      rxbytes = dst[8] + (dst[9] << 8);
      // check if the header exists and a minimum of 60 bytes are available
      if (dst[4] == 0xaa && dst[5] == 0xaa && dst[6] == 0xaa && dst[7] == 0xaa && rxbytes >= 60) {
        memcpy(dst, dst+12, rxbytes);
        Serial.printf("rxbuffer bytes: %u\n", rxbytes);
        return rxbytes;
      }
    }
    return 0;
}

void setNmkAt(uint16_t index) {
    // sets the Network Membership Key (NMK) at a certain position in the transmit buffer
    for (uint16_t i=0; i<16; i++) buffer[index+i] = NMK_EVSE[i]; // NMK 
}

void setNidAt(uint16_t index) {
    // copies the network ID (NID, 7 bytes) into the wished position in the transmit buffer
    for (uint16_t i=0; i<7 ;i++) buffer[index+i] = NID[i];
}

uint16_t getManagementMessageType() {
    // calculates the MMTYPE (base value + lower two bits), see Table 11-2 of homeplug spec
    return (rxbuffer[16]<<8) + rxbuffer[15];
}

void composeSetKey() {
    
    memset(buffer,0x00,100);  // clear buffer
    // Destination MAC
    buffer[0]=0x00; 
    buffer[1]=0xB0;
    buffer[2]=0x52;
    buffer[3]=0x00;
    buffer[4]=0x00;
    buffer[5]=0x01;                
    // Source MAC
    buffer[6]=0x02; // fixed MAC for now 
    buffer[7]=0x25; // we should receive the actual MAC of the module in the CM_SET_KEY.CNF
    buffer[8]=0xd7;
    buffer[9]=0x68;
    buffer[10]=0xee;
    buffer[11]=0x55;                
    // Protocol
    buffer[12]=0x88; // Protocol HomeplugAV
    buffer[13]=0xE1;
    buffer[14]=0x01; // version
    buffer[15]=0x08; // CM_SET_KEY.REQ
    buffer[16]=0x60; 
    buffer[17]=0x00; // frag_index
    buffer[18]=0x00; // frag_seqnum
    buffer[19]=0x01; // 0 key info type

    buffer[20]=0xaa; // 1 my nonce (0x00 in spec!)
    buffer[21]=0xaa; // 2
    buffer[22]=0xaa; // 3
    buffer[23]=0xaa; // 4

    buffer[24]=0x00; // 5 your nonce
    buffer[25]=0x00; // 6
    buffer[26]=0x00; // 7
    buffer[27]=0x00; // 8
        
    buffer[28]=0x04; // 9 nw info pid
        
    buffer[29]=0x00; // 10 info prn
    buffer[30]=0x00; // 11
    buffer[31]=0x00; // 12 pmn
    buffer[32]=0x00; // 13 CCo capability
    setNidAt(33);
                    // 14-20 nid  7 bytes from 33 to 39
                    // Network ID to be associated with the key distributed herein.
                    // The 54 LSBs of this field contain the NID (refer to Section 3.4.3.1). The
                    // two MSBs shall be set to 0b00.
    buffer[40]=0x01;// peks (payload encryption key select) Table 11-83. 01 is NMK. We had 02 here, why???
                    // with 0x0F we could choose "no key, payload is sent in the clear"
    setNmkAt(41); 
}

// Task
// 
// called every 100ms
//
void Timer100ms(void * parameter) {

  uint16_t reg16, mnt, x;

  while(1)  // infinite loop
  {
    switch(modem_state) {
      
      case MODEM_POWERUP:
        Serial.printf("Searching for modem.. ");
        reg16 = qcaspi_read_register16(SPI_REG_SIGNATURE);
        if (reg16 == QCASPI_GOOD_SIGNATURE) {
          Serial.printf("QCA700X modem found\n");
          modem_state = MODEM_WRITESPACE;
        }    
        break;

      case MODEM_WRITESPACE:
        reg16 = qcaspi_read_register16(SPI_REG_WRBUF_SPC_AVA);
        if (reg16 == 3163) {
          Serial.printf("QCA700X write space ok\n"); 
          modem_state = MODEM_CM_SET_KEY_REQ;
        }  
        break;

      case MODEM_CM_SET_KEY_REQ:
        // set up buffer with CM_SET_KEY.REQ request data
        composeSetKey();
        // Send data to modem
        qcaspi_write_burst(buffer, 60);    // minimal 60 bytes according to an4_rev5.pdf
        Serial.printf("transmitting SET_KEY.REQ, to configure the EVSE modem with random NMK\n"); 
        modem_state = MODEM_CM_SET_KEY_CNF;
        break;

      case MODEM_CM_SET_KEY_CNF:
      case MODEM_CONFIGURED:
        reg16 = qcaspi_read_burst(rxbuffer);
        mnt = getManagementMessageType();

        if (reg16) {
          for (x=0; x<reg16; x++) Serial.printf("%02x ",rxbuffer[x]);
          Serial.printf("\n");

          if (mnt == (CM_SET_KEY + MMTYPE_CNF)) {
            Serial.printf("received SET_KEY.CNF\n");
            if (rxbuffer[19] == 0x01) {
              modem_state = MODEM_CONFIGURED;
              Serial.printf("NMK set\n");
            } else Serial.printf("NMK -NOT- set\n");
          } else if (mnt == (CM_SLAC_PARAM + MMTYPE_REQ)) {
            Serial.printf("received CM_SLAC_PARAM.REQ\n");

          }
        }
        break;

      default:
        break;
    }

    // Pause the task for 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);

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
    
    Serial.begin();
    Serial.printf("\npowerup\n");

    // Create Task 100ms Timer
    xTaskCreate(
        Timer100ms,     // Function that should be called
        "Timer100ms",   // Name of the task (for debugging)
        3072,           // Stack size (bytes)                              
        NULL,           // Parameter to pass
        1,              // Task priority
        NULL            // Task handle
    );

    modem_state = MODEM_POWERUP;
}

void loop() {

  //  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  //  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  //  Serial.printf("Flash Size: %d\n", ESP.getFlashChipSize());
  //  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  //  Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());

    delay(1000);
  
}