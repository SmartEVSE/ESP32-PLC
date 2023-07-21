# ESP32-PLC
a ESP32-S3 talks to a QCA7005 to read the SOC of a car

Work in Progress

- Communicating with the modem (No interrupts used).
- So far SET_KEY.REQ, SET_KEY.CNF, and CM_SLAC_PARAM.REQ are read and set.
- SOUNDS are received from the PEV, average attentuation level is calculated, and sent back in CM_ATTEN_CHAR.IND
- All Homeplug stuff done.
- SDP request/response done. IPv6 link-local address was generated from ESP's MAC.
- We receive the IPv6 address from the car.
- TCP connection from the car to the EVSE is set up.
- Car starts sending EXI encoded messages over TCP.
- First exi encoded message is decoded, and tells us what charging options the car supports. We only support DIN for now.
- Checkpoint403: Schema negotiated.
