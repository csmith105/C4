#ifndef C4_H
#define C4_H

#include <iostream>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "Packet.h"

// How large is our sliding window?
#define TX_WINDOW_SIZE 8

// How often is the
#define UPDATE_INTERVAL 1000

#define superhex uppercase << setfill('0') << setw(2) << hex

typedef struct {

	// Where are we reading / writting to / from?
    int fileDescriptor;

	// Recieved byte buffer, used to store the incomming bytes of the next packet
	uint8_t rxBuffer[PACKET_RAW_MAX_LENGTH];
	uint16_t rxLength;
    
    uint8_t nextExpectedPacketNum;

	// Packet either to be sent or currently waiting for ACK
	uint8_t txBuffer[TX_WINDOW_SIZE][PACKET_RAW_MAX_LENGTH];
	uint16_t txLength[TX_WINDOW_SIZE];
    
    // "Highest" pending packet number
    uint8_t pendingMax;
    
    // "Lowest" pending packet number
    uint8_t pendingMin;

	// Packet handler function, will be called whenever a valid packet is recieved
	// Must be capable of executing entirely within a 1ms window
	void (*packetHandler)(Packet packet);

} C4Port;

// Easy port setup
bool initC4Port(C4Port * port, char * filename, void (*packetHandler)(Packet packet));

// Called every 1ms, does lots of things, not calling this every 1ms would be bad
void * updateThread(void * port);

inline bool isACK(uint8_t * data, uint8_t length);

void sendACK(C4Port * port, uint8_t number);

// Checks the provided packet's data against it's checksum
// Returns true if a packet checks out, false otherwise
inline bool validateChecksum(uint8_t * data, uint8_t length, uint8_t providedChecksum);

// Computes and writes the checksum to the specified packet
// Returns true on success, false otherwise
inline uint8_t computeChecksum(uint8_t * data, uint8_t length);

// Takes in a packet struct and writes an encoded packet to a buffer
// Returns true on success, false otherwise
uint8_t encodePacket(Packet * packet, uint8_t * buffer);

// Takes in an encoded packet and writes a regular packet
// Returns true on success, false otherwise
bool decodePacket(Packet * packet, uint8_t * buffer, uint8_t length);

void writeDataToPort(C4Port * port, uint8_t * data, size_t length);

bool sendReliablePacket(C4Port * port, Packet * packet);

void sendUnreliablePacket(C4Port * port, Packet * packet);

void resendReliablePacket(C4Port * port);

#endif /* C4_H */