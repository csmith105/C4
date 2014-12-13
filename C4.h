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

// Timing (in uS)
#define UPDATE_INTERVAL 1000

// Timing (in ticks)
#define PACKET_TIMEOUT 400
#define IDLE_TIMEOUT 10000

#define superhex uppercase << setfill('0') << setw(2) << hex

typedef struct {

	// Where are we reading / writting to / from?
    int fileDescriptor;
    
    // Packet handler function, will be called whenever a valid packet is recieved
    // Must be capable of executing entirely within a 1ms window
    void (*packetHandler) (Packet packet);

	// Recieved byte buffer, used to store the incomming bytes of the next packet
	uint8_t rxBuffer[PACKET_RAW_MAX_LENGTH];
	uint16_t rxLength;
    
    uint8_t nextExpectedPacketNum;

	// Packet either to be sent or currently waiting for ACK
	uint8_t txBuffer[TX_WINDOW_SIZE][PACKET_RAW_MAX_LENGTH];
	uint16_t txLength[TX_WINDOW_SIZE];
    
    // # of slots currently in use
    uint8_t windowSize;
    
    // Lowest slot # currently in use
    uint8_t lowestSlot;
    
    // Timeouts
    uint16_t ticksSinceLastACK;

} C4Port;

// Easy port setup
bool initC4Port(C4Port * port, char * filename, void (*packetHandler)(Packet packet));

// Called every 1ms, does lots of things, not calling this every 1ms would be bad
void * updateThread(void * port);

bool sendReliablePacket(C4Port * port, Packet * packet);

void sendUnreliablePacket(C4Port * port, Packet * packet);

#endif /* C4_H */