#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stddef.h>

// Data only (can be set up to 247)
#define PACKET_DATA_MAX_LENGTH 16

// Type + Meta + Length + Data
#define PACKET_DECODED_MAX_LENGTH (PACKET_DATA_MAX_LENGTH + 3)

// Stuffing + Number + Type + Meta + Length + Data + Checksum
#define PACKET_ENCODED_MAX_LENGTH (PACKET_DECODED_MAX_LENGTH + 3)

// Stuffing + Number + Type + Meta + Length + Data + Checksum + Frame
#define RACKET_RAW_MAX_LENGTH (PACKET_ENCODED_MAX_LENGTH + 1)

// Declaration of system used message bytes
extern const uint8_t ACK;
extern const uint8_t FRAME;

// Packet Formast:
//
// Reliable packet (will be resent):
// ---------------------------------------------------------------------------------
// | Number | Stuffing | Type | Meta | Length | Data ... | Checksum | Frame (0x00) |
// ---------------------------------------------------------------------------------
//
// Unreliable packet (will not be ACK'd or resent):
// ---------------------------------------------------------------------------------
// | 0xFF | Stuffing | Type | Meta | Length | Data ... | Checksum | Frame (0x00) |
// ---------------------------------------------------------------------------------
//
// The entire packet is encoded in COBS during transmission, eliminating any zero values,
// in this since COBS is kind of the outer shell of a packet, and as such the packet must
// be decoded before any of the data makes sense.

typedef struct {
    
	// Packet type
	uint8_t type;

	// May or may not have specific meaning depending on the type of this packet
	uint8_t meta;

	// Length of data, 0 means no data at all, 1 means 1 byte of data @ data[0]
	uint8_t length;

	// 0 - 251 bytes of data
	uint8_t * data;

} Packet;

#endif /* PACKET_H */