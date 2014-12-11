#include "C4.h"

#include <iomanip>
#include <cstdlib>
#include <stdio.h>      // Standard input/output definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <string.h>

#include "Packet.h"

using namespace std;

// Const byte values for internal protocol messages
const uint8_t ACK = 0xAA, FRAME = 0x00, URP_HEADER = 0xFF;

// Non-public method stubs
void updatePort(C4Port * port);

bool initC4Port(C4Port * port, char * filename, void (*packetHandler)(Packet packet)) {
    
    // Open the given file as read/write, don't become the controlling terminal, don't block
    int fileDescriptor = open(filename, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);

    if(fileDescriptor == -1) {
        cerr << "open_port: Unable to open " << filename << " " << strerror(errno) << endl;
        return false;
    }
    
    struct termios tty;
    
    memset(&tty, 0, sizeof tty);
    
    if(tcgetattr(fileDescriptor, &tty) != 0) {
        cerr << strerror(errno) << endl;
        return false;
    }
    
    // Baud
    //cfsetispeed(&tty, B115200);
    //cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);
    
    
    // Character size
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    // No canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout
    
    // Shut off xon/xoff ctrl
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Ignore modem controls
    tty.c_cflag |= (CLOCAL | CREAD);
    
    // Input processing
    tty.c_iflag &= ~INLCR;
    tty.c_iflag &= ~IGNCR;
    tty.c_iflag &= ~ICRNL;
    
    // Parity
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    
    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    if(tcsetattr(fileDescriptor, TCSANOW, &tty) != 0) {
        cerr << strerror(errno) << endl;
        return false;
    }
    
    // Setup genaric port stuff
    port->fileDescriptor = fileDescriptor;
    port->packetHandler = packetHandler;
    port->nextExpectedPacketNum = 1;
    port->pendingMin = 1;
    port->pendingMax = 1;
    port->rxLength = 0;
    
    return true;
    
}

inline uint8_t getNextPacketNumber(uint8_t previous) {
    
    // Generates numbers from 1 to TX_WINDOW_SIZE
    return (previous % TX_WINDOW_SIZE) + 1;
    
}

inline bool isSlotAve(C4Port * port) {
    
    return (getNextPacketNumber(port->pendingMax) != port->pendingMin);
    
}

// Returns true if a packet checks out, false otherwise
inline bool validateChecksum(uint8_t * data, uint8_t length, uint8_t providedChecksum) {

	// Does the computed checksum match the provided checksum?
	return (computeChecksum(data, length) == providedChecksum);

}

// Generates a checksum for the given data
// Returns the checksum byte
inline uint8_t computeChecksum(uint8_t * data, uint8_t length) {
    
	// Init the checksum
	uint8_t checksum = 0;

	// Process all the bytes
    for(uint8_t i = 0; i < length; ++i)
        checksum ^= data[i];

	return checksum;

}

// Takes in a packet struct and writes an encoded packet to the specified buffer
// Returns the length of bytes written
uint8_t encodePacket(Packet * packet, uint8_t * buffer) {
    
	// Check the data length
    if(packet->length > PACKET_DATA_MAX_LENGTH) {
        
        cerr << "Packet data length (" << (unsigned) packet->length << ") is grater than allowed (" << PACKET_DATA_MAX_LENGTH << ")! Aborting encoding" << endl;
        
        return 0;
        
    }

	// Grab enough space to store any packet
	uint8_t data[PACKET_ENCODED_MAX_LENGTH];

	// Going to reserve the first byte for the stuffing, starting at 1
	uint8_t length = 4;

	// Store the header stuff
	data[1] = packet->type;
	data[2] = packet->meta;
	data[3] = packet->length;
    
	// Store the data
    for(; length < packet->length + 4; ++length)
        data[length] = packet->data[length - 4];

	// Add the checksum byte
	data[length] = computeChecksum(data + 1, length - 1);
	length++;
    
    //cout << "Encoding packet of length " << dec << (unsigned) length << endl << endl;

	// At this point all the data to be sent is stored in data,
	// we'll now enode that data

	// Encode!

	// Vars needed for encoding
	size_t rIndex = 1, wIndex = 1, lastZeroAddr = 0, numBytesSinceLastZero = 1;

	while(rIndex < length) {

		if(data[rIndex] == 0) {

			buffer[lastZeroAddr] = numBytesSinceLastZero;

			numBytesSinceLastZero = 1;

			lastZeroAddr = wIndex++;

			rIndex++;

		} else {

			buffer[wIndex++] = data[rIndex++];

			numBytesSinceLastZero++;

		}

	}

	buffer[lastZeroAddr] = numBytesSinceLastZero;
    
    // Add the packet framing byte
    buffer[wIndex++] = FRAME;

	return wIndex;

}

// Takes in a packet buffer and writes a regular packet
// Returns true on success, false otherwise
bool decodePacket(Packet * packet, uint8_t * buffer, uint8_t length) {
    
    //cout << "Decoding..." << endl;
    
    //for(uint8_t i = 0; i < length; ++i)
    //    cout << superhex << (unsigned) buffer[i] << " ";
    //cout << endl << endl;

	// Vars needed for the decode step
	size_t rIndex = 0, wIndex = 0, lastZeroEnc;

	// Decode!

	while(rIndex < length) {

		// Get the number of bytes to the next 0x00
		lastZeroEnc = buffer[rIndex];

		// Does the length of bytes from our current position fit within our domain (length)?
        if(rIndex + lastZeroEnc > length && lastZeroEnc != 1) {
            
            cerr << "BAD LAST ZERO ENC!" << endl;
            
            return 0;
            
        }

		// Move along...
		rIndex++;

		// Read/write the bytes between here and the next zero encounter
		// They should all shift down by 1 byte
		for(uint8_t i = 1; i < lastZeroEnc; i++)
			buffer[wIndex++] = buffer[rIndex++];

		// Write the zero byte back
		if(lastZeroEnc != 0xFF && rIndex != length)
			buffer[wIndex++] = '\0';

	}

	// The stuffing byte gets consumed in the decode step, and all the data bytes
	// are shifted down. Therefore the length of our decoded payload is one byte less.
	--length;

	// We need to validate the data now
    if(!validateChecksum(buffer, length - 1, buffer[length - 1])) {
        
        cerr << "FAILED VALIDATION!" << endl;
        
        return false;
    }

	// At this point we passes our checksum, time to create the packet

	// Fetch the header info
	packet->type = buffer[0];
	packet->meta = buffer[1];
	packet->length = buffer[2];

	// Allocate the space for the packet's data
    packet->data = new uint8_t[packet->length];

	// Store the data
	for(uint8_t i = 0; i < packet->length; ++i)
		packet->data[i] = buffer[i + 3];

	return true;

}

// Returns true on packet send, false otherwise
void sendUnreliablePacket(C4Port * port, Packet * packet) {
    
    uint8_t buffer[PACKET_RAW_MAX_LENGTH];
    uint8_t length;
    
    length = encodePacket(packet, buffer + 1) + 1;
    
    buffer[0] = URP_HEADER;
    
    // Send it
    writeDataToPort(port, buffer, length);
    
}

// Returns true on packet send, false otherwise
bool sendReliablePacket(C4Port * port, Packet * packet) {

	// We can only send packets when a slot is empty
    if(!isSlotAve(port)) {
        //cout << "Cannot send packet, all slots filled" << endl;
        return false;
    }
    
	port->txLength[port->pendingMax - 1] = encodePacket(packet, port->txBuffer[port->pendingMax - 1] + 1) + 1;
    
    port->txBuffer[port->pendingMax - 1][0] = port->pendingMax;
    
    //cout << dec << port->txLength << endl;
    
    //cout << "S\t" << (unsigned) port->pendingMax << endl;
    
	// Send it
	writeDataToPort(port, port->txBuffer[port->pendingMax - 1], port->txLength[port->pendingMax - 1]);
    
    // Increment pendingMax, since we just stored a new packet
    port->pendingMax = getNextPacketNumber(port->pendingMax);
    
	return true;

}

void writeDataToPort(C4Port * port, uint8_t * data, size_t length) {
    
    // Write the packet to the source stream
    if(write(port->fileDescriptor, data, length) == -1 && write(port->fileDescriptor, data, 1) == -1)
        cerr << "Write error: " << strerror(errno) << endl;
    
}

void resendReliablePacket(C4Port * port, uint8_t slot) {
    
	writeDataToPort(port, port->txBuffer[slot], port->txLength[slot]);
    
}

inline bool isACK(uint8_t * data, uint8_t length) {
    
    return (length == 3 && data[0] == ACK);
    
}

void sendACK(C4Port * port, uint8_t number) {
    
    uint8_t data[] = { ACK, number, FRAME };
    
    write(port->fileDescriptor, data, 3);

}

// Must clear the rx buffer on completion
void evaluateRxData(C4Port * port) {
    
    //cout << "RXD:\t" << endl;
    //for(uint8_t i = 0; i < port->rxLength; ++i)
    //    cout << superhex << (unsigned) port->rxBuffer[i] << " ";
    //cout << endl << endl;
    
	if(isACK(port->rxBuffer, port->rxLength)) {
        
        //cout << "Recieved ACK for packet #" << (unsigned) port->rxBuffer[1] << endl;
        
        // Is this the ACK we're expecting?
        if(port->pendingMin == port->rxBuffer[1]) {
            
            port->pendingMin = getNextPacketNumber(port->pendingMin);
            
        } else {
            
            cerr << "Recieved not expected, rec: " << (unsigned) port->rxBuffer[2] << ", exp: " << (unsigned) port->nextExpectedPacketNum << "!!!!!!!!!!!!!!!!!" << endl;
            
        }

	} else {
        
		// We've recieved something that isn't an ACK, decode it
		Packet packet;
        
        //cout << "Recieving packet..." << endl;

		if(decodePacket(&packet, port->rxBuffer + 1, port->rxLength - 1)) {
            
            if(port->rxBuffer[0] == URP_HEADER) {
                
                // Is this an unreliable packet?
                
                cout << "Rec URP" << endl;
                
                // Call the packet handler
                port->packetHandler(packet);
                
            } else if(port->rxBuffer[0] == port->nextExpectedPacketNum) {
                
                // Is this the packet we're expecting?
                
                cout << "REC\t" << (unsigned) port->rxBuffer[0] << endl;
                
                // Good packet - Send ACK
                sendACK(port, port->rxBuffer[0]);
                
                // Advance nextExpectedPacketNum
                port->nextExpectedPacketNum = getNextPacketNumber(port->nextExpectedPacketNum);
                
                // Call the packet handler
                port->packetHandler(packet);
                
            } else {
                
                cerr << "Good packet, but it is not the expected value. Exp: " << (unsigned) port->nextExpectedPacketNum << " Rec: " << (unsigned) port->rxBuffer[0] << endl;
                
            }

		}

	}
    
    // Clear the RX buffer
    port->rxLength = 0;

}

void updatePort(C4Port * port) {
    
    // Attempt to read in however many bytes are left in our packet buffer
    while(port->rxLength < PACKET_RAW_MAX_LENGTH) {
        
        ssize_t readVal = read(port->fileDescriptor, port->rxBuffer + port->rxLength, 1);
        
        if(readVal == -1) {
            
            cerr << "read() returned error condition" << strerror(errno) << endl;
            
            break;
            
        } else if(!readVal) break;
        
        ++(port->rxLength);
        
        // Did we just recieve a packet framing byte?
        if(port->rxBuffer[port->rxLength - 1] == FRAME) evaluateRxData(port);
        
    }
    
    if(port->rxLength >= PACKET_RAW_MAX_LENGTH) {
        
        // We've overflowed the buffer, clear it - data loss will happen
        port->rxLength = 0;
        
        cerr << "ERR: RX Buffer overflow, clearing (data will be lost!)\r\n" << endl;
        
    }
    
}

void * updateThread(void * port) {
    
    while(true) {
        
        // Run update tick
        updatePort((C4Port *) port);
        
        // Sleep
        usleep(UPDATE_INTERVAL);
        
    }
    
    return NULL;
    
}