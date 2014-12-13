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
    
    // Seek to end of file
    lseek(fileDescriptor, 0, SEEK_END);
    
    // Setup genaric port stuff
    port->fileDescriptor = fileDescriptor;
    port->packetHandler = packetHandler;
    port->nextExpectedPacketNum = 1;
    
    port->windowSize = 0;
    port->lowestSlot = 0;
    
    port->rxLength = 0;
    port->ticksSinceLastACK = 0;
    
    // Init packet numbers
    for(uint8_t i = 0; i < TX_WINDOW_SIZE; ++i)
        port->txBuffer[i][0] = i + 1;
    
    if(pthread_mutex_init(&port->lock, NULL) != 0)
    {
        printf("\n mutex init failed\n");
        return false;
    }
    
    return true;
    
}

inline uint8_t getNextSlot(uint8_t previous) {
    
    // Generates numbers from 0 to TX_WINDOW_SIZE - 1
    return ((previous + 1) % TX_WINDOW_SIZE);
    
}

inline bool isSlotAve(C4Port * port) {
    
    return (port->windowSize != TX_WINDOW_SIZE);
    
}

void writeDataToPort(C4Port * port, uint8_t * data, size_t length) {
    
    // Write the packet to the source stream
    if(write(port->fileDescriptor, data, length) == -1 && write(port->fileDescriptor, data, 1) == -1)
        cerr << "Write error: " << strerror(errno) << endl;
    
}

// Generates a checksum for the given data
// Returns the checksum byte
inline uint8_t computeChecksum(uint8_t * data, uint8_t length) {
    
    // Init the checksum
    uint8_t checksum = 0;
    
    // Process all the bytes
    for(uint8_t i = 0; i < length; ++i) checksum ^= data[i];
    
    return checksum;
    
}

// Returns true if a packet checks out, false otherwise
inline bool validateChecksum(uint8_t * data, uint8_t length, uint8_t providedChecksum) {
	return (computeChecksum(data, length) == providedChecksum);
}

// Takes in a Packet struct and writes an encoded packet to the specified buffer
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

uint8_t getNextAveSlot(C4Port * port) {
    
    return (port->lowestSlot + port->windowSize) % TX_WINDOW_SIZE;
    
}

// Returns true on packet send, false otherwise
bool sendReliablePacket(C4Port * port, Packet * packet) {

	// We can only send packets when a slot is empty
    if(!isSlotAve(port)) return false;
    
    pthread_mutex_lock(&port->lock);
    
	port->txLength[getNextAveSlot(port)] =
        encodePacket(packet, port->txBuffer[getNextAveSlot(port)] + 1) + 1;
    
    //port->txBuffer[getNextAveSlot(port)][0] = getNextAveSlot(port) + 1;
    
    //cout << dec << port->txLength << endl;
    
    //cout << "S\t" << (unsigned) port->pendingMax << endl;
    
	// Send it
	writeDataToPort(port, port->txBuffer[getNextAveSlot(port)], port->txLength[getNextAveSlot(port)]);
    
    // Increment pendingMax, since we are going to store a new packet
    ++ port->windowSize;
    
    pthread_mutex_unlock(&port->lock);
    
	return true;

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
    
    pthread_mutex_lock(&port->lock);
    
	if(isACK(port->rxBuffer, port->rxLength)) {
        
        //cout << "Recieved ACK for packet #" << (unsigned) port->rxBuffer[1] << endl;
        
        // Reset timeout
        port->ticksSinceLastACK = 0;
        
        // Is this the ACK we're expecting?
        if((port->lowestSlot + 1) == port->rxBuffer[1]) {
            
            //cout << "Recieved ACK for packet #" << (unsigned) port->rxBuffer[1] << endl;
            
            port->lowestSlot = getNextSlot(port->lowestSlot);
            -- port->windowSize;
            
        } else {
            
            cerr << "ACK not expected, rec: " << (unsigned) port->rxBuffer[1] << ", exp: " << (unsigned) port->lowestSlot + 1 << endl;
            
        }

	} else {
        
		// We've recieved something that isn't an ACK, decode it
		Packet packet;

		if(decodePacket(&packet, port->rxBuffer + 1, port->rxLength - 1)) {
            
            if(port->rxBuffer[0] == URP_HEADER) {
                
                // Is this an unreliable packet?
                
                // Call the packet handler
                port->packetHandler(packet);
                
            } else if(port->rxBuffer[0] == (port->nextExpectedPacketNum)) {
                
                // This is the packet we're expecting
                
                // Good packet - Send ACK
                sendACK(port, port->rxBuffer[0]);
                
                // Advance nextExpectedPacketNum
                port->nextExpectedPacketNum = 1 + getNextSlot(port->nextExpectedPacketNum - 1);
                
                // Call the packet handler
                port->packetHandler(packet);
                
            } else {
                
                cerr << "Good packet, but it is not the expected value. Exp: " << (unsigned) (port->nextExpectedPacketNum) << " Rec: " << (unsigned) port->rxBuffer[0] << endl;
                
            }

		}

	}
    
    // Clear the RX buffer
    port->rxLength = 0;
    
    pthread_mutex_unlock(&port->lock);

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
    
    // Did we fill the RX buffer?
    if(port->rxLength >= PACKET_RAW_MAX_LENGTH) {
        
        // We've overflowed the buffer, clear it - data loss will happen
        port->rxLength = 0;
        
        cerr << "ERR: RX Buffer overflow, clearing (data will be lost!)\r\n" << endl;
        
    }
    
    pthread_mutex_lock(&port->lock);
    
    // Are packets pending?
    if(port->windowSize)
        ++(port->ticksSinceLastACK);
    
    // Do we need to resend packets?
    if(port->ticksSinceLastACK > PACKET_TIMEOUT) {
        
        cerr << "TIMEOUT: Resending " << (unsigned) port->windowSize << " packets starting at " << (unsigned) port->lowestSlot << endl;
        
        // We must assume that a packet was lost and that all other sent packets were rejected
        // so we must resend all of the packets currently pending
        
        uint8_t slot = port->lowestSlot;
        
        for(uint8_t i = 0; i < port->windowSize; ++i) {
            
            cout << "Resending: " << (unsigned) slot << " | " << (unsigned) port->txBuffer[slot][0] << " | ";
            
            for(uint8_t i = 0; i < port->txLength[slot]; ++i)
                cout << superhex << (unsigned) port->txBuffer[slot][i] << " ";
            
            cout << endl;
            
            // Resend a packet
            writeDataToPort(port, port->txBuffer[slot], port->txLength[slot]);
            
            slot = getNextSlot(slot);
            
        }
        
        // Reset timeout
        port->ticksSinceLastACK = 0;
        
    }
    
    pthread_mutex_unlock(&port->lock);
    
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