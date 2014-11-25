#include <stdlib.h>

#include <iostream>
#include <iomanip>

#include <pthread.h>    // POSIX threads
#include <fcntl.h>
#include <unistd.h>

#include "C4.h"

using namespace std;

void packetHandler(Packet packet) {
    
    //cout << "TYPE:\t\t" << superhex << (unsigned) packet.type << endl;
    //cout << "META:\t\t" << superhex << (unsigned) packet.meta << endl;
    //cout << "LENGTH:\t\t" << superhex << (unsigned) packet.length << endl;
    //cout << "DATA:\t\t";
    
    //for(uint8_t i = 0; i < packet.length; ++i)
    //    cout << superhex << (unsigned) packet.data[i] << " ";
    
    //cout << endl << endl;
    
    delete[] packet.data;

}

int main(int argc, const char * argv[]) {
    
    // Cody's COBS Communication Code
    
    C4Port * port1 = new C4Port;
    C4Port * port2 = new C4Port;
    
    char fileToOpen1[] = "/dev/tty.usbserial-A603UC7L";
    char fileToOpen2[] = "/dev/tty.usbserial-A603UBJX";
    
    if(!initC4Port(port1, fileToOpen1, packetHandler)) {
        cout << "Failed to init port1" << endl;
        return 1;
    }
    
    if(!initC4Port(port2, fileToOpen2, packetHandler)) {
        cout << "Failed to init port2" << endl;
        return 1;
    }
    
    pthread_t thread1, thread2;
    
    int rc1 = pthread_create(&thread1, NULL, updateThread, (void *) port1);
    int rc2 = pthread_create(&thread2, NULL, updateThread, (void *) port2);
    
    if(rc1) {
        printf("ERROR; return code from pthread_create() is %d\n", rc1);
        exit(-1);
    }
    
    if(rc2) {
        printf("ERROR; return code from pthread_create() is %d\n", rc2);
        exit(-1);
    }
    
    uint8_t data[16];
    
    Packet packet;
    
    packet.type = 0xAB;
    packet.meta = 0xCD;
    packet.length = 8;
    packet.data = data;
    
    long packetsSentSinceLastSecond = 0;
    long totalPackets = 0;
    
    // Get some random
    int fd = open("/dev/random", O_RDONLY);
    
    int i = 0;
    while(true) {
        
        // Populate data randomly
        read(fd, data, 16);
        
        packet.length = data[0] % 17;
        
        if(sendPacket(port2, &packet)) {
            ++totalPackets;
            ++packetsSentSinceLastSecond;
        }
        
        ++i;
        
        if(i == 10000) {
            cout << setw(7) << packetsSentSinceLastSecond << " pps | " << setw(10) << totalPackets << " total" << endl;
            packetsSentSinceLastSecond = 0;
            i = 0;
        }
        
        usleep(100);
        
    }
    
    
    /*for(int i = 0; i <= PACKET_DATA_MAX_LENGTH; ++i) {
        
        packet.length = i;
        
        if(sendPacket(port2, &packet)) {
            cout << "Packet of length " << dec << i << " sent" << endl;
        } else {
            cout << "Packet of length " << dec << i << " NOT sent" << endl;
        }
        usleep(40000);
    }*/
    
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    
    close(port1->fileDescriptor);
    close(port2->fileDescriptor);
    
    delete port1;
    delete port2;
    
    return 0;
    
}
