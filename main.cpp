#include <stdlib.h>

#include <iostream>
#include <iomanip>

#include <pthread.h>    // POSIX threads
#include <fcntl.h>
#include <unistd.h>

#include "C4.h"
#include "Commands.h"

using namespace std;

int main(int argc, char * argv[3]) {
    
    // Cody's COBS Communication Code
    
    C4Port * port1 = new C4Port;
    //C4Port * port2 = new C4Port;
    
    if(argc == 1 || argc == 2)
        argv[2] = "/dev/tty.usbserial-A6026PNO";
    
    cout << argv[2] << endl;
    
    int pps = 100;
    
    if(argc >= 1)
        pps = atoi(argv[1]);
        
    //char fileToOpen2[] = "/dev/tty.usbserial-A603UBJX";
    
    if(!initC4Port(port1, argv[2], packetHandler)) {
        cout << "Failed to init port1" << endl;
        return 1;
    }
    
    /*if(!initC4Port(port2, fileToOpen2, packetHandler)) {
        cout << "Failed to init port2" << endl;
        return 1;
    }*/
    
    pthread_t thread1;
    
    int rc1 = pthread_create(&thread1, NULL, updateThread, (void *) port1);
    //int rc2 = pthread_create(&thread2, NULL, updateThread, (void *) port2);
    
    if(rc1) {
        printf("ERROR; return code from pthread_create() is %d\n", rc1);
        exit(-1);
    }
    
    /*if(rc2) {
        printf("ERROR; return code from pthread_create() is %d\n", rc2);
        exit(-1);
    }*/
    
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
        
        
        if(sendReliablePacket(port1, &packet)) {
            ++totalPackets;
            ++packetsSentSinceLastSecond;
        }

        ++i;
        
        if(i >= pps) {
            cout << dec << setfill(' ') << setw(7) << (unsigned) packetsSentSinceLastSecond << " pps | " << setw(10) << (unsigned) totalPackets << " total" << endl;
            packetsSentSinceLastSecond = 0;
            i = 0;
        }
        
        usleep(1000000 / pps);
        
    }
    
    //pthread_join(thread1, NULL);
    //pthread_join(thread2, NULL);
    
    //close(port1->fileDescriptor);
    //close(port2->fileDescriptor);
    
    //delete port1;
    //delete port2;
    
    return 0;
    
}
