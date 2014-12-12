#include "Commands.h"

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