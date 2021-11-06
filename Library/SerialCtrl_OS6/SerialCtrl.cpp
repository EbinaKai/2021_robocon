#include <SerialCtrl.h>
#include "mbed.h"

SerialCtrl::SerialCtrl(BufferedSerial *pts, Timer *ptt) {
    _Serial = pts;
    _Timer = ptt;
    TIMEOUT_MS = 8;
    RETRY_MAX = RETRY_DEF;
}
    
void SerialCtrl::param(unsigned char header, unsigned char footer) {
    STX = header;
    ETX = footer;
}

bool SerialCtrl::get() {
    int retry = 0;
    
    do {
        //judge start of text
        if( !(input() == STX) ) {
            ++retry;
        } else {
            unsigned char sum = 0;
            
            //get data
            for(int i=0; i<DATA_N;   ++i) data[i] = input();
            for(int i=0; i<DATA_N-1; ++i) sum += data[i];
    
            //judge checksum and end of text
            if( (sum == data[SUM]) && (input() == ETX) ) {
                return true;
            } else {
                ++retry;
            }
        }
    } while(retry < RETRY_MAX);
    return false;
}

void SerialCtrl::setFailMax(int n) {
    if(n >= 0) {
        RETRY_MAX = n;
    } else {
        return;
    }
}
void SerialCtrl::setFailMax() {
    RETRY_MAX = RETRY_DEF;
}

unsigned char SerialCtrl::input() {
    unsigned char temp;
    
    _Timer->reset();
    while(duration_cast<std::chrono::milliseconds>(_Timer->elapsed_time()).count() < TIMEOUT_MS) {
        if( _Serial->readable() ) {
            _Serial->read(&temp, 1);
            return temp;
        }
    }
    return NUL;
}
