/*
  TinySP.h - Library for serial communication protocol.
  Created by H. G. de Marina.
*/

#ifndef TinySP_h
#define TinySP_h

#include "Arduino.h"

#define MAX_BUF 64

struct Packet
{
    unsigned char id;
    size_t len;
    unsigned char buf[MAX_BUF];
    Packet(unsigned char *, int);
};

class TinySP
{
  public:
    TinySP();

    unsigned char _buf[MAX_BUF];
    unsigned char _read_byte();
    int _decode_byte();
    int _state;
    int _ctr;

    Packet read_packet();
    void send_packet(const Packet &);
};
#endif
