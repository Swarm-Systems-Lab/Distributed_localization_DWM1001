#include "Arduino.h"
#include "TinySP.h"

enum
{
    ST_BEFOREPKT,
    ST_INPKTID,
    ST_INPKT,
    ST_BYTESTUFF,
    ST_PKTDONE,
    ST_PKTERR,
    ST_PKTERR_END,
};

enum
{
    CONTROL1 = 0x10,
    CONTROL2 = 0x03,
};

Packet::Packet(unsigned char *b, int n) :
    id(b[0]),
    len(n - 1)
{
    memcpy(buf, b + 1, len);
}

TinySP::TinySP()
{
}

unsigned char TinySP::_read_byte()
{
   unsigned char r;

   if(Serial.available())
        r = Serial.read();

   return r;
}

Packet TinySP::read_packet()
{
    _state = ST_BEFOREPKT;
    _ctr = 0;
    while (0 == _decode_byte())
        /* EMPTY */;
    Packet ret(_buf, _ctr);
    return ret;
}

int TinySP::_decode_byte()
{
    unsigned char ch(_read_byte());
    switch (_state)
    {
        case ST_BEFOREPKT:
            if (ch == CONTROL1)
                _state = ST_INPKTID;
            break;
        case ST_INPKTID:
            _buf[_ctr++] = ch;
            _state = ST_INPKT;
            break;
        case ST_INPKT:
            if (ch == CONTROL1)
                _state = ST_BYTESTUFF;
            else
                _buf[_ctr++] = ch;
            break;
        case ST_BYTESTUFF:
            if (ch == CONTROL1)
            {
                _buf[_ctr++] = ch;
                _state = ST_INPKT;
            }
            else
                _state = ch == CONTROL2 ? ST_PKTDONE : ST_PKTERR;
            break;
        case ST_PKTERR:
            _state = ch == CONTROL1 ? ST_PKTERR_END : ST_PKTERR;
            break;
        case ST_PKTERR_END:
            _state = ch == CONTROL2 ? ST_BEFOREPKT : ST_PKTERR;
            break;
        default:
            _state = ST_PKTERR;
            break;
    }

    if (_state == ST_PKTERR
            || _state == ST_PKTERR_END)
    {
    }

    if (_state == ST_PKTDONE)
    {
        _state = ST_BEFOREPKT;
        return _ctr;
    }

    return 0;
}

void TinySP::send_packet(const Packet & p)
{
    if (p.len + 4 >= MAX_BUF)
        return;

    unsigned char pkt[MAX_BUF];

    unsigned i(0);
    int j(0);

    pkt[j++] = CONTROL1;
    pkt[j++] = p.id;

    while (i < p.len)
    {
        if (p.buf[i] == CONTROL1)
            pkt[j++] = CONTROL1;
        pkt[j++] = p.buf[i++];
    }

    pkt[j++] = CONTROL1;
    pkt[j++] = CONTROL2;

    Serial.write(pkt, j);
}
