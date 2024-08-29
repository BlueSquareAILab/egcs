#ifndef PACKET_HPP
#define PACKET_HPP

//16 byte
struct S_REQ_PACKET  {
    u32 header;
    u32 chipId;
    byte code;
    byte status; 
    byte index;
    byte pkt_size; 
    u16 count;
    byte extra[2];
};

//16 byte
struct S_RES_SYS_PACKET {
    u32 header;
    u32 chipId;
    byte code;
    byte version;
    byte ipAddress[4];
    byte extra[2];
};

//8 byte
struct S_RES_PACKET {
    u32 header;
    byte code;
    byte param[2];
    byte extra;
};

#endif