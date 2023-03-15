#include "packet.h"

Packet_t make_packet(uint8_t dest_address,
                     uint8_t src_address,
                     uint8_t sysflags,
                     uint8_t packet_id,
                     uint8_t userflags,
                     uint8_t *data,
                     uint8_t length) {
    Packet_t packet;
    packet.header.dest_address = dest_address;
    packet.header.src_address  = src_address;
    packet.header.sysflags     = sysflags; // only message type for now
    packet.header.packet_id    = packet_id;
    packet.header.userflags    = userflags;
    packet.length              = length;
    if (data != NULL)
        memcpy(&packet.payload, data, length);
    return packet;
}

uint8_t packet_nbytes(const Packet_t& packet) {
    // payload length, plus header plus length.
    // the snr and the rssi are not sent
    return packet.length + sizeof(Header) + 1;
}