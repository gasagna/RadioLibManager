#ifndef _PACKET_H
#define _PACKET_H

#include "Arduino.h"

/*
    Default settings and parameters
*/

// number of bits of header parameters. They can be chosen to reduce the size of the 
// header as much as possible, given the expected number of nodes in a network.
#define DEST_ADDRESS_NBITS 4
#define  SRC_ADDRESS_NBITS 4
#define    PACKET_ID_NBITS 8

// The maximum payload length
#define MAX_PAYLOAD_LEN  255 

// Packet types
#define RADIOLIBMANAGER_MSG_UNCONFIRMED 1
#define RADIOLIBMANAGER_MSG_CONFIRMED   2
#define RADIOLIBMANAGER_MSG_ACK         3

// get message type from system flags, first two bits (least significant bit)
#define RADIOLIBMANAGER_MSGTYPE(sysflags) (sysflags & 0b00000011)

/// @brief A bit field struct to store packet information compactly in 
// as little space as possible. The size in bits of each field also 
// determines the range of values that the fields can have.
struct Header {
    uint8_t     sysflags : 4;                  ///< header flags for the system
    uint8_t    userflags : 4;                  ///< header flags reserved to the user
   uint32_t dest_address : DEST_ADDRESS_NBITS; ///< the destination address
   uint32_t  src_address : SRC_ADDRESS_NBITS;  ///< the address of the source device
   uint32_t    packet_id : PACKET_ID_NBITS;    ///< the packet id
} __attribute__((packed)); // force this structure to be packed

/// @brief Equality operator for headers. Used for checking if a packet has been seen before
inline bool operator==(const Header& lhs, const Header& rhs) {
    return (lhs.dest_address == rhs.dest_address &&
               lhs.packet_id == rhs.packet_id &&
             lhs.src_address == rhs.src_address &&
                lhs.sysflags == rhs.sysflags &&
               lhs.userflags == rhs.userflags);
}

/// @brief Data structure for packets sent with RadioLibManager. This is
/// part of the private interface and this type is not exposed to user code.
typedef struct {
    Header  header;                   ///< the header
    uint8_t length;                   ///< the length of the payload
    uint8_t payload[MAX_PAYLOAD_LEN]; ///< the buffer array where data is stored – Not all is sent
    int16_t rssi, snr;                ///< these never get sent
} Packet_t;

/// @brief Return the number of bytes in the packet.
/// @param packet [IN]
/// @return the number of bytes in the packet
uint8_t packet_nbytes(const Packet_t& packet); 

/// @brief Construct a packet.
/// @param dest_address [IN] the address of the destination device
/// @param src_address [IN] the address of the source device
/// @param msgtype [IN] the type of message
/// @param packet_id [IN] the id of the packet
/// @param userflags [IN] user customisable packet flags
/// @param data [IN] the payload
/// @param len [IN] the length of the payload (must be <= MAX_BUF_LEN)
/// @return a `Packet_t` object
Packet_t make_packet(uint8_t dest_address,
                     uint8_t src_address,
                     uint8_t msgtype,
                     uint8_t packet_id,
                     uint8_t userflags = 0,
                    uint8_t* data = NULL,
                     uint8_t length = 0);

#endif