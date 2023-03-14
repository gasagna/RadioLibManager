#ifndef _PACKET_H
#define _PACKET_H

#include "Arduino.h"

/*
    Default settings
*/

// number of bits of header parameters. They can be chosen to reduce the size of the 
// header as much as possible, given the expected number of nodes in a network.
#define DEST_ADDRESS_NBITS 5
#define PACKET_ID_NBITS    3
#define SRC_ADDRESS_NBITS  5
#define MSG_TYPE_NBITS     3 // should not be less then 2

#define MAX_PAYLOAD_LEN  255 ///< The maximum payload length


/// @brief The type of a message
typedef enum {
        UNCONFIRMED,  ///< the type of messages sent that do not require to be acknowledged
          CONFIRMED,  ///< the type of messages sent that require to be acknowledged
    ACKNOWLEDGEMENT,  ///< the type of messages that acknowledge other messages
} Message_t;

/// @brief A bit field struct to store packet information compactly in 
// as little space as possible, i.e. two bytes. The size in bits of each
// field also determines the range of values that the fields can have.
struct Header {
   uint8_t dest_address : DEST_ADDRESS_NBITS; ///< the destination address
   uint8_t    packet_id : PACKET_ID_NBITS;    ///< the packet id
   uint8_t  src_address : SRC_ADDRESS_NBITS;  ///< the address of the source device
   Message_t   msg_type : MSG_TYPE_NBITS;     ///< the type of the message, see ::Message_t
} __attribute__((packed)); // this is necessary to force this structure to be packed

/// @brief Equality operator for headers. Used for checking if a packet has been seen before
inline bool operator==(const Header& lhs, const Header& rhs) {
    return (lhs.dest_address == rhs.dest_address &&
               lhs.packet_id == rhs.packet_id &&
             lhs.src_address == rhs.src_address &&
                lhs.msg_type == rhs.msg_type);
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
/// @param msg_type [IN] the type of message, see ::Message_t
/// @param packet_id [IN] the id of the packet, from 0 to 7
/// @param data [IN] the payload
/// @param len [IN] the length of the payload (must be <= MAX_BUF_LEN)
/// @return a `Packet_t` object
Packet_t make_packet(uint8_t dest_address,
                     uint8_t src_address,
                   Message_t message_type,
                     uint8_t packet_id,
                    uint8_t* data = NULL,
                     uint8_t length = 0);

#endif