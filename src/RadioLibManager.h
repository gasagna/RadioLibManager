#ifndef _RADIOLIBMANAGER_H
#define _RADIOLIBMANAGER_H

/*
    Default settings
*/

// the size of the header cache
#define HEADER_CACHE_SIZE 5

#include "CircularBuffer.h"
#include "RadioLib.h"
#include "packet.h"

/// @brief the state of the ::RadioLibManager object
typedef enum {
        DONE, ///< manager has finished sending a packet
        IDLE, ///< the manager is idle
       START, ///< the manager is setting up to start receiving
       ERROR, ///< the manager has encountered some unhandled error
     PROCESS, ///< manager is processing a packet just received
     RECEIVE, ///< the manager is receiving
    TIMEDOUT, ///< manager has timedout while waiting for an acknowledgment
    TRANSMIT  ///< the manager is trasmitting
} RadioLibManagerState;

// for pretty printing
#ifdef DEBUGRADIO
    const char STATEMAP[8][9] = {"DONE",
                                 "IDLE",
                                 "START",
                                 "ERROR",
                                 "PROCESS",
                                 "RECEIVE",
                                 "TIMEDOUT",
                                 "TRANSMIT"};
#endif

/*
   Interrupt Service Routines and other global flag variables
*/

// we got a packet, set the rx flag!
#if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
#endif
void __set_rx_flag__(void);

// we send a packet, set the tx flag!
#if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
#endif
void __set_tx_flag__(void);

/*
    The  RadioLibManager class!
*/
class RadioLibManager {
    public:
        /// @brief Construct the radio manager
        /// @param radio [IN] a radio object from RadioLib, subclass of PhysicalLayer
        /// @param address [IN] the address of this device, from 0 to 31
        /// @param max_retries [IN] number of times to send a packet without receiving
        ///        an acknowledgment before giving up (defaults to 3)
        RadioLibManager(PhysicalLayer& radio, uint8_t address, int max_retries = 3);

    public:
        /// @brief Set the radio in continuous receive mode and call a user provided
        /// callback function on each packet received.  As an examples of such a callback
        /// the user could print . This function never returns, unless a RadioLib error
        /// is raised on a call to RadioLib's `readData` function. Corrupted packets received 
        /// with inconsistent CRC sum, for which RadioLib returns RADIOLIB_ERR_CRC_MISMATCH 
        /// are silently dropped, and the radio returns to receive mode.
        /// @param callback 
        /// @param ack_delay_ms [in] number of milliseconds to wait before sending the acknowledgment.
        /// Setting this to zero, might lead the node that sent the message to miss the acknowledgment.
        /// @return 
        int16_t recvContinuous(void (*callback)(Packet_t& packet), uint32_t ack_delay_ms = 10);

        /// @brief Send data
        /// @param data
        /// @param length
        /// @param dest_address
        /// @param confirmed
        /// @param ack_timeout_ms
        /// @param userflags
        /// @param rssi
        /// @param snr
        /// @return
        int16_t send(uint8_t* data, uint8_t length, uint8_t to, bool confirmed, uint32_t ack_timeout_ms,
                     uint8_t userflags, int* rssi = NULL, int* snr = NULL);

    private:
        /// @brief Check if the same packet header has been seen in 
        /// the last ::HEADER_CACHE_SIZE packets
        /// @param packet 
        /// @return true if the packet has been seen before
        bool _has_seen_packet(Packet_t& packet);

        /// @brief Store the header of a packet. We store the last ::_header_cache_size hashes.
        /// @param packet 
        void _store_packet(Packet_t& packet);

        /// @brief Wait until DIO1 activates and read data.
        /// @param packet [in] location where the packet receivd is stored
        /// @return the return code from RadioLib's readData function
        int16_t _receiveWait(Packet_t& packet); 

        /// @brief Set interrupt service routine to call when DIO1 activates, clear
        /// IRQ flags and start receiving
        /// @param timeout_ms [IN] number of milliseconds to wait before timing out
        int16_t _startReceive(uint32_t timeout_ms = 0);

        /// @brief Start RX mode and wait until a packet is received or until timeout is reached.
        /// @param packet [in] location where the packet receivd is stored
        /// @param timeout_ms [IN] number of milliseconds to wait before timing out
        /// @return the error code from RadioLib's startReceive or readData, or RADIOLIB_ERR_NONE
        int16_t _startReceiveWait(Packet_t& packet, uint32_t timeout_ms = 0);

        /// @brief Helper function to transmit a packet with an interrupt
        /// @param packet
        /// @return the return code from RadioLib finishTransmit function
        int16_t _transmit(Packet_t& packet);

    private:
        uint16_t             _max_retries;                       ///< max number of retries for sending a packet
        uint8_t              _address;                           ///< this device address
        uint8_t              _this_packet_id;                    ///< id of the packet to be sent
        RadioLibManagerState _state;                             ///< internal state
        PhysicalLayer&       _radio;                             ///< the RadioLib radio driver
        CircularBuffer<Header, HEADER_CACHE_SIZE> _header_cache; ///< cache for the headers of the last few previously seen packets.
                                                                 ///< it should be smaller than the maximum packet id sent by a node, 
                                                                 // otherwise we would be dropping packets because the packet count 
                                                                 // was reset, and not because a packet was sent twice
};

#endif