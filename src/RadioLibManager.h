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
         CAD, ///< the manager is doing channel activity detection
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
    const char STATEMAP[9][9] = {"CAD",
                                 "DONE",
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

volatile bool __rx_flag__ = false;
volatile bool __tx_flag__ = false;

// we got a packet, set the rx flag!
#if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
#endif
void __set_rx_flag__(void) {
    __rx_flag__ = true;
}

// we send a packet, set the tx flag!
#if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
#endif
void __set_tx_flag__(void) {
    __tx_flag__ = true;
}

/*
    The  RadioLibManager class!
*/
template <class RADIO>
class RadioLibManager {
    public:
        /// @brief Construct the radio manager
        /// @param radio [IN] a radio object from RadioLib, subclass of PhysicalLayer
        /// @param address [IN] the address of this device, from 0 to 31
        /// @param max_retries [IN] number of times to send a packet without receiving
        /// @param detect_channel_activity [IN] perform channel activity detection before sending data
        ///        an acknowledgment before giving up (defaults to 3)
        RadioLibManager(RADIO& radio, uint8_t address, int max_retries = 3, bool detect_channel_activity = true) 
            : _radio(radio) {
                _max_retries = max_retries;
                _detect_channel_activity = detect_channel_activity;
                _address = address;
                _this_packet_id = 0;
                _state = RadioLibManagerState::IDLE;
        }

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
        int16_t recvContinuous(void (*callback)(Packet_t& packet), uint32_t ack_delay_ms=10) {
            // store packet here
            Packet_t packet;

            // initially no error
            int16_t retcode = RADIOLIB_ERR_NONE;

            // go to RX state
            RadioLibManagerState _state = RadioLibManagerState::START;

            // flag to activate executing the callback
            bool has_packet = false;

            while (true) {
                #ifdef DEBUGRADIO
                    Serial.print("RadioLibManager state: "); Serial.println(STATEMAP[_state]);
                #endif
                switch (_state) {
                    // first thing is to check if we need to enable the receiver
                    case RadioLibManagerState::START: {
                        // need to call startReceive again because when we called
                        // readData we have put the radio in standby
                        retcode = _startReceive();

                        if (retcode != RADIOLIB_ERR_NONE) {
                            _state = RadioLibManagerState::ERROR;
                        }

                        // after turning on the radio we might need to process a packet
                        if (has_packet == true) {
                            _state = RadioLibManagerState::PROCESS;
                        } else {
                            _state = RadioLibManagerState::RECEIVE;
                        }

                        break;
                    }

                    // wait for packets
                    case RadioLibManagerState::RECEIVE: {
                         
                        // get a packet – assumes we called _startReceive() before this
                        retcode = _receiveWait(packet);

                        switch (retcode) {
                            // if we have received a packet with CRC errors, stay in RX
                            case(RADIOLIB_ERR_CRC_MISMATCH): {
                                _state = RadioLibManagerState::START;
                                break;
                            }

                            // if it's a packet we expect
                            case(RADIOLIB_ERR_NONE): {
                                if (packet.header.dest_address == _address){
                                    // we will need to run the callback
                                    has_packet = true;

                                    // send the ACK first or execute the callback
                                    if (RADIOLIBMANAGER_MSGTYPE(packet.header.sysflags) == RADIOLIBMANAGER_MSG_CONFIRMED) {
                                        _state = _detect_channel_activity == true ? RadioLibManagerState::CAD : RadioLibManagerState::TRANSMIT;
                                    } else {
                                        _state = RadioLibManagerState::START;
                                    }
                                } else {
                                   _state = RadioLibManagerState::START;
                                }
                                break;
                            }

                            // unhandled error
                            default: {
                                _state = RadioLibManagerState::ERROR;
                            }
                        }
                        break; // RECEIVE case
                    }

                    // perform channel activity detection
                    case RadioLibManagerState::CAD: {
                        switch (_detectChannelActivity()) {
                            // wait and repeat
                            case RADIOLIB_LORA_DETECTED: {
                                delay(100);
                                _state = RadioLibManagerState::CAD;
                                break;
                            }
                            // send data
                            case RADIOLIB_CHANNEL_FREE: {
                                _state = RadioLibManagerState::TRANSMIT;
                                break;
                            }
                            // assume it's all good
                            default: {// RADIOLIB_ERR_UNKNOWN or other
                                _state = RadioLibManagerState::TRANSMIT;
                                break;
                            }
                        } 
                        break;
                    }

                    // send acknowledgment
                    case RadioLibManagerState::TRANSMIT: {
                        // wait tiny bit before sending the acknowledgment
                        delay(ack_delay_ms);

                        // make acknowledgment packet and send it
                        Packet_t ackn = make_packet(packet.header.src_address,
                                                   _address,
                                                   RADIOLIBMANAGER_MSG_ACK,
                                                   packet.header.packet_id);

                        // blocking transmit
                        _transmit(ackn);

                        // put the transceiver in rx mode
                        _state = RadioLibManagerState::START;

                        break; // TRANSMIT case
                    }

                    // after turning the radio back on in rx mode, execute callback
                    case RadioLibManagerState::PROCESS: {
                        // only execute the callback function on packets we 
                        // have not seen before, e.g. the first time we 
                        // receive it. Then also store the packet header, so we 
                        // can check if it has been already seen
                        if (!_has_seen_packet(packet)) {
                            if (callback != NULL) {
                                callback(packet);
                                _store_packet(packet);
                            }
                        }

                        // reset flag
                        has_packet = false;

                        // go to packet listening (we have already gone through START)
                        _state = RadioLibManagerState::RECEIVE;

                        break;
                    } // PROCESS case

                    // exit on error
                    case RadioLibManagerState::ERROR: {
                        _radio.sleep();
                        return retcode;
                    } // ERROR case
                }
            }
        }

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
        int16_t send(uint8_t* data, uint8_t length, uint8_t to, bool confirmed, uint32_t ack_timeout_ms, uint8_t userflags, int* rssi=NULL, int* snr=NULL) {

            // make packet
            Packet_t pckt = make_packet(to,
                                        _address,
                                        (confirmed == true) ? RADIOLIBMANAGER_MSG_CONFIRMED : RADIOLIBMANAGER_MSG_UNCONFIRMED,
                                        _this_packet_id,
                                        userflags,
                                        data,
                                        length);

            // initially no error 
            int16_t retcode = RADIOLIB_ERR_NONE;

            // start from CAD or go directly to TRANSMIT
            _state = _detect_channel_activity == true ? RadioLibManagerState::CAD : RadioLibManagerState::TRANSMIT;

            // number of sending attempts 
            uint8_t attempt = 0;

            while (true) {
                #ifdef DEBUGRADIO
                    Serial.print("RadioLibManager state: "); Serial.println(STATEMAP[_state]);
                #endif
                switch(_state) {
                   // perform channel activity detection
                    case RadioLibManagerState::CAD: {
                        switch (_detectChannelActivity()) {
                            // wait and repeat
                            case RADIOLIB_LORA_DETECTED: {
                                delay(100);
                                _state = RadioLibManagerState::CAD;
                                break;
                            }
                            // send data
                            case RADIOLIB_CHANNEL_FREE: {
                                _state = RadioLibManagerState::TRANSMIT;
                                break;
                            }
                            // assume it's all good
                            default: { // RADIOLIB_ERR_UNKNOWN or other
                                _state = RadioLibManagerState::TRANSMIT;
                                break;
                            }
                        } 
                        break;
                    }

                    // send packet
                    case RadioLibManagerState::TRANSMIT: {
                        // update attempts
                        attempt += 1;

                        // blocking call
                        _transmit(pckt);

                        // listen for the acknowledgment or exit
                        _state = confirmed == true ?  RadioLibManagerState::RECEIVE : RadioLibManagerState::DONE;

                        break;
                    }

                    // receive acknowledgment
                    case RadioLibManagerState::RECEIVE: {

                        // store ACK packet here
                        Packet_t ackn;

                        // enable RX mode and wait for the ACK or timeout
                        retcode = _startReceiveWait(ackn, ack_timeout_ms);

                        switch (retcode) {
                            // on time out resend, but not too many times
                            case(RADIOLIB_ERR_RX_TIMEOUT): {
                                if (attempt >= _max_retries) {
                                    _state = RadioLibManagerState::TIMEDOUT;
                                } else {
                                    _state = _detect_channel_activity == true ? RadioLibManagerState::CAD : RadioLibManagerState::TRANSMIT;
                                }
                                break;
                            }
                            // if we have received a packet with CRC errors, resend packet
                            // as it might a corrupted acknowledgment packet. The receiver
                            // will know that it has to drop the packet if it gets sent more
                            // than once
                            case(RADIOLIB_ERR_CRC_MISMATCH): {
                                _state = _detect_channel_activity == true ? RadioLibManagerState::CAD : RadioLibManagerState::TRANSMIT;
                                break;
                            }
                            // if we received a packet successfully and it's the packet we expect
                            case(RADIOLIB_ERR_NONE): {
                                if (ackn.header.dest_address  == _address         &&
                                    RADIOLIBMANAGER_MSGTYPE(ackn.header.sysflags) == RADIOLIBMANAGER_MSG_ACK &&
                                    ackn.header.packet_id     == _this_packet_id) {
                                    _state = RadioLibManagerState::DONE;
                                }
                                break;
                            }
                            default: {
                                _state = RadioLibManagerState::ERROR;
                            }
                        }
                        break; //break RECEIVE case
                    }

                    // exit on ERROR, TIMEDOUT and DONE
                    default: {
                        // send radio to sleep after we have sent
                        _radio.sleep();

                        // increment packet counter, noting we have at most PACKET_ID_NBITS
                        _this_packet_id = (_this_packet_id + 1) % (1 << PACKET_ID_NBITS);

                        // store rssi and snr of the last received packet
                        // this only makes sense if we received an acknowledgment
                        // but the user would not notice if they have not passed argument
                        // for rssi and snr when they do not need an acknowledment
                        if (rssi != NULL) {*rssi = _radio.getRSSI();}
                        if (snr  != NULL) {*snr  = _radio.getSNR();}

                        // return radio_state
                        return retcode;
                    }
                }
            }
        }


    private:
        /// @brief Check if the same packet header has been seen in 
        /// the last ::HEADER_CACHE_SIZE packets
        /// @param packet 
        /// @return true if the packet has been seen before
        bool _has_seen_packet(Packet_t& packet) {
            for (int i = 0; i <= _header_cache.size(); i++) {
                if (_header_cache[i] == packet.header)
                    return true;
            }
            return false;
        }

        /// @brief Store the header of a packet. We store the last ::_header_cache_size hashes.
        /// @param packet 
        void _store_packet(Packet_t& packet) {
            // add to the head
            _header_cache.unshift(packet.header);
        }

        /// @brief Detect activity on the channel
        /// @return the return code from RadioLib's scanChannel function
        int16_t _detectChannelActivity() {
            // clear action to avoid false positive when going to rx mode
            _radio.clearDio1Action();
            return _radio.scanChannel();
        }

        /// @brief Wait until DIO1 activates and read data.
        /// @param packet [in] location where the packet receivd is stored
        /// @return the return code from RadioLib's readData function
        int16_t _receiveWait(Packet_t& packet) {
            #ifdef DEBUGRADIO
                Serial.println("IN : _receiveWait");
            #endif
            // wait for the interrupt to fire
            while (true) {
                if (__rx_flag__ == true) {
                    // clear flag
                    __rx_flag__ = false;

                    // attempt to read stuff
                    int16_t retcode = _radio.readData((uint8_t*)&packet, _radio.getPacketLength());

                    // store rssi and snr
                    packet.rssi = _radio.getRSSI();
                    packet.snr  = _radio.getSNR();

                    #ifdef DEBUGRADIO
                        Serial.println("OUT: _receiveWait");
                    #endif
                    // return whatever readData returned
                    return retcode;
                }
            }
        }

        /// @brief Set interrupt service routine to call when DIO1 activates, clear
        /// IRQ flags and start receiving
        /// @param timeout_ms [IN] number of milliseconds to wait before timing out
        int16_t _startReceive(uint32_t timeout_ms=0) {
            // set the function that will be called when a new packet is received
            _radio.clearDio1Action();
            _radio.setDio1Action(__set_rx_flag__);

            // start rx mode – timeouts are given in multiples of 15.625 us
            uint32_t timeout = timeout_ms == 0 ? RADIOLIB_SX126X_RX_TIMEOUT_INF : 1000 * timeout_ms / 15.625;
            return _radio.startReceive(timeout);
        }

        /// @brief Start RX mode and wait until a packet is received or until timeout is reached.
        /// @param packet [in] location where the packet receivd is stored
        /// @param timeout_ms [IN] number of milliseconds to wait before timing out
        /// @return the error code from RadioLib's startReceive or readData, or RADIOLIB_ERR_NONE
        int16_t _startReceiveWait(Packet_t& packet, uint32_t timeout_ms=0) {
            int16_t retcode = _startReceive(timeout_ms);
            if (retcode != RADIOLIB_ERR_NONE)
                return retcode;
            return _receiveWait(packet);
        }

        /// @brief Helper function to transmit a packet with an interrupt
        /// @param packet
        /// @return the return code from RadioLib finishTransmit function
        int16_t _transmit(Packet_t& packet) {
            // set the function that will be called when a new packet is transmitted
            _radio.clearDio1Action();
            _radio.setDio1Action(__set_tx_flag__);

            // start tx mode 
            int16_t retcode = _radio.startTransmit((uint8_t*)(&packet), packet_nbytes(packet), 0);

            // unless this raised an error
            if (retcode != RADIOLIB_ERR_NONE) {
                return retcode;
            }

            // wait for the interrupt to fire, then clean up and clear IRQ status flag, 
            // disable transmitted RF
            while (true) {
                if (__tx_flag__ == true) {
                    __tx_flag__ = false;
                    return _radio.finishTransmit();
                }
            }
        }

    private:
        uint16_t             _max_retries;                       ///< max number of retries for sending a packet
        uint8_t              _address;                           ///< this device address
        uint8_t              _this_packet_id;                    ///< id of the packet to be sent
        bool                 _detect_channel_activity;
        RadioLibManagerState _state;                             ///< internal state
        RADIO&               _radio;                             ///< the RadioLib radio driver
        CircularBuffer<Header, HEADER_CACHE_SIZE> _header_cache; ///< cache for the headers of the last few previously seen packets.
                                                                 ///< it should be smaller than the maximum packet id sent by a node, 
                                                                 // otherwise we would be dropping packets because the packet count 
                                                                 // was reset, and not because a packet was sent twice
};

#endif