#include "RadioLibManager.h"

volatile bool __rx_flag__ = false;
volatile bool __tx_flag__ = false;

#if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
#endif
void __set_rx_flag__(void) {
    __rx_flag__ = true;
}

#if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
#endif
void __set_tx_flag__(void) {
    __tx_flag__ = true;
}

RadioLibManager::RadioLibManager(PhysicalLayer& radio, uint8_t address, int max_retries) 
    : _radio(radio) {
        _max_retries = max_retries;
        _address = address;
        _this_packet_id = 0;
        _state = RadioLibManagerState::IDLE;
}

int16_t RadioLibManager::recvContinuous(void (*callback)(Packet_t& packet), uint32_t ack_delay_ms) {
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
                        _state = RadioLibManagerState::RECEIVE;
                        break;
                    }

                    // if it's a packet we expect
                    case(RADIOLIB_ERR_NONE): {
                        if (packet.header.dest_address == _address){
                            // we will need to run the callback
                            has_packet = true;

                            // send the ACK first or execute the callback
                            if (RADIOLIBMANAGER_MSGTYPE(packet.header.sysflags) == RADIOLIBMANAGER_MSG_CONFIRMED) {
                                _state = RadioLibManagerState::TRANSMIT;
                            } else {
                                _state = RadioLibManagerState::START;
                            }
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

                // go to packet listening
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

int16_t RadioLibManager::send(uint8_t* data, uint8_t length, uint8_t to, bool confirmed, uint32_t ack_timeout_ms, uint8_t userflags, int* rssi, int* snr) {

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

    // go to TX state
    _state = RadioLibManagerState::TRANSMIT;

    // number of sending attempts 
    uint8_t attempt = 0;

    while (true) {
        #ifdef DEBUGRADIO
            Serial.print("RadioLibManager state: "); Serial.println(STATEMAP[_state]);
        #endif
        switch(_state) {
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
                            _state = RadioLibManagerState::TRANSMIT;
                        }
                        break;
                    }
                    // if we have received a packet with CRC errors, resend packet
                    // as it might a corrupted acknowledgment packet. The receiver
                    // will know that it has to drop the packet if it gets sent more
                    // than once
                    case(RADIOLIB_ERR_CRC_MISMATCH): {
                        _state = RadioLibManagerState::TRANSMIT;
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

bool RadioLibManager::_has_seen_packet(Packet_t& packet) {
    for (int i = 0; i <= _header_cache.size(); i++) {
        if (_header_cache[i] == packet.header)
            return true;
    }
    return false;
}

void RadioLibManager::_store_packet(Packet_t& packet) {
    // add to the head
    _header_cache.unshift(packet.header);
}

int16_t RadioLibManager::_receiveWait(Packet_t& packet) {
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

            // return whatever readData returned
            return retcode;
        }
    }
}

int16_t RadioLibManager::_startReceive(uint32_t timeout_ms ) {
    // set the function that will be called when a new packet is received
    _radio.clearDio1Action();
    _radio.setDio1Action(__set_rx_flag__);

    // start rx mode – timeouts are given in multiples of 15.625 us
    uint32_t timeout = timeout_ms == 0 ? RADIOLIB_SX126X_RX_TIMEOUT_INF : 1000 * timeout_ms / 15.625;
    return radio.startReceive(timeout, RADIOLIB_SX126X_IRQ_RX_DEFAULT, RADIOLIB_SX126X_IRQ_RX_DONE);
}

int16_t RadioLibManager::_startReceiveWait(Packet_t& packet, uint32_t timeout_ms) {
    int16_t retcode = _startReceive(timeout_ms);
    if (retcode != RADIOLIB_ERR_NONE)
        return retcode;
    return _receiveWait(packet);
}

int16_t RadioLibManager::_transmit(Packet_t& packet) {
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