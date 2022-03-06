#include "EasyTransfer.h"

const static uint8_t HEADER_BYTE_1 = 0x06;
const static uint8_t HEADER_BYTE_2 = 0x85;
const static EASY_TRANSFER_PRINT_DEBUG = false;

void EasyTransfer::begin(uint8_t * ptr, uint8_t length, Stream *theStream){
	address = ptr;
	size = length;
	_stream = theStream;
	
	//dynamic creation of rx parsing buffer in RAM
	rx_buffer = (uint8_t*) malloc(size + 1);
    for (uint8_t i = 0; i < size + 1; i++)
    {
        rx_buffer[i] = 0x00;
    }

    current_stage = ReceiveState::RX_STATE_HEADER_1;
}

void EasyTransfer::sendData()
{
    _stream->write(HEADER_BYTE_1);
    _stream->write(HEADER_BYTE_2);
    _stream->write(size);
    for (uint8_t i = 0; i < size; i++)
    {
        _stream->write(*(address+i));
    }
    uint8_t crc = CRC8.smbus(address, size);
    _stream->write(crc);

    if (EASY_TRANSFER_PRINT_DEBUG)
    {
        Serial.print("Transmitting: ");
        for (int i = 0; i < size; i++)
        {
            Serial.printf("%02x ", *(address+i));
        }
        Serial.println(); 
    }
}

boolean EasyTransfer::receiveData()
{
    // This is a rudamentary state machine.  If at any point the stream
    // is entirely emptied, the function exits and can pick up where it left
    // off the next time it's called.

    // First check for header byte #1
    if (current_stage == ReceiveState::RX_STATE_HEADER_1)
    {
        // This blocks until the header is found or the stream has been emptied
        while (_stream->available())
        {
            if (_stream->read() == HEADER_BYTE_1)
            {
                current_stage = ReceiveState::RX_STATE_HEADER_2;
                break;
            }
        }
    }

    // Next check for header byte #2
    if (current_stage == ReceiveState::RX_STATE_HEADER_2)
    {
        if (_stream->available())
        {
            if (_stream->read() == HEADER_BYTE_2)
            {
                current_stage = ReceiveState::RX_STATE_SIZE;
            }
            else
            {
                // Header is corrupted
                current_stage = ReceiveState::RX_STATE_HEADER_1;
                return false;  // Return immediately for performance reasons
            }
        }
        else
        {
            return false;  // Return immediately for performance reasons
        }
    }

    // Next check for header byte #3
    if (current_stage == ReceiveState::RX_STATE_SIZE)
    {
        if (_stream->available())
        {
            if (_stream->read() == size)
            {
                current_stage = ReceiveState::RX_STATE_PACKET;
            }
            else
            {
                // Header is corrupted
                current_stage = ReceiveState::RX_STATE_HEADER_1;
                return false;  // Return immediately for performance reasons
            }
        }
        else
        {
            return false;  // Return immediately for performance reasons
        }
    }

    // Finally read all data bytes
    if (current_stage == ReceiveState::RX_STATE_PACKET)
    {
        // This blocks until the buffer is filled or the stream has been emptied.
        // Note that one more byte than "size" is read in. The last byte is the CRC.
        while (_stream->available() && rx_array_inx <= size)
        {
            rx_buffer[rx_array_inx] = _stream->read();
            rx_array_inx++;
        }

        // Need to check if the loop exits due to running out of data and exit now
        if (rx_array_inx - 1 != size)
        {
            return false;  // Return immediately for performance reasons
        }

        // Seem to have got whole message. Calculate the CRC on the whole data packet minus the CRC byte.
        uint8_t crc = CRC8.smbus(rx_buffer, size);

        // Check if calculated CRC matches received value
        boolean result = false;
        if (crc == rx_buffer[size])
        {
            // CRC good
            memcpy(address, rx_buffer, size);
            result = true;
        }

        // We're done with the packet regardless of its validity so reset instance variables
        rx_array_inx = 0;
        looking_for_new_packet = true;
        current_stage = ReceiveState::RX_STATE_HEADER_1;

        if (EASY_TRANSFER_PRINT_DEBUG)
        {
            for (int i = 0; i < size; i++)
            {
                Serial.printf("%02x ", rx_buffer[i]);
            }
            Serial.printf("| CRC-Rx %02x | CRC-cal %02x | Valid %d", rx_buffer[size], crc, result);
            Serial.println();
        }

        return result;
    }

    return false;
}
