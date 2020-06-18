#ifndef SIMPLEHDLC_H
#define SIMPLEHDLC_H

#include <Arduino.h>

#include <stdint.h>
#include <FastCRC.h>

#define FRAME_FLAG 0x7e             /**< Message start flag */
#define CONTROL_ESCAPE_BYTE 0x7D    /**< A "control escape octet", has the bit sequence '01111101', (7D hexadecimal) */
#define INVERT_BYTE 0x20            /**<  */
#define CRC16_CCITT_INIT_VAL 0xFFFF /**< Initial value for CRC checksum */
#define MAX_FRAME_LENGTH 64        /**< Maximum length of a frame including start and end flags */
#define FRAME_HEADER_LENGTH 4       /**< Length of header (node id, node type, command, payload length)

/**
 * Defines the structure of the message
 */
typedef struct hdlcMessage
{
    uint8_t node_id;
    uint8_t node_type;
    uint8_t command;                        /**< The type of message */
    uint8_t length;                         /**< Must include the command and length itself */
    uint8_t payload[MAX_FRAME_LENGTH - 8];  /**< The data of the message */
} hdlcMessage;

typedef void (* message_callback_type)(hdlcMessage message);

/**
 * @brief      Simple HDLC message interface class
 */
class SimpleHDLC
{
    public:
        /**
         * @brief      Constructor of SimpleHDLC object
         *
         * @param      input_stream  Pointer to the input stream the object should read from
         */
        SimpleHDLC(Stream& input_stream, message_callback_type);

        /**
         * @brief      Processes the data stream to find messages.
         */
        void receive();

        /**
         * @brief      Function to send a message via the stream, will be wrapped in HDLC frame
         *
         * @param[in]  message  The message to be sent
         */
        void send(const hdlcMessage& message);
    private:
        /**
         * @brief      Sends a single byte through the serial port
         * @param[in]  data  The data bute to be sent
         * @param[in]  special If special character then checks will be skipped
         */
        void sendByte_(const uint8_t data, bool special);

        /**
         * @brief      Serializes an HDLC message as a series of bytes
         *
         * @param[in]   message    The message to serialize
         * @param[out]  buffer     The buffer to output serial data to
         * @param[out]  length     The length of the output buffer
         */
        void serializeMessage_(const hdlcMessage& message, uint8_t buffer[], uint8_t buffer_length);

        /**
         * @brief      Deserializes an HDLC message from a series of bytes
         *
         * @param[out] message  The message object to populate with deserialized data
         * @param[in]  buffer   The buffer to deserialize from
         * @param[in]  length   The length of the input buffer
         */
        void deserializeMessage_(hdlcMessage& message, const uint8_t buffer[], const uint8_t buffer_length);

        Stream& data_stream_;                                   /**< Stream to read data from and publish data to */
        uint8_t frame_receive_buffer_[MAX_FRAME_LENGTH + 1];    /**< Buffer to receive frame data into from stream */
        FastCRC16 fast_crc16_;                                  /**< FastCRC16 object */
        uint8_t frame_position_;                                /**< Position within frame **/
        bool invert_next_byte_;                                 /**< Tracks if byte should be inverted **/
        message_callback_type handleMessageCallback_;           /**< User defined message handler callback function */
};

#endif