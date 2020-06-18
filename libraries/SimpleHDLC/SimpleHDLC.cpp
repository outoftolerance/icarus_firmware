#include "SimpleHDLC.h"

#define low(x)    ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

/*------------------------------Constructor Methods------------------------------*/

SimpleHDLC::SimpleHDLC(Stream& input_stream, message_callback_type callback_function):
	data_stream_(input_stream),
	handleMessageCallback_(callback_function)
{
	frame_position_ = 0;
    invert_next_byte_ = false;
}

/*------------------------------Private Methods------------------------------*/

void SimpleHDLC::sendByte_(const uint8_t data, bool special)
{
	//Write byte to output stream but check for reserved characters
	if(!special && ((data == CONTROL_ESCAPE_BYTE) || (data == FRAME_FLAG)))
    {
        data_stream_.write(CONTROL_ESCAPE_BYTE);
        data_stream_.write(data ^ INVERT_BYTE);
    }
    else
    {
    	data_stream_.write(data);
    }
}

void SimpleHDLC::serializeMessage_(const hdlcMessage& message, uint8_t buffer[], uint8_t buffer_length)
{
	for(int i = 0; i < buffer_length; i++)
	{
		if(i == 0)
		{
			buffer[i] = message.node_id;
		}
		else if (i == 1)
		{
			buffer[i] = message.node_type;
		}
		else if(i == 2)
		{
			buffer[i] = message.command;
		}
		else if (i == 3)
		{
			buffer[i] = message.length;
		}
		else
		{
			buffer[i] = message.payload[i - FRAME_HEADER_LENGTH];
		}
	}
}

void SimpleHDLC::deserializeMessage_(hdlcMessage& message, const uint8_t buffer[], uint8_t buffer_length)
{
	for(int i = 0; i < buffer_length; i++)
	{
		if(i == 0)
		{
			message.node_id = buffer[i];
		}
		else if(i == 1)
		{
			message.node_type = buffer[i];
		}
		else if(i == 2)
		{
			message.command = buffer[i];
		}
		else if(i == 3)
		{
			message.length = buffer[i];
		}
		else
		{
			message.payload[i - FRAME_HEADER_LENGTH] = buffer[i];
		}
	}
}

/*------------------------------Public Methods------------------------------*/

void SimpleHDLC::receive()
{
	uint8_t new_byte;
	uint16_t frame_crc16;

	//Loop through all the available bytes in the serial port
	while(data_stream_.available() > 0)
	{
		//Read new byte from data_stream_
		new_byte = data_stream_.read();

		//Check for start of new frame
		if(new_byte == FRAME_FLAG)
		{
			//Start of a new frame! Reset control variables
			invert_next_byte_ = false;
			frame_position_ = 0;

			//Skip adding this to the frame buffer
			continue;
		}
		//Check if the next byte needs to be inverted
		else if (new_byte == CONTROL_ESCAPE_BYTE)
		{
			invert_next_byte_ = true;

			//Skip adding this to the frame buffer
			continue;
		}
		//Check if this byte needs inverting
		else if(invert_next_byte_ == true)
		{
			new_byte ^= INVERT_BYTE;
			invert_next_byte_ = false;
		}

		//Add the new byte to the frame receive buffer
		frame_receive_buffer_[frame_position_] = new_byte;
		frame_position_++;

		//Validate if we have found a complete frame with CRC, need at least FRAME_HEADER_LENGTH bytes for valid frame
		if(frame_position_ >= FRAME_HEADER_LENGTH)
		{
			/**
			 * Here we get away with frame_position - 2 because data is bytes and function takes 
			 * in # of bytes in input data array, usually want to do sizeof(data[])
			 */
			//Calculate the CRC16 for received data
			frame_crc16 = fast_crc16_.kermit(frame_receive_buffer_, frame_position_ - 2);

			//Check if a valid frame is found
			if( (frame_position_ >= 2) && ( frame_crc16 == (uint16_t)((frame_receive_buffer_[frame_position_ - 1] << 8 ) | (frame_receive_buffer_[frame_position_ - 2] & 0xff)) ) )  // (msb << 8 ) | (lsb & 0xff)
			{
				//Decode new frame into message
				hdlcMessage new_message;
				deserializeMessage_(new_message, frame_receive_buffer_, (uint8_t)(frame_position_ - 2));

				//Execute message callback function
				(*handleMessageCallback_)(new_message);
			}
		}

		//Check if we hit the max length of the frame
		if(frame_position_ > MAX_FRAME_LENGTH)
		{
			//Reset to start of frame and start again
			frame_position_ = 0;
			invert_next_byte_ = false;
		}
	}
}

void SimpleHDLC::send(const hdlcMessage& message)
{
    uint16_t frame_crc16;

    //Convert message to serial bytes
    uint8_t buffer[message.length + FRAME_HEADER_LENGTH];
    serializeMessage_(message, buffer, message.length + FRAME_HEADER_LENGTH);

    //Calculate frame CRC16
    frame_crc16 = fast_crc16_.kermit(buffer, sizeof(buffer));

    //Send initial frame flag to open frame
    sendByte_(FRAME_FLAG, true);

    //Loop through the serialized data buffer and send all bytes making sure to convert
    for(int i = 0; i < message.length + FRAME_HEADER_LENGTH; i++)
    {
        sendByte_(buffer[i], false);
    }

    //Send bottom 8 bits of CRC
    sendByte_(low(frame_crc16), false);

    //Send top 8 bits of CRC
    sendByte_(high(frame_crc16), false);

    //Send final frame flag to close frame (don't need this?)
    sendByte_(FRAME_FLAG, true);
}