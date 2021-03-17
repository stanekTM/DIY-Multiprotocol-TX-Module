/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Compatible with EAchine H8 mini, H10, BayangToys X6/X7/X9, JJRC JJ850 ...
// Last sync with hexfet new_protocols/bayang_nrf24l01.c dated 2015-12-22

#if defined(BAYANG_NRF24L01_INO)

#include "iface_xn297.h"

#define BAYANG_BIND_COUNT		1000
#define BAYANG_PACKET_PERIOD	2000
#define BAYANG_PACKET_TELEM_PERIOD	5000
#define BAYANG_INITIAL_WAIT		500
#define BAYANG_PACKET_SIZE		15
#define BAYANG_RF_NUM_CHANNELS	4
#define BAYANG_RF_BIND_CHANNEL	0
#define BAYANG_RF_BIND_CHANNEL_X16_AH 10
#define BAYANG_ADDRESS_LENGTH	5

enum BAYANG_FLAGS {
	// flags going to packet[2]
	BAYANG_FLAG_RTH			= 0x01,
	BAYANG_FLAG_HEADLESS	= 0x02, 
	BAYANG_FLAG_FLIP		= 0x08,
	BAYANG_FLAG_VIDEO		= 0x10, 
	BAYANG_FLAG_PICTURE		= 0x20, 
	// flags going to packet[3]
	BAYANG_FLAG_INVERTED	= 0x80,			// inverted flight on Floureon H101
	BAYANG_FLAG_TAKE_OFF	= 0x20,			// take off / landing on X16 AH
	BAYANG_FLAG_EMG_STOP	= 0x04|0x08,	// 0x08 for VISUO XS809H-W-HD-G
};

enum BAYANG_OPTION_FLAGS {
	BAYANG_OPTION_FLAG_TELEMETRY	= 0x01,
	BAYANG_OPTION_FLAG_ANALOGAUX	= 0x02,
};

static void __attribute__((unused)) BAYANG_send_packet()
{
	uint8_t i;
	if (IS_BIND_IN_PROGRESS)
	{
	#ifdef BAYANG_HUB_TELEMETRY
		if(option & BAYANG_OPTION_FLAG_TELEMETRY)
			if(option & BAYANG_OPTION_FLAG_ANALOGAUX)
				packet[0]= 0xA1;	// telemetry and analog aux are enabled
			else
				packet[0]= 0xA3;	// telemetry is enabled
		else if(option & BAYANG_OPTION_FLAG_ANALOGAUX)
				packet[0]= 0xA2;	// analog aux is enabled
			else
	#else
		if(option & BAYANG_OPTION_FLAG_ANALOGAUX)
			packet[0]= 0xA2;		// analog aux is enabled
		else
	#endif
			packet[0]= 0xA4;
		if(sub_protocol==QX100)
			packet[0] = 0x53;

		for(i=0;i<5;i++)
			packet[i+1]=rx_tx_addr[i];
		for(i=0;i<4;i++)
			packet[i+6]=hopping_frequency[i];
		switch (sub_protocol)
		{
			case QX100:
			case X16_AH:
				packet[10] = 0x00;
				packet[11] = 0x00;
				break;
			case IRDRONE:
				packet[10] = 0x30;
				packet[11] = 0x01;
				break;
			case DHD_D4:
				packet[10] = 0xC8;
				packet[11] = 0x99;
				break;
			default:
				packet[10] = rx_tx_addr[0];	// txid[0]
				packet[11] = rx_tx_addr[1];	// txid[1]
				break;
		}
	}
	else
	{
		XN297_Hopping(hopping_frequency_no++);
		hopping_frequency_no%=BAYANG_RF_NUM_CHANNELS;
		uint16_t val;
		uint8_t dyntrim = 1;
		switch (sub_protocol)
		{
			case X16_AH:
			case IRDRONE:
				packet[0] = 0xA6;
				break;
			default:
				packet[0] = 0xA5;
				break;
		}
		if (option & BAYANG_OPTION_FLAG_ANALOGAUX)
		{
			// Analog aux channel 1 (channel 14)
			packet[1] = convert_channel_8b(CH14);
		}
		else
			packet[1] = 0xFA;		// normal mode is 0xF7, expert 0xFa , D4 normal is 0xF4

		//Flags packet[2]
		packet[2] = 0x00;
		if(CH5_SW)
			packet[2] = BAYANG_FLAG_FLIP;
		if(CH6_SW)
			packet[2] |= BAYANG_FLAG_RTH;
		if(CH7_SW)
			packet[2] |= BAYANG_FLAG_PICTURE;
		if(CH8_SW)
			packet[2] |= BAYANG_FLAG_VIDEO;
		if(CH9_SW)
		{
			packet[2] |= BAYANG_FLAG_HEADLESS;
			dyntrim = 0;
		}
		//Flags packet[3]
		packet[3] = 0x00;
		if(CH10_SW)
			packet[3] = BAYANG_FLAG_INVERTED;
		if(CH11_SW)
			dyntrim = 0;
		if(CH12_SW)
		  packet[3] |= BAYANG_FLAG_TAKE_OFF;
		if(CH13_SW)
			packet[3] |= BAYANG_FLAG_EMG_STOP;
		//Aileron
		val = convert_channel_10b(AILERON, false);
		packet[4] = (val>>8) + (dyntrim ? ((val>>2) & 0xFC) : 0x7C);
		packet[5] = val & 0xFF;
		//Elevator
		val = convert_channel_10b(ELEVATOR, false);
		packet[6] = (val>>8) + (dyntrim ? ((val>>2) & 0xFC) : 0x7C);
		packet[7] = val & 0xFF;
		//Throttle
		val = convert_channel_10b(THROTTLE, false);
		packet[8] = (val>>8) + 0x7C;
		packet[9] = val & 0xFF;
		//Rudder
		val = convert_channel_10b(RUDDER, false);
		packet[10] = (val>>8) + (dyntrim ? ((val>>2) & 0xFC) : 0x7C);
		packet[11] = val & 0xFF;
	}
	switch (sub_protocol)
	{
		case H8S3D:
			packet[12] = rx_tx_addr[2];	// txid[2]
			packet[13] = 0x34;
			break;
		case QX100:
		case X16_AH:
			packet[12] = 0;
			packet[13] = 0;
			break;
		case IRDRONE:
			packet[12] = 0xE0;
			packet[13] = 0x2E;
			break;
		case DHD_D4:
			packet[12] = 0x37;	//0x17 during bind
			packet[13] = 0xED;
			break;
		default:
			packet[12] = rx_tx_addr[2];	// txid[2]
			if (option & BAYANG_OPTION_FLAG_ANALOGAUX)
			{	// Analog aux channel 2 (channel 15)
				packet[13] = convert_channel_8b(CH15);
			}
			else
				packet[13] = 0x0A;
			break;
	}
	packet[14] = 0;
	for (uint8_t i=0; i < BAYANG_PACKET_SIZE-1; i++)
		packet[14] += packet[i];

	// Send
	XN297_SetPower();
	XN297_SetTxRxMode(TX_EN);
	XN297_WritePayload(packet, BAYANG_PACKET_SIZE);
}

#ifdef BAYANG_HUB_TELEMETRY
static void __attribute__((unused)) BAYANG_check_rx(void)
{
	if( XN297_IsRX() )
	{ // data received from model
		XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);	// Strange can't test the CRC since it seems to be disabled on telemetry packets...
		uint8_t check = packet[0];
		for (uint8_t i=1; i < BAYANG_PACKET_SIZE-1; i++)
			check += packet[i];
		// decode data , check sum is ok as well, since there is no crc
		if (packet[0] == 0x85 && packet[14] == check && telemetry_link == 0)
		{
			// uncompensated battery volts*100/2
			v_lipo1 = (packet[3]<<7) + (packet[4]>>1);
			// compensated battery volts*100/2
			v_lipo2 = (packet[5]<<7) + (packet[6]>>1);
			// reception in packets / sec
			RX_LQI = packet[7];
			RX_RSSI = RX_LQI;
			//Flags
			//uint8_t flags = packet[3] >> 3;
			// battery low: flags & 1
			telemetry_link=1;
			#if defined HUB_TELEMETRY
				// Multiplexed P, I, D values in packet[8] and packet[9].
    	        // The two most significant bits specify which term is sent.
        	    // Remaining 14 bits represent the value: 0 .. 16383
				frsky_send_user_frame(0x24+(packet[8]>>6), packet[9], packet[8] & 0x3F );	//0x24 = ACCEL_X_ID, so ACCEL_X_ID=P, ACCEL_Y_ID=I, ACCEL_Z_ID=D
			#endif
			telemetry_counter++;
			if(telemetry_lost)
				telemetry_link=0;	// Don't send anything yet
		}
	}
	XN297_SetTxRxMode(TXRX_OFF);
}
#endif

static void __attribute__((unused)) BAYANG_RF_init()
{
	XN297_Configure(XN297_CRCEN, XN297_SCRAMBLED, XN297_1M);
	XN297_SetTXAddr((uint8_t *)"\x00\x00\x00\x00\x00", BAYANG_ADDRESS_LENGTH);
	//XN297_HoppingCalib(BAYANG_RF_NUM_CHANNELS);
	
	//Set bind channel
	uint8_t ch = BAYANG_RF_BIND_CHANNEL;
	if(sub_protocol == X16_AH || sub_protocol == IRDRONE)
		ch = BAYANG_RF_BIND_CHANNEL_X16_AH;
	XN297_RFChannel(ch);
}

enum {
	BAYANG_BIND=0,
	BAYANG_WRITE,
	BAYANG_CHECK,
	BAYANG_READ,
};

#define BAYANG_CHECK_DELAY		1000		// Time after write phase to check write complete
#define BAYANG_READ_DELAY		600			// Time before read phase

uint16_t BAYANG_callback()
{
	#ifdef BAYANG_HUB_TELEMETRY
		uint16_t start;
	#endif
	switch(phase)
	{
		case BAYANG_BIND:
			if (--bind_counter == 0)
			{
				XN297_SetTXAddr(rx_tx_addr, BAYANG_ADDRESS_LENGTH);
				#ifdef BAYANG_HUB_TELEMETRY
					XN297_SetRXAddr(rx_tx_addr, BAYANG_PACKET_SIZE);
				#endif
				BIND_DONE;
				phase++;	//WRITE
			}
			else
				BAYANG_send_packet();
			break;
		case BAYANG_WRITE:
			#ifdef MULTI_SYNC
				telemetry_set_input_sync((option & BAYANG_OPTION_FLAG_TELEMETRY)?BAYANG_PACKET_TELEM_PERIOD:BAYANG_PACKET_PERIOD);
			#endif
			BAYANG_send_packet();
			#ifdef BAYANG_HUB_TELEMETRY
				if (option & BAYANG_OPTION_FLAG_TELEMETRY)
				{	// telemetry is enabled
					state++;
					if (state > 200)
					{
						state = 0;
						//telemetry reception packet rate - packets per second
						TX_LQI = telemetry_counter>>1;
						telemetry_counter = 0;
						telemetry_lost=0;
					}
					phase++;	//CHECK
					return BAYANG_CHECK_DELAY;
				}
			#endif
			break;
	#ifdef BAYANG_HUB_TELEMETRY
		case BAYANG_CHECK:
			// switch radio to rx as soon as packet is sent
			start=(uint16_t)micros();
			while ((uint16_t)((uint16_t)micros()-(uint16_t)start) < 1000)			// Wait max 1ms
				if(XN297_IsPacketSent())
					break;
			XN297_SetTxRxMode(RX_EN);
			phase++;	// READ
			return BAYANG_PACKET_TELEM_PERIOD - BAYANG_CHECK_DELAY - BAYANG_READ_DELAY;
		case BAYANG_READ:
			BAYANG_check_rx();
			phase=BAYANG_WRITE;
			return BAYANG_READ_DELAY;
	#endif
	}
	return BAYANG_PACKET_PERIOD;
}

static void __attribute__((unused)) BAYANG_initialize_txid()
{
	//Could be using txid[0..2] but using rx_tx_addr everywhere instead...
	if(sub_protocol==DHD_D4)
		hopping_frequency[0]=(rx_tx_addr[2]&0x07)|0x01;
	else
		hopping_frequency[0]=0;
	hopping_frequency[1]=(rx_tx_addr[3]&0x1F)+0x10;
	hopping_frequency[2]=hopping_frequency[1]+0x20;
	hopping_frequency[3]=hopping_frequency[2]+0x20;
	hopping_frequency_no=0;
}

void BAYANG_init(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
	phase=BAYANG_BIND;
    bind_counter = BAYANG_BIND_COUNT;
	BAYANG_initialize_txid();
	BAYANG_RF_init();
	packet_count=0;
}

#endif
