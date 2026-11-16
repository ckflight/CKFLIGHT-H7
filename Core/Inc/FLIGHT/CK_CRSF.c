#include "COMMON/maths.h"
#include "COMMON/crc.h"

#include "DRIVERS/CK_UART.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_CIRCULARBUFFER.h"

#include "FLIGHT/CK_CRSF.h"

#include "COMMUNICATION/CK_PRINTER.h"

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

#define CRSF_MAX_FRAME					64U
#define CRSF_MAX_CHANNEL				16U
#define CRSF_HEADER_LEN					2
#define CRSF_RC_CHANNEL_SCALE_LEGACY	0.62477120195241f

#define CRSF_CHECK_INTERVAL				1500

#define CRSF_TIME_NEEDED_PER_FRAME_US   1750 // a maximally sized 64byte payload will take ~1550us, round up to 1750.

typedef enum{

    CRSF_FRAMETYPE_GPS 					= 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR 		= 0x08,
    CRSF_FRAMETYPE_HEARTBEAT 			= 0x0B,
    CRSF_FRAMETYPE_VTX 					= 0x0F,
    CRSF_FRAMETYPE_VTX_TELEM 			= 0x10,
    CRSF_FRAMETYPE_LINK_STATISTICS 		= 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED 	= 0x16,
    CRSF_FRAMETYPE_ATTITUDE 			= 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE 			= 0x21,

    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_PARAM_DEVICE_PING 		= 0x28,
    CRSF_FRAMETYPE_PARAM_DEVICE_INFO 		= 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ 			= 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE 			= 0x2D,
    CRSF_FRAMETYPE_COMMAND 					= 0x32,

	// Custom Telemetry Frames 0x7F,0x80
    CRSF_FRAMETYPE_AP_CUSTOM_TELEM_LEGACY 	= 0x7F,   // as suggested by Remo Masina for fw < 4.06
    CRSF_FRAMETYPE_AP_CUSTOM_TELEM 			= 0x80,   // reserved for ArduPilot by TBS, requires fw >= 4.06

}FrameType;

typedef enum {
    CRSF_ADDRESS_BROADCAST 			= 0x00,
    CRSF_ADDRESS_USB 				= 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO 	= 0x80,
    CRSF_ADDRESS_RESERVED1 			= 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR 	= 0xC0,
    CRSF_ADDRESS_GPS 				= 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX 		= 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER 	= 0xC8,
    CRSF_ADDRESS_RESERVED2 			= 0xCA,
    CRSF_ADDRESS_RACE_TAG 			= 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER 	= 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER 		= 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER 	= 0xEE
} crsfAddress_e;

typedef struct{

	uint8_t device_address;

	uint8_t length;

	FrameType type;

	uint8_t payload[CRSF_MAX_FRAME - 3]; // +1 CRC

	uint8_t crc;

}CRSF_FRAME;

struct crsfPayloadRcChannelsPacked_s {

    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    uint32_t chan0 : 11;
    uint32_t chan1 : 11;
    uint32_t chan2 : 11;
    uint32_t chan3 : 11;
    uint32_t chan4 : 11;
    uint32_t chan5 : 11;
    uint32_t chan6 : 11;
    uint32_t chan7 : 11;
    uint32_t chan8 : 11;
    uint32_t chan9 : 11;
    uint32_t chan10 : 11;
    uint32_t chan11 : 11;
    uint32_t chan12 : 11;
    uint32_t chan13 : 11;
    uint32_t chan14 : 11;
    uint32_t chan15 : 11;
}__attribute__ ((__packed__));

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */

struct crsfLinkStatisticsFrame {
    uint8_t uplink_rssi_ant1; 		// ( dBm * -1 )
    uint8_t uplink_rssi_ant2; 		// ( dBm * -1 )
    uint8_t uplink_status; 			// Package success rate / Link quality ( % )
    int8_t uplink_snr; 				// ( db )
    uint8_t active_antenna; 		// Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
    uint8_t rf_mode; 				// ( enum 4fps = 0 , 50fps, 150hz)
    uint8_t uplink_tx_power; 		// ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
    uint8_t downlink_rssi; 			// ( dBm * -1 )
    uint8_t downlink_status; 		// Downlink package success rate / Link quality ( % )
    int8_t downlink_dnr; 			// ( db )
}__attribute__ ((__packed__));

typedef struct{
    int16_t rssi_dbm;
    CRSF_RFMode rf_mode;
    uint8_t link_quality;

}LinkStatus;

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;
typedef struct crsfLinkStatisticsFrame crsfPayloadLinkStatisticsFrame_t;
LinkStatus crsfLinkStatus;

circularBuffer_t crsf_cb;

CRSF_Method crsf_method;

// volatile: written by the UART interrupt (CK_CRSF_NewByte2), read and cleared by the main loop
volatile bool crsf_dataReceived = false;

uint8_t crsf_frame_ofs = 0;

uint16_t _channels_crsf[16];

uint32_t crsfChannelData[CRSF_MAX_CHANNEL];

USART_TypeDef* CK_CRSF_UART;

CRSF_FRAME crsf_frame;          // ISR working buffer: only the UART interrupt writes it, byte by byte

/*
 * DOUBLE BUFFER (same idea as Betaflight crsf.c)
 *
 *  crsf_frame        : working buffer filled byte by byte by the UART interrupt.
 *  crsf_frame_ready  : when the last byte (CRC) arrives the ISR copies the finished frame here.
 *  crsf_frame_main   : the main loop takes crsf_frame_ready here; CRC check and decode run on this copy.
 *
 * Why: at 420k baud a frame lasts 619 us, at 1 kHz the gap between frames is ~381 us. The main loop
 * used to run the CRC directly on crsf_frame; if the next frame had already started, the payload was
 * overwritten -> CRC error -> frame dropped. Loop and receiver clocks are almost synchronous, so the
 * phase could stay in this bad window for a long time and no valid frame arrived for seconds.
 * Now the main loop has a whole frame time (~1 ms) to take the frame.
 *
 * crsf_ready_seq (seqlock): the ISR increments it before and after the copy. The main loop reads it
 * before and after its own copy and copies again if it changed. No lock, interrupts stay enabled,
 * the ISR never waits.
 */
static CRSF_FRAME crsf_frame_ready;
static CRSF_FRAME crsf_frame_main;
static volatile uint32_t crsf_ready_seq = 0;

// Compiler barrier: keeps memory accesses on their side of this point.
// Enough between an ISR and the main loop on a single core.
#ifndef CRSF_COMPILER_BARRIER
#define CRSF_COMPILER_BARRIER()     __asm volatile ("" ::: "memory")
#endif

// Diagnostic counters (count only, never change behaviour). Read with CK_CRSF_GetStats().
static volatile uint32_t crsf_stat_frames_ok = 0;
static volatile uint32_t crsf_stat_crc_errors = 0;
static volatile uint32_t crsf_stat_uart_overruns = 0;

// ISR context: publish the finished frame
static inline void CK_CRSF_PublishFrame(void){

	crsf_ready_seq++;                   // odd: being written
	CRSF_COMPILER_BARRIER();
	crsf_frame_ready = crsf_frame;
	CRSF_COMPILER_BARRIER();
	crsf_ready_seq++;                   // even: ready

	crsf_dataReceived = true;
}

// Main loop context: take the last finished frame into crsf_frame_main. False if no consistent copy.
static bool CK_CRSF_TakeFrame(void){

	uint32_t seq_before;
	uint32_t seq_after;
	uint8_t tries = 0;

	do{
		seq_before = crsf_ready_seq;
		CRSF_COMPILER_BARRIER();
		crsf_frame_main = crsf_frame_ready;
		CRSF_COMPILER_BARRIER();
		seq_after = crsf_ready_seq;
	}while((seq_before != seq_after || (seq_before & 1u)) && ++tries < 4u);

	return (seq_before == seq_after) && ((seq_before & 1u) == 0u);
}

uint8_t crsf_payload_counter = 0;
uint8_t crsf_state = 0;

uint8_t crsf_dma_buffer[32] __attribute__((aligned(32)));   // one cache line (only used with USE_DMA_CRSF)

void CK_CRSF_Init(CRSF_Method method){

	crsf_method = method;

	int crsf_baud = 420000;

	USART_CONFIGURATION_ config;
	config.interrupt 			= RX_INTERRUPT;
	config.mode 				= RX_ONLY;
	config.parity 				= NO_PARITY;
	config.stop_bit 			= STOP_BIT1;
	config.baudrate 			= crsf_baud;
	#if CRSF_
	config.rx_gpio_type			= CRSF_UART_RX_GPIO;
	config.rx_gpio_pin			= CRSF_UART_RX_PIN;
	config.rx_af				= CRSF_UART_RX_AF;
	config.usart 				= CRSF_UART;
	CK_CRSF_UART 				= CRSF_UART;

	config.preempt_priority		= RECEIVER_PreemptPriority;
	config.sub_priority			= RECEIVER_SubPriority;
	#endif
	config.use_circular_buffer 	= true;

	CK_UART_Init(&config, &crsf_cb);

	#if USE_INTERRUPT_CRSF
	CK_UART_RXInterruptEnable(CK_CRSF_UART);
	CK_UART_RXEnable(CK_CRSF_UART);
	#endif

	#if USE_DMA_CRSF
	CK_UART_DMA_EnableClock(CRSF_DMA);

	CK_UART_DMA_ClearFlag(CRSF_DMA, CRSF_DMA_Stream);

	CK_UART_DMA_InitRX(CRSF_DMA_Stream, CRSF_DMA_Channel);

	CK_UART_DMA_TCInterruptEnable(CRSF_DMA_Stream);

	HAL_NVIC_EnableIRQ(CRSF_DMA_IRQn);

	CK_UART_DMA_SetPeripheralAddress(CRSF_DMA_Stream, (uint32_t)(&CK_CRSF_UART->DR));

	CK_CRSF_StartDMA();
	#endif

}

#if USE_DMA_CRSF
void CK_CRSF_StartDMA(){

	// RX
	CK_UART_DMA_ClearFlag(CRSF_DMA, CRSF_DMA_Stream);

	CK_UART_DMA_SetBuffer(CRSF_DMA_Stream, crsf_dma_buffer, 26);

	CK_UART_RXEnable(CK_CRSF_UART);

	CK_UART_DMA_Enable(CRSF_DMA_Stream);

	CK_UART_DMA_RXEnable(CK_CRSF_UART);
}
#endif

bool CK_CRSF_Update(uint32_t current_time){

	bool isNewData = false;

    if(crsf_method == CRSF_INTERRUPT){

    	if(crsf_dataReceived){

    		// Clear the flag first: if a new frame completes while this one is processed the flag
    		// rises again and it is handled in the next loop (it used to be cleared after decode and lost).
    		crsf_dataReceived = false;

    		if(CK_CRSF_TakeFrame()){

	    		uint8_t crc = crc8_dvb_s2(0, crsf_frame_main.type);

				for (uint8_t i = 0; i < crsf_frame_main.length - 2; i++){
					crc = crc8_dvb_s2(crc, crsf_frame_main.payload[i]);
				}

				if(crc == crsf_frame_main.crc){

					CK_CRSF_Decode();

					isNewData = true;

					crsf_stat_frames_ok++;

					#if SCOPE_CHECK_CRSF_PACKET == 1
					CK_GPIO_TogglePin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
					#endif

				}
				else{
					crsf_stat_crc_errors++;
				}
    		}

        }

    }

    else if(crsf_method == CRSF_POLLING){

    	static uint32_t previousTime = 0;

        if(current_time >= previousTime + CRSF_CHECK_INTERVAL){

            previousTime = current_time;

            if(crsf_dataReceived){

            	crsf_dataReceived = false;

            	if(CK_CRSF_TakeFrame()){
            		CK_CRSF_Decode();
            		isNewData = true;
            	}
            }

        }

    }

    static uint32_t pre_time = 0;
    if(current_time - pre_time >= 35000){

    	pre_time = current_time;
    	//CK_CRSF_SendTelemetry();
    }

    return isNewData;

}

void CK_CRSF_SendTelemetry(void){

    // calculate crc
    uint8_t crc = crc8_dvb_s2(0, crsf_frame.type);

    for (uint8_t i = 0; i < crsf_frame.type - 2; i++) {
        crc = crc8_dvb_s2(crc, crsf_frame.payload[i]);
    }

    crsf_frame.payload[crsf_frame.length - 2] = crc;

    //CK_UART_SendInterrupt(CK_CRSF_UART, 0x00);

    for(int i = 0; i < crsf_frame.type + 2; i++){

    	uint8_t current_data = ((uint8_t*)&crsf_frame)[i];
    	CK_UART_SendPolling(CK_CRSF_UART, current_data);
    }

}

void CK_CRSF_AddByte(uint8_t index, uint8_t byte) {
	((uint8_t*)&crsf_frame)[index] = byte;
}

void CK_CRSF_NewByte(uint8_t data){

	// Overflow check
	if(crsf_frame_ofs >= CRSF_MAX_FRAME){
		crsf_frame_ofs = 0;
	}

	CK_CRSF_AddByte(crsf_frame_ofs++, data);

	// need a header to get the length
	if(crsf_frame_ofs < CRSF_HEADER_LEN){
		return;
	}

	if(crsf_frame_ofs == CRSF_HEADER_LEN){

		// check for wrong frame
		if(crsf_frame.length > CRSF_MAX_FRAME){
			crsf_frame_ofs = 0;
		}
		return;
	}

	// Overflow check
	if(crsf_frame_ofs > crsf_frame.length + CRSF_HEADER_LEN){
		crsf_frame_ofs = 0;
		return;
	}

	// Decode since now frame is completed
	if(crsf_frame_ofs == crsf_frame.length + CRSF_HEADER_LEN){

		crsf_frame_ofs = 0;

		uint8_t crc = crc8_dvb_s2(0, crsf_frame.type);

		for (uint8_t i = 0; i < crsf_frame.length - 2; i++) {
			crc = crc8_dvb_s2(crc, crsf_frame.payload[i]);
		}

		// incorrect crc
		if(crc != crsf_frame.payload[crsf_frame.length - CRSF_HEADER_LEN]){
			return;
		}

		CK_CRSF_PublishFrame();

	}

}

void CK_CRSF_NewByte2(uint8_t data){

	// Crossfire frame <Device address><Frame length><Type><Payload><CRC>
	// 0xC8(CRSF_ADDRESS_FLIGHT_CONTROLLER) + 0x18(24) + 0x16(CRSF_FRAMETYPE_RC_CHANNELS_PACKED) + 22 Byte + 1 CRC
	static uint8_t crc = 0;

    switch(crsf_state){

    case 0:

    	// Flight controlle only needs to check data with flight controller header
    	// Since only the data with this header is meant to be received by flight controller,

        if(data == CRSF_ADDRESS_FLIGHT_CONTROLLER){

        	crsf_frame.device_address = data;
            crsf_state++;
        }

        break;

    case 1:

    	// Second data is frame length and it should be bigger than 2
        if(data > 2 && data < CRSF_MAX_FRAME){

        	crsf_frame.length = data;
        	crsf_state++;
        }
        else{
			crsf_state = 0;
		}
        break;

    case 2:

    	// Frame Type data
    	if(data == CRSF_FRAMETYPE_RC_CHANNELS_PACKED || data == CRSF_FRAMETYPE_LINK_STATISTICS){
    		crsf_frame.type = data;
			crsf_state++;
		}
    	else{
    		crsf_state = 0;
    	}

    	break;

    case 3:

    	// Receive payload + 1 crc bytes
    	if(crsf_payload_counter < crsf_frame.length - 2){
    		crsf_frame.payload[crsf_payload_counter++] = data;
    	}
    	else if(crsf_payload_counter == crsf_frame.length - 2){

    		crc = data;

    		crsf_frame.crc = crc;

    		CK_CRSF_PublishFrame();     // hand the finished frame to the main loop (double buffer)

    		crsf_state = 0;

    		crsf_payload_counter = 0;

    		/*
    		crc = crc8_dvb_s2(0, crsf_frame.type);

    		for (uint8_t i = 0; i < crsf_frame.length - 2; i++){
    			crc = crc8_dvb_s2(crc, crsf_frame.payload[i]);
    		}

    		if(crc == crsf_frame.crc){

    			crsf_state = 0;

        		crsf_payload_counter = 0;

        		crsf_dataReceived = true;

    		}
    		else{

    			crsf_state = 0;

        		crsf_payload_counter = 0;

        		crsf_dataReceived = false;

    		}
    		*/

    	}

    	break;

    default:
    	break;

    }

}

void CK_CRSF_Decode(void){

    switch(crsf_frame_main.type){

        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:

            CK_CRSF_Decode_11bit_Channels();

            break;

        case CRSF_FRAMETYPE_LINK_STATISTICS:

        	CK_CRSF_Decode_LinkStatusFrame();

        	break;

        case CRSF_FRAMETYPE_GPS:
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
        case CRSF_FRAMETYPE_HEARTBEAT:
        case CRSF_FRAMETYPE_VTX:
        case CRSF_FRAMETYPE_VTX_TELEM:
        case CRSF_FRAMETYPE_ATTITUDE:
        case CRSF_FRAMETYPE_FLIGHT_MODE:

        	break;

        default:

            break;
    }
}

void CK_CRSF_Decode_11bit_Channels(void){

	// use ordinary RC frame structure (0x16)
	// scale factors defined by TBS - TICKS_TO_US(x) ((x) * 5 / 8 + 880)

	const crsfPayloadRcChannelsPacked_t* const rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsf_frame_main.payload;

	crsfChannelData[0] = (rcChannels->chan0 * 5) / 8 + 880;
	crsfChannelData[1] = (rcChannels->chan1 * 5) / 8 + 880;
	crsfChannelData[2] = (rcChannels->chan2 * 5) / 8 + 880;
	crsfChannelData[3] = (rcChannels->chan3 * 5) / 8 + 880;
	crsfChannelData[4] = (rcChannels->chan4 * 5) / 8 + 880;
	crsfChannelData[5] = (rcChannels->chan5 * 5) / 8 + 880;
	crsfChannelData[6] = (rcChannels->chan6 * 5) / 8 + 880;
	crsfChannelData[7] = (rcChannels->chan7 * 5) / 8 + 880;
	crsfChannelData[8] = (rcChannels->chan8 * 5) / 8 + 880;
	crsfChannelData[9] = (rcChannels->chan9 * 5) / 8 + 880;
	crsfChannelData[10] = (rcChannels->chan10 * 5) / 8 + 880;
	crsfChannelData[11] = (rcChannels->chan11 * 5) / 8 + 880;
	crsfChannelData[12] = (rcChannels->chan12 * 5) / 8 + 880;
	crsfChannelData[13] = (rcChannels->chan13 * 5) / 8 + 880;
	crsfChannelData[14] = (rcChannels->chan14 * 5) / 8 + 880;
	crsfChannelData[15] = (rcChannels->chan15 * 5) / 8 + 880;

}

static uint32_t crsfLinkStatsMs = 0;      // time of the last valid (CRC checked) link statistics frame
static uint32_t crsfDegradedSinceMs = 0;

/*
 * Real link loss, debounced: fresh link statistics show LQ <= CRSF_LQ_LOST for at least
 * CRSF_LINK_DEGRADED_MS. A single low RSSI reading or one bad statistics frame never triggers it.
 * (Frames that stop completely are caught by the receiver timeout in CK_RECEIVER.)
 */
bool CK_CRSF_LinkDegraded(uint32_t now_ms){

	const bool fresh = crsfLinkStatsMs != 0 && (now_ms - crsfLinkStatsMs) < CRSF_LINK_STATS_FRESH_MS;
	const bool bad = fresh && crsfLinkStatus.link_quality <= CRSF_LQ_LOST;

	if(!bad){
		crsfDegradedSinceMs = 0;
		return false;
	}
	if(crsfDegradedSinceMs == 0){
		crsfDegradedSinceMs = now_ms ? now_ms : 1u;
		return false;
	}
	return (now_ms - crsfDegradedSinceMs) >= CRSF_LINK_DEGRADED_MS;
}

void CK_CRSF_Decode_LinkStatusFrame(void){

	const crsfPayloadLinkStatisticsFrame_t* link = (const crsfPayloadLinkStatisticsFrame_t*)&crsf_frame_main.payload;
	uint8_t rssi_dbm;

	if (link->active_antenna == 0) {
		rssi_dbm = link->uplink_rssi_ant1;
	}
	else{
		rssi_dbm = link->uplink_rssi_ant2;
	}

	crsfLinkStatus.rssi_dbm = -1 * rssi_dbm;
	crsfLinkStatus.rf_mode = MIN(link->rf_mode, 3U);
	crsfLinkStatus.link_quality = link->uplink_status;
	crsfLinkStatsMs = CK_TIME_GetMilliSec();
}

int CK_CRSF_GetChannelRaw(int channel){

	if(channel < 1 || channel > 16){
		return 0;
	}
	else{
		return crsfChannelData[channel - 1];
	}
}

int16_t CK_CRSF_GetRSSI_dBm(void){
	return crsfLinkStatus.rssi_dbm;
}

CRSF_RFMode CK_CRSF_GetRFMode(void){
	return crsfLinkStatus.rf_mode;
}

uint8_t CK_CRSF_GetLinkQuality(void){
	return crsfLinkStatus.link_quality;
}

void CK_CRSF_GetStats(uint32_t* frames_ok, uint32_t* crc_errors, uint32_t* uart_overruns){
	*frames_ok     = crsf_stat_frames_ok;
	*crc_errors    = crsf_stat_crc_errors;
	*uart_overruns = crsf_stat_uart_overruns;
}

#if CRSF_

#if USE_INTERRUPT_CRSF

#if CRSF_INTERRUPT_ == 1
void USART1_IRQHandler(void){
#endif

#if CRSF_INTERRUPT_ == 2
void USART2_IRQHandler(void){
#endif

#if CRSF_INTERRUPT_ == 3
void USART3_IRQHandler(void){
#endif

#if CRSF_INTERRUPT_ == 4
void UART4_IRQHandler(void){
#endif

#if CRSF_INTERRUPT_ == 5
void UART5_IRQHandler(void){
#endif

#if CRSF_INTERRUPT_ == 6
void USART6_IRQHandler(void){
#endif

	#if USE_H7 == 1
	const uint32_t crsf_isr = CK_CRSF_UART->ISR;
	if(crsf_isr & USART_ISR_ORE){
		crsf_stat_uart_overruns++;      // a byte was lost (RDR not read in time)
	}
	if(crsf_isr & CK_USART_SR_RXNE){
	#endif

	#if USE_F4 == 1
	const uint32_t crsf_sr = CK_CRSF_UART->SR;
	if(crsf_sr & USART_SR_ORE){
		crsf_stat_uart_overruns++;      // a byte was lost; reading SR then DR clears ORE
	}
	if(crsf_sr & CK_USART_SR_RXNE){
	#endif

		#if USE_H7 == 1
		CK_UART_ClearFlags(CK_CRSF_UART);

		uint8_t rxData = CK_CRSF_UART->RDR;
		#endif

		#if USE_F4 == 1
		uint8_t rxData = CK_CRSF_UART->DR;
		#endif

		if(crsf_method == CRSF_INTERRUPT){

			//CK_CRSF_NewByte(rxData);

			CK_CRSF_NewByte2(rxData);

		}

		else if(crsf_method == CRSF_POLLING){

			if(!CK_CIRCULARBUFFER_IsBufferFull(&crsf_cb)){
				CK_CIRCULARBUFFER_BufferWrite(&crsf_cb, rxData);
			}
		}
	}

	/*
	 * ORE can stay set without RXNE: if a new byte arrives after the flags were read and just before
	 * the data register is read, ORE=1 and RXNE=0. RXNEIE also raises an interrupt for ORE; the RXNE
	 * check above never clears it, so the interrupt would re-enter forever (interrupt storm -> the
	 * main loop stops). This branch never runs in normal operation, it costs one compare.
	 *   F4: ORE is cleared by "read SR + read DR" (SR was read above, DR here).
	 *   H7: ORE is cleared only by ICR.ORECF.
	 */
	#if USE_F4 == 1
	else if(crsf_sr & USART_SR_ORE){
		(void)CK_CRSF_UART->DR;
	}
	#endif

	#if USE_H7 == 1
	else if(crsf_isr & USART_ISR_ORE){
		CK_CRSF_UART->ICR = USART_ICR_ORECF;
	}
	#endif

}
#endif

#if USE_DMA_CRSF

void CRSF_DMA_Handler(void){

    if(CK_UART_DMA_IsTransferComplete(CRSF_DMA, CRSF_DMA_Stream)){ // Transfer of one sector is done.

        //CK_UART_DisableRXDMA(SPI_ICM20602);

        //CK_UART_DMA_Disable(CRSF_DMA_Stream);

    	CK_UART_DMA_ClearFlag(CRSF_DMA, CRSF_DMA_Stream);

		#if USE_H7
		//CK_UART_Disable(SPI_IIM42652);
		#endif

		#if USE_H7 == 1
    	// Invalidate before rx operation when dcache is enabled
        // DMA is done so data is ready on sram send it to cache so cpu can use it.
    	SCB_InvalidateDCache_by_Addr((uint32_t*)crsf_dma_buffer, sizeof(crsf_dma_buffer));   // its own buffer (was the gyro array)
		#endif


    	uint8_t temp = crsf_dma_buffer[0];
    	CK_CRSF_StartDMA();

    }
}

#endif

#endif























