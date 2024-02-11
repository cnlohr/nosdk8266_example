
//#include "c_types.h"

#ifdef TESTSTRAP

#include <stdint.h>
#include <stdio.h>
#include "chirpbuff.h"
#include <stdlib.h>

#define uint32 uint32_t

#else

#include "esp8266_auxrom.h"
#include "eagle_soc.h"
#include "nosdk8266.h"
#include "esp8266_rom.h"

// TODO: Use float number (related to 8) to fix the drift
#define call_delay_us(time) { asm volatile("mov.n a2, %0\n_call0 delay4clk" : : "r"(time * (MAIN_MHZ / 8)) : "a2" ); }

#include "ets_sys.h"
#include "pin_mux_register.h"

#endif

#include "slc_register.h"
#include "dmastuff.h"


#include "chirpbuffinfo.h"

#include "LoRa-SDR-Code.h"

#define DMABUFFERDEPTH 3

void testi2s_init();

//These contol the speed at which the bus comms.
#define WS_I2S_BCK 1  //Can't be less than 1.
#define WS_I2S_DIV 1

//I2S DMA buffer descriptors
static struct sdio_queue i2sBufDescTX[DMABUFFERDEPTH] __attribute__((aligned(128)));;

uint32_t chirpbuffUP[CHIRPLENGTH_WORDS_WITH_PADDING];
uint32_t chirpbuffDOWN[CHIRPLENGTH_WORDS_WITH_PADDING];
uint32_t dummy[DMA_SIZE_WORDS];

volatile int fxcycle;
int etx;


#define MAX_SYMBOLS 532


// Our table is bespoke for the specific SF.
#define CHIPSSPREAD CHIRPLENGTH_WORDS// QUARTER_CHIRP_LENGTH_WORDS (TODO: Use the quater value elsewhere in the code)
#define MARK_FROM_SF0 (1<<SF_NUMBER) // SF7

// For some reason, adding a small time offset too symbols and header makes them more readable.
// On the ESP8266, this appaers to not be needed.
#define DATA_PHASE_OFFSET (CHIPSSPREAD/256)

#define PREAMBLE_CHIRPS 10
#define CODEWORD_LENGTH 2

uint32_t quadsetcount;
int32_t quadsets[MAX_SYMBOLS*4+PREAMBLE_CHIRPS*4+9+CODEWORD_LENGTH*4];

int32_t * AddChirp( int32_t * qso, int offset, int verneer )
{
	offset = offset * CHIPSSPREAD / (MARK_FROM_SF0);
	offset += verneer;
	*(qso++) = (CHIPSSPREAD * 0 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 1 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 2 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	*(qso++) = (CHIPSSPREAD * 3 / 4 + offset + CHIPSSPREAD ) % CHIPSSPREAD;
	return qso;
}


volatile int quadsetplace = -1;

int runningcount_bits = 0;

void slc_isr(void * v) {

	uint32_t * sendbuff = 0;
	uint32_t sendlen = 0;
	struct sdio_queue *finishedDesc;
//	slc_intr_status = READ_PERI_REG(SLC_INT_STATUS); -> We should check to make sure we are SLC_RX_EOF_INT_ST, but we are only getting one interrupt.
#ifdef TESTSTRAP
	struct sdio_queue tmp;
	finishedDesc = &tmp;
#else
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
#endif

	etx++;

	if( quadsetplace < 0 )
	{
		goto dump0;
	}

	// LoRa symbols are in quarters of a chirp.
	if( fxcycle>= NUM_DMAS_PER_QUARTER_CHIRP )
	{
		fxcycle = 0;
		quadsetplace++;
		if( quadsetplace >= quadsetcount ) goto dump0;
	}

	int symbol = quadsets[quadsetplace];

	// Select down- or up-chirp.
	if( symbol < 0 )
	{
		int word = fxcycle * DMA_SIZE_WORDS - symbol - 1;
		if( word >= CHIPSSPREAD ) word -= CHIPSSPREAD;
		word++;
		sendbuff = (chirpbuffDOWN + word);
	}
	else
	{
		int word = fxcycle * DMA_SIZE_WORDS + symbol;
		if( word >= CHIPSSPREAD ) word -= CHIPSSPREAD;
		sendbuff = (chirpbuffUP + word);
	}


	// Sometimes we do the full length, of all of the needed DMAs
	// Sometimes we overshoot the time window, so we peel off 4 bytes.
	int running_bits_after = runningcount_bits + DMA_SIZE_WORDS*32;
	int overflow = running_bits_after - IDEAL_QUARTER_CHIRP_LENGTH_BITS;
	if( overflow > 0 )
	{
		int overflow_amount = overflow / 32;
		int overflow_remainder = overflow % 32;
		sendlen = DMA_SIZE_WORDS*4 - 4*overflow_amount;
		runningcount_bits = overflow_remainder;

		// XXX TODO: Why can't I put the logic for advancing the group in here?
	}
	else
	{
		sendlen = DMA_SIZE_WORDS*4;
		runningcount_bits = running_bits_after;
	}

#ifdef TESTSTRAP
	static FILE * fappendlog;
	if( !fappendlog ) fappendlog = fopen( "fappendlog.csv", "w" );
	{
		if( symbol < 0 )
			fprintf( fappendlog, "2, %d, %d\n", (int)(CHIRPLENGTH_WORDS - (sendbuff - chirpbuffDOWN) - 1), sendlen );
		else
			fprintf( fappendlog, "1, %d, %d\n", (int)(sendbuff - chirpbuffUP), sendlen );
	}
#else
	finishedDesc->buf_ptr = (uint32_t)sendbuff;
	finishedDesc->datalen = sendlen;
#endif

	fxcycle++;
	return;


dump0:
#ifdef TESTSTRAP
	printf( "Hit dummy %d %d\n", quadsetplace, quadsetcount );
	exit( 0 );
#else
	// This location just always reads as zeroes.
	finishedDesc->buf_ptr = (uint32_t)dummy;
	quadsetplace = -1;
#endif
	return;
}


#ifdef TESTSTRAP

void SPIRead( uint32_t pos, uint32_t * buff, int len )
{
	memcpy( buff, (pos - 0x00020000) + (uint8_t*)chirpbuff, len );
}

void nosdk8266_init()
{
}


void testi2s_init()
{
}

#else

//Initialize I2S subsystem for DMA circular buffer use
void testi2s_init() {
	int x, y;
	//Bits are shifted out

	//Initialize DMA buffer descriptors in such a way that they will form a circular
	//buffer.

	for (x=0; x<DMABUFFERDEPTH; x++) {
		i2sBufDescTX[x].owner=1;
		i2sBufDescTX[x].eof=1;  // Trigger interrupt on packet complete.
		i2sBufDescTX[x].sub_sof=0;
		i2sBufDescTX[x].datalen=DMA_SIZE_WORDS*4;
		i2sBufDescTX[x].blocksize=4;
		i2sBufDescTX[x].buf_ptr= ((uint32_t)dummy);
		i2sBufDescTX[x].unused=0;
		i2sBufDescTX[x].next_link_ptr=(int)((x<(DMABUFFERDEPTH-1))?(&i2sBufDescTX[x+1]):(&i2sBufDescTX[0]));
	}

	//Reset DMA )
	//SLC_TX_LOOP_TEST = IF this isn't set, SO will occasionally get unrecoverable errors when you underflow.
	//Originally this little tidbit was found at https://github.com/pvvx/esp8266web/blob/master/info/libs/bios/sip_slc.c
	//
	//I have not tried without SLC_AHBM_RST | SLC_AHBM_FIFO_RST.  I just assume they are useful?
	SET_PERI_REG_MASK(SLC_CONF0, SLC_TX_LOOP_TEST |SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST | SLC_AHBM_FIFO_RST);
	CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST | SLC_AHBM_FIFO_RST);

	//Clear DMA int flags
	SET_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);
	CLEAR_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);

	//Enable and configure DMA
	CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE<<SLC_MODE_S));
	SET_PERI_REG_MASK(SLC_CONF0,(1<<SLC_MODE_S));
	
	// We have to do this, otherwise, when the end of a "RX" packet is hit, it will skip outputting a few random frames.
	SET_PERI_REG_MASK(SLC_RX_DSCR_CONF,SLC_INFOR_NO_REPLACE|SLC_TOKEN_NO_REPLACE);
	CLEAR_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_RX_FILL_EN|SLC_RX_EOF_MODE | SLC_RX_FILL_MODE);
	
	CLEAR_PERI_REG_MASK(SLC_RX_LINK,SLC_RXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)&i2sBufDescTX[0]) & SLC_RXLINK_DESCADDR_MASK);

	//Attach the DMA interrupt
	ets_isr_attach(ETS_SLC_INUM, slc_isr, 0);
	WRITE_PERI_REG(SLC_INT_ENA,  SLC_RX_EOF_INT_ENA );		// Not including SLC_RX_UDF_INT_ENA

	//clear any interrupt flags that are set
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	///enable DMA intr in cpu
	ets_isr_unmask(1<<ETS_SLC_INUM);

	//Start transmission
	SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);


	//Init pins to i2s functions
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA); // GPIO3
//	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_I2SO_WS);   // GPIO2
//	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_I2SO_BCK);   // GPIO15

	//Enable clock to i2s subsystem
	i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

	//Reset I2S subsystem
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);

	CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S));
	SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);
	WRITE_PERI_REG(I2SRXEOF_NUM, DMA_SIZE_WORDS*4);

	CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));

	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|I2S_RECE_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
                                    	(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((WS_I2S_BCK&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						((WS_I2S_DIV&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S) );

	//Start transmission
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START|I2S_I2S_RX_START);
}

#endif

int main()
{
	// We store the bit pattern at flash:0x20000, so we don't have to constantly
	// re-write it when working on code.
	SPIRead( MEMORY_START_OFFSET, chirpbuffUP, sizeof( chirpbuffUP ) );
	SPIRead( REVERSE_START_OFFSET, chirpbuffDOWN, sizeof( chirpbuffDOWN ) );
	memset( dummy, 0, sizeof( dummy ) );

	// Don't crank up clock speed til we're done with flash.
	nosdk8266_init();


	int i = 0;
	fxcycle = 0;
	etx = 0;


#ifndef TESTSTRAP
	// Configure GPIO5 (TX) and GPIO2 (LED)
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);
	PIN_DIR_OUTPUT = _BV(2); //Enable GPIO2 light off.

	// Run the I2S bus at 1040/6 = 173.333 MHz.
	// It looks like, at least on my part, if I try running
	// hotter it can get to 1040/5.1 but not all the way to
	// 5 so it's unstable there.
#endif
	testi2s_init();

	int frame = 0;
	uint16_t lora_symbols[MAX_SYMBOLS];
	int lora_symbols_count;

	while(1) {
		//12x this speed.
		frame++;
#ifndef TESTSTRAP
		PIN_OUT_SET = _BV(2); //Turn GPIO2 light off.
		//call_delay_us(1000000);
		printf("ETX: %d %08x\n", fxcycle, chirpbuffUP[10] );
		PIN_OUT_CLEAR = _BV(2); //Turn GPIO2 light off.
		call_delay_us(1000000);
#endif
		// Just some random data.
		uint8_t payload_in[259] = { 0xbb, 0xcc, 0xde, 0x55, 0x22,}; 
		int payload_in_size = 6;

		static int msgno = 0;
		payload_in[4] = msgno++;

		memset( lora_symbols, 0, sizeof(lora_symbols) );
		lora_symbols_count = 0;
		int r = CreateMessageFromPayload( lora_symbols, &lora_symbols_count, MAX_SYMBOLS, SF_NUMBER, 0, payload_in, payload_in_size );

		if( r < 0 )
		{
			printf( "Failed to generate message (%d)\n", r );
			// Failed
			continue;
		}

		int j;
		//for( j = 0; j < symbols_len; j++ )
		//	symbols[j] = 255 - symbols[j];

		quadsetcount = 0;
		int32_t * qso = quadsets;
		for( j = 0; j < PREAMBLE_CHIRPS; j++ )
		{
			qso = AddChirp( qso, 0, 0 );
		}

		uint8_t syncword = 0x43;

	#if SF_NUMBE == 7
		#define CODEWORD_SHIFT 3
	#elif SF_NUMBE == 8
		#define CODEWORD_SHIFT 4
	#else
		#define CODEWORD_SHIFT 3
	#endif

		if( CODEWORD_LENGTH > 0 )
			qso = AddChirp( qso,  ( ( syncword & 0xf ) << CODEWORD_SHIFT ), DATA_PHASE_OFFSET * 2 );
		if( CODEWORD_LENGTH > 1 )
			qso = AddChirp( qso, ( ( ( syncword & 0xf0 ) >> 4 ) << CODEWORD_SHIFT ), DATA_PHASE_OFFSET * 2 );


		*(qso++) = -(CHIPSSPREAD * 0 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 1 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 2 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 3 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 0 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 1 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 2 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 3 / 4 )-1;
		*(qso++) = -(CHIPSSPREAD * 0 / 4 )-1;

		//if( ADDSF <= 6 )
		//{
		//	// Two additional upchirps with SF6 https://github.com/tapparelj/gr-lora_sdr/issues/74#issuecomment-1891569580
		//	for( j = 0; j < 2; j++ )
		//	{
		//		qso = AddChirp( qso, 0, 0 );
		//	}
		//}

		for( j = 0; j < lora_symbols_count; j++ )
		{
			int ofs = lora_symbols[j];
			//ofs = ofs ^ ((MARK_FROM_SF6<<6) -1);
			//ofs &= (MARK_FROM_SF6<<6) -1;
			qso = AddChirp( qso, ofs, DATA_PHASE_OFFSET );
			printf( "%02x ", ofs );
		}
		printf( "\n" );

/* GOOD (CR 5/4), 20 Bytes 58 30 60 00 68 10 60 48 70 14 5f 0f 2a ... */
/* GOOD (CR 5/4), 21 Bytes 57 0f 7f 00 77 10 58 28 70 14 5f 0f 2a ... */
/*  BAD (CR 5/4), 22 Bytes 27 37 60 0f 70 60 67 48 70 14 5f 0f 2a ... */
/*  BAD (CR 5/4), 23 Bytes 28 08 7f 0f 6f 60 5f 28 70 14 5f 0f 2a ... */
/*   EH (CR 5/4), 24 Bytes 58 0f 00 3f 77 10 58 57 70 14 5f 0f 2a ... */
/* GOOD (CR 5/4), 25 Bytes 57 30 1f 3f 68 10 60 37 70 14 5f 0f 2a ... */
/*   EH (CR 5/4), 26 Bytes 27 08 00 30 6f 60 5f 57 70 14 5f 0f 2a ... <<< CRC is bogus but payload is good. */
/*  BAD (CR 5/4), 27 Bytes 28 37 1f 30 70 60 67 37 70 14 5f 0f 2a ... */
/*  BAD (CR 5/4), 28 Bytes 58 30 7f 30 6f 1f 58 37 70 14 5f 0f 2a ... */
/*  BAD (CR 5/4), 29 Bytes 57 0f 60 30 70 1f 60 57 70 14 5f 0f 2a ... */
/*  BAD (CR 5/4), 30 Bytes 27 37 7f 3f 77 6f 5f 37 70 14 5f 0f 2a ... */
/*  BAD (CR 5/4), 31 Bytes 28 08 60 3f 68 6f 67 57 70 14 5f 0f 2a ... */
/*   PG (CR 5/4), 32 Bytes 20 4f 1f 0f 70 67 60 08 70 14 5f 0f 2a ... */
/*  BAD (CR 5/4), 33 Bytes 2f 70 00 0f 6f 67 58 68 70 14 5f 0f 2a ... */
/*   EH (CR 5/4), 34 Bytes 5f 48 1f 00 68 17 67 08 70 14 5f 0f 2a ... << CRC is bogus but payload is good. */
/*   EH (CR 5/4), 35 Bytes 50 77 00 00 77 17 5f 68 70 14 5f 0f 2a ... << CRC is bogus but payload is good. */
/* GOOD (CR 5/4), 36 Bytes 20 70 60 00 68 68 60 68 70 14 5f 0f 2a ... */
/*   EH (CR 5/4), 37 Bytes 2f 4f 7f 00 77 68 58 08 70 14 5f 0f 2a ... << CRC is bogus but payload is good. */

		runningcount_bits = 0;

		// This tells the interrupt we have data.
		quadsetcount = qso - quadsets + 0;
		printf( "--- %d [%d] %d\n", lora_symbols_count, quadsetcount, CHIPSSPREAD/4 );
		quadsetplace = 0;
#ifdef TESTSTRAP
		while(1)
		{
			slc_isr( 0 );
		}
#endif
	}

}

