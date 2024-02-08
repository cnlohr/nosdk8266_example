
//#include "c_types.h"
#include "esp8266_auxrom.h"
#include "eagle_soc.h"
#include "nosdk8266.h"
#include "esp8266_rom.h"

// TODO: Use float number (related to 8) to fix the drift
#define call_delay_us(time) { asm volatile("mov.n a2, %0\n_call0 delay4clk" : : "r"(time * (MAIN_MHZ / 8)) : "a2" ); }

#include "ets_sys.h"
#include "slc_register.h"
#include "dmastuff.h"
#include "pin_mux_register.h"

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





#define MAX_SYMBOLS 270


// Our table is bespoke for the specific SF.
#define CHIPSSPREAD CHIRPLENGTH_WORDS// QUARTER_CHIRP_LENGTH_WORDS (TODO: Use the quater value elsewhere in the code)
#define MARK_FROM_SF0 (1<<7) // SF7

// For some reason, adding a small time offset too symbols and header makes them more readable.
#define DATA_PHASE_OFFSET ( CHIPSSPREAD / 512 )

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

void slc_isr(void * v) {
	struct sdio_queue *finishedDesc;
//	slc_intr_status = READ_PERI_REG(SLC_INT_STATUS); -> We should check to make sure we are SLC_RX_EOF_INT_ST, but we are only getting one interrupt.
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);

	//#define DMA_SIZE_WORDS (129)
	//#define NUM_DMAS_PER_QUARTER_CHIRP (11)
	//#define CHIRPLENGTH_WORDS_WITH_PADDING (5675)

	finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
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

	if( symbol < 0 )
	{
		int word = fxcycle * DMA_SIZE_WORDS - symbol;
		if( word >= CHIPSSPREAD ) word -= CHIPSSPREAD;
		finishedDesc->buf_ptr = (uint32_t)(chirpbuffDOWN + word);
	}
	else
	{
		int word = fxcycle * DMA_SIZE_WORDS + symbol;
		if( word >= CHIPSSPREAD ) word -= CHIPSSPREAD;
		finishedDesc->buf_ptr = (uint32_t)(chirpbuffUP + word);
	}
	fxcycle++;
	return;
dump0:
	// This location just always reads as zeroes.
	finishedDesc->buf_ptr = (uint32_t)dummy;
	quadsetplace = -1;
	return;
}

//Initialize I2S subsystem for DMA circular buffer use
void testi2s_init() {
	int x, y;
	//Bits are shifted out

	//Initialize DMA buffer descriptors in such a way that they will form a circular
	//buffer.

	fxcycle = 0;
	for (x=0; x<DMABUFFERDEPTH; x++) {
		i2sBufDescTX[x].owner=1;
		i2sBufDescTX[x].eof=1;  // Trigger interrupt on packet complete.
		i2sBufDescTX[x].sub_sof=0;
		i2sBufDescTX[x].datalen=DMA_SIZE_WORDS*4;
		i2sBufDescTX[x].blocksize=DMA_SIZE_WORDS*4;
		i2sBufDescTX[x].buf_ptr= ((uint32_t)(chirpbuffUP)) + (fxcycle++) * DMA_SIZE_WORDS * 4;
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

int main()
{
	int i = 0;

	// We store the bit pattern at flash:0x20000, so we don't have to constantly
	// re-write it when working on code.
	SPIRead( MEMORY_START_OFFSET, chirpbuffUP, sizeof( chirpbuffUP ) );
	SPIRead( REVERSE_START_OFFSET, chirpbuffDOWN, sizeof( chirpbuffDOWN ) );
	memset( dummy, 0, sizeof( dummy ) );
	nosdk8266_init();

	// Configure GPIO5 (TX) and GPIO2 (LED)
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);
	PIN_DIR_OUTPUT = _BV(2); //Enable GPIO2 light off.

	// Run the I2S bus at 1040/6 = 173.333 MHz.
	// It looks like, at least on my part, if I try running
	// hotter it can get to 1040/5.1 but not all the way to
	// 5 so it's unstable there.

	testi2s_init();

	int frame = 0;
	uint16_t lora_symbols[MAX_SYMBOLS];
	int lora_symbols_count;

	while(1) {
		//12x this speed.
		frame++;
		PIN_OUT_SET = _BV(2); //Turn GPIO2 light off.
		//call_delay_us(1000000);
		printf("ETX: %d %08x\n", fxcycle, chirpbuffUP[10]);
		PIN_OUT_CLEAR = _BV(2); //Turn GPIO2 light off.
		call_delay_us(1000000);
		int r = CreateMessageFromPayload( lora_symbols, &lora_symbols_count, MAX_SYMBOLS, 7 /* Hard-coded because of our table */ );

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

	#if ADDSF <= 6
		#define CODEWORD_SHIFT 2 // XXX TODO: No idea what this would do here! XXX This is probably wrong.
	#elif ADDSF >= 11
		#define CODEWORD_SHIFT 3 // XXX TODO: Unknown for SF11, SF12 Might be 3?
	#else
		#define CODEWORD_SHIFT 3
	#endif

		if( CODEWORD_LENGTH > 0 )
			qso = AddChirp( qso,  ( ( syncword & 0xf ) << CODEWORD_SHIFT ), 0 );
		if( CODEWORD_LENGTH > 1 )
			qso = AddChirp( qso, ( ( ( syncword & 0xf0 ) >> 4 ) << CODEWORD_SHIFT ), 0 );


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
		}
		
		quadsetcount = qso - quadsets;
		printf( "--- %d %d %d\n", lora_symbols_count, quadsetcount, CHIPSSPREAD/4 );


		quadsetplace = 0;
	}

}

