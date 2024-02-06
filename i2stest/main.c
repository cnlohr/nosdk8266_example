//#include "c_types.h"
#include "esp8266_auxrom.h"
#include "eagle_soc.h"
#include "nosdk8266.h"

// TODO: Use float number (related to 8) to fix the drift
#define call_delay_us(time) { asm volatile("mov.n a2, %0\n_call0 delay4clk" : : "r"(time * (MAIN_MHZ / 8)) : "a2" ); }


#include "ets_sys.h"
#include "slc_register.h"
#include "dmastuff.h"
#include "pin_mux_register.h"

#define DMABUFFERDEPTH 3
#define I2SDMABUFLEN (16)
#define LINE32LEN I2SDMABUFLEN
#define RX_NUM (I2SDMABUFLEN)

extern int fxcycle;
extern int erx, etx;

void testi2s_init();

//These contol the speed at which the bus comms.
#define WS_I2S_BCK 16  //Can't be less than 1.
#define WS_I2S_DIV 16

//I2S DMA buffer descriptors
static struct sdio_queue i2sBufDescRX[DMABUFFERDEPTH] __attribute__((aligned(128)));;
static struct sdio_queue i2sBufDescTX[DMABUFFERDEPTH] __attribute__((aligned(128)));;
uint32_t i2sBDRX[I2SDMABUFLEN*DMABUFFERDEPTH] __attribute__((aligned(128)));
uint32_t i2sBDTX[I2SDMABUFLEN*DMABUFFERDEPTH] __attribute__((aligned(128)));
int fxcycle;
int erx, etx;

void slc_isr(void * v) {
	//portBASE_TYPE HPTaskAwoken=0;
	struct sdio_queue *finishedDesc;
	uint32 slc_intr_status;
	int x;
	fxcycle++;

	slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);

//printf( "%08x\n", slc_intr_status );
	if ( (slc_intr_status & SLC_RX_EOF_INT_ST))
	{
		finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
		//finishedDesc->owner=1;

		etx++;	//I know it's wacky, but the nomeclature is backwards, this is for TX packets in here.
	}
	if ( (slc_intr_status & SLC_TX_EOF_INT_ST))
	{
		finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_TX_EOF_DES_ADDR);
		//finishedDesc=finishedDesc->next_link_ptr;

		//Don't know why - but this MUST be done, otherwise everything comes to a screeching halt.
		//finishedDesc->owner=1;

		//Nomaclature weird. this is actually RX packets.
		erx++;
	}



}

//Initialize I2S subsystem for DMA circular buffer use
void testi2s_init() {
	int x, y;
	//Bits are shifted out

	//Initialize DMA buffer descriptors in such a way that they will form a circular
	//buffer.

	for (x=0; x<DMABUFFERDEPTH; x++) {
		i2sBufDescRX[x].owner=1;
		i2sBufDescRX[x].eof=0;
		i2sBufDescRX[x].sub_sof=0;
		i2sBufDescRX[x].datalen=I2SDMABUFLEN*4;
		i2sBufDescRX[x].blocksize=I2SDMABUFLEN*4;
		i2sBufDescRX[x].buf_ptr=(uint32_t)&i2sBDRX[x*I2SDMABUFLEN];
		i2sBufDescRX[x].unused=0;
		i2sBufDescRX[x].next_link_ptr=(int)((x<(DMABUFFERDEPTH-1))?(&i2sBufDescRX[x+1]):(&i2sBufDescRX[0]));
		for( y = 0; y < I2SDMABUFLEN; y++ )
		{
			i2sBDRX[y+x*I2SDMABUFLEN] = 0xAAAAAAAA;
		}
	}

	for (x=0; x<DMABUFFERDEPTH; x++) {
		i2sBufDescTX[x].owner=1;
		i2sBufDescTX[x].eof=1;  // Trigger interrupt on packet complete.
		i2sBufDescTX[x].sub_sof=0;
		i2sBufDescTX[x].datalen=I2SDMABUFLEN*4;
		i2sBufDescTX[x].blocksize=I2SDMABUFLEN*4;
		i2sBufDescTX[x].buf_ptr=(uint32_t)&i2sBDTX[x*I2SDMABUFLEN];
		i2sBufDescTX[x].unused=0;
		i2sBufDescTX[x].next_link_ptr=(int)((x<(DMABUFFERDEPTH-1))?(&i2sBufDescTX[x+1]):(&i2sBufDescTX[0]));
		for( y = 0; y < I2SDMABUFLEN; y++ )
		{
			i2sBDTX[y+x*I2SDMABUFLEN] = 0xAAAAAAAA;
		}
		i2sBDTX[x*I2SDMABUFLEN + I2SDMABUFLEN - 1] = 0xffffffff;
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
	
	CLEAR_PERI_REG_MASK(SLC_TX_LINK,SLC_TXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_TX_LINK, ((uint32)&i2sBufDescRX[0]) & SLC_TXLINK_DESCADDR_MASK); //any random desc is OK, we don't use TX but it needs something valid
	CLEAR_PERI_REG_MASK(SLC_RX_LINK,SLC_RXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)&i2sBufDescTX[0]) & SLC_RXLINK_DESCADDR_MASK);

	//Attach the DMA interrupt
	ets_isr_attach(ETS_SLC_INUM, slc_isr, 0);
	WRITE_PERI_REG(SLC_INT_ENA,  SLC_TX_EOF_INT_ENA | SLC_RX_EOF_INT_ENA );
		// Not including SLC_RX_UDF_INT_ENA | SLC_TX_DSCR_ERR_INT_ENA

	//clear any interrupt flags that are set
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	///enable DMA intr in cpu
	ets_isr_unmask(1<<ETS_SLC_INUM);

	//Start transmission
	SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
	SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);


	//Init pins to i2s functions
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);  // GPIO12
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);    // GPIO14 //Dunno why - this is needed.  If it's not enabled, nothing will be read.

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA); // GPIO3
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_I2SO_WS);   // GPIO2
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_I2SO_BCK);   // GPIO15

	//Enable clock to i2s subsystem
	i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

	//Reset I2S subsystem
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);

	CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S)|(I2S_I2S_TX_FIFO_MOD<<I2S_I2S_TX_FIFO_MOD_S));
	SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);
	WRITE_PERI_REG(I2SRXEOF_NUM, RX_NUM);

	CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_TX_CHAN_MOD<<I2S_TX_CHAN_MOD_S)|(I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));

	//Clear int
	SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);


	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|I2S_RECE_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
                                    	(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((WS_I2S_BCK&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						((WS_I2S_DIV&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S) );



	//enable int
//	SET_PERI_REG_MASK(I2SINT_ENA,   I2S_I2S_TX_REMPTY_INT_ENA|I2S_I2S_TX_WFULL_INT_ENA|
//			I2S_I2S_RX_REMPTY_INT_ENA|I2S_I2S_TX_PUT_DATA_INT_ENA|I2S_I2S_RX_TAKE_DATA_INT_ENA);

	//Start transmission
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START|I2S_I2S_RX_START);
}

int main()
{
	int i = 0;
	nosdk8266_init();

	// Configure GPIO5 (TX) and GPIO2 (LED)
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);
	PIN_DIR_OUTPUT = _BV(2); //Enable GPIO2 light off.

	//call_delay_us( 3000000 );
	//int j = 0;
	//for (j = 0; j < 200; j++) {
	//	uart_div_modify(0, (j * 1000000) / 115200);
	//	printf("pllworkingfreq: %d ", j);
	//}

	testi2s_init();

	while(1) {
		//12x this speed.

		PIN_OUT_SET = _BV(2); //Turn GPIO2 light off.
		//call_delay_us(1000000);
		printf("ERX: %d   ETX: %d  input buffer: %08x %08x %08x\n",  erx, etx, i2sBDRX[0], i2sBDRX, slc_isr );
/*
		int i;
		for( i = 0; i < I2SDMABUFLEN*DMABUFFERDEPTH; i++ )
		{
			printf( "%08x ", i2sBDTX[i] );
		}
*/
		//printf("PLL divider register values: (1)0x%x | (2)0x%x\n", rom_i2c_readReg(103, 4, 1), rom_i2c_readReg(103, 4, 2));
		PIN_OUT_CLEAR = _BV(2); //Turn GPIO2 light off.
		call_delay_us(1000000);
		//i++;
	}
}

