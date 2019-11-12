#include <stdint.h>
#include <stddef.h>
#include "em_core.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "main.h"
#include "uartdrv.h"
#include "mainctrl.h"
#include "string.h"

#define UART_FRAMR_QUEUE_LEN_10 10

/*
 * uart frame queue
 * @num: data item counter
 * @in: enqueue index
 * @out: dequeue index
 * @RS422frame[]: array for storing items
 * */
typedef struct {
	volatile int8_t num;
	int8_t in, out;
	UartFrame RS422frame[UART_FRAMR_QUEUE_LEN_10];
}UartFrameQueueDef;

UartFrameQueueDef uartFrameQueue = {0};

static bool UartFrameEnqueueEmpty(void)
{
	if (uartFrameQueue.num == 0)
		return true;
	else
		return false;
}

static bool UartFrameEnqueueFull(void)
{
	if (uartFrameQueue.num == UART_FRAMR_QUEUE_LEN_10)
		return true;
	else
		return false;
}

int UartFrameEnqueue(UartFrame *uFrame)
{
	if (UartFrameEnqueueFull())
		return -1;

	uartFrameQueue.RS422frame[uartFrameQueue.in] = *uFrame;
	uartFrameQueue.in++;
	if (uartFrameQueue.in > UART_FRAMR_QUEUE_LEN_10 - 1)
		uartFrameQueue.in = 0;

	CORE_CriticalDisableIrq();
	uartFrameQueue.num += 1;
	CORE_CriticalEnableIrq();

	return 0;
}

UartFrame* UartFrameDequeue(void)
{
	UartFrame *uFrame = NULL;

	if (UartFrameEnqueueEmpty())
		goto out;

	uFrame = &uartFrameQueue.RS422frame[uartFrameQueue.out];
	uartFrameQueue.out++;
	if (uartFrameQueue.out > UART_FRAMR_QUEUE_LEN_10 - 1)
		uartFrameQueue.out = 0;

	CORE_CriticalDisableIrq();
	uartFrameQueue.num -= 1;
	CORE_CriticalEnableIrq();

out:
	return uFrame;
}

volatile struct circularBuffer rxBuf = { {0}, 0, 0, 0, false }, txBuf = { {0}, 0, 0, 0, false };

/* Setup UART0 in async mode for RS232*/
static USART_TypeDef *uart = USART0;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

void UART_DMAConfig(void);
/*
 * uartSetup function
 */
void uartSetup(void)
{
	/*
	 * Enable clock for GPIO module (required for pin configuration)
	 * */
	CMU_ClockEnable(cmuClock_USART0, true);
	/*
	 * Configure GPIO pins
	 * */
	GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortE, 11, gpioModeInput, 1);

	/*
	 * Prepare struct for initializing UART in asynchronous mode
	 * */
	uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
	uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
	uartInit.baudrate     = 256000;//256000;         /* Baud rate *///115200 transfers to 148720
	uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
	uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
	uartInit.parity       = usartNoParity; /* Parity mode */
	uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
	uartInit.mvdis        = false;          /* Disable majority voting */
	uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
	uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

	/*
	 * Initialize USART with uartInit struct
	 * */
	USART_InitAsync(uart, &uartInit);

	/*
	 * Prepare UART Rx and Tx interrupts
	 * */
	USART_IntClear(uart, _USART_IFC_MASK);
	USART_IntEnable(uart, USART_IEN_RXDATAV);
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	//NVIC_ClearPendingIRQ(USART0_TX_IRQn);
	NVIC_EnableIRQ(USART0_RX_IRQn);
	//NVIC_EnableIRQ(USART0_TX_IRQn);

	/*
	 * Enable I/O pins at UART1 location #2
	 * */
	uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;

	/*
	 * config DMA for USART0
	 * */
	UART_DMAConfig();

	/*
	 * Enable UART
	 * */
	//USART_Enable(uart, usartEnableTx);
}


/*
 * @brief  uartGetChar function
 *
 *  Note that if there are no pending characters in the receive buffer, this
 *  function will hang until a character is received.
 *
 * */
uint8_t uartGetChar( )
{
	uint8_t ch;

	/*
	 * Check if there is a byte that is ready to be fetched. If no byte is ready, wait for incoming data
	 * */
	if (rxBuf.pendingBytes < 1) {
		while (rxBuf.pendingBytes < 1) ;
	}

	/* Copy data from buffer */
	ch = rxBuf.data[rxBuf.rdI];
	rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

	/* Decrement pending byte counter */
	rxBuf.pendingBytes--;

	return ch;
}


/*
 * @brief  uartGetChar function
 *
 *  Note that if there are no pending characters in the receive buffer, this
 *  function will return 0, else return 1.
 * */
uint32_t uartReadChar(uint8_t *data)
{
	/*
	 * Check if there is a byte that is ready to be fetched. If no byte is ready, return 0
	 * */
	if (rxBuf.pendingBytes < 1){
		return 0 ;
	}

	/* Copy data from buffer */
	*data = rxBuf.data[rxBuf.rdI];
	rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

	/* Decrement pending byte counter */
	rxBuf.pendingBytes--;

	return 1;
}

/******************************************************************************
 * @brief  uartPutChar function
 *
 *****************************************************************************/
void uartPutChar(uint8_t ch)
{
	/* Check if Tx queue has room for new data */
	if ((txBuf.pendingBytes + 1) > BUFFERSIZE) {
		/* Wait until there is room in queue */
		while ((txBuf.pendingBytes + 1) > BUFFERSIZE) ;
	}

	/* Copy ch into txBuffer */
	txBuf.data[txBuf.wrI] = ch;
	txBuf.wrI = (txBuf.wrI + 1) % BUFFERSIZE;

	/* Increment pending byte counter */
	txBuf.pendingBytes++;

	/* Enable interrupt on USART TX Buffer*/
	USART_IntEnable(uart, USART_IEN_TXBL);
}

/******************************************************************************
 * @brief  uartPutData function
 *
 *****************************************************************************/
void uartPutData(volatile uint8_t * dataPtr, uint32_t dataLen)
{
	uint32_t i = 0;

	/* Check if buffer is large enough for data */
	if (dataLen > BUFFERSIZE)
		return;

	/* Check if buffer has room for new data */
	if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) {
		/* Wait until room */
		while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE);
	}

	/* Fill dataPtr[0:dataLen-1] into txBuffer */
	while (i < dataLen) {
		txBuf.data[txBuf.wrI] = *(dataPtr + i);
		txBuf.wrI = (txBuf.wrI + 1) % BUFFERSIZE;
		i++;
	}

	/* Increment pending byte counter */
	txBuf.pendingBytes += dataLen;

	/* Enable interrupt on USART TX Buffer*/
	USART_IntEnable(uart, USART_IEN_TXBL);
}

/*
 * @brief  uartGetData function
 * */
uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen)
{
	uint32_t i = 0;

	/* Wait until the requested number of bytes are available */
	if (rxBuf.pendingBytes < dataLen) {
		while (rxBuf.pendingBytes < dataLen) ;
	}

	if (dataLen == 0) {
		dataLen = rxBuf.pendingBytes;
	}

	/* Copy data from Rx buffer to dataPtr */
	while (i < dataLen) {
		*(dataPtr + i) = rxBuf.data[rxBuf.rdI];
		rxBuf.rdI      = (rxBuf.rdI + 1) % BUFFERSIZE;
		i++;
	}

	/* Decrement pending byte counter */
	rxBuf.pendingBytes -= dataLen;

	return i;
}

sleepCMD_t sleepCMD = {
		.begin = "begin",
		.frameLen = 0x0c,
		.reserve = {0},
		.sleepCmd = {0x91, 0xee},
		.crc = {0x9d, 0xee},
		.end = "end-",
};

bool parseSleepCMD(sleepCMD_t *cmd)
{
	if (!strncmp((char *)cmd->begin, (char *)sleepCMD.begin, 5)
		&& !strncmp((char *)cmd->sleepCmd, (char *)sleepCMD.sleepCmd, 2)) {
		return true;
	}

	return false;
}

uint32_t checkSleepCMD(rcvMsg_t *rcvMessage)
{
	uint32_t i = 0;
	uint32_t dataLen;
	uint8_t *str = "here\n";
	uint8_t *str2 = "here2\n";
	static int rdi1 = -1, rdi2 = -1;

	if (rxBuf.pendingBytes < SLEEPCMD_LEN)
		return 0;

	dataLen = rxBuf.pendingBytes;

	/* Copy data from Rx buffer to dataPtr */
	while (i < dataLen) {
		if (rxBuf.data[rxBuf.rdI] == 0x62) {
			rcvMessage->searchHeadFlag = true;
			rcvMessage->len = 0;
			uartPutData(str, 5);
			if (rdi1 == -1)
				rdi1 = rxBuf.rdI;
			else if (rdi2 == -1)
				rdi2 = rxBuf.rdI;
		}

		if (rcvMessage->searchHeadFlag) {
			rcvMessage->rcvBytes[rcvMessage->len++] = rxBuf.data[rxBuf.rdI];
			if (rcvMessage->len == SLEEPCMD_LEN) {
				if (parseSleepCMD((sleepCMD_t *)rcvMessage->rcvBytes)) {
					g_cur_mode = MAIN_SLEEPMODE;
					NVIC_DisableIRQ(USART0_RX_IRQn);
					rxBuf.pendingBytes -= SLEEPCMD_LEN;
					rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;
					i++;
					NVIC_EnableIRQ(USART0_RX_IRQn);
					//uartPutData((uint8_t *)&g_recvSlaveFr, sizeof(struct MainCtrlFrame));
					uartPutData(str2, 6);
					rdi1 = rdi2 = -1;
					return i;
				} else {
					rcvMessage->searchHeadFlag = false;
				}
			}
		}

		rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;
		i++;
	}

	/* Decrement pending byte counter */
	NVIC_DisableIRQ(USART0_RX_IRQn);
	//CORE_CriticalDisableIrq();
	rxBuf.pendingBytes -= dataLen;
	//CORE_CriticalEnableIrq();
	NVIC_EnableIRQ(USART0_RX_IRQn);

	return i;
}

/*
 * the below logic is for UART-Rx DMA feature.
 * */
#if ( (DMA_CHAN_COUNT > 0) && (DMA_CHAN_COUNT <= 4) )
#define DMACTRL_CH_CNT      4
#define DMACTRL_ALIGNMENT   128

#elif ( (DMA_CHAN_COUNT > 4) && (DMA_CHAN_COUNT <= 8) )
#define DMACTRL_CH_CNT      8
#define DMACTRL_ALIGNMENT   256

#elif ( (DMA_CHAN_COUNT > 8) && (DMA_CHAN_COUNT <= 12) )
#define DMACTRL_CH_CNT      16
#define DMACTRL_ALIGNMENT   256

#else
#error "Unsupported DMA channel count (dmactrl.c)."
#endif

/** DMA control block array, requires proper alignment. */
SL_ALIGN(DMACTRL_ALIGNMENT)
DMA_DESCRIPTOR_TypeDef dmaControlBlock1[DMACTRL_CH_CNT * 2] SL_ATTRIBUTE_ALIGN(DMACTRL_ALIGNMENT);

#if 0
#define CMD_LEN 22
uint8_t g_primaryResultBuffer[CMD_LEN] = {0}, g_alterResultBuffer[CMD_LEN] = {0};
#endif

DMA_CB_TypeDef dma_uart_cb;

/*
 * starts the UART-tx in UART DMA done callback function if there is new data in rxBuf,
 * in order to more quickly transferring the data.
 * */
void TryQuickUART_Tx(void)
{
	if ((rxBuf.wrI + BUFFERSIZE - rxBuf.rdI) % BUFFERSIZE > 0) {
		if (!g_uartDMAStarted) {
			if (rxBuf.wrI > rxBuf.rdI) {
				g_uartDMATransferNum = rxBuf.wrI - rxBuf.rdI;
			} else {
				g_uartDMATransferNum = BUFFERSIZE - rxBuf.rdI;
			}

			DMA_ActivateBasic(
				DMA_CHANNEL,
				true,
				false,
				(void *)&(USART0->TXDATA), // primary destination
				(void *)&rxBuf.data[rxBuf.rdI], // primary source
				g_uartDMATransferNum - 1
				);

			USART_Enable(USART0, usartEnableTx);
			g_uartDMAStarted = true;
		}
	}
}

/*
 * normally check if there is new data in rxBuf within the main loop of main function.
 * */
void CheckUARTTx(void)
{
	if ((rxBuf.wrI + BUFFERSIZE - rxBuf.rdI) % BUFFERSIZE > 0) {
		CORE_CriticalDisableIrq();
		if (!g_uartDMAStarted) {
			if (rxBuf.wrI > rxBuf.rdI) {
				g_uartDMATransferNum = rxBuf.wrI - rxBuf.rdI;
			} else {
				g_uartDMATransferNum = BUFFERSIZE - rxBuf.rdI;
			}

			DMA_ActivateBasic(
				DMA_CHANNEL,
				true,
				false,
				(void *)&(USART0->TXDATA), // primary destination
				(void *)&rxBuf.data[rxBuf.rdI], // primary source
				g_uartDMATransferNum - 1
				);

			USART_Enable(USART0, usartEnableTx);
			g_uartDMAStarted = true;
		}
		CORE_CriticalEnableIrq();
	}
}

void UART_DMA_callback(unsigned int channel, bool primary, void *user)
{
	/*
	 * update 'rxBuf.rdI' offset
	 * */
	rxBuf.rdI = (rxBuf.rdI + g_uartDMATransferNum) % BUFFERSIZE;
	g_uartDMATransferNum = 0;
	g_uartDMAStarted = false;

	TryQuickUART_Tx();

#if 0
	if (primary == true)
		memcpy((void *)&rxBuf.data[rxBuf.wrI], (void *)g_primaryResultBuffer, CMD_LEN);
	else
		memcpy((void *)&rxBuf.data[rxBuf.wrI], (void *)g_alterResultBuffer, CMD_LEN);

	rxBuf.wrI = (rxBuf.wrI + CMD_LEN) % BUFFERSIZE;
	rxBuf.pendingBytes += CMD_LEN;

	/* Re-activate the DMA */
	DMA_RefreshPingPong(
		channel,
		primary,
		false,
		NULL,
		NULL,
		CMD_LEN - 1,
		false);

	USART0->CMD |= USART_CMD_RXEN;
#endif
}

void DMAInit(void)
{
	DMA_Init_TypeDef dmaInit;

	/*
	* Configure general DMA issues
	* */
	dmaInit.hprot = 0;
	dmaInit.controlBlock = dmaControlBlock1;
	DMA_Init(&dmaInit);
}

void UART_DMAConfig(void)
{
	DMA_CfgDescr_TypeDef descrCfg;
	DMA_CfgChannel_TypeDef chnlCfg;

	CMU_ClockEnable(cmuClock_DMA, true);

	/*
	* Configure DMA channel used
	* */
	dma_uart_cb.cbFunc = UART_DMA_callback;
	dma_uart_cb.userPtr = NULL;

	chnlCfg.highPri = false;
	chnlCfg.enableInt = true;
	chnlCfg.select = DMAREQ_USART0_TXBL;
	chnlCfg.cb = &dma_uart_cb;
	DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);

	/*
	* one byte per transfer
	* */
	descrCfg.dstInc = dmaDataIncNone;
	descrCfg.srcInc = dmaDataInc1;
	descrCfg.size = dmaDataSize1;
	descrCfg.arbRate = dmaArbitrate1;
	descrCfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);

#if 0
	DMA_CfgDescr(DMA_CHANNEL, false, &descrCfg);
	// Start DMA
	DMA_ActivatePingPong(
		DMA_CHANNEL,
		false,
		(void *)&g_primaryResultBuffer, // primary destination
		(void *)&(USART0->RXDATA), // primary source
		CMD_LEN - 1,
		(void *)&g_alterResultBuffer, // alternate destination
		(void *)&(USART0->RXDATA), // alternate source
		CMD_LEN - 1);
#endif
//	DMA_ActivateBasic(
//		DMA_CHANNEL,
//		true,
//		false,
//		(void *)&(USART0->TXDATA), // primary destination
//		(void *)&rxBuf.data[0], // primary source
//		CMD_LEN - 1
//		);
}

/*
 * @brief UART0 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 * */
void USART0_RX_IRQHandler(void)
{
	uint32_t flag = 0;

	flag = USART_IntGet(uart);
	USART_IntClear(uart, flag);

	/* Check for RX data valid interrupt */
	if (uart->IF & USART_IF_RXDATAV) {
		/* Copy data into RX Buffer */
		uint8_t rxData = USART_Rx(uart);
		rxBuf.data[rxBuf.wrI] = rxData;
		rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
		rxBuf.pendingBytes++;

		/* Flag Rx overflow */
		if (rxBuf.pendingBytes > BUFFERSIZE) {
			rxBuf.overflow = true;
		}
	}
}

/*
 * @brief UART0 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 * */
void USART0_TX_IRQHandler(void)
{
	/* Check TX buffer level status */
	if (uart->IF & USART_IF_TXBL) {
		if (txBuf.pendingBytes > 0) {
			/* Transmit pending character */
			USART_Tx(uart, txBuf.data[txBuf.rdI]);
			txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
			txBuf.pendingBytes--;
		}

		/* Disable Tx interrupt if no more bytes in queue */
		if (txBuf.pendingBytes == 0) {
			USART_IntDisable(uart, USART_IEN_TXBL);
		}
	}
}
