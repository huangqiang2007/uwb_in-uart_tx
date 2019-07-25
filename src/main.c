#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "mainctrl.h"
#include "uartdrv.h"
#include "spidrv.h"
#include "Typedefs.h"
#include "libdw1000.h"

extern volatile int8_t g_slaveWkup;

void clockConfig(void)
{
	SystemCoreClockUpdate();

	/*
	 * chose external crystal oscillator as clock source.
	 * */
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_OscillatorEnable(cmuSelect_ULFRCO, true, true);
	CMU_ClockEnable(cmuClock_HFLE, true);

	/*
	 * Enable clocks required
	 * */
	//CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
}


#if 1

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
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMACTRL_CH_CNT * 2] SL_ATTRIBUTE_ALIGN(DMACTRL_ALIGNMENT);

#define TX_BUFFER_SIZE   10
#define RX_BUFFER_SIZE   TX_BUFFER_SIZE

uint8_t TxBuffer[TX_BUFFER_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8_t RxBuffer[RX_BUFFER_SIZE];

/**************************************************************************//**
 * @brief callback function that will trigger after a completed DMA transfer
 *****************************************************************************/
void refreshRxTransfer(uint32_t channelNum,
                     bool isPrimaryDescriptor,
                     void *userPtr)
{
	bool isUseBurst = false;
	DMA_ActivateBasic(channelNum,
					isPrimaryDescriptor,
					isUseBurst,
					(void *) RxBuffer,         // Destination address to transfer to
					(void *) &USART1->RXDATA,  // Source address to transfer from
					RX_BUFFER_SIZE - 1);       // Number of DMA transfers minus 1
}

/**************************************************************************//**
 * @brief callback function that will trigger after a completed DMA transfer
 *****************************************************************************/
void refreshTxTransfer(uint32_t channelNum,
                     bool isPrimaryDescriptor,
                     void *userPtr)
{
	bool isUseBurst = false;
	DMA_ActivateBasic(channelNum,
					isPrimaryDescriptor,
					isUseBurst,
					(void *) &USART1->TXDATA,  // Destination address to transfer to
					(void *) TxBuffer,         // Source address to transfer from
					RX_BUFFER_SIZE - 1);       // Number of DMA transfers minus 1
}

/**************************************************************************//**
 * @brief
 *    Initialize the DMA module to transfer packets via USART whenever there
 *    is room in the TX Register
 *
 * @note
 *    The callback object needs to at least have static scope persistence so
 *    that the reference to the object is valid beyond its first use in
 *    initialization. This is because the handler needs access to the callback
 *    function. If reference isn't valid anymore, then all dma transfers after
 *    the first one will fail.
 *****************************************************************************/
void initTransferDma(void)
{
	// Callback configuration for TX
	static DMA_CB_TypeDef callbackTX;
	callbackTX.cbFunc = (DMA_FuncPtr_TypeDef) refreshTxTransfer;
	callbackTX.userPtr = NULL;

	// Channel configuration for TX transmission
	DMA_CfgChannel_TypeDef channelConfigTX;
	channelConfigTX.highPri   = false;                // Set high priority for the channel
	channelConfigTX.enableInt = false;                // Interrupt used to reset the transfer
	channelConfigTX.select    = DMAREQ_USART1_TXBL;   // Select DMA trigger
	channelConfigTX.cb        = &callbackTX;  	    // Callback to refresh the DMA transfer
	uint32_t channelNumTX     = 1;
	DMA_CfgChannel(channelNumTX, &channelConfigTX);

	// Channel descriptor configuration for TX transmission
	static DMA_CfgDescr_TypeDef descriptorConfigTX;
	descriptorConfigTX.dstInc  = dmaDataIncNone;  // Destination doesn't move
	descriptorConfigTX.srcInc  = dmaDataInc1;     // Source doesn't move
	descriptorConfigTX.size    = dmaDataSize1;    // Transfer 8 bits each time
	descriptorConfigTX.arbRate = dmaArbitrate1;   // Arbitrate after every DMA transfer
	descriptorConfigTX.hprot   = 0;               // Access level/protection not an issue
	bool isPrimaryDescriptor = true;
	DMA_CfgDescr(channelNumTX, isPrimaryDescriptor, &descriptorConfigTX);

	bool isUseBurst = false;
	DMA_ActivateBasic(channelNumTX,
					isPrimaryDescriptor,
					isUseBurst,
					(void *) &USART1->TXDATA,  // Destination address to transfer to
					(void *) TxBuffer,         // Source address to transfer from
					TX_BUFFER_SIZE - 1);       // Number of DMA transfers minus 1
}

/**************************************************************************//**
 * @brief
 *    Initialize the DMA module to receive packets via USART and transfer
 *    them to a buffer
 *****************************************************************************/
void initReceiveDma(void)
{
	// Callback configuration for RX
	static DMA_CB_TypeDef callbackRX;
	callbackRX.cbFunc = (DMA_FuncPtr_TypeDef) refreshRxTransfer;
	callbackRX.userPtr = NULL;

	// Channel configuration for RX transmission
	DMA_CfgChannel_TypeDef channelConfigRX;
	channelConfigRX.highPri   = false;                    // Set high priority for the channel
	channelConfigRX.enableInt = false;                    // Interrupt used to reset the transfer
	channelConfigRX.select    = DMAREQ_USART1_RXDATAV;    // Select DMA trigger
	channelConfigRX.cb        = &callbackRX;  	        // Callback to refresh the DMA transfer
	uint32_t channelNumRX     = 0;
	DMA_CfgChannel(channelNumRX, &channelConfigRX);

	// Channel descriptor configuration for RX transmission
	static DMA_CfgDescr_TypeDef descriptorConfigRX;
	descriptorConfigRX.dstInc  = dmaDataInc1;     // Destination moves one spot in the buffer every transmit
	descriptorConfigRX.srcInc  = dmaDataIncNone;  // Source doesn't move
	descriptorConfigRX.size    = dmaDataSize1;    // Transfer 8 bits each time
	descriptorConfigRX.arbRate = dmaArbitrate1;   // Arbitrate after every DMA transfer
	descriptorConfigRX.hprot   = 0;               // Access level/protection not an issue
	bool isPrimaryDescriptor = true;
	DMA_CfgDescr(channelNumRX, isPrimaryDescriptor, &descriptorConfigRX);

	// Activate basic DMA cycle (used for memory-peripheral transfers)
	bool isUseBurst = false;
	DMA_ActivateBasic(channelNumRX,
					isPrimaryDescriptor,
					isUseBurst,
					(void *) RxBuffer,         // Destination address to transfer to
					(void *) &USART1->RXDATA,  // Source address to transfer from
					RX_BUFFER_SIZE - 1);       // Number of DMA transfers minus 1
}

/**************************************************************************//**
 * @brief Initialize USART1
 *****************************************************************************/
void initUSART1 (void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	// Configure GPIO mode
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1); //tx
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 1);    //rx
	GPIO_PinModeSet(gpioPortB, 7, gpioModePushPull, 0); //clk
	GPIO_PinModeSet(gpioPortB, 8, gpioModePushPull, 1); //cs

	// Start with default config, then modify as necessary
	USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
	//config.master       = true;            // master mode
	config.baudrate     = 1000000;         // CLK freq is 1 MHz
	config.autoCsEnable = true;            // CS pin controlled by hardware, not firmware
	config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
	config.msbf         = true;            // send MSB first
	config.enable       = usartDisable;    // making sure USART isn't enabled until we set it up
	USART_InitSync(USART1, &config);

	// Set and enable USART pin locations
	USART1->ROUTE = USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | USART_ROUTE_TXPEN
		| USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC0;

	// Enable USART1
	//USART_Enable(USART1, usartEnable);
	USART1->CMD |= (usartEnable | USART_CMD_MASTEREN);
}

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
	// Initialize chip
	CHIP_Init();

	// Initialize USART1 as SPI slave
	initUSART1();

	// Initializing the DMA
	DMA_Init_TypeDef init;
	init.hprot = 0;                      // Access level/protection not an issue
	init.controlBlock = dmaControlBlock; // Make sure control block is properly aligned
	DMA_Init(&init);

	// Setup LDMA channels for transfer across SPI
	initReceiveDma();
	initTransferDma();

	// Place breakpoint here and observe RxBuffer
	// RxBuffer should contain 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9
	while(1);
}

#elif 0

#define TX_BUFFER_SIZE   10
#define RX_BUFFER_SIZE   TX_BUFFER_SIZE

uint8_t TxBuffer[TX_BUFFER_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint32_t TxBufferIndex = 0;

uint8_t RxBuffer[RX_BUFFER_SIZE] = {0};
uint32_t RxBufferIndex = 0;

/**************************************************************************//**
 * @brief USART1 TX IRQ Handler
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  // Send and receive incoming data
  USART_Tx(USART1, TxBuffer[TxBufferIndex++]);

  // Stop sending once we've gone through the whole TxBuffer
  if (TxBufferIndex == TX_BUFFER_SIZE)
  {
    TxBufferIndex = 0;
  }
}

/**************************************************************************//**
 * @brief USART1 RX IRQ Handler
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  // Send and receive incoming data
  RxBuffer[RxBufferIndex++] = USART_Rx(USART1);

  // Stop once we've filled up our buffer
  if (RxBufferIndex == RX_BUFFER_SIZE)
  {
    // Place breakpoint here and observe RxBuffer
    // RxBuffer should contain 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9
    RxBufferIndex = 0;
  }
}

/**************************************************************************//**
 * @brief Initialize USART1
 *****************************************************************************/
void initUSART1 (void)
{
	usart_spi = USART1;

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	// Configure GPIO mode
	//  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0); // US1_CLK is push pull
	//  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1);  // US1_CS is push pull
	//  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 1); // US1_TX (MOSI) is push pull
	//  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 1);    // US1_RX (MISO) is input

	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1); //tx
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 1);    //rx
	GPIO_PinModeSet(gpioPortB, 7, gpioModePushPull, 0); //clk
	GPIO_PinModeSet(gpioPortB, 8, gpioModePushPull, 1); //cs

	// Start with default config, then modify as necessary
	USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
	config.master       = true;            // master mode
	config.baudrate     = 1000000;         // CLK freq is 1 MHz
	//config.autoCsEnable = true;            // CS pin controlled by hardware, not firmware
	config.autoTx = true;	// AUTOTX
	config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
	config.msbf         = true;            // send MSB first
	config.enable       = usartDisable;    // Make sure to keep USART disabled until it's all set up
	USART_InitSync(USART1, &config);

	// Set and enable USART pin locations
	USART1->ROUTE = USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC0;

	// Enabling TX interrupts to transfer whenever
	// there is room in the transmit buffer
	USART_IntClear(USART1, USART_IF_TXBL);
	USART_IntEnable(USART1, USART_IF_TXBL);
	NVIC_ClearPendingIRQ(USART1_TX_IRQn);
	//NVIC_EnableIRQ(USART1_TX_IRQn);

	// Enabling RX interrupts to trigger whenever
	// there is new data from the slave
	USART_IntClear(USART1, USART_IF_RXDATAV);
	USART_IntEnable(USART1, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART1_RX_IRQn);
	//NVIC_EnableIRQ(USART1_RX_IRQn);

	// Enable USART1
	//USART_Enable(USART1, usartEnable);
	dwReadID(&g_dwDev);
}

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
  // Initialize chip
  CHIP_Init();

  // Initialize USART1 as SPI master
  initUSART1();

  // After the USART is initialized transfers will automatically start based off of certain interrupt flag conditions
  // A new packet will be sent to the slave whenever the TX Buffer is empty after a previous successful transfer
  // and a packet will be sent from the slave back whenever it receives a packet from the master and will be handled
  // by the USART_RX_IRQHandler
  while(1);
}

#elif 0

int main(void)
{
	/*
	 * Chip errata
	 * */
	CHIP_Init();

	/*
	 * global var init
	 * */
	globalInit();

	/*
	 * power supply for AD and UWB
	 * */
	powerADandUWB(1);

	/*
	 * config needed clock
	 * */
	clockConfig();

	/*
	 * RS422 Uart init for delivering converted data
	 * */
	uartSetup();

	/*
	 * SPI master config
	 * */
	SPIConfig(SPI_CLK);

	dwReadID(&g_dwDev);

	/*
	 * config one DMA channel for transferring ADC sample results
	 * to specific RAM buffer.
	 * */
	//DMAConfig();

	/*
	 * DW100 wireless device init
	 * */
	dwDeviceInit(&g_dwDev);

  	UDELAY_Calibrate();
  	Delay_ms(500);

	while (1) {
		switch(g_cur_mode)
		{
			case MAIN_WKUPMODE:
				WakeupSlave(&g_dwDev);
				break;

			case MAIN_SAMPLEMODE:
				RecvFromSlave(&g_dwDev);
				break;

			case MAIN_IDLEMODE:
				g_cur_mode = MAIN_WKUPMODE;
				break;

			default:
				g_cur_mode = MAIN_WKUPMODE;
				break;
		}

		Delay_ms(2);
	}
}
#endif
