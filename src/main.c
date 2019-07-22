#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_cmu.h"
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
		if (!g_slaveWkup)
			WakeupSlave(&g_dwDev);
		else
			RecvFromSlave(&g_dwDev);

		Delay_ms(2);
	}
}





