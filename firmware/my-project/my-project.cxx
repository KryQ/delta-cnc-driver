//Only for intelisense to stop tripping
#define STM32F7

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/crc.h>

#include "defines.h"
#include "stepgen.h"

#include "string.h"
#include "math.h"

class COMMANDS
{
public:
	static const uint32_t CFG = 0x21314101;
	static const uint32_t MOT = 0x21314102;
	static const uint32_t OUT = 0x21314103;
	static const uint32_t SPIN = 0x21314104;

	static const uint32_t RESET_BOARD = 0x5453523E;
	static const uint32_t CM1 = 0x314D433E;
	static const uint32_t CM2 = 0x324D433E;
	static const uint32_t CM3_PWM1 = 0x334D433E;
	static const uint32_t CM3_PWM2 = 0x344D433E;
	static const uint32_t CM3_PWM3 = 0x354D433E;
	static const uint32_t CM3_PWM4 = 0x364D433E;

	static const uint32_t TST = 0x5453543E;
};

class Spindle
{
public:
	bool isEnabled = false;
	uint8_t direction = 0; //0 = CW, 1 = CCW;
	uint16_t channel_duty = 2000;
	uint32_t desired_speed = 0, actual_speed = 0;
	float pid_integral = 0, pid_error = 0;
	volatile uint32_t measure_delay_counter = 0, control_delay_counter = 0, freq_counter = 0;
	const uint8_t desired_measure_interval = 100, desired_control_interval = 100;
	const float Kp = 0.25, Ki = 0.000025, Kd = 0.008;

	void setup()
	{
		/**
	 * PB5 - Spindle speed input
	 * PB7 - Spindle enable
	 * PB10 - Spindle set speed TIM2 CH3
	 * PB11 - Spindle direction
	 */

		//Setup Spindle speed interrupt
		rcc_periph_clock_enable(RCC_GPIOB);
		gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5);
		nvic_enable_irq(NVIC_EXTI9_5_IRQ);
		exti_enable_request(EXTI5);
		exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
		exti_select_source(EXTI5, GPIOB);

		gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11 | GPIO7);
		gpio_clear(GPIOB, GPIO11 | GPIO7);

		gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
		gpio_set_af(GPIOB, GPIO_AF1, GPIO10);

		rcc_periph_clock_enable(RCC_TIM2);
		timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT_MUL_4, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
		timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM2);
		timer_enable_oc_output(TIM2, TIM_OC3);
		timer_enable_break_main_output(TIM2);
		timer_set_oc_value(TIM2, TIM_OC3, 4000);
		timer_set_period(TIM2, 10000);
		timer_enable_counter(TIM2);
	}

	void enable()
	{
		if (!this->isEnabled)
		{
			this->isEnabled = true;
			this->channel_duty = 3800;
			timer_set_oc_value(TIM2, TIM_OC3, this->channel_duty);

			gpio_set(GPIOB, GPIO7);
		}
	}

	void disable()
	{
		if (this->isEnabled)
		{
			this->isEnabled = false;
			gpio_clear(GPIOB, GPIO7);
		}
	}

	void setDirection(uint8_t dir)
	{
		if (dir)
		{
			this->direction = 1;
			gpio_clear(GPIOB, GPIO11);
		}
		else
		{
			this->direction = 0;
			gpio_set(GPIOB, GPIO11);
		}
	}

	void setSpeed(uint32_t speed)
	{
		this->desired_speed = speed;
	}

	uint32_t getActualSpeed() { return this->actual_speed; }

	void calculateRPM()
	{
		if (this->measure_delay_counter >= this->desired_measure_interval)
		{
			this->measure_delay_counter = 0;

			/**
			 * 3330 -> 942
			 * 4305 -> 1200
			 * 
			 */
			//The output frequency is set to P (Hz), the motor class logarithm is N, and the speed is F (RPM), then the output speed frequency is P=F*N/60.

			this->actual_speed = (float)this->freq_counter * 600 / 2; //WAS 7
			this->freq_counter = 0;
		}
	}

	void controlSpeed()
	{
		if (this->control_delay_counter >= this->desired_control_interval && this->isEnabled)
		{
			this->control_delay_counter = 0;

			float error = (float)this->desired_speed - this->actual_speed;								// determine error
			this->pid_integral += error * this->desired_control_interval;									// compute integral
			float rateError = (error - this->pid_error) / this->desired_control_interval; // compute derivative

			float out = this->Kp * error + this->Ki * this->pid_integral + this->Kd * rateError; //PID output

			this->pid_error = error; //remember current error

			this->channel_duty = this->channel_duty + out;

			timer_set_oc_value(TIM2, TIM_OC3, this->channel_duty);
		}
	}
};

bool configured = false, timeouted = false;
volatile bool rxBufComplete = false, priRxBuffer = false, priTxBuffer = false, doStep = false;
volatile uint32_t rx1Buf[BUFSIZE] = {0}, rx2Buf[BUFSIZE] = {0}, tx1Buf[BUFSIZE] = {0}, tx2Buf[BUFSIZE] = {0};
uint32_t spiTimeout = 0;

volatile uint16_t inputStatus = 0;
Spindle spindle;

void dma2_stream0_isr()
{
	if (dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TCIF))
	{
		dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_TCIF);
		/* Toggle PC1 just to keep aware of activity and frequency. */

		rxBufComplete = true;
		priRxBuffer = !priRxBuffer;
	}

	if (dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_HTIF))
	{
		dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_HTIF);
	}
}

void dma2_stream3_isr()
{
	if (dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_TCIF))
	{
		dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_TCIF);
		priTxBuffer = !priTxBuffer;
		//memcpy((void *)btxBuf, (void *)txBuf, SPIBUFSIZE);
	}
}

void exti0_isr(void)
{
	if (exti_get_flag_status(EXTI0))
	{
		exti_reset_request(EXTI0);
		gpio_get(Y_MIN_PORT, Y_MIN_PIN)
				? inputStatus &= ~(1 << 2)
				: inputStatus |= (1 << 2);
	}
}

void exti1_isr(void)
{
	if (exti_get_flag_status(EXTI1))
	{
		exti_reset_request(EXTI1);
		gpio_get(Y_MIN_PORT, Y_MAX_PIN)
				? inputStatus &= ~(1 << 3)
				: inputStatus |= (1 << 3);
	}
}

void exti2_isr(void)
{
	if (exti_get_flag_status(EXTI2))
	{
		exti_reset_request(EXTI2);
		gpio_get(Z_MIN_PORT, Z_MIN_PIN)
				? inputStatus &= ~(1 << 4)
				: inputStatus |= (1 << 4);
	}
}

void exti3_isr(void)
{
	if (exti_get_flag_status(EXTI3))
	{
		exti_reset_request(EXTI3);
		gpio_get(Z_MAX_PORT, Z_MAX_PIN)
				? inputStatus &= ~(1 << 5)
				: inputStatus |= (1 << 5);
	}
}

void exti9_5_isr(void)
{
	if (exti_get_flag_status(EXTI5))
	{
		exti_reset_request(EXTI5);
		spindle.freq_counter++;
	}
}

void exti15_10_isr(void)
{
	if (exti_get_flag_status(EXTI15))
	{
		exti_reset_request(EXTI15);
		gpio_get(X_MAX_PORT, X_MAX_PIN)
				? inputStatus &= ~(1 << 1)
				: inputStatus |= (1 << 1);
	}

	if (exti_get_flag_status(EXTI14))
	{
		exti_reset_request(EXTI14);
		gpio_get(X_MIN_PORT, X_MIN_PIN)
				? inputStatus &= ~(1 << 0)
				: inputStatus |= (1 << 0);
	}
}

void setup_outputs(void)
{
	/**
	 * OUTPUTS
	 * 
	 * PA8 A_STEP
	 * PA9 A_DIR
	 * PA10 Z_STEP
	 * PA11 Z_DIR
	 * PA12 Y_STEP
	 * PA15 Y_DIR
	 * PC6 5_OUT
	 * PC7 B_STEP
	 * PC8 B_DIR
	 * PC9 STEPPER_EN
	 * PC10 X_STEP
	 * PC11 X_DIR
	 * PC12 LED_1
	 * 
	 * PB3 LED_3
	 * PB12 1_OUT
	 * PB13 2_OUT
	 * PB14 3_OUT
	 * PB15 4_OUT
	 * 
	 * PD2 LED_2
	 * 
	 */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO12 | GPIO11 | GPIO10 | GPIO9 | GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO12 | GPIO10 | GPIO8);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12 | GPIO3);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO11 | GPIO10 | GPIO9 | GPIO8 | GPIO7 | GPIO6);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO10 | GPIO7);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);

	gpio_set(GPIOC, GPIO12);
	gpio_set(GPIOD, GPIO2);
	gpio_set(GPIOB, GPIO3);
}

void setup_inputs(void)
{
	/**
	 * INPUT PORTS PINS
	 * 
	 * PA0 - B_LIM_MIN  1
	 * PA1 - A_LIM_MAX  3
	 * PA2 - A_LIM_MIN  2
	 * PA3 - B_LIM_MAX  0
	 * PC0 - Y_LIM_MIN  7 EXTI0
	 * PC1 - Y_LIM_MAX  6 EXTI1
	 * PC2 - Z_LIM_MIN  5 EXTI2
	 * PC3 - Z_LIM_MAX  4 EXTI3
	 * PC14 - X_LIM_MIN 9 EXTI14
	 * PC15 - X_LIM_MAX 8 EXTI15
	 * 
	 */

	//rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);

	//gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO3 | GPIO2 | GPIO1 | GPIO0);
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO15 | GPIO14 | GPIO3 | GPIO2 | GPIO1 | GPIO0);

	rcc_periph_clock_enable(RCC_SYSCFG);

	nvic_enable_irq(NVIC_EXTI0_IRQ);
	nvic_enable_irq(NVIC_EXTI1_IRQ);
	nvic_enable_irq(NVIC_EXTI2_IRQ);
	nvic_enable_irq(NVIC_EXTI3_IRQ);
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);

	exti_enable_request(EXTI15 | EXTI14 | EXTI3 | EXTI2 | EXTI1 | EXTI0);
	exti_set_trigger(EXTI15 | EXTI14 | EXTI3 | EXTI2 | EXTI1 | EXTI0, EXTI_TRIGGER_BOTH);
	exti_select_source(EXTI15, GPIOC);
	exti_select_source(EXTI14, GPIOC);
	exti_select_source(EXTI3, GPIOC);
	exti_select_source(EXTI2, GPIOC);
	exti_select_source(EXTI1, GPIOC);
	exti_select_source(EXTI0, GPIOC);
}

void sys_tick_handler(void)
{
	spiTimeout++;
	spindle.measure_delay_counter++;
	spindle.control_delay_counter++;
	//stepgen();
}

static void setup_systick(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(216000000 / 1000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

//todo: READ INPUTS
uint32_t read_inputs()
{
	uint32_t x = 0;

	x |= gpio_get(X_MIN_PORT, X_MIN_PIN) ? 1 : 0;
	x |= gpio_get(X_MAX_PORT, X_MAX_PIN) ? 1 << 1 : 0;
	x |= gpio_get(Y_MIN_PORT, Y_MIN_PIN) ? 1 << 2 : 0;
	x |= gpio_get(Y_MAX_PORT, Y_MAX_PIN) ? 1 << 3 : 0;
	x |= gpio_get(Z_MIN_PORT, Z_MIN_PIN) ? 1 << 4 : 0;
	x |= gpio_get(Z_MAX_PORT, Z_MAX_PIN) ? 1 << 5 : 0;
	x |= gpio_get(A_MIN_PORT, A_MIN_PIN) ? 1 << 6 : 0;
	x |= gpio_get(A_MAX_PORT, A_MAX_PIN) ? 1 << 7 : 0;
	x |= gpio_get(B_MIN_PORT, B_MIN_PIN) ? 1 << 8 : 0;
	x |= gpio_get(B_MAX_PORT, B_MAX_PIN) ? 1 << 9 : 0;

	return x ^ ~0;
}

void update_outputs(uint32_t outputs)
{
	if (outputs & 0x0001)
	{
		gpio_set(OUT_1_PORT, OUT_1_PIN);
	}
	else
	{
		gpio_clear(OUT_1_PORT, OUT_1_PIN);
	}

	if (outputs & 0x0002)
	{
		gpio_set(OUT_2_PORT, OUT_2_PIN);
	}
	else
	{
		gpio_clear(OUT_2_PORT, OUT_2_PIN);
	}

	if (outputs & 0x0004)
	{
		gpio_set(OUT_2_PORT, OUT_2_PIN);
	}
	else
	{
		gpio_clear(OUT_2_PORT, OUT_2_PIN);
	}

	if (outputs & 0x0008)
	{
		gpio_set(OUT_3_PORT, OUT_3_PIN);
	}
	else
	{
		gpio_clear(OUT_3_PORT, OUT_3_PIN);
	}

	if (outputs & 0x0010)
	{
		gpio_set(OUT_4_PORT, OUT_4_PIN);
	}
	else
	{
		gpio_clear(OUT_4_PORT, OUT_4_PIN);
	}

	if (outputs & 0x0020)
	{
		gpio_set(OUT_5_PORT, OUT_5_PIN);
	}
	else
	{
		gpio_clear(OUT_5_PORT, OUT_5_PIN);
	}
}

void reset_board()
{
	stepgen_reset();
	spindle.disable();
}

void setup_spi()
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_SPI1);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
									GPIO7 | GPIO6 | GPIO5 | GPIO4);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO7 | GPIO6 | GPIO5 | GPIO4);

	spi_set_slave_mode(SPI1);
	spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_128);
	spi_set_clock_polarity_0(SPI1);
	spi_set_clock_phase_0(SPI1);
	spi_set_full_duplex_mode(SPI1);
	//spi_set_unidirectional_mode(SPI1); /* bidirectional but in 3-wire */
	spi_send_msb_first(SPI1);
	spi_enable_rx_dma(SPI1);
	spi_enable_tx_dma(SPI1);
	spi_enable(SPI1);
}

static void setup_dma(void)
{
	rcc_periph_clock_enable(RCC_DMA2);

	//RX SPI STREAM
	nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
	dma_stream_reset(DMA2, DMA_STREAM0);
	dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_HIGH);
	dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
	dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_32BIT);
	dma_enable_circular_mode(DMA2, DMA_STREAM0);
	dma_set_transfer_mode(DMA2, DMA_STREAM0,
												DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	/* The register to target is the DAC1 8-bit right justified data
	   register */
	dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t)&SPI1_DR);
	/* The array v[] is filled with the waveform data to be output */
	dma_enable_double_buffer_mode(DMA2, DMA_STREAM0);
	dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t)&rx1Buf);
	dma_set_memory_address_1(DMA2, DMA_STREAM0, (uint32_t)&rx2Buf);
	dma_set_number_of_data(DMA2, DMA_STREAM0, SPIBUFSIZE);

	dma_enable_circular_mode(DMA2, DMA_STREAM0);

	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
	//dma_enable_half_transfer_interrupt(DMA2, DMA_STREAM0);
	dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_3);
	dma_enable_stream(DMA2, DMA_STREAM0);

	//TX SPI STREAM
	nvic_enable_irq(NVIC_DMA2_STREAM3_IRQ);
	dma_stream_reset(DMA2, DMA_STREAM3);
	dma_set_priority(DMA2, DMA_STREAM3, DMA_SxCR_PL_MEDIUM);
	dma_set_memory_size(DMA2, DMA_STREAM3, DMA_SxCR_MSIZE_32BIT);
	dma_set_peripheral_size(DMA2, DMA_STREAM3, DMA_SxCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA2, DMA_STREAM3);
	dma_enable_circular_mode(DMA2, DMA_STREAM3);
	dma_enable_double_buffer_mode(DMA2, DMA_STREAM3);
	dma_set_transfer_mode(DMA2, DMA_STREAM3,
												DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_set_peripheral_address(DMA2, DMA_STREAM3, (uint32_t)&SPI1_DR);
	dma_set_memory_address(DMA2, DMA_STREAM3, (uint32_t)&tx1Buf);
	dma_set_memory_address_1(DMA2, DMA_STREAM3, (uint32_t)&tx2Buf);
	dma_set_number_of_data(DMA2, DMA_STREAM3, SPIBUFSIZE);

	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM3);
	dma_channel_select(DMA2, DMA_STREAM3, DMA_SxCR_CHSEL_3);
	dma_enable_stream(DMA2, DMA_STREAM3);
}

void tim1_brk_tim9_isr()
{
	if (timer_get_flag(TIM9, TIM_SR_UIF))
	{
		timer_clear_flag(TIM9, TIM_SR_UIF);

		//gpio_toggle(LED3_PORT, LED3_PIN);
		doStep = true;
		//stepgen();
	}
}

void setup_timer()
{
	//nvic_set_priority(NVIC_TIM1_BRK_TIM9_IRQ, 0x10);
	nvic_enable_irq(NVIC_TIM1_BRK_TIM9_IRQ);
	rcc_periph_clock_enable(RCC_TIM9);
	timer_set_mode(TIM9, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_continuous_mode(TIM9);
	timer_set_period(TIM9, 1079); //1079 for 200kHz stepgen
	timer_enable_counter(TIM9);
	timer_enable_irq(TIM9, TIM_DIER_UIE);
}

void setup_crc()
{
	rcc_periph_clock_enable(RCC_CRC);
	crc_set_initial(0xffffffffl);
	crc_set_polysize(CRC_CR_POLYSIZE_32);
	crc_reverse_output_enable();
	crc_set_reverse_input(CRC_CR_REV_IN_BYTE);
}

int main(void)
{
	rcc_clock_setup_hsi(&rcc_3v3[RCC_CLOCK_3V3_216MHZ]);
	flash_art_enable();

	setup_outputs();
	setup_inputs();
	setup_systick();
	setup_timer();

	setup_spi();
	setup_dma();
	setup_crc();
	spindle.setup();

	inputStatus = read_inputs();

	uint32_t *txBufPtr = (uint32_t *)tx1Buf;
	uint32_t *rxBufPtr = (uint32_t *)rx1Buf;

	while (1)
	{
		if (rxBufComplete)
		{
			rxBufComplete = false;

			if (priRxBuffer)
				rxBufPtr = (uint32_t *)rx1Buf;
			else
				rxBufPtr = (uint32_t *)rx2Buf;

			if (priTxBuffer)
				txBufPtr = (uint32_t *)tx1Buf;
			else
				txBufPtr = (uint32_t *)tx2Buf;

			crc_reset();
			if (rxBufPtr[6] == (crc_calculate_block((uint32_t *)rxBufPtr, 6) ^ ~0))
			{
				txBufPtr[0] = rxBufPtr[0] ^ ~0;
				spiTimeout = 0;

				switch (rxBufPtr[0])
				{
				case COMMANDS::RESET_BOARD:
					reset_board();
					break;
				case COMMANDS::MOT:
					stepgen_update_input((const void *)&rxBufPtr[1]);
					stepgen_get_position((void *)&txBufPtr[1]);
					//txBufPtr[4] = rxBufPtr[1];
					break;
				case COMMANDS::OUT:
					update_outputs(rxBufPtr[1]);
					txBufPtr[1] = inputStatus;
					break;
				case COMMANDS::SPIN:
					spindle.setDirection(rxBufPtr[1] & 0x02);
					(rxBufPtr[1] & 0x01)
							? spindle.enable()
							: spindle.disable();

					spindle.setSpeed(rxBufPtr[2]);
					txBufPtr[1] = spindle.getActualSpeed();
					break;
				case COMMANDS::CFG:
					stepgen_update_stepwidth(rxBufPtr[1]);
					stepgen_reset();

					configured = true;
					break;
				default:
					gpio_toggle(LED2_PORT, LED2_PIN);
					break;
				}

				crc_reset();
				txBufPtr[6] = crc_calculate_block((uint32_t *)txBufPtr, 6) ^ ~0;
			}
			else
				gpio_clear(LED3_PORT, LED3_PIN);
		}

		if (doStep)
		{
			doStep = false;
			stepgen();
		}

		//configured ? gpio_clear(LED2_PORT, LED2_PIN) : gpio_set(LED2_PORT, LED2_PIN);

		spindle.calculateRPM();
		spindle.controlSpeed();

		if (spiTimeout > 200)
		{
			spiTimeout = 0;
			reset_board();

			gpio_toggle(LED1_PORT, LED1_PIN);
		}
	}
}
