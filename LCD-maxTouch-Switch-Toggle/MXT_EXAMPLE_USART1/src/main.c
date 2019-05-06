#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"

typedef struct {
	 const uint8_t *data;
	 uint16_t width;
	 uint16_t height;
	 uint8_t dataSize;
 } tImage;
 
#include "img/ffimg.h"
#include "img/fullimg.h"
#include "img/muteimg.h"
#include "img/pauseimg.h"
#include "img/playimg.h"
#include "img/rewindimg.h"
#include "img/skipimg.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

#define VOLT_REF (3300)
#define MAX_DIGITAL (4095)

#define TREM_PIO PIOA
#define TREM_PIO_ID ID_PIOA
#define TREM_PIO_PIN 19                                                                                                                                                  
#define TREM_PIO_PIN_MASK (1u << TREM_PIO_PIN)

#define POT_PIO PIOA
#define POT_PIO_ID ID_PIOA
#define POT_PIO_PIN 0
#define POT_PIO_PIN_MASK (1u << POT_PIO_PIN)

#define LED_PIO           PIOC
#define LED_PIO_ID        ID_PIOC
#define LED_PIO_PIN       8u
#define LED_PIO_PIN_MASK  (1u << LED_PIO_PIN)



/************************************************************************/
/* constants                                                            */
/************************************************************************/

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

const uint32_t RAIO = 40;
const uint32_t B1_X = ILI9488_LCD_WIDTH/2;
const uint32_t B1_Y = 60;

const uint32_t B2_X = ILI9488_LCD_WIDTH/3-15;
const uint32_t B2_Y = 170;

const uint32_t B3_X = ILI9488_LCD_WIDTH*2/3+15;
const uint32_t B3_Y = 170;

const uint32_t B4_X = ILI9488_LCD_WIDTH/3-15;
const uint32_t B4_Y = 280;

const uint32_t B5_X = ILI9488_LCD_WIDTH*2/3+15;
const uint32_t B5_Y = 280;

const uint32_t B6_X = ILI9488_LCD_WIDTH/2;
const uint32_t B6_Y = 390;


/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile long g_systimer = 0;
volatile uint32_t last_status = 0;
volatile Bool pause = false;
volatile Bool press = false;
volatile Bool cd = false;

volatile Bool redraw = false;

volatile Bool time_flag = false;


volatile char pause_play = '0';
volatile char rrewind = '0';
volatile char ff = '0';
volatile char skip = '0';
volatile char fullscreen = '0';
volatile char mute = '0';

volatile bool g_is_conversion_done_pot = false;
volatile uint32_t g_ul_value_pot = 0;

#define AFEC_CHANNEL_POT 5

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

static void configure_lcd();
static void mxt_init(struct mxt_device *device);
void draw_screen();
uint32_t convert_axis_system_x(uint32_t touch_y);
uint32_t convert_axis_system_y(uint32_t touch_x);
void update_screen(uint32_t tx, uint32_t ty);
void SysTick_Handler();
void config_console();
void usart_put_string(Usart *usart, char str[]);
int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms);
void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout);
void usart_log(char* name, char* log);
void hc05_config_server();
int hc05_server_init();


/************************************************************************/
/* Handlers                                                             */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

static int32_t convert_adc_to_temp(int32_t ADC_value){

  int32_t ul_vol;

  /*
   * converte bits -> tensão (Volts)
   */
	ul_vol = ADC_value * 100 / 4096;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  //ul_temp = (ul_vol - 720)  * 100 / 233 + 27;
  return(ul_vol);
}

static void AFEC_Pot_callback(void)
{
	g_ul_value_pot = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT);
	g_is_conversion_done_pot = true;
}

static void config_ADC_POT(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_5,	AFEC_Pot_callback, 1);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_POT, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_POT, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT);
}
/**
*  Interrupt handler for TC1 interrupt.
*/
void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT);

	afec_start_software_conversion(AFEC0);
}


void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	* maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
		
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		// eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.y);
		uint32_t conv_y = convert_axis_system_y(touch_event.x);
		
		
		/* Format a new entry in the data string that will be sent over USART */
		//sprintf(buf, "Nr: %1d, X:%4d, Y:%4d, Status: %d conv X:%3d Y:%3d\n\r\n", touch_event.id, touch_event.x, touch_event.y, touch_event.status, conv_x, conv_y);
		
		if(last_status <= 60){
			update_screen(conv_x, conv_y);
			press = true;
		}
		last_status = touch_event.status;

		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		* if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}

void SysTick_Handler() {
	g_systimer++;
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	time_flag = true;
	tc_stop(TC0, 1);
	cd = false;
	//pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);	
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void update_screen(uint32_t tx, uint32_t ty) {
	if(!cd){
		if(tx >= B1_X-RAIO && tx <= B1_X+RAIO) {
			if(ty >= B1_Y-RAIO && ty <= B1_Y+RAIO) {
				tc_start(TC0, 1);
				cd = true;
				pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
				pin_toggle(TREM_PIO, TREM_PIO_PIN_MASK);
				if(!pause){
					pause = true;
				}
				else{
					pause = false;
				}
				pause_play = '1';
				redraw = true;
			}
		}
		
		if(tx >= B2_X-RAIO && tx <= B2_X+RAIO) {
			if(ty >= B2_Y-RAIO && ty <= B2_Y+RAIO) {
				tc_start(TC0, 1);
				cd = true;
				pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
				pin_toggle(TREM_PIO, TREM_PIO_PIN_MASK);
				rrewind = '1';
				redraw = true;
			}
		}
		
		if(tx >= B3_X-RAIO && tx <= B3_X+RAIO) {
			if(ty >= B3_Y-RAIO && ty <= B3_Y+RAIO) {
				tc_start(TC0, 1);
				cd = true;
				pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
				pin_toggle(TREM_PIO, TREM_PIO_PIN_MASK);
				ff = '1';
				redraw = true;
			}
		}
		
		if(tx >= B4_X-RAIO && tx <= B4_X+RAIO) {
			if(ty >= B4_Y-RAIO && ty <= B4_Y+RAIO) {
				tc_start(TC0, 1);
				cd = true;
				pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
				pin_toggle(TREM_PIO, TREM_PIO_PIN_MASK);
				mute = '1';
				redraw = true;
			}
		}
		
		if(tx >= B5_X-RAIO && tx <= B5_X+RAIO) {
			if(ty >= B5_Y-RAIO && ty <= B5_Y+RAIO) {
				tc_start(TC0, 1);
				cd = true;
				pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
				pin_toggle(TREM_PIO, TREM_PIO_PIN_MASK);
				fullscreen = '1';
				redraw = true;
			}
		}
		if(tx >= B6_X-RAIO && tx <= B6_X+RAIO) {
			if(ty >= B6_Y-RAIO && ty <= B6_Y+RAIO) {
				tc_start(TC0, 1);
				cd = true;
				pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
				pin_toggle(TREM_PIO, TREM_PIO_PIN_MASK);
				skip = '1';
				redraw = true;
			}
		}
		
	}
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEsub2pewds", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN0000", 1000);
	usart_log("hc05_server_init", buffer_rx);
}

void draw_buttons(){
	if(!pause){
		ili9488_draw_pixmap(B1_X-playimg.width/2, B1_Y-playimg.height/2, playimg.width, playimg.height+2, playimg.data);
	}
	else{
		ili9488_draw_pixmap(B1_X-pauseimg.width/2, B1_Y-pauseimg.height/2, pauseimg.width, pauseimg.height+2, pauseimg.data);
	}
	
	ili9488_draw_pixmap(B2_X-rewindimg.width/2, B2_Y-rewindimg.height/2, rewindimg.width, rewindimg.height+2, rewindimg.data);
	
	
	ili9488_draw_pixmap(B3_X-ffimg.width/2, B3_Y-ffimg.height/2, ffimg.width, ffimg.height+2, ffimg.data);
	
	ili9488_draw_pixmap(B4_X-muteimg.width/2, B4_Y-muteimg.height/2, muteimg.width, muteimg.height+2, muteimg.data);
	
	ili9488_draw_pixmap(B5_X-fullimg.width/2, B5_Y-fullimg.height/2, fullimg.width, fullimg.height+2, fullimg.data);
	
	ili9488_draw_pixmap(B6_X-skipimg.width/2, B6_Y-skipimg.height/2, skipimg.width, skipimg.height+2, skipimg.data);
}

void trem_toggle(Pio *pio, uint32_t mask){
	if(!pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/

int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	TC_init(TC0, ID_TC1, 1, 10);
	
	TC_init(TC0, ID_TC2, 2, 10);

	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
		
	#ifndef DEBUG_SERIAL
	printf("Inicializando...\r\n");
	printf("Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	configure_lcd();
	draw_screen();
	draw_buttons();
	
	/* Initialize the mXT touch device */
	mxt_init(&device);
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIO_PIN_MASK, 1, 0, 0 );
	
	pmc_enable_periph_clk(TREM_PIO_ID);
	pio_set_output(TREM_PIO, TREM_PIO_PIN_MASK, 0, 0, 0 );
	
	
	config_ADC_POT();
	/* incializa conversão ADC */
	afec_start_software_conversion(AFEC0);

	char eof = 'X';
	char buffer[1024];
	
	while (true) {
		/* Check for any pending messages and run message handler if any
		 * message is found in the queue */
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device);
		}
		
		if(redraw){
			draw_buttons();
			redraw = false;
		}
		if (time_flag){
			pin_toggle(LED_PIO, LED_PIO_PIN_MASK);
			pin_toggle(TREM_PIO, TREM_PIO_PIN_MASK);
			time_flag = false;
		}
		
		if (press){
			const pot_val = convert_adc_to_temp(g_ul_value_pot);
			
			
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, (uint32_t) pause_play);
			printf("%c", pause_play);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM,  (uint32_t)  rrewind);
			printf("%c", rrewind);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM,  (uint32_t) ff);
			printf("%c", ff);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM,  (uint32_t) skip);
			printf("%c", skip);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM,  (uint32_t) fullscreen);
			printf("%c", fullscreen);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM,  (uint32_t) mute);
			printf("%c", mute);
			//if (pot_val < 10){
				//while(!usart_is_tx_ready(UART_COMM));
			 	//usart_write(UART_COMM,  (uint32_t) 0);
			 	//printf("%d",'0') ;
			//}
			while(!usart_is_tx_ready(UART_COMM));
 			usart_write(UART_COMM,  (uint32_t) pot_val);
 			printf("%d",pot_val) ;
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM,  (uint32_t)  eof);
			printf("%c", eof);

			press = false;
			
		}
		
		
		
		pause_play = '0';
		rrewind = '0';
		ff = '0';
		skip = '0';
		fullscreen = '0';
		mute = '0';
	}

	return 0;
}
