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

const uint32_t B6_X = ILI9488_LCD_WIDTH/3-15;
const uint32_t B6_Y = 390;

const uint32_t B7_X = ILI9488_LCD_WIDTH*2/3+15;
const uint32_t B7_Y = 390;

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile long g_systimer = 0;
volatile uint32_t last_status = 0;
volatile Bool pause = false;
volatile Bool press = false;

volatile Bool redraw = false;

volatile char pause_play = '0';
volatile char rrewind = '0';
volatile char ff = '0';
volatile char skip = '0';
volatile char fullscreen = '0';
volatile char mute = '0';
volatile char nextep = '0';
	
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
	if(tx >= B1_X-RAIO && tx <= B1_X+RAIO) {
		if(ty >= B1_Y-RAIO && ty <= B1_Y+RAIO) {
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
			rrewind = '1';
			redraw = true;
		}
	}
	
	if(tx >= B3_X-RAIO && tx <= B3_X+RAIO) {
		if(ty >= B3_Y-RAIO && ty <= B3_Y+RAIO) {
			ff = '1';
			redraw = true;
		}
	}
	
	if(tx >= B4_X-RAIO && tx <= B4_X+RAIO) {
		if(ty >= B4_Y-RAIO && ty <= B4_Y+RAIO) {
			skip = '1';
			redraw = true;
		}
	}
	
	if(tx >= B5_X-RAIO && tx <= B5_X+RAIO) {
		if(ty >= B5_Y-RAIO && ty <= B5_Y+RAIO) {
			fullscreen = '1';
			redraw = true;
		}
	}
	if(tx >= B6_X-RAIO && tx <= B6_X+RAIO) {
		if(ty >= B6_Y-RAIO && ty <= B6_Y+RAIO) {
			mute = '1';
			redraw = true;
		}
	}
	if(tx >= B7_X-RAIO && tx <= B7_X+RAIO) {
		if(ty >= B7_Y-RAIO && ty <= B7_Y+RAIO) {
			nextep = '1';
			redraw = true;
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
		ili9488_draw_pixmap(B1_X-playimg.width/2, B1_Y-playimg.height/2, playimg.width, playimg.height, playimg.data);
	}
	else{
		ili9488_draw_pixmap(B1_X-pauseimg.width/2, B1_Y-pauseimg.height/2, pauseimg.width, pauseimg.height, pauseimg.data);
	}
	
	ili9488_draw_pixmap(B2_X-rewindimg.width/2, B2_Y-rewindimg.height/2, rewindimg.width, rewindimg.height, rewindimg.data);
	
	
	ili9488_draw_pixmap(B3_X-ffimg.width/2, B3_Y-ffimg.height/2, ffimg.width, ffimg.height, ffimg.data);
	
	ili9488_draw_pixmap(B4_X-skipimg.width/2, B4_Y-skipimg.height/2, skipimg.width, skipimg.height, skipimg.data);
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_MAGENTA));
	ili9488_draw_filled_circle(B5_X, B5_Y, RAIO);
	
	ili9488_draw_pixmap(B6_X-muteimg.width/2, B6_Y-muteimg.height/2, muteimg.width, muteimg.height, muteimg.data);
	
	ili9488_draw_pixmap(B7_X-skipimg.width/2, B7_Y-skipimg.height/2, skipimg.width, skipimg.height, skipimg.data);
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
		if (press){
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
		nextep = '0';
	}

	return 0;
}