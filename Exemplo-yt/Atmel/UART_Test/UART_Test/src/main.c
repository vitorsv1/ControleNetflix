/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/
// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

// Pause play
#define PP_PIO      PIOA
#define PP_PIO_ID   ID_PIOA
#define PP_IDX      4
#define PP_IDX_MASK (1 << PP_IDX)

// Rewind
#define REWIND_PIO      PIOA
#define REWIND_PIO_ID   ID_PIOA
#define REWIND_IDX      2
#define REWIND_IDX_MASK (1 << REWIND_IDX)

// FastFoward
#define FF_PIO      PIOA
#define FF_PIO_ID   ID_PIOA
#define FF_IDX      3
#define FF_IDX_MASK (1 << FF_IDX)

// Skip
#define SKIP_PIO      PIOA
#define SKIP_PIO_ID   ID_PIOA
#define SKIP_IDX      24
#define SKIP_IDX_MASK (1 << SKIP_IDX)

// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)


/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

volatile long g_systimer = 0;
volatile int pp_flag = 0;
volatile int rewind_flag = 0;
volatile int ff_flag = 0;
volatile int skip_flag = 0;
volatile int fullscreen_flag = 0;
volatile int mute_flag = 0;

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

void pp_callback(void){
	pp_flag = 1;
}
void rewind_callback(void){
	rewind_flag = 1;
}
void ff_callback(void){
	ff_flag = 1;
}
void skip_callback(void){
	skip_flag = 1;
}
void fullscreen_callback(void){
	fullscreen_flag = 1;
}
void mute_callback(void){
	mute_flag = 1;
}

/************************************************************************/
/* funções                                                              */
/************************************************************************/

void SysTick_Handler() {
	g_systimer++;
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
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN2502", 1000);
	usart_log("hc05_server_init", buffer_rx);
}

void io_init(void)
{

	// Configura led
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOC);
	
	pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT);
	
	pio_configure(PP_PIO, PIO_INPUT, PP_IDX_MASK, PIO_PULLUP);
	pio_configure(REWIND_PIO, PIO_INPUT, REWIND_IDX_MASK, PIO_PULLUP);
	pio_configure(FF_PIO, PIO_INPUT, FF_IDX_MASK, PIO_PULLUP);
	pio_configure(SKIP_PIO, PIO_INPUT, SKIP_IDX_MASK, PIO_PULLUP);

	// Ativa interrupção
	pio_enable_interrupt(PP_PIO, PP_IDX_MASK);
	pio_enable_interrupt(REWIND_PIO, REWIND_IDX_MASK);
	pio_enable_interrupt(FF_PIO, FF_IDX_MASK);
	pio_enable_interrupt(SKIP_PIO, SKIP_IDX_MASK);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(ID_PIOA);
	NVIC_SetPriority(ID_PIOA, 4); // Prioridade 4
	
	// Configura interrupção no o referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(PP_PIO, PP_PIO_ID, PP_IDX_MASK, PIO_IT_FALL_EDGE, pp_callback);
	pio_handler_set(REWIND_PIO, REWIND_PIO_ID, REWIND_IDX_MASK, PIO_IT_FALL_EDGE, rewind_callback);
	pio_handler_set(FF_PIO, FF_PIO_ID, FF_IDX_MASK, PIO_IT_FALL_EDGE, ff_callback);
	pio_handler_set(SKIP_PIO, SKIP_PIO_ID, SKIP_IDX_MASK, PIO_IT_FALL_EDGE, skip_callback);
}
/************************************************************************/
/* Main                                                                 */
/************************************************************************/

int main (void)
{
	
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	io_init();
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	char pause_play = '0';
	char rewind = '0';
	char ff = '0';
	char skip = '0';
	char fullscreen = '0';
	char mute = '0';
	
	char eof = 'X';
	char buffer[1024];
	
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		if(pp_flag == 1){
			pause_play = '1';/*
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, pause_play);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, eof);*/
			pause_play = '0';
			pp_flag = 0;
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
		}
		else if(rewind_flag == 1){
			rewind = '1';
			rewind = '0';
			rewind_flag = 0;
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
		}
		else if(ff_flag == 1){
			ff = '1';
			ff = '0';
			ff_flag = 0;
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
		}
		else if(skip_flag == 1){
			skip = '1';
			skip = '0';
			skip_flag = 0;
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(200);
		}
		if(fullscreen_flag){
			
			fullscreen_flag = 0;
		}
		if(mute_flag){
			
			mute_flag = 0;
		}
		
	}
}
