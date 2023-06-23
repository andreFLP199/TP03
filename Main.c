/*
 * TP02.c
 *
 * Created: 02/06/2023 14:15:21
 * Author : andre
 */ 

/*
 * Arquitetura de Computadores
 * Aula Pr?tica 03
 * Trabalho Pr?tico 2
 * Author : Rafael Baptista & Andr? Passaradas
 */ 

#include <avr/io.h>
#include "OS.h"
#include <util/delay.h>
#include <avr/interrupt.h>


// Vari?veis globais
uint8_t ledState = 0;			  // Estado atual do LED
uint8_t lastButtonDirState;	  // Estado anterior do bot?o direito
uint8_t currentButtonDirState;  // Estado atual do bot?o direito
uint8_t lastButtonEsqState;	  // Estado anterior do bot?o esquerdo
uint8_t currentButtonEsqState;  // Estado atual do bot?o esquerdo

/************************************************************************/
/* Initializes the ADC (Vref=AVcc, Prescaller=128) */
/************************************************************************/
void InitADC()
{
	ADMUX |= (1<<REFS0); // Select Vref=AVcc
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Prescaller=128
	ADCSRA |= (1<<ADEN); // Enable ADC
}
/************************************************************************/
/* Read ADC Channel (waits conversion complete) */
/************************************************************************/
uint16_t ReadADC(uint8_t channel)
{
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
	ADCSRA |= (1<<ADSC); // Single conversion mode
	while( ADCSRA & (1<<ADSC) ); // Wait ADC conversion
	ADCSRA |= (1<<ADIF); // Clear interrupt flag
	return ADC; // ADC = ADCH<<8 | ADCL;
}



// Fun??es de transmiss?o e recep??o UART

void UART_putc(uint8_t data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void UART_puts(char* s)
{
	while(*s > 0) UART_putc(*s++);
}

uint8_t UART_getc(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}


void UART_getLine(char* buf, uint8_t n)
{
uint8_t bufIdx = 0;
char c;
// while received character is not carriage return
// and end of buffer has not been reached
do
{
// receive character
c = UART_getc();
// store character in buffer
buf[bufIdx++] = c;
}
while((bufIdx < n) && (c != '\r'));
// ensure buffer is null terminated
buf[bufIdx] = 0;
}

void UART_newLine(){
	
	//UART_putc(0x0D);  //Carriage return (TAB)
	UART_putc(0x0A);    //Line Feed (Muda de linha)
	
}




// Inicializa??o da UART
void UART_Init( uint16_t UBRR) {
	UBRR0H = (uint8_t)(UBRR >> 8);
	UBRR0L = (uint8_t)(UBRR & 0xFF);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);  // Habilita transmiss?o e recep??o
}


// Envio da mensagem de identifica??o e menu de instru??es
void sendMenu() {
	 UART_puts("********************************************************************************");
	 UART_newLine();
	 UART_puts("*** Eng. Informatica, Arquitetura de Computadores, TP2 ***Turma 02, Grupo 06 ***");
	 UART_newLine();
	 UART_puts("********************************************************************************");
	 UART_newLine();
	 UART_puts("(1) Modo Manual");
	 UART_newLine();
	 UART_puts("(2) Ligar Rele");
	 UART_newLine();
	 UART_puts("(3) Apagar Rele");
	 UART_newLine();
	 UART_puts("(4) Modo Automatico");	
	 UART_newLine();
	 UART_newLine();
	 UART_puts("(9) Imprimir menu");
	 UART_newLine();
	UART_puts(">>");
	 
}
void ligarRele() {
	// Ligar o relé
	PORTB |= (1 << RELAY_PIN);
}

void desligarRele() {
	// Desligar o relé
	PORTB &= ~(1 << RELAY_PIN);
}
void manual(){	
	desligarRele();
	ligarRele();

	
	UART_puts("entrei\n");
		PORTB |= (1 << LED_B);
		//_delay_ms(10000); // Esperar 1 segundo
		
		UART_puts("entrfkufdei\n");
		sendMenu();
	
}

int main(void) {
	UART_Init(MYUBRR);
	
	DDRD = INPUT;
	DDRB = OUTPUT;

	currentButtonDirState = (PIND & (1 << BOTDIR));
	currentButtonEsqState = (PIND & (1 << BOTESQ));
	
	// Envia mensagem de identifica o e menu de instru??es
	
	sendMenu();
	
	while (1) {
		
		InitADC();
		
		
			/*lastButtonEsqState    = currentButtonEsqState;
			currentButtonEsqState = (PIND & (1<<BOTESQ));
			
			lastButtonDirState    = currentButtonDirState;
			currentButtonDirState = (PIND & (1<<BOTDIR));
		
		
			// Botao ESQ Premido
			if((lastButtonEsqState == OFF && currentButtonEsqState == (1<<BOTESQ))) {
				ledState = (ledState == 0) ? 8 : LED_SHIFTESQ();    //    If the led state is 0 (OFF) sets it to 8 (last LED) ; Else do the left shift
				UART_puts("ESQUERDO");
			}

			// Botao DIR Premido
			if(lastButtonDirState == OFF && currentButtonDirState == (1<<BOTDIR)){
					switch (ledState) {
						case 8:                //    If the led state is 8 (last led) sets it to 0 (OFF)
						ledState = 0;
						break;

						case 0:                //    If the led state is 0 (OFF) sets it to 1 (first led)
						ledState = 1;
						break;

						default:            //    Else do the right shift
						ledState = LED_SHIFTDIR();
						break;
					}
					UART_puts("DIREITO");
			}*/

		if((UCSR0A & (1 << UDRE0))){
			 command = UART_getc();
			 UART_putc(command);  // ECO do comando recebido
			 UART_newLine();
			}
		
		

		switch (command) {
			case '1':  // Movimento para a DIREITA
				manual();
				UART_puts("OK\n");
				break;
			case '2': 
				switch (ledState){ // Movimento para a ESQUERDA
					case 8:
						ledState = LED_SHIFTESQ();
						break;
					case 0:
						ledState = 8;
						break;
					case 255:
						ledState = 0;
					default:
						ledState = LED_SHIFTESQ();
						break;
				}  // Se o estado do LED for 0, alterna para 8; caso contr?rio, faz o deslocamento para a esquerda
				PORTB = ledState;
				UART_puts("OK\n");
				break;
			case '3':  // Apagar LEDs
				ledState = 0;
				PORTB = ledState;
				UART_puts("OK\n");
				break;
			case '4':  // Acender TODOS
				ledState = 255;
				PORTB = ledState;
				UART_puts("OK\n");
				break;
			case '5':  // Imprimir MENU
				UART_puts("OK\n");
				sendMenu();
				break;
			default:
				break;	
		}
		command = ' ';
	


		
	}
}

