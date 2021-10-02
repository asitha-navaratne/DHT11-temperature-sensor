#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>

#define DHT11_DDR		DDRC									///< DDR of DHT11 sensor.
#define LCD_DDR 		DDRB									///< DDR of LCD display.
#define	DHT11_INPUT		PINC									///< Port of DHT11 sensor for input operations.
#define	DHT11_OUTPUT	PORTC									///< Port of DHT11 sensor for output operations.
#define LCD_PORT 		PORTB									///< Port of LCD display.
#define DHT11_PIN		PC0										///< Pin for DHT11 sensor.
#define RS 				PB0										///< Pin for LCD Register Select terminal.
#define EN 				PB1										///< Pin for LCD Enable terminal.

void DHT11_START_PULSE(void);
unsigned char DHT11_DATA_BYTE(void);
void LCD_DISPLAY_INIT(void);
void LCD_SEND_COMMAND(unsigned char command);
void LCD_SEND_CHARACTER(unsigned char character);
void LCD_SEND_STRING(char *string);
void LCD_SEND_NUMBER(unsigned char number,uint8_t decimal);

int main(void){
	LCD_DISPLAY_INIT();
	
	unsigned char humidity_int,humidity_decimal,temperature_int,temperature_decimal;
	uint8_t checksum;
	
	while(1){
		DHT11_START_PULSE();
		
		humidity_int = DHT11_DATA_BYTE();					///< Receive integer value of humidity.
		humidity_decimal = DHT11_DATA_BYTE();				///< Receive decimal value of humidity.
		temperature_int = DHT11_DATA_BYTE();				///< Receive integer value of temperature.
		temperature_decimal = DHT11_DATA_BYTE();			///< Receive decimal value of temperature.
		checksum = DHT11_DATA_BYTE();						///< Receive checksum  value.
		
		LCD_SEND_COMMAND(0x01);								///< Clear display screen.
		LCD_SEND_COMMAND(0x80);								///< Move cursor to beginning of first line.
		
		if(checksum != (uint8_t)(humidity_int + humidity_decimal + temperature_int + temperature_decimal)){
			LCD_SEND_STRING("Error!");
		}
		else{
			LCD_SEND_STRING("Temp = ");
			LCD_SEND_NUMBER(temperature_int,0);
			LCD_SEND_CHARACTER('.');
			
			LCD_SEND_NUMBER(temperature_decimal,1);
			LCD_SEND_CHARACTER(0xDF);						///< Send a degree symbol to the LCD display.
			LCD_SEND_CHARACTER('C');
			
			LCD_SEND_COMMAND(0xC0);
			
			LCD_SEND_STRING("RH = ");
			LCD_SEND_NUMBER(humidity_int,0);
			LCD_SEND_CHARACTER('.');
			
			LCD_SEND_NUMBER(humidity_decimal,1);
			LCD_SEND_CHARACTER('%');
			
		}
		_delay_ms(500);
	}
}

/*!
 *	@brief Initialize LCD in 4-bit mode.
 */

void LCD_DISPLAY_INIT(void){
	LCD_DDR |= (1<<RS)|(1<<EN)|(1<<4)|(1<<5)|(1<<6)|(1<<7);
	
	LCD_SEND_COMMAND(0x01);										///< Clear display screen.
	LCD_SEND_COMMAND(0x02);										///< Initialize LCD in 4-bit mode.
	LCD_SEND_COMMAND(0x28);										///< 2 line, 5x7 matrix of characters in 4-bit mode.
	LCD_SEND_COMMAND(0x80);										///< Move cursor to beginning of first line.
	LCD_SEND_COMMAND(0x0C);										///< Turn display and cursor on.
	_delay_ms(2);
}

/*!
 *	@brief Generate a start pulse and await response to begin communication with DHT11.
 */

void DHT11_START_PULSE(void){
	DHT11_DDR |= (1<<DHT11_PIN);
	DHT11_OUTPUT &= ~(1<<DHT11_PIN);
	_delay_ms(20);
	DHT11_OUTPUT |= (1<<DHT11_PIN);				///< Send a 20 ms LOW pulse to the sensor.
	
	DHT11_DDR &= ~(1<<DHT11_PIN);
	
	while(DHT11_INPUT & (1<<DHT11_PIN));
	while(!(DHT11_INPUT & (1<<DHT11_PIN)));
	while(DHT11_INPUT & (1<<DHT11_PIN));			///< Wait until the response pulse is received.
}

/*!
 *	@brief Receive a byte of data from the DHT11 sensor.
 *	@return Received data byte (unsigned char).
 */

unsigned char DHT11_DATA_BYTE(void){
	unsigned char databyte = 0;
	for(uint8_t i=0;i<8;i++){
		while(!(DHT11_INPUT & (1<<DHT11_PIN)));
		_delay_us(30);
		if(DHT11_INPUT & (1<<DHT11_PIN)){
			databyte = (databyte<<1)|(0x01);					///< If data line is HIGH for more than 30 us, received bit is a 1.
		}
		else{
			databyte = (databyte<<1);							///< If data line is LOW after 30 us, received bit is a 0.
		}
		while(DHT11_INPUT & (1<<DHT11_PIN));
	}
	return databyte;
}

/*!
 *	@brief Send a command to the LCD display.
 *	@param Command to be sent (unsigned char).
 */

void LCD_SEND_COMMAND(unsigned char command){
	LCD_PORT = (LCD_PORT & 0x0F)|(command & 0xF0);				///< Load upper nibble of the command to the port.
	LCD_PORT &= ~(1<<RS);										///< Set RS pin low to enter command mode.
	LCD_PORT |= (1<<EN);
	_delay_us(1);
	LCD_PORT &= ~(1<<EN);										///< Create a momentary pulse at EN pin to send the data to the LCD.
 
	_delay_us(200);
 
	LCD_PORT = (LCD_PORT & 0x0F)|(command << 4);				///< Load lower nibble of the command to the port.
	LCD_PORT |= (1<<EN);
	_delay_us(1);
	LCD_PORT &= ~(1<<EN);										///< Pulse to send the data.
 
	_delay_ms(2);
}

/*!
 *	@brief Send a single character to the LCD display.
 *	@param Character to be sent (unsigned char).
 */

void LCD_SEND_CHARACTER(unsigned char character){
	LCD_PORT = (LCD_PORT & 0x0F)|(character & 0xF0);			///< Load upper nibble of the character to the port. 
	LCD_PORT |= (1<<RS);										///< Set RS pin high to enter character mode.
	LCD_PORT |= (1<<EN);
	_delay_us(1);
	LCD_PORT &= ~(1<<EN);										///< Create a momentary pulse at EN pin to send the data to the LCD.
	
	_delay_us(200);

	LCD_PORT = (LCD_PORT & 0x0F)|(character << 4);				///< Load lower nibble of the character to the port. 
	LCD_PORT |= (1<<EN);
	_delay_us(1);
	LCD_PORT &= ~(1<<EN);										///< Pulse to send the data.
	
	_delay_ms(2);
}

/*!
 *	@brief Send a string of characters to the LCD display.
 *	@param String to be sent (char).
 */

void LCD_SEND_STRING(char *string){
	for(uint8_t i=0;string[i]!=0;i++){
		LCD_SEND_CHARACTER(string[i]);
		_delay_ms(2);
	}
}

/*!
 *	@brief Send a number received from the DHT11 sensor to the LCD display.
 *	@param Number to be sent (unsigned char) and integer or decimal condition (uint8_t).
 */
 
 void LCD_SEND_NUMBER(unsigned char number,uint8_t decimal){
	uint8_t	dig[3];
	uint8_t integer = (uint8_t)number;
	
	dig[0] = (integer/100);								///< Save first digit to array.
	dig[1] = (integer%100)/10;								///< Save second digit to array.
	dig[2] = (integer%10);									///< Save third digit to array.
	
	if(decimal){
		for(uint8_t i=0;i<3;i++){
			LCD_SEND_CHARACTER(((0x03<<4)|dig[i]));		///< Map digit to corresponding symbol in LCD character map.
			_delay_ms(2);
		}
	}
	else{
		for(uint8_t i=1;i<3;i++){
			LCD_SEND_CHARACTER(((0x03<<4)|dig[i]));
			_delay_ms(2);
		}
	}
 }