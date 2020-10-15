/*
 * lcd.c
 *
 *  Created on: Oct 6, 2020
 *      Author: Тлехас Алий
 */

#include "lcd.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};

/************************************** Static declarations (Internal Func) **************************************/

static void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data);
static void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command);
static void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len);

static char* convert(unsigned int num, int base);  //Convert integer number into octal, hex, etc.


/************************************** Function definitions **************************************/

/**
 * Create new Lcd_HandleTypeDef and initialize the Lcd
 */
Lcd_HandleTypeDef Lcd_create(
		Lcd_PortType port[], Lcd_PinType pin[],
		Lcd_PortType rs_port, Lcd_PinType rs_pin,
		Lcd_PortType en_port, Lcd_PinType en_pin)
{
	Lcd_HandleTypeDef lcd;

	lcd.en_pin = en_pin;
	lcd.en_port = en_port;

	lcd.rs_pin = rs_pin;
	lcd.rs_port = rs_port;

	lcd.data_pin = pin;
	lcd.data_port = port;

	//Lcd_init(&lcd);

	return lcd;
}

/**
 * Initialize 16x2-lcd without cursor
 */
void Lcd_init(Lcd_HandleTypeDef * lcd)
{
	lcd_write_command(lcd, 0x33);
	lcd_write_command(lcd, 0x32);
	lcd_write_command(lcd, FUNCTION_SET | OPT_N);				// 4-bit mode

	lcd_write_command(lcd, CLEAR_DISPLAY);						// Clear screen
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D);		// Lcd-on, cursor-off, no-blink
	lcd_write_command(lcd, ENTRY_MODE_SET | OPT_INC);			// Increment cursor
}

/**
 * Set the cursor position
 */
void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col)
{
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_16[row] + col);
}

/**
 * Blinking and noBlinking cursor or disable cursor
 */
void Lcd_blink(Lcd_HandleTypeDef * lcd){
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_B);		// Lcd-on, cursor-on, blink
}

void Lcd_no_blink(Lcd_HandleTypeDef * lcd){
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_C);		// Lcd-on, cursor-on, no-blink
}

void Lcd_disable_cursor(Lcd_HandleTypeDef * lcd){
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D);		// Lcd-on, cursor-off, no-blink
}

/**
 * Printf() for lcd
 */
void Lcd_printf(Lcd_HandleTypeDef * lcd, char * format, ...) {
	char *traverse;
	unsigned int i;
	char *s;

	//Initializing Myprintf's arguments
	    va_list arg;
	    va_start(arg, format);

	    for(traverse = format; *traverse != '\0'; traverse++)
	    {
	        while( *traverse != '%' )
	        {
	        	Lcd_string(lcd, traverse);
	            traverse++;
	        }

	        traverse++;

	        //Fetching and executing arguments
	        switch(*traverse)
	        {
	            case 'c' : i = va_arg(arg,int);     //Fetch char argument
	            			Lcd_int(lcd, i);
	                        break;

	            case 'd' : i = va_arg(arg,int);         //Fetch Decimal/Integer argument
	                        if(i<0)
	                        {
	                            i = -i;
	                            Lcd_string(lcd, "-");
	                        }
	                        Lcd_string(lcd, convert(i,10));
	                        break;

	            case 'o': i = va_arg(arg,unsigned int); //Fetch Octal representation
	            			Lcd_string(lcd, convert(i,8));
	                        break;

	            case 's': s = va_arg(arg,char *);       //Fetch string
	            			Lcd_string(lcd, s);
	                        break;

	            case 'x': i = va_arg(arg,unsigned int); //Fetch Hexadecimal representation
	            			Lcd_string(lcd, convert(i,16));
	                        break;
	        }
	    }

	    //Closing argument list to necessary clean-up
	    va_end(arg);
}

void Lcd_printAt(Lcd_HandleTypeDef * lcd, char * format, ...) {
	char buf[PRINTF_BUF];
	va_list ap;
	va_start(ap, format);
    vsnprintf(buf, sizeof(buf), format, ap);
    Lcd_string(lcd, buf);
    va_end(ap);
}

void Lcd_int(Lcd_HandleTypeDef * lcd, int number)
{
	char buffer[11];
	sprintf(buffer, "%d", number);

	Lcd_string(lcd, buffer);
}

/**
 * Write a string on the current position
 */
void Lcd_string(Lcd_HandleTypeDef * lcd, char * string)
{
	for(uint8_t i = 0; i < strlen(string); i++)
	{
		lcd_write_data(lcd, string[i]);
	}
}

/**
 * Clear the screen
 */
void Lcd_clear(Lcd_HandleTypeDef * lcd) {
	lcd_write_command(lcd, CLEAR_DISPLAY);
}

/**
 * Define and add a new 5x10 character
 */
void Lcd_define_char(Lcd_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]){
	lcd_write_command(lcd, SETCGRAM_ADDR + (code << 3));
	for(uint8_t i=0;i<8;++i){
		lcd_write_data(lcd, bitmap[i]);
	}

}

/************************************** Static function definition **************************************/

/**
 * Write a byte to the command register
 */
void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_COMMAND_REG);		// Write to command register

	lcd_write(lcd, (command >> 4), LCD_NIB);
	lcd_write(lcd, command & 0x0F, LCD_NIB);

}

/**
 * Write a byte to the data register
 */
void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_DATA_REG);			// Write to data register

	lcd_write(lcd, data >> 4, LCD_NIB);
	lcd_write(lcd, data & 0x0F, LCD_NIB);

}

/**
 * Set len bits on the bus and toggle the enable line
 */
void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 0x01);
	}

	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 1);
	DELAY(1);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 0); 		// Data receive on falling edge
}

char* convert(unsigned int num, int base) {
	static char Representation[]= "0123456789ABCDEF";
	    static char buffer[50];
	    char *ptr;

	    ptr = &buffer[49];
	    *ptr = '\0';

	    do
	    {
	        *--ptr = Representation[num%base];
	        num /= base;
	    }while(num != 0);

	    return(ptr);

}
