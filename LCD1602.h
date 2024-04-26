/*
 * LCD1602.h
 *
 *  Created on: Apr 26, 2024
 *      Author: dhara
 */

#ifndef LCD1602_H_
#define LCD1602_H_



#include "STM32F103IO.h"



#define           en            0x200
#define           rs            0x100



void LCD_Command (volatile uint8_t cmnd)
{
	GPIOB_OUT &= ~(0x3F << 4);       // Clear the LCD connected pins
	GPIOB_OUT |= (cmnd & 0xF0);      // Send upper nibble
	GPIOB_OUT |= en;                 // en = 1, rs = 0
	delay_us(50);
	GPIOB_OUT &= ~(en);              // en = 0, rs = 0
	delay_us(50);

	GPIOB_OUT &= ~(0x3F << 4);            // Clear the LCD connected pins
	GPIOB_OUT |= ((cmnd & 0x0F) << 4);    // Send lower nibble
	GPIOB_OUT |= en;                      // en = 1, rs = 0
	delay_us(50);
	GPIOB_OUT &= ~(en);                   // en = 0, rs = 0
	delay_us(50);
}



void LCD_Char(volatile uint8_t data)
{
	GPIOB_OUT &= ~(0x3F << 4);       // Clear the LCD connected pins
	GPIOB_OUT |= (data & 0xF0);      // Send upper nibble
	GPIOB_OUT |= (en + rs);          // en = 1, rs = 1
	delay_us(50);
	GPIOB_OUT &= ~(en);              // en = 0, rs = 1
	delay_us(50);

	GPIOB_OUT &= ~(0x3F << 4);            // Clear the LCD connected pins
	GPIOB_OUT |= ((data & 0x0F) << 4);    // Send lower nibble
	GPIOB_OUT |= (en + rs);               // en = 1, rs = 1
	delay_us(50);
	GPIOB_OUT &= ~(en);                   // en = 0, rs = 1
	delay_us(50);
}



void LCD_String(volatile const char *str)	// Send string to LCD function
{
	int i;
	for(i=0;str[i]!=0;i++)  // Send each volatile uint8_t of string till the NULL
	{
		LCD_Char (str[i]);  // Call LCD data write
	}
}



void LCD_Init(){
	GPIO_mode(GPIOB, 4, OUT);
	GPIO_mode(GPIOB, 5, OUT);
	GPIO_mode(GPIOB, 6, OUT);
	GPIO_mode(GPIOB, 7, OUT);
	GPIO_mode(GPIOB, 8, OUT);
	GPIO_mode(GPIOB, 9, OUT);

	delay_us(50);	         	// LCD Power ON Initialization time >15ms
	LCD_Command (0x02);    	// 4bit mode
	LCD_Command (0x28);	    //  4bit mode
	LCD_Command (0x0C);	    //Display ON Cursor OFF
	LCD_Command (0x06);   	// Auto Increment cursor
	LCD_Command (0x01);	    // Clear display
	LCD_Command (0x80);	    // Cursor at home position
}



void LCD_Cursor(volatile uint8_t row, volatile uint8_t column){
  if(row==0) LCD_Command(0x80 + column);
  else if(row==1) LCD_Command(0xc0 + column);
}



void LCD_Clear(){
  LCD_Command(0x01);
}




#endif /* LCD1602_H_ */
