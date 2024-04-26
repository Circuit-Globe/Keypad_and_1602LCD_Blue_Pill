/*
 * Keypad.h
 *
 *  Created on: Apr 26, 2024
 *      Author: dhara
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_



#include "STM32F103IO.h"



void keypad_Init()
{
  GPIO_mode(GPIOA, 4, OUT);
	GPIO_mode(GPIOA, 5, OUT);
	GPIO_mode(GPIOA, 6, OUT);
	GPIO_mode(GPIOA, 7, OUT);

	GPIO_mode(GPIOA, 0, IN);
	GPIO_mode(GPIOA, 1, IN);
	GPIO_mode(GPIOA, 2, IN);
	GPIO_mode(GPIOA, 3, IN);
}


volatile char keypad()
{
	volatile char row, col, key;

	for(row=0; row<4; row++){
		GPIOA_OUT &= ~(0x0F << 4);        // clear the keypad connected output pin
		GPIOA_OUT |= (1 << (row + 4));
		if(GPIOA_IN  & 0x0F){ col = (GPIOA_IN  & 0x0F); break;}
	}


	if(row == 0){
         if(col == 1) key = '1';
    else if(col == 2) key = '2';
    else if(col == 4) key = '3';
    else if(col == 8) key = 'A';
  }
  else if(row == 1){
         if(col == 1) key = '4';
    else if(col == 2) key = '5';
    else if(col == 4) key = '6';
    else if(col == 8) key = 'B';
  }
  else if(row==2){
         if(col == 1) key = '7';
    else if(col == 2) key = '8';
    else if(col == 4) key = '9';
    else if(col == 8) key = 'C';
  }
  else if(row == 3){
         if(col == 1) key = '*';
    else if(col == 2) key = '0';
    else if(col == 4) key = '#';
    else if(col == 8) key = 'D';
  }
  else key = 0;
  return key;
}



#endif /* KEYPAD_H_ */
