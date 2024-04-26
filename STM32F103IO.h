/*
 * STM32F103IO.h
 *
 *  Created on: Apr 26, 2024
 *      Author: dhara
 */

#ifndef STM32F103IO_H_
#define STM32F103IO_H_



#define       GPIOA_IN             *porta_input_reg
#define       GPIOA_OUT            *porta_output_reg
#define       GPIOB_IN             *portb_input_reg
#define       GPIOB_OUT            *portb_output_reg
#define       GPIOC_IN             *portc_input_reg
#define       GPIOC_OUT            *portc_output_reg

#define       RCC_base_add         0x40021000
#define       APB2_offset          0x18
#define       PORTA_base_add       0x40010800
#define       PORTB_base_add       0x40010C00
#define       PORTC_base_add       0x40011000
#define       GPIO_CRL_offset      0x00
#define       GPIO_CRH_offset      0x04
#define       GPIO_ODR_offset      0x0C
#define       GPIO_IDR_offset      0x08

#define       input_mode           8
#define       output_mode          2
#define       clear                15
#define       GPIOA               'A'
#define       GPIOB               'B'
#define       GPIOC               'C'
#define       IN                   0
#define       OUT                  1





volatile uint32_t *clock_reg    =   (uint32_t*) (RCC_base_add + APB2_offset);

volatile uint32_t *porta_mode_reg_low     =    (uint32_t*) (PORTA_base_add + GPIO_CRL_offset);
volatile uint32_t *porta_mode_reg_high    =    (uint32_t*) (PORTA_base_add + GPIO_CRH_offset);
volatile uint32_t *porta_input_reg        =    (uint32_t*) (PORTA_base_add + GPIO_IDR_offset);
volatile uint32_t *porta_output_reg       =    (uint32_t*) (PORTA_base_add + GPIO_ODR_offset);

volatile uint32_t *portb_mode_reg_low     =    (uint32_t*) (PORTB_base_add + GPIO_CRL_offset);
volatile uint32_t *portb_mode_reg_high    =    (uint32_t*) (PORTB_base_add + GPIO_CRH_offset);
volatile uint32_t *portb_input_reg        =    (uint32_t*) (PORTB_base_add + GPIO_IDR_offset);
volatile uint32_t *portb_output_reg       =    (uint32_t*) (PORTB_base_add + GPIO_ODR_offset);

volatile uint32_t *portc_mode_reg_low     =    (uint32_t*) (PORTC_base_add + GPIO_CRL_offset);
volatile uint32_t *portc_mode_reg_high    =    (uint32_t*) (PORTC_base_add + GPIO_CRH_offset);
volatile uint32_t *portc_input_reg        =    (uint32_t*) (PORTC_base_add + GPIO_IDR_offset);
volatile uint32_t *portc_output_reg       =    (uint32_t*) (PORTC_base_add + GPIO_ODR_offset);





void GPIO_mode(volatile char port_name, volatile uint32_t pin_number, volatile uint32_t mode){


/*---------------------------------------------------------------------------------------------------------*/
/*                                           PORTA pin configuration                                       */
/*---------------------------------------------------------------------------------------------------------*/


  if(port_name == 'A'){
    /* Enable PORTA */
    *clock_reg |=  (1 << 2);

/* If the pin is belongs to LOWER BYTE */
    if((pin_number >= 0) && (pin_number <= 7)){
    /* Clear the respective pin mode & config bits */
      *porta_mode_reg_low    &=     ~(clear << (4*pin_number));

           if(mode == 1) *porta_mode_reg_low |= (output_mode << (4*pin_number));  /* Mode == 1 means OUTPUT */
      else if(mode == 0) *porta_mode_reg_low |= (input_mode << (4*pin_number));    /* Mode == 0 means INPUT */
    }


/* If the pin is belongs to HIGHER BYTE */
    else if((pin_number >= 8) && (pin_number <= 15)){
      /* for clear the respective pin mode & config bits */
      *porta_mode_reg_high &= ~(clear << (4*(pin_number - 8)));

           if(mode == 1) *porta_mode_reg_high |= (output_mode << (4*(pin_number - 8)));   /* Mode == 1 means OUTPUT */
      else if(mode == 0) *porta_mode_reg_high |= (input_mode << (4*(pin_number - 8)));    /* Mode == 0 means INPUT */
    }
  }


/*---------------------------------------------------------------------------------------------------------*/
/*                                           PORTB pin configuration                                       */
/*---------------------------------------------------------------------------------------------------------*/


    else if(port_name == 'B'){
    /* Enable PORTB */
    *clock_reg |=  (1 << 3);

/* If the pin is belongs to LOWER BYTE */
    if((pin_number >= 0) && (pin_number <= 7)){
    /* Clear the respective pin mode & config bits */
      *portb_mode_reg_low    &=     ~(clear << (4*pin_number));

           if(mode == 1) *portb_mode_reg_low |= (output_mode << (4*pin_number));   /* Mode == 1 means OUTPUT */
      else if(mode == 0) *portb_mode_reg_low |= (input_mode << (4*pin_number));    /* Mode == 0 means INPUT */
    }


/* If the pin is belongs to HIGHER BYTE */
    else if((pin_number >= 8) && (pin_number <= 15)){
      /* for clear the respective pin mode & config bits */
      *portb_mode_reg_high &= ~(clear << (4*(pin_number - 8)));

           if(mode == 1) *portb_mode_reg_high |= (output_mode << (4*(pin_number - 8)));   /* Mode == 1 means OUTPUT */
      else if(mode == 0) *portb_mode_reg_high |= (input_mode << (4*(pin_number - 8)));    /* Mode == 0 means INPUT */
    }
  }


/*---------------------------------------------------------------------------------------------------------*/
/*                                           PORTC pin configuration                                       */
/*---------------------------------------------------------------------------------------------------------*/


    else if(port_name == 'C'){
    /* Enable PORTC */
    *clock_reg |=  (1 << 4);

/* If the pin is belongs to LOWER BYTE */
    if((pin_number >= 0) && (pin_number <= 7)){
    /* Clear the respective pin mode & config bits */
      *portc_mode_reg_low    &=     ~(clear << (4*pin_number));

           if(mode == 1) *portc_mode_reg_low |= (output_mode << (4*pin_number));   /* Mode == 1 means OUTPUT */
      else if(mode == 0) *portc_mode_reg_low |= (input_mode << (4*pin_number));    /* Mode == 0 means INPUT */
    }


/* If the pin is belongs to HIGHER BYTE */
    else if((pin_number >= 8) && (pin_number <= 15)){
      /* for clear the respective pin mode & config bits */
      *portc_mode_reg_high &= ~(clear << (4*(pin_number - 8)));

           if(mode == 1) *portc_mode_reg_high |= (output_mode << (4*(pin_number - 8)));   /* Mode == 1 means OUTPUT */
      else if(mode == 0) *portc_mode_reg_high |= (input_mode << (4*(pin_number - 8)));    /* Mode == 0 means INPUT */
    }
  }
}



void GPIO_write(volatile char port_name, volatile uint32_t pin_number, volatile uint32_t pin_state){
  if(port_name == 'A'){
         if(pin_state == 1) *porta_output_reg |=  (1 << pin_number);
    else if(pin_state == 0) *porta_output_reg &= ~(1 << pin_number);
  }

  else if(port_name == 'B'){
         if(pin_state == 1) *portb_output_reg |=  (1 << pin_number);
    else if(pin_state == 0) *portb_output_reg &= ~(1 << pin_number);
  }

  if(port_name == 'C'){
         if(pin_state == 1) *portc_output_reg |=  (1 << pin_number);
    else if(pin_state == 0) *portc_output_reg &= ~(1 << pin_number);
  }
}



volatile uint32_t GPIO_read(volatile char port_name, volatile uint32_t pin_number){
       if(port_name == 'A') return ((*porta_input_reg >> pin_number) & 1);
  else if(port_name == 'B') return ((*portb_input_reg >> pin_number) & 1);
  else if(port_name == 'C') return ((*portc_input_reg >> pin_number) & 1);
}



/* Mili second delay */
void delay_ms(volatile uint16_t time){
	volatile uint16_t x, y;
	for(x=0; x<time; x++)
	for(y=0; y<235; y++);
}


/* Micro second delay */
void delay_us(volatile uint16_t time){
	for(volatile uint16_t x=0; x<time; x++);
}



#endif /* STM32F103IO_H_ */
