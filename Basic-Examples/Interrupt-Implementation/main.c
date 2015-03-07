/**
 * @author Carlos Alvarez (2/28/2015) 
 *  
 * @details This example uses timers and interrupt function to toggle the Blue 
 *          LED on the board every second.
 *
 *        See detail tutorial at: https://www.youtube.com/watch?v=bQ50pzqFA5Q
 */

#include <lm4f120h5qr.h>

#define FLAG_NONE 0x00000000
#define FLAG_TOGGLE_LED 0x00000001

volatile int flags = FLAG_NONE;


void TIMER0A_Handler(void)
{
  flags = FLAG_TOGGLE_LED;
  TIMER0->ICR |= (1<<0);
}

int main()
{
  SYSCTL->RCGCGPIO |= (1<<5);
  GPIOF->AFSEL &= ~(1<<2);
  GPIOF->DIR = (1<<2); //0b0000 0100 (determines if pin is output or input)
  GPIOF->DEN = (1<<2);
  GPIOF->DATA |= (1<<2);
  
  /*****One-Shot/Periodic Timer Mode*****/ //[pg.683]
  
  //Enable and provide a clock to 16/32-bit general purpose timer module 0 in Run mode (enable timer0)
  SYSCTL->RCGCTIMER |= (1<<0); //0b0000 0001
  
  //Step1:Ensure the time is disabld (the TnEN bit in the GPTMCTL register is cleared) before making any changes [pg.698]
  //disable timer0 by giving a value of 0 to bit 0
  TIMER0->CTL &= ~(1<<0);
  
  //Step2:Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0000 [pg.688]
  //For a 16/32 bit timer, this value selects the 32 bit timer configuration
  //For a 32/64 bit wide timer, this value selects the 64 bit timer configuration
  TIMER0->CFG = 0x00000000;
  
  //Step3:Configure the TnMR field in the GPTM Timer n Mode Register (GPTMTnMR)
  //This value set to the GPTMTAMR register allows us to choose periodic mode
  TIMER0->TAMR |= (0x2<<0);
  
  //Step4: 0 - timer counts down: 1 - timer counts up, starting from 0x0
  //Setting TACDIR(bit 4) to 0 in order to make the timer count down
  TIMER0->TAMR &= ~(1<<4);
  
  //Step5: Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR)
  //Loads the start value into to the GPTMTnILR
  TIMER0->TAILR = 0x00F42400; //16,000,000 = 1 sec for this microcontroller
  
  //Step6: If interrupts are required, set the appropriate bits in the GPTM Interrupt Mask Register (GPTMIMR)
  TIMER0->IMR |= (1<<0); //Enabled Interrupt
  //NVIC->ISER[0] |= (1<<19); //Interrupts from registers directly
  NVIC_EnableIRQ(TIMER0A_IRQn); //from CMSIS Library
  
  //Step7: Set the TnEN bit in the GPTMCTL register to enable the timer and start counting
  //Enable timer0 by giving a vaule of 1 to bit 0, this will get the timer to start counting
  TIMER0->CTL |= (1<<0);
  
  //Step8: Clear status flag (GPTMICR) by writing a 1 to the appropriate bit
  //Wrinting a 1 to bit 0 of this register clears the TATORIS bit in the GPTMRIS register and the TATOMIS bit in the GPTMMIS register
  //By giving TATORIS bit a value of 1, we say that Timer A has timed out. This interrupt is asserted when a one-shot or
  //periodic mode timer reaches it's count limit
  while(1)
  {
    if(flags == FLAG_TOGGLE_LED)
    {
      GPIOF->DATA ^= (1<<2);
      flags = FLAG_NONE;
    }

  }
  
  return 0;
}
