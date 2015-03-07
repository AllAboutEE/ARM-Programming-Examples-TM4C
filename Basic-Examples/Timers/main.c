/**
 * @author Carlos Alvarez (2/28/2015) 
 *  
 * @details This example uses timers (Periodic Mode) to toggle the Blue LED on 
 *          the board every second.
 *
 *        See detail tutorial at: https://www.youtube.com/watch?v=fUac_C1aZP0
 */

#include <lm4f120h5qr.h>

int main()
{
  //configure GPIO F blue LED
  SYSCTL->RCGCGPIO |= (1<<5); //enable clock on Port F
  GPIOF->DIR = (1<<2); //make PF2 output
  GPIOF->AFSEL &= ~(1<<2); //disable alternate function on PF2
  GPIOF->DEN = (1<<2); //digital i/o enable PF2
  GPIOF->DATA |= (1<<2); //PF1 set to HIGH (led on)
  
  //To use GPTM the appropirate TIMERn bit must be set
  SYSCTL->RCGCTIMER |= (1<<0); 
  //1: Disable the timer (clear the TnEN bit in the GPTMCTL register)
  TIMER0->CTL &= ~(1<<0); 
  //2: Write the GPTMCFG witha a value of 0x00000000
  TIMER0->CFG = 0x00000000;
  //3: Configure the TnMR field in the GPTMTnMR. 0x2 for Periodic
  TIMER0->TAMR |= (0x2<<0);
  //4:Count down timer
  TIMER0->TAMR &= ~(1<<4);
  //5: Load the value that you will count down/up to
  TIMER0->TAILR = 0x00F42400; //16,000,000 this corresponds to the clk speed of this microcontroller
  //6: If interrupts are required, set the appropriate bits in the GPTMIMR
  //7: Set the TnEN bit in the GPTMCTL register to enable the timer and start counting
  TIMER0->CTL |= (1<<0);
  
  while(1)
  {
    if((TIMER0->RIS & 0x00000001) == 1) //0 TIMER A HAS NOT TIME OUT
    {
      TIMER0->ICR |= (1<<0);
      GPIOF->DATA ^= (1<<2);
    }
  }
        
  return 0;
}
