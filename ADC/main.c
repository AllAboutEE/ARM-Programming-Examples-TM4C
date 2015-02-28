/**
 * @author Carlos Alvarez (2/28/2015) 
 *  
 * @details This example uses a potentiometer to turn on the Red LED.
 *        See detail tutorial at: https://www.youtube.com/watch?v=xkAT9MXh8UY
 */

#include <lm4f120h5qr.h>

volatile static uint32_t adcResult = 0;

void ADC1SS3_Handler(void)
{
  adcResult = ADC1->SSFIFO3;
  ADC1->ISC = (1<<3);
}

void SystemInit()
{
  
}

int main()
{
  /****** MODULE INITIALIZATION ******/
  //This register provides software the capability to enable and disable the ADC modules in Run mode
  //The ADC module 1 is enabled by setting bit 1. ADC module 0 is enabled by setting bit 0.
  SYSCTL->RCGCADC = (1<<1); //Step 1
  
  //This register provides software the capability to enable and disable GPIO modules in Run Mode
  //Enable PortE (1<<4), will be used for pot: Enable PortF (1<<5), will be used for LED
  SYSCTL->RCGCGPIO = (1<<4)|(1<<5); //Step 2 
  
  //This register is the data direction register
  //We clear pin One in order to make it an input
  GPIOE->DIR &= ~(1<<1);
  
  /****** LED ******/
  GPIOF->DEN = 0xFF; //Enable digital functions for the corresponding pin
  GPIOF->AFSEL = 0x00; //Disable alternate functions, we are using digital
  GPIOF->DIR = 0xFF; //Setting the pins makes them an output
  GPIOF->DATA = (1<<1);
  /****************/
  
  //Selects the alternate function, in this case analog
  GPIOE->AFSEL = (1<<1); //Step 3
  
  //Using analog => disable the digital function of that pin (I am using PE1 which corresponds to AIN2)
  GPIOE->DEN &= ~(1<<1); //Step 4
  
  //Analog function of the pin is enabled, the isolation is disabled, and the pin is capable of analog functions (pin PE1)
  GPIOE->AMSEL = (1<<1); //Step 5
  
  /****** Sample Sequencer Configuration ******/

  //Disable sample sequencer 3 (I used SS3 because SS3 allows us to sample from one pin)
  //Disabling the sequencer during programming prevents erroneous execution if a trigger event
  //were to occur during the configuration process
  ADC1->ACTSS &= ~(1<<3); //Step 1
  
  //I am using SS3 whos bits start from 12->15, => I assigned 0xF<<12
  // Always (countinuously sample)
  ADC1->EMUX = (0xF<<12); //Step 2
  
  //Select my analog input. I am using PE1 which corresponds to AIN2
  ADC1->SSMUX3 = 2; //Step 3
  
  //I set bit END0(bit 1), and bit IE0(bit 2) to be used for interrupts
  ADC1->SSCTL3 = 0x6; //Step 4
  
  //Interrupts are used, => I set bit 3 (SS3 Interrupt Mask)
  ADC1->IM = (1<<3);//Step 5
  
  //Sample Sequencer 3 is enabled again
  ADC1->ACTSS |= (1<<3);//Step 6
  
  //This bit is cleared by writing a 1. Clearing thi bit also clears the INR3 bit in the ADCRIS register.
  ADC1->ISC = (1<<3);
  
  NVIC_EnableIRQ(ADC1SS3_IRQn);
  
  while(1)
  {
    if(adcResult > 2047)
    {
      GPIOF->DATA |= (1<<1);
    }
    else
    {
      GPIOF->DATA &= ~(1<<1);
    }
  }
  
  return 0;
}
