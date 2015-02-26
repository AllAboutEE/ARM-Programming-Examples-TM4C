/**
 * @author Carlos Alvarez (2/26/2015) 
 *  
 * @details This example toggles the RGB LED's on the board.
 *        See detail tutorial at: https:https://www.youtube.com/watch?v=M-p5xOiXrks
 */

#include <lm4f120h5qr.h>

void wait(void);

int main()
{
  SYSCTL->RCGCGPIO = 0x20; //0b100000 (enable PORT F only)
  GPIOF->DIR = 0xE; //0b00001110 (making pins PF1,2,3 outputs)
  GPIOF->DEN = 0xE; //Digital enabling pints PF1,2,3
  
  while(1)
  {
    GPIOF->DATA = 0x02;
    wait();
    GPIOF->DATA = 0x04;
    wait();
    GPIOF->DATA = 0x08;
    wait();
  }

  
  return 0;
}

void wait()
{
  int clockCounter = 0;
  while(clockCounter++ < 1000000);
}
