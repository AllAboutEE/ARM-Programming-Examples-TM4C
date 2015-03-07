#include <Driver_USART.h>

extern ARM_DRIVER_USART Driver_USART0;

int main(void) 
{
    ARM_DRIVER_USART* USARTpc = &Driver_USART0;

    USARTpc->Initialize(NULL);
    USARTpc->PowerControl(ARM_POWER_FULL);
    USARTpc->Control(
        ARM_USART_MODE_ASYNCHRONOUS | 
        ARM_USART_DATA_BITS_8 | 
        ARM_USART_PARITY_NONE | 
        ARM_USART_STOP_BITS_1 , 9600
    );

    char data[] = "Hello World!\r\n";

    USARTpc->Send(data, sizeof(data)/sizeof(data[0]));
    // Create an array where the data will be read to. 
    char dataR[5];
    while(1)
    {
        if( USARTpc->Receive(dataR,sizeof(dataR)/sizeof(dataR[0])) != ARM_DRIVER_ERROR_BUSY)
        {
                USARTpc->Send(dataR,sizeof(dataR)/sizeof(dataR[0]));
        }
    }
}

