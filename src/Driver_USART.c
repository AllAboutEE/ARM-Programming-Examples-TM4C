#include "Driver_USART.h"
#include "USART_LM4F.h"

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 00)

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };


// UART0 declarations
static USART_INFO USART0_Info = {0};

static const USART_RESOURCES USART0_Resources = {
  UART0_IRQn,
  UART0,
  &USART0_Info
};


void UART0_Handler(void)
{
    USART_IRQHandler(&USART0_Resources);
}



ARM_DRIVER_VERSION ARM_USART0_GetVersion(void)
{
}

ARM_USART_CAPABILITIES ARM_USART0_GetCapabilities(void)
{
}

int32_t ARM_USART0_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    // 1. Enable the UART module using the RCGCUART register (see page 344).
    SYSCTL->RCGCUART |= (1<<0);
    // 2. Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 340).
    // To find out which GPIO port to enable, refer to Table 23-5 on page 1351.
    SYSCTL->RCGCGPIO |= (1<<0); 
    // 3. Set the GPIO AFSEL bits for the appropriate pins (see page 671). To determine which GPIOs to
    // configure, see Table 23-4 on page 1344
    GPIOA->AFSEL |= (1<<1)|(1<<0); 
	// 4. Configure the GPIO current level and/or slew rate as specified for the mode selected (see
    // page 673 and page 681
    
    // 5. Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate
    // pins (see page 688 and Table 23-5 on page 1351).
    GPIOA->PCTL |= (1<<0)|(1<<4);  
    GPIOA->DEN |= (1<<0)|(1<<1); 
    
    UART0->ICR |= (1<<5)|(1<<4); // Transmit and receive Interrupt Clear
    UART0->IM |= (1<<5)|(1<<4); // Enable transmit and receive interrupts
    
    return ARM_DRIVER_OK;
}

int32_t ARM_USART0_Uninitialize(void)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}



int32_t ARM_USART0_PowerControl(ARM_POWER_STATE state)
{
    return USART_PowerControl(state, &USART0_Resources);
}

int32_t ARM_USART0_Send(const void *data, uint32_t num)
{
    return USART_Send(data,num,&USART0_Resources);
}


int32_t ARM_USART0_Receive(void *data, uint32_t num)
{

  return USART_Receive(data,num,&USART0_Resources);
}

int32_t ARM_USART0_Transfer(const void *data_out, void *data_in, uint32_t num)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

uint32_t ARM_USART0_GetTxCount(void)
{
  return 0;
}

uint32_t ARM_USART0_GetRxCount(void)
{
  return 0;
}

int32_t ARM_USART0_Control(uint32_t control, uint32_t arg)
{
    return USART_Control(control,arg,&USART0_Resources);
}

ARM_USART_STATUS ARM_USART0_GetStatus(void)
{
}

int32_t ARM_USART0_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

ARM_USART_MODEM_STATUS ARM_USART0_GetModemStatus(void)
{
}

void ARM_USART_SignalEvent(uint32_t event)
{
    // function body
}

// End USART Interface

ARM_DRIVER_USART Driver_USART0 = {
    ARM_USART0_GetVersion,
    ARM_USART0_GetCapabilities,
    ARM_USART0_Initialize,
    ARM_USART0_Uninitialize,
    ARM_USART0_PowerControl,
    ARM_USART0_Send,
    ARM_USART0_Receive,
    ARM_USART0_Transfer,
    ARM_USART0_GetTxCount,
    ARM_USART0_GetRxCount,
    ARM_USART0_Control,
    ARM_USART0_GetStatus,
    ARM_USART0_SetModemControl,
    ARM_USART0_GetModemStatus
};
