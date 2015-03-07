/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        12. December 2014
 * $Revision:    V1.00
 *
 * Driver:       Driver_USART0, Driver_USART1, Driver_USART2, Driver_USART3
 * Configured:   via RTE_Device.h configuration file
 * Project:      USART Driver for LM4F and TM4C
 * -----------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                Value     UART Interface
 *   ---------------------                -----     --------------
 *   Connect to hardware via Driver_UART# = 0       use USART0
 *   Connect to hardware via Driver_UART# = 1       use USART1
 *   Connect to hardware via Driver_UART# = 2       use USART2
 *   Connect to hardware via Driver_UART# = 3       use USART3
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    - Initial release
 */
#include "USART_LM4F.h"
#include "LM4F120H5QR.h"



void USART_IRQHandler(USART_RESOURCES *usart) 
{

    // check the type of interrupt we have
    if( (usart->uart_reg->CTL & (1<<4)) && (usart->uart_reg->MIS & (1<<5)) ) // check if we have a transmit interrupt (check UARTCTL EOT, and  UARTMIS TXRIS)
    {
        // check if the entire data buffer has transmitted
        if(usart->info->xfer.tx_cnt == usart->info->xfer.tx_num)
        {
           // all the data has been transmitted
           usart->uart_reg->ICR |= (1<<5); //  TXIC - TXMIX and TXRIS Transmit Interrupt Clear
           usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;
        }
        else
        {
            // there's still data that needs to be transmitted
          
            if( !(usart->uart_reg->FR & (1<<5))) // check TXFF bit (FF = FIFO Full)
            {
                usart->uart_reg->DR = usart->info->xfer.tx_buf[usart->info->xfer.tx_cnt++];
            }
        }
        return;
    }
    
    if(usart->info->xfer.rx_cnt == usart->info->xfer.rx_num)
    {
        usart->uart_reg->ICR |= (1<<4); 
    }
    else if( !(usart->uart_reg->FR & (1<<4))) 
    {
        // See if there are any characters in the receive FIFO.

      usart->info->xfer.rx_buf[usart->info->xfer.rx_cnt++] = (uint8_t)usart->uart_reg->DR;
      
    }

      
}


int32_t USART_Send (const void *data, uint32_t num, USART_RESOURCES *usart)
{

    if((data == NULL)||(num==0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    
    if( ((UART0->FR & (1<<5)) == 0) &&
       ( !(usart->info->flags & USART_FLAG_SEND_ACTIVE)) )
    {
        usart->info->xfer.tx_buf = (uint8_t*)data;
        usart->info->xfer.tx_num = num;
        usart->info->xfer.tx_cnt = 0;
        usart->info->flags |= USART_FLAG_SEND_ACTIVE;
        
        usart->uart_reg->DR = usart->info->xfer.tx_buf[usart->info->xfer.tx_cnt++];
    }
    else
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
    
    
    return ARM_DRIVER_OK; 
}

int32_t USART_PowerControl (ARM_POWER_STATE  state, USART_RESOURCES *usart)
{
    switch (state)
    {
    case ARM_POWER_OFF:
      NVIC_DisableIRQ(usart->irq_num);
        break;

    case ARM_POWER_LOW:
        break;

    case ARM_POWER_FULL:
        NVIC_ClearPendingIRQ(usart->irq_num);
        NVIC_EnableIRQ(usart->irq_num);
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    
    return ARM_DRIVER_OK;
}

int32_t USART_Control(uint32_t control, uint32_t arg, USART_RESOURCES *usart)
{

    uint8_t wlen=0x0, // word length 5 bits(default)
        fen = 0x1,
        stp2 = 0x0,
        eps=0x0, // even parity select
        pen=0x0, // parity enable
        brk = 0x0;

    int32_t status = ARM_DRIVER_OK;
    
    uint32_t control_bits = control & ARM_USART_CONTROL_Msk,
             data_bits = control & ARM_USART_DATA_BITS_Msk,
             parity_bits = control & ARM_USART_PARITY_Msk,
             stop_bits = control & ARM_USART_STOP_BITS_Msk,
             flow_control_bits = control & ARM_USART_FLOW_CONTROL_Msk;
             
    
    // 1. Disable the UART by clearing the UARTEN bit in the UARTCTL register
    usart->uart_reg->CTL &= ~(1<<0);
        
    switch(control_bits)
    {
    case ARM_USART_MODE_ASYNCHRONOUS:
              // Example calculation for baud=9600
        // Find  the Baud-Rate Divisor
        // BRD = 16,000,000 / (16 * 9600) = 104.16666666666666666666666666666666666666666666666666
        // UARTFBRD[DIVFRAC] = integer(0.166667 * 64 + 0.5) = 11
                        
        // With the BRD values in hand, the UART configuration is written to the module in the following order
                                   

        // 2. Write the integer portion of the BRD to the UARTIBRD register
        usart->uart_reg->IBRD = (uint16_t)( ((float)SystemCoreClock)/ (16*arg));      
        // 3. Write the fractional portion of the BRD to the UARTFBRD register.
        usart->uart_reg->FBRD = (uint8_t)( (( ((float)SystemCoreClock) / (16*arg)) - (uint16_t)( ((float)SystemCoreClock) / (16*arg)) )*64+0.5);  
        
        switch(data_bits)
        {
        case ARM_USART_DATA_BITS_8:
          wlen = 0x3;
          break;
        case ARM_USART_DATA_BITS_5:
          wlen = 0x0;
        case ARM_USART_DATA_BITS_6:
          wlen = 0x1;
        case ARM_USART_DATA_BITS_7:
          wlen = 0x2;
        case ARM_USART_DATA_BITS_9:
        default:
          status |= ARM_USART_ERROR_DATA_BITS;
          break;
        }
    
        switch(parity_bits)
        {
        case ARM_USART_PARITY_NONE:
          pen = 0;
          break;
        case ARM_USART_PARITY_EVEN:
          eps = 1;
        case ARM_USART_PARITY_ODD:
          eps = 0;
        default:
          status |= ARM_USART_ERROR_PARITY;
          break;
        }
    
        switch(stop_bits)
        {
        case ARM_USART_STOP_BITS_1:
          stp2 = 0;
          break;
        case ARM_USART_STOP_BITS_2:
          stp2 = 1;
        default:
          status |= ARM_USART_ERROR_STOP_BITS;
          break;
        }
    
        switch(flow_control_bits)
        {
        case ARM_USART_FLOW_CONTROL_NONE:
          usart->uart_reg->CTL &= ~((1<<14)|(1<<15));
          break;
        default:
          status |= ARM_USART_ERROR_FLOW_CONTROL;
          break;
        }
    
   
        
        // 4. Write the desired serial parameters to the UARTLCRH register

        // select stop bits
        usart->uart_reg->LCRH &= ~(1<<3);  // clear stp2 bit
        usart->uart_reg->LCRH |= (stp2<<3);

                
        // select word length / data bits
        usart->uart_reg->LCRH &= ~(0x3<<5); // clear the wlen bits
        usart->uart_reg->LCRH |= (wlen<<5);
        
        // select parity            
        usart->uart_reg->LCRH &= ~((1<<2)|(1<<1)); // clear eps and pen bits
        usart->uart_reg->LCRH |= (eps<<2)|(pen<<1);
        
        // enable transmist and receive
        usart->uart_reg->CTL |= (1<<8)|(1<<9);
        
        
        break;
    case ARM_USART_CONTROL_TX:
      usart->uart_reg->CTL &= ~(1<<8); // clear TXE bit
      usart->uart_reg->CTL |= (arg<<8);
      break;
    case ARM_USART_CONTROL_RX:
      usart->uart_reg->CTL &= ~(1<<9); // clear RXE bit
      usart->uart_reg->CTL |= (arg<<9);
      break;      
    default:
      status |= ARM_USART_ERROR_MODE;
      break;
    }
      
     // 5. Configure the UART clock source by writing to the UARTCC register
    usart->uart_reg->CC = 0x0; 

    // 6. Optionally, configure the µDMA channel (see “Micro Direct Memory Access (µDMA)” on page 585)
    // and enable the DMA option(s) in the UARTDMACTL register

    // 7. Enable the UART by setting the UARTEN bit in the UARTCTL register.
    usart->uart_reg->CTL |= (1<<0)|(1<<4); 

    return status;
}

int32_t USART_Receive(void *data, uint32_t num, USART_RESOURCES *usart)
{
    if ((data == NULL) || (num == 0)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
    }
    
    if(usart->info->xfer.rx_cnt == usart->info->xfer.rx_num)
    {
      usart->info->status.rx_busy = 0;
    }

    
    if(usart->info->status.rx_busy == 1)
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
    
    usart->info->xfer.rx_buf = (uint8_t*)data;
    usart->info->xfer.rx_cnt = 0;
    usart->info->xfer.rx_num = num;
    usart->info->status.rx_busy = 1;
    

    
}

// UART1 Begins
