#ifndef __USART_LM4F_H
#define __USART_LM4F_H

#include "LM4F120H5QR.h"
#include "Driver_USART.h"

// USART flags
#define USART_FLAG_INITIALIZED       (1 << 0)
#define USART_FLAG_POWERED           (1 << 1)
#define USART_FLAG_CONFIGURED        (1 << 2)
#define USART_FLAG_TX_ENABLED        (1 << 3)
#define USART_FLAG_RX_ENABLED        (1 << 4)
#define USART_FLAG_SEND_ACTIVE       (1 << 5)
#define USART_FLAG_RECEIVE_ACTIVE    (1 << 6)

// USART Transfer Information (Run-Time)
typedef struct _USART_TRANSFER_INFO {
  uint32_t                rx_num;        // Total number of data to be received
  uint32_t                tx_num;        // Total number of data to be send
  uint8_t                *rx_buf;        // Pointer to in data buffer
  uint8_t                *tx_buf;        // Pointer to out data buffer
  uint32_t                rx_cnt;        // Number of data received
  uint32_t                tx_cnt;        // Number of data sent
  uint8_t                 tx_def_val;    // Transmit default value (used in USART_SYNC_MASTER_MODE_RX)
  uint8_t                 rx_dump_val;   // Receive dump value (used in USART_SYNC_MASTER_MODE_TX)
  uint8_t                 sync_mode;     // Synchronous mode
} USART_TRANSFER_INFO;


// USART Information (Run-Time)
typedef struct _USART_INFO {
  ARM_USART_SignalEvent_t cb_event;      // Event callback
  ARM_USART_STATUS        status;        // Status flags
  USART_TRANSFER_INFO     xfer;          // Transfer information
  uint8_t                 mode;          // USART mode
  uint8_t                 flags;         // USART driver flags
  uint32_t                baudrate;      // Baudrate
} USART_INFO;

// USART Resources definitions
typedef struct {
  IRQn_Type               irq_num;       // USART IRQ Number
  UART0_Type         *uart_reg;      // Pointer to UART peripheral
  USART_INFO             *info;          // Run-Time Information
} const USART_RESOURCES;


void USART_IRQHandler(USART_RESOURCES *usart);
int32_t USART_Send (const void *data, uint32_t num, USART_RESOURCES *usart);
int32_t USART_PowerControl (ARM_POWER_STATE  state, USART_RESOURCES *usart);
int32_t USART_Control(uint32_t control, uint32_t arg, USART_RESOURCES *usart);
int32_t USART_Receive(void *data, uint32_t num, USART_RESOURCES *usart);



#endif 
