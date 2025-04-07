
extern "C" {
#include <string.h>
}
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

//#include "wiring_private.h"
#include "Uart.h"
//#include <hal_gpio.h>
//#include <SEGGER_RTT.h>

#define NO_RTS_PIN 255
#define NO_CTS_PIN 255
#define RTS_RX_THRESHOLD 10

void Uart::pin_set_peripheral_function(uint32_t pinmux)
{
	uint8_t port = (uint8_t)((pinmux >> 16)/32);
	PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >> 16) & 0x01u)));
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux &0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));
}


Uart::Uart(SERCOM *_s, uint32_t _pinRX, uint32_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX)
{
	sercom = _s;
  	uc_pinRX = _pinRX;
  	uc_pinTX = _pinTX;
	uc_padRX = _padRX ;
  	uc_padTX = _padTX;

}

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{  
  	pin_set_peripheral_function(uc_pinRX);
	pin_set_peripheral_function(uc_pinTX);  
	
    sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
    sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
    sercom->initPads(uc_padTX, uc_padRX);

  sercom->enableUART();
}

/*void Uart::end()
{
  sercom->resetUART();
  rxBuffer.clear();
  txBuffer.clear();
}*/

void Uart::flush()
{
  while(txBuffer.available()); // wait until TX buffer is empty

  sercom->flushUART();
}

void Uart::IrqHandler()
{
  if (sercom->isFrameErrorUART()) {
    // frame error, next byte is invalid so read and discard it
    sercom->readDataUART();

    sercom->clearFrameErrorUART();
  }

  if (sercom->availableDataUART()) {
    rxBuffer.store_char(sercom->readDataUART());

    /*if (uc_pinRTS != NO_RTS_PIN) {
      // RX buffer space is below the threshold, de-assert RTS
      if (rxBuffer.availableForStore() < RTS_RX_THRESHOLD) {
        *pul_outsetRTS = ul_pinMaskRTS;
      }
    }*/	
  }

  if (sercom->isDataRegisterEmptyUART()) {
    if (txBuffer.available()) {
      uint8_t data = txBuffer.read_char();

      sercom->writeDataUART(data);
    } else {
      sercom->disableDataRegisterEmptyInterruptUART();
    }	 
  }

  if (sercom->isUARTError()) {
    sercom->acknowledgeUARTError();
    // TODO: if (sercom->isBufferOverflowErrorUART()) ....
    // TODO: if (sercom->isParityErrorUART()) ....
    sercom->clearStatusUART();	
  }
}

int Uart::available()
{
  return rxBuffer.available();
}

int Uart::availableForWrite()
{
  return txBuffer.availableForStore();
}
/*
int Uart::peek()
{
  return rxBuffer.peek();
}*/

int Uart::read()
{
  int c = rxBuffer.read_char(); 

  return c;
}


size_t Uart::write(const uint8_t data)
{
  if (sercom->isDataRegisterEmptyUART() && txBuffer.available() == 0) {
    sercom->writeDataUART(data);
  } else {
    // spin lock until a spot opens up in the buffer
    while(txBuffer.isFull()) {
      uint8_t interruptsEnabled = ((__get_PRIMASK() & 0x1) == 0);

      if (interruptsEnabled) {
        uint32_t exceptionNumber = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);

        if (exceptionNumber == 0 ||
              NVIC_GetPriority((IRQn_Type)(exceptionNumber - 16)) > SERCOM_NVIC_PRIORITY) {
          // no exception or called from an ISR with lower priority,
          // wait for free buffer spot via IRQ
          continue;
        }
      }

      // interrupts are disabled or called from ISR with higher or equal priority than the SERCOM IRQ
      // manually call the UART IRQ handler when the data register is empty
      if (sercom->isDataRegisterEmptyUART()) {
        IrqHandler();
      }
    }

    txBuffer.store_char(data);

    sercom->enableDataRegisterEmptyInterruptUART();
  }

  return 1;
}

size_t Uart::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--) {
	if (write(*buffer++)){
	  flush();
	  n++;
	}
    else break;
  }
  return n;
  //for(int i=0; i<size;i++)write(buffer[i]);
}



SercomNumberStopBit Uart::extractNbStopBit(uint16_t config)
{
  switch(config & SERIAL_STOP_BIT_MASK)
  {
    case SERIAL_STOP_BIT_1:
    default:
      return SERCOM_STOP_BIT_1;

    case SERIAL_STOP_BIT_2:
      return SERCOM_STOP_BITS_2;
  }
}

SercomUartCharSize Uart::extractCharSize(uint16_t config)
{
  switch(config & SERIAL_DATA_MASK)
  {
    case SERIAL_DATA_5:
      return UART_CHAR_SIZE_5_BITS;

    case SERIAL_DATA_6:
      return UART_CHAR_SIZE_6_BITS;

    case SERIAL_DATA_7:
      return UART_CHAR_SIZE_7_BITS;

    case SERIAL_DATA_8:
    default:
      return UART_CHAR_SIZE_8_BITS;

  }
}

SercomParityMode Uart::extractParity(uint16_t config)
{
  switch(config & SERIAL_PARITY_MASK)
  {
    case SERIAL_PARITY_NONE:
    default:
      return SERCOM_NO_PARITY;

    case SERIAL_PARITY_EVEN:
      return SERCOM_EVEN_PARITY;

    case SERIAL_PARITY_ODD:
      return SERCOM_ODD_PARITY;
  }
}
	
int Uart::print(const char* format, ...)
{
    char dest[1024];
    va_list argptr;
    va_start(argptr, format);
    int len=vsprintf(dest, format, argptr);
    va_end(argptr);    
	write((uint8_t*)dest, len);	
	return len;
}
	  
