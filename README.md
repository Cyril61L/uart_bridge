# **UART transparent bridge for STM32**
Target : NUCLEO-F103RB  
This programs realize a bridge beetween USART1 and USART2 using interruption (not DMA).  
Use ST Low layer API.  


### **USART_irqHandler**  
**Event RXNE** : Rx register conains byte, at each byte received the programs push the bytes received into rxBuffer.

**Event IDLE** : The uart IDLE event is used to detect the end of a chunk (trame), when this event is triggred the program send the rxBuffer to the other UART, then it clear the buffer.
