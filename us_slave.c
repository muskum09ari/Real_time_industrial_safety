#include "stm32f4xx.h"
#include "lcd.h"
#include <stdint.h>
 
// Initialize USART3 on PC11 (RX)
void usart3_init(void) {
    // Enable GPIOC and USART3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
 
    // Set PC11 to alternate function (AF7 for USART3 RX)
    GPIOC->MODER &= ~(3 << (2 * 11));
    GPIOC->MODER |=  (2 << (2 * 11));  // AF mode
    GPIOC->AFR[1] &= ~(0xF << ((11 - 8) * 4));
    GPIOC->AFR[1] |=  (7 << ((11 - 8) * 4));  // AF7
 
    // Configure USART3: 9600 baud, 8N1
    USART3->BRR = 0x683;  // for 16 MHz system clock
    USART3->CR1 |= USART_CR1_RE;  // Enable Receiver
    USART3->CR1 |= USART_CR1_UE;  // Enable USART
}
 
// Blocking USART3 character read
uint8_t usart3_read_char(void) {
    while (!(USART3->SR & USART_SR_RXNE));
    return (uint8_t)USART3->DR;
}
 
int main(void) {
    // Initialize LCD
    lcd_gpio_init();
    lcd_init();
 
    // Initialize USART3
    usart3_init();
 
    lcd(0x01, 0);  // Clear display
    lcd_string("Receiver Ready");
 
    // Receive characters
    char buffer[32];
    uint8_t idx = 0;
    char c;
 
    // Read until '\r' or '\n' or buffer is full
    while (idx < sizeof(buffer) - 1) {
        c = usart3_read_char();
        if (c == '\r' || c == '\n') break;
        buffer[idx++] = c;
    }
    buffer[idx] = '\0';  // Null-terminate
 
    // Display received string
    lcd(0x01, 0);         // Clear LCD
    lcd_string("Received:");
    lcd(0xC0, 0);
    lcd_string(buffer);
 
    while (1);  // Idle
}
