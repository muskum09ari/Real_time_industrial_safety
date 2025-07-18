#include "stm32f4xx.h"
#include "lcd.h"
#include <stdint.h>
 
// -------- USART3 Initializer for PC11 (RX)
void usart3_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
 
    GPIOC->MODER &= ~((3 << (2*10)) | (3 << (2*11)));
    GPIOC->MODER |=  (2 << (2*10)) | (2 << (2*11));
    GPIOC->AFR[1] &= ~((0xF << ((10-8)*4)) | (0xF << ((11-8)*4)));
    GPIOC->AFR[1] |=  (7 << ((10-8)*4)) | (7 << ((11-8)*4));
 
    USART3->BRR = 0x683;
    USART3->CR1 |= USART_CR1_RE; // Receiver enable
    USART3->CR1 |= USART_CR1_UE; // USART enable
}
 
uint8_t usart_read(void) {
    while(!(USART3->SR & USART_SR_RXNE));
    return USART3->DR;
}
 
int main(void) {
    lcd_gpio_init();
    lcd_init();
    usart3_init();
 
    lcd(0x01,0);
    lcd_string("Receiver Ready");
 
    char buffer[32];
    uint8_t idx = 0, c = 0;
 
    // Read characters until '\n' received or buffer full
    while(idx < sizeof(buffer)-1) {
        c = usart_read();
        if(c == '\n' || c == '\r') break;
        buffer[idx++] = c;
        // Optional: show receiving char for debug
    }
    buffer[idx] = '\0';  // Null terminate
 
    lcd(0x01,0);
    lcd_string("Received:");
    lcd(0xC0,0);
    lcd_string(buffer);
 
    while(1);
}
