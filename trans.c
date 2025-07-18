#include "stm32f4xx.h"
 
#include "lcd.h"
 
#include <stdint.h>
 
// -------- USART3 Initializer for PC10 (TX)
 
void usart3_init(void) {
 
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Enable GPIOC clock
 
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 clock
 
    // PC10 (TX) & PC11 (RX) alternate function 7
 
    GPIOC->MODER &= ~((3 << (2*10)) | (3 << (2*11)));
 
    GPIOC->MODER |=  (2 << (2*10)) | (2 << (2*11));
 
    GPIOC->AFR[1] &= ~((0xF << ((10-8)*4)) | (0xF << ((11-8)*4)));
 
    GPIOC->AFR[1] |=  (7 << ((10-8)*4)) | (7 << ((11-8)*4));
 
    USART3->BRR = 0x683; // 9600 baud @ 16MHz
 
    USART3->CR1 |= USART_CR1_TE;  // Transmitter enable
 
    USART3->CR1 |= USART_CR1_UE;  // USART enable
 
}
 
void usart_write(uint8_t val) {
 
    while(!(USART3->SR & USART_SR_TXE));
 
    USART3->DR = val;
 
}
 
void usart_send_string(const char *str) {
 
    while(*str) {
 
        usart_write((uint8_t)*str++);
 
    }
 
}
 
int main(void) {
 
    lcd_gpio_init();
 
    lcd_init();
 
    usart3_init();
 
    lcd(0x01,0);
 
    lcd_string("Mater Ready");
 
    char *text = "Welcome";
 
    usart_send_string(text);   // Send string
 
    usart_write('\n');         // Send delimiter (\n) at end
 
    lcd(0xC0,0);
 
    lcd_string("Sent!");
 
    while(1);
 
