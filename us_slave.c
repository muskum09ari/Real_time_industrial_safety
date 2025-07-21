#include "stm32f4xx.h"
#include "lcd.h"
#include <stdint.h>

// USART3 Init
void usart3_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    // PC10 (TX), PC11 (RX) as AF7
    GPIOC->MODER &= ~((3 << (2 * 10)) | (3 << (2 * 11)));
    GPIOC->MODER |=  (2 << (2 * 10)) | (2 << (2 * 11));
    GPIOC->AFR[1] &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((11 - 8) * 4)));
    GPIOC->AFR[1] |=  (7 << ((10 - 8) * 4)) | (7 << ((11 - 8) * 4));

    USART3->BRR = 0x683;  // 9600 baud @ 16MHz
    USART3->CR1 |= USART_CR1_RE | USART_CR1_TE;  // Enable RX and TX
    USART3->CR1 |= USART_CR1_UE;  // Enable USART
}

uint8_t usart3_read_char(void) {
    while (!(USART3->SR & USART_SR_RXNE));
    return (uint8_t)USART3->DR;
}

void usart3_send_char(char c) {
    while (!(USART3->SR & USART_SR_TXE));
    USART3->DR = c;
}

int main(void) {
    lcd_gpio_init();
    lcd_init();
    usart3_init();

    lcd(0x01, 0);
    lcd_string("Receiver Ready");

    char buffer[32];
    uint8_t idx = 0;
    char c;

    // Receive message
    while (idx < sizeof(buffer) - 1) {
        c = usart3_read_char();
        if (c == '\r' || c == '\n') break;
        buffer[idx++] = c;
    }
    buffer[idx] = '\0';

    // Display received msg
    lcd(0x01, 0);
    lcd_string("Received:");
    lcd(0xC0, 0);
    lcd_string(buffer);

    // Echo back to confirm connection
    usart3_send_char('A');

    while (1);
}
