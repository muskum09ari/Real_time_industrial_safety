#include "stm32f4xx.h"
#include "lcd.h"
#include "secq.h"
#include "test.h"

extern void delay_timer_ms(uint32_t ms);
extern void tim3_init(void);

// Declare the function from secq.c
void delay_timer_ms(uint32_t ms);  // Uses Timer 3
void buttons_gpio_init(void);
void security_check_loop(void);
void display_code(void);
uint8_t read_button(GPIO_TypeDef *port, uint16_t pin_mask);
uint8_t check_code(void);
void security_check_loop(void);


int main(void) {
    SystemInit();   // Initialize system clock (CMSIS)
    tim3_init();
    security_check_loop();  // Block here until correct passcode entered

    // After successful login
    lcd(0x01, 0);             // Clear LCD
    lcd_string("Mode Select"); // Just a placeholder text



    while (1) {
        // Infinite loop - System runs after passcode
    }
}
