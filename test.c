#include "stm32f4xx.h"
#include "lcd.h"
#include "test.h"

#define UP_BUTTON_PIN     (1 << 7)   // PB7
#define DOWN_BUTTON_PIN   (1 << 3)   // PB3
#define LEFT_BUTTON_PIN   (1 << 4)   // PB4
#define ENTER_BUTTON_PIN  (1 << 15)  // PA15

#define NUM_DIGITS        4

uint32_t last_press_up = 0;
uint32_t last_press_down = 0;
uint32_t last_press_left = 0;
uint32_t last_press_enter = 0;
const uint32_t debounce_delay = 300; // 300 ms


uint8_t current_digit_index = 0;
uint8_t entered_code[NUM_DIGITS] = {0, 0, 0, 0};
const uint8_t target_passcode[NUM_DIGITS] = {1, 2, 3, 4
};

volatile uint32_t ms_counter = 0;

void tim3_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 clock

    TIM3->PSC = 8399;     // Prescaler: 84 MHz / (8399 + 1) = 10 kHz
    TIM3->ARR = 9;        // Auto-reload: 10 kHz / (9 + 1) = 1 kHz = 1ms tick

    TIM3->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    TIM3->CR1 |= TIM_CR1_CEN;    // Enable timer

    NVIC_EnableIRQ(TIM3_IRQn);   // Enable TIM3 interrupt in NVIC
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;  // Clear update interrupt flag
        ms_counter++;
    }
}

void delay_timer_ms(uint32_t ms) {
    ms_counter = 0;
    while (ms_counter < ms);
}

// Initialize GPIO for buttons
void buttons_gpio_init(void) {
    // Enable clocks for GPIOB and GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;

    // Set PB7, PB3, PB4 as input with pull-down
    GPIOB->MODER &= ~( (3 << (7 * 2)) | (3 << (3 * 2)) | (3 << (4 * 2)) );
    GPIOB->PUPDR &= ~( (3 << (7 * 2)) | (3 << (3 * 2)) | (3 << (4 * 2)) );
    GPIOB->PUPDR |=  (1 << (7 * 2)) | (1 << (3 * 2)) | (1 << (4 * 2));  // Pull-down

    // Set PA15 as input with pull-down
    GPIOA->MODER &= ~(3 << (15 * 2));
    GPIOA->PUPDR &= ~(3 << (15 * 2));
    GPIOA->PUPDR |=  (1 << (15 * 2));  // Pull-down
}

// Read input pin state
uint8_t read_button(GPIO_TypeDef *port, uint16_t pin_mask) {
	return (port->IDR & pin_mask) ? 0 : 1;

}

void display_code(void) {
    lcd(0x01, 0);
    lcd(0x80, 0);
    lcd_string("Code:");
    lcd(0xC0,0);
    for (int i = 0; i < NUM_DIGITS; i++) {
        if (i == current_digit_index) {
            lcd('(', 1);
            lcd(entered_code[i] + '0', 1);
            lcd(')', 1);
        } else {
            lcd(' ', 1);
            lcd(entered_code[i] + '0', 1);
            lcd(' ', 1);
        }
    }
}

uint8_t check_code(void) {
    for (int i = 0; i < NUM_DIGITS; i++) {
        if (entered_code[i] != target_passcode[i]) return 0;
    }
    return 1;
}
extern void test_mode_run(void);
// Main security loop
void security_check_loop(void) {
    lcd_gpio_init();
    lcd_init();
    buttons_gpio_init();
    tim3_init();
    display_code();

    while (1) {
            if (read_button(GPIOB, UP_BUTTON_PIN)) {
                entered_code[current_digit_index]++;
                if (entered_code[current_digit_index] > 9)
                    entered_code[current_digit_index] = 0;
                display_code();
                last_press_up = ms_counter;
            }

            if (read_button(GPIOB, DOWN_BUTTON_PIN)) {
                if (entered_code[current_digit_index] == 0)
                    entered_code[current_digit_index] = 9;
                else
                    entered_code[current_digit_index]--;
                display_code();
                last_press_up = ms_counter;
            }

            if (read_button(GPIOB, LEFT_BUTTON_PIN) && (ms_counter - last_press_left > debounce_delay)) {
                current_digit_index = (current_digit_index + 1) % NUM_DIGITS;
                display_code();
                last_press_left = ms_counter;
            }


            if (read_button(GPIOA, ENTER_BUTTON_PIN)) {
                lcd(0x01, 0);
                if (check_code()) {
                    lcd_string("ACCESS GRANTED");
                    break;
                } else {
                    lcd_string("WRONG CODE");
                    delay_timer_ms(1000);
                    for (int i = 0; i < NUM_DIGITS; i++) entered_code[i] = 0;
                    current_digit_index = 0;
                    display_code();
                }
                last_press_up = ms_counter;
            }
        }
    // Mode Selection Menu
        uint8_t selected = 0;
        lcd(0x01, 0);
        lcd_string("Select Mode:");

        while (1) {
            lcd(0xC0, 0);
            if (selected == 0) {
                lcd_string("> Test   Run ");
            } else {
                lcd_string("  Test  >Run ");
            }

            if ((read_button(GPIOB, UP_BUTTON_PIN) || read_button(GPIOB, DOWN_BUTTON_PIN)) && (ms_counter - last_press_up > debounce_delay)) {
                selected ^= 1;
                //delay_timer_ms(300);
                last_press_up = ms_counter;
            }

            if (read_button(GPIOA, ENTER_BUTTON_PIN) && (ms_counter - last_press_enter > debounce_delay)) {
                lcd(0x01, 0);
                if (selected == 0) {
                    lcd_string("Test Mode");
                    lcd(0xC0, 0);
                    test_mode_run();

                } else {
                    lcd_string("Run Mode");
                   // delay_timer_ms(1000);
                    // Placeholder for run_mode_run();
                }
                //break;
            }
        }
    }



