#include "stm32f4xx.h"
#include "lcd.h"
#include "test.h"
#include <stdint.h>
// External variables from secq.c
extern volatile uint32_t ms_counter;
extern void delay_timer_ms(uint32_t ms);
extern uint8_t read_button(GPIO_TypeDef *port, uint16_t pin_mask);

// TB6612FNG Motor Driver Pins - Fixed definitions
#define STDBY_PIN       (1 << 4)   // PA4
#define FAN_AIN1        (1 << 6)   // PA6
#define FAN_AIN2        (1 << 7)   // PA7
#define PUMP_BIN1       (1 << 7)   // PC7
#define PUMP_BIN2       (1 << 8)   // PC8
#define FAN_PWM         (1 << 8)   // PA8
#define PUMP_PWM        (1 << 9)   // PA9

#define RED_LED_PIN     (1 << 2)   // PA2
#define BLUE_LED_PIN    (1 << 3)   // PA3
#define BUZZER_PIN      (1 << 9)   // PC9

#define FIRE_SWITCH_PIN (1 << 7)   // PB7
#define SMOKE_SWITCH_PIN (1 << 3)  // PB3
#define GAS_POT_PIN      (1 << 1)  // PC1

#define UART3_TX_PIN     (1 << 10)  // PC10
#define UART3_RX_PIN     (1 << 11)  // PC11
#define ENTER_BUTTON_PIN  (1 << 15) // PA15

#define SWITCH_TEST_COUNT 5
#define SWITCH_TIMEOUT_MS 10000

// PWM Timer definitions
#define TIM1_PSC_VALUE  (84 - 1)    // 1 MHz PWM clock
#define TIM1_ARR_VALUE  (1000 - 1)  // ~1kHz PWM frequency

// Button definitions from secq.c
#define UP_BUTTON_PIN     (1 << 7)   // PB7
#define DOWN_BUTTON_PIN   (1 << 3)   // PB3
#define LEFT_BUTTON_PIN   (1 << 4)   // PB4

typedef enum {
    TEST_PASS = 0,
    TEST_FAIL = 1,
    TEST_TIMEOUT = 2
} test_result_t;

// Function prototypes
void test_gpio_init(void);
void tim1_pwm_init(void);
void adc_init(void);
void test_uart_init(void);
void motor_a_set_speed_dir(uint16_t speed_percent, uint8_t direction);
void motor_b_set_speed_dir(uint16_t speed_percent, uint8_t direction);
void test_buzzer(void);
void test_leds(void);
void test_motor(void);
void test_pump(void);
void test_fire_switch(void);
void test_smoke_switch(void);
void test_gas_sensor(void);
void test_uart_communication(void);
void uart3_send_string(const char *str);
void uart3_send_char(char c);
char uart3_receive_char(void);
uint16_t read_adc(void);
void display_test_result(const char* test_name, test_result_t result);

void test_gpio_init(void) {
    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // Motor A direction pins: PA6, PA7 (Output mode)
    GPIOA->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOA->MODER |= (1 << (6 * 2)) | (1 << (7 * 2));

    // Motor A PWM: PA8 -> AF1 (TIM1_CH1)
    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->MODER |= (2 << (8 * 2)); // Alternate function
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    GPIOA->AFR[1] |= (1 << ((8 - 8) * 4));  // AF1

    // Motor B direction pins: PC7, PC8 (Output mode)
    GPIOC->MODER &= ~((3 << (7 * 2)) | (3 << (8 * 2)));
    GPIOC->MODER |= (1 << (7 * 2)) | (1 << (8 * 2));

    // Motor B PWM: PA9 -> AF1 (TIM1_CH2)
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |= (2 << (9 * 2)); // Alternate function
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));
    GPIOA->AFR[1] |= (1 << ((9 - 8) * 4));  // AF1

    // STBY: PA4 as output
    GPIOA->MODER &= ~(3 << (4 * 2));
    GPIOA->MODER |= (1 << (4 * 2));

    // LEDs: PA2, PA3 (Output mode)
    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
    GPIOA->MODER |= (1 << (2 * 2)) | (1 << (3 * 2));

    // Buzzer: PC9 (Output mode)
    GPIOC->MODER &= ~(3 << (9 * 2));
    GPIOC->MODER |= (1 << (9 * 2));

    // Input: Fire & Smoke switches with pull-up
    GPIOB->MODER &= ~((3 << (7 * 2)) | (3 << (3 * 2)));
    GPIOB->PUPDR |= (1 << (7 * 2)) | (1 << (3 * 2));

    // Enter button: PA15 with pull-up
    GPIOA->MODER &= ~(3 << (15 * 2));
    GPIOA->PUPDR |= (1 << (15 * 2));

    // ADC pin for gas sensor: PC1 (Analog mode)
    GPIOC->MODER |= (3 << (1 * 2));

    // UART3 alternate function (PC10, PC11)
    GPIOC->MODER |= (2 << (10 * 2)) | (2 << (11 * 2));
    GPIOC->AFR[1] |= (7 << ((10 - 8) * 4)) | (7 << ((11 - 8) * 4));

    // Initialize all outputs to low
    GPIOA->ODR &= ~(RED_LED_PIN | BLUE_LED_PIN | STDBY_PIN | FAN_AIN1 | FAN_AIN2);
    GPIOC->ODR &= ~(PUMP_BIN1 | PUMP_BIN2 | BUZZER_PIN);
}

// Initialize TIM1 for PWM generation
void tim1_pwm_init(void) {
    // Enable TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Configure timer
    TIM1->PSC = TIM1_PSC_VALUE;
    TIM1->ARR = TIM1_ARR_VALUE;

    // Channel 1 - PA8 (Motor A PWM)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCER |= TIM_CCER_CC1E;

    // Channel 2 - PA9 (Motor B PWM)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCER |= TIM_CCER_CC2E;

    // Enable timer
    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->BDTR |= TIM_BDTR_MOE;  // Main output enable
    TIM1->CR1 |= TIM_CR1_CEN;    // Counter enable
}

// Motor A control function
void motor_a_set_speed_dir(uint16_t speed_percent, uint8_t direction) {
    if (speed_percent > 100) speed_percent = 100;
    uint32_t pwm_value = ((TIM1->ARR + 1) * speed_percent) / 100;
    TIM1->CCR1 = pwm_value;

    if (direction) {
        // Forward
        GPIOA->BSRR = (1 << 6);      // Set AIN1
        GPIOA->BSRR = (1 << (7 + 16)); // Reset AIN2
    } else {
        // Backward
        GPIOA->BSRR = (1 << (6 + 16)); // Reset AIN1
        GPIOA->BSRR = (1 << 7);        // Set AIN2
    }
}

// Motor B control function
void motor_b_set_speed_dir(uint16_t speed_percent, uint8_t direction) {
    if (speed_percent > 100) speed_percent = 100;
    uint32_t pwm_value = ((TIM1->ARR + 1) * speed_percent) / 100;
    TIM1->CCR2 = pwm_value;

    if (direction) {
        // Forward
        GPIOC->BSRR = (1 << 7);        // Set BIN1
        GPIOC->BSRR = (1 << (8 + 16)); // Reset BIN2
    } else {
        // Backward
        GPIOC->BSRR = (1 << (7 + 16)); // Reset BIN1
        GPIOC->BSRR = (1 << 8);        // Set BIN2
    }
}

// Initialize ADC for gas sensor
void adc_init(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC1
    ADC1->CR2 |= ADC_CR2_ADON;      // Turn on ADC
    ADC1->SQR1 = 0;                 // 1 conversion
    ADC1->SQR3 = 11;                // Channel 11 (PC1)
    ADC1->SMPR1 |= (7 << (1 * 3));  // Sampling time for channel 11

    delay_timer_ms(10);  // Wait for ADC to stabilize
}

// Initialize UART3
void test_uart_init(void) {
    // Enable USART3 clock
    RCC->APB1ENR |= (1 << 18);  // Enable clock for USART3

    // Configure USART3
    USART3->BRR = 0X683;   // Baud rate setting (9600 @16MHz)
    USART3->CR1 |= (1 << 3);  // Enable Transmitter
   USART3->CR1 |= (1 << 2);  // Enable Receiver
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->CR1 |= (1 << 13); // Enable USART3
}

// Test 1: Buzzer test
void test_buzzer(void) {
    lcd(0x01, 0);  // Clear display
    lcd_string("Buzzer Test");
    lcd(0xC0, 0);  // Second line
    lcd_string("Beeping...");

    // Beep for 1 second
    GPIOC->ODR |= BUZZER_PIN;  // Turn on buzzer
    delay_timer_ms(100);      // Wait 1 second
    GPIOC->ODR &= ~BUZZER_PIN; // Turn off buzzer

    display_test_result("Buzzer", TEST_PASS);
    delay_timer_ms(1000);
}

// Test 2: LED blink test
void test_leds(void) {
    lcd(0x01, 0);
    lcd_string("LED Blink Test");
    lcd(0xC0, 0);
    lcd_string("Blinking...");

    // Blink for 5 seconds
    for (int i = 0; i < 10; i++) {
        GPIOA->ODR ^= (RED_LED_PIN | BLUE_LED_PIN);
        delay_timer_ms(250);
    }

    // Turn off LEDs
    GPIOA->ODR &= ~(RED_LED_PIN | BLUE_LED_PIN);

    display_test_result("LED Blink", TEST_PASS);
    delay_timer_ms(1000);
}

// Test 3: DC Motor test - Fixed
void test_motor(void) {
    lcd(0x01, 0);
    lcd_string("DC Motor Test");
    lcd(0xC0, 0);
    lcd_string("Running...");

    // Enable standby pin
    GPIOA->ODR |= STDBY_PIN;

    // Run motor A forward at 50% speed for 1 seconds
    motor_a_set_speed_dir(50, 1);
    delay_timer_ms(1000);

    // Stop motor A
    motor_a_set_speed_dir(0, 1);

    // Run motor A backward at 50% speed for 1 seconds
    motor_a_set_speed_dir(50, 0);
    delay_timer_ms(1000);

    // Stop motor A
    motor_a_set_speed_dir(0, 1);

    // Disable standby pin
    GPIOA->ODR &= ~STDBY_PIN;

    display_test_result("DC Motor", TEST_PASS);
    delay_timer_ms(100);
}

// Test 4: Water pump test - Fixed
void test_pump(void) {
    lcd(0x01, 0);
    lcd_string("Water Pump Test");
    lcd(0xC0, 0);
    lcd_string("Running...");

    // Enable standby pin
    GPIOA->ODR |= STDBY_PIN;

    // Run pump (Motor B) forward at 70% speed for 1 seconds
    motor_b_set_speed_dir(70, 1);
    delay_timer_ms(1000);

    // Stop pump
    motor_b_set_speed_dir(0, 1);

    // Run pump backward at 70% speed for 1 seconds
    motor_b_set_speed_dir(70, 0);
    delay_timer_ms(1000);

    // Stop pump
    motor_b_set_speed_dir(0, 1);

    // Disable standby pin
    GPIOA->ODR &= ~STDBY_PIN;

    display_test_result("Water Pump", TEST_PASS);
    delay_timer_ms(100);
}

// Test 5: Fire sensor test
void test_fire_switch(void) {
    lcd(0x01, 0);
    lcd_string("Fire Sensor Test");
    lcd(0xC0, 0);
    lcd_string("Press 5 times");

    uint8_t press_count = 0;
    uint32_t start_time = ms_counter;
    uint32_t last_press_time = 0;
    test_result_t result = TEST_PASS;

    while (press_count < SWITCH_TEST_COUNT) {
        // Check timeout
        if ((ms_counter - start_time) > SWITCH_TIMEOUT_MS) {
            result = TEST_TIMEOUT;
            break;
        }

        // Check for button press with debouncing
        if (read_button(GPIOB, FIRE_SWITCH_PIN) &&
            (ms_counter - last_press_time > 300)) {
            press_count++;
            last_press_time = ms_counter;

            // Update display
            lcd(0xC0, 0);
            lcd_string("Count: ");
            lcd(press_count + '0', 1);
            lcd_string("/5    ");
        }
    }

    display_test_result("Fire Sensor", result);
    delay_timer_ms(100);
}

// Test 6: Smoke sensor test
void test_smoke_switch(void) {
    lcd(0x01, 0);
    lcd_string("Smoke Sensor Test");
    lcd(0xC0, 0);
    lcd_string("Press 5 times");

    uint8_t press_count = 0;
    uint32_t start_time = ms_counter;
    uint32_t last_press_time = 0;
    test_result_t result = TEST_PASS;

    while (press_count < SWITCH_TEST_COUNT) {
        // Check timeout
        if ((ms_counter - start_time) > SWITCH_TIMEOUT_MS) {
            result = TEST_TIMEOUT;
            break;
        }

        // Check for button press with debouncing
        if (read_button(GPIOB, SMOKE_SWITCH_PIN) &&
            (ms_counter - last_press_time > 300)) {
            press_count++;
            last_press_time = ms_counter;

            // Update display
            lcd(0xC0, 0);
            lcd_string("Count: ");
            lcd(press_count + '0', 1);
            lcd_string("/5    ");
        }
    }

    display_test_result("Smoke Sensor", result);
    delay_timer_ms(100);
}

// Test 7: Gas sensor test
void test_gas_sensor(void) {
    lcd(0x01, 0);
    lcd_string("Gas Sensor Test");

    for (int i = 0; i < 10; i++) {
        uint16_t adc_value = read_adc();
        float voltage = (adc_value * 3.3) / 4095.0;

        lcd(0xC0, 0);
        lcd_string("V:");

        // Display voltage with 2 decimal places
        int volt_int = (int)voltage;
        int volt_frac = (int)((voltage - volt_int) * 100);

        lcd(volt_int + '0', 1);
        lcd('.', 1);
        lcd((volt_frac / 10) + '0', 1);
        lcd((volt_frac % 10) + '0', 1);
        lcd('V', 1);
        lcd_string("    ");

        delay_timer_ms(100);
    }

    display_test_result("Gas Sensor", TEST_PASS);
    delay_timer_ms(100);
}

// Test 8: UART communication test
void test_uart_communication(void) {
    lcd(0x01, 0);
    lcd_string("UART Test");
    lcd(0xC0, 0);
    lcd_string("Sending...");

    // Send message
    uart3_send_string("Welcome\r\n");

    delay_timer_ms(500);  // Give slave time to process

    lcd(0xC0, 0);
    lcd_string("Waiting...");

    // Wait for slave ACK
    uint32_t start_time = ms_counter;
    char received = 0;

    while ((ms_counter - start_time) < 1000) {  // 1s timeout
        if (USART3->SR & USART_SR_RXNE) {
            received = uart3_receive_char();
            break;
        }
    }

    if (received == 'A') {  // Only consider 'A' as valid ACK
        display_test_result("UART Comm", TEST_PASS);
    } else {
        display_test_result("UART Comm", TEST_TIMEOUT);
    }

    delay_timer_ms(1000);
}


// Read ADC value
uint16_t read_adc(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;  // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));  // Wait for conversion complete
    return ADC1->DR;
}

// UART3 send character
void uart3_send_char(char c) {
    while (!(USART3->SR & USART_SR_TXE));
    USART3->DR = c;
}

// UART3 send string
void uart3_send_string(const char *str) {
    while (*str) {
        uart3_send_char(*str++);
    }
}

// UART3 receive character
char uart3_receive_char(void) {
    while (!(USART3->SR & USART_SR_RXNE));
    return USART3->DR;
}

// Display test result

void display_test_result(const char* test_name, test_result_t result) {
    lcd(0x01, 0);
    lcd_string((char*)test_name);
    lcd(0xC0, 0);

    switch (result) {
        case TEST_PASS:
            lcd_string("PASS");
            break;
        case TEST_FAIL:
            lcd_string("FAIL");
            break;
        case TEST_TIMEOUT:
            lcd_string("TIMEOUT");
            break;
    }

    delay_timer_ms(1000);
}

// Main test mode function
void test_mode_run(void) {
    // Initialize all peripherals
    test_gpio_init();
    tim1_pwm_init();  // Initialize PWM for motors
    adc_init();
    test_uart_init();

    lcd(0x01, 0);
    lcd_string("Test Mode Start");
    delay_timer_ms(1000);

    // Run all tests in sequence

    test_buzzer();
    test_leds();
    test_motor();
    test_pump();
    test_fire_switch();
    test_smoke_switch();
    test_gas_sensor();
    test_uart_communication();


    // Display final results
    lcd(0x01, 0);
    lcd_string("All Tests Done");
    lcd(0xC0, 0);
    lcd_string("Press ENTER");

    // Wait for user input to return
    while (!read_button(GPIOA, ENTER_BUTTON_PIN)) {
        delay_timer_ms(10);
    }

    delay_timer_ms(300);  // Debounce delay
}
