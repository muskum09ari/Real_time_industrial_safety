#include "stm32f4xx.h"
#include "lcd.h"
#include "test.h"

// External variables from secq.c
extern volatile uint32_t ms_counter;
extern void delay_timer_ms(uint32_t ms);
extern uint8_t read_button(GPIO_TypeDef *port, uint16_t pin_mask);



#define BUZZER_PIN       (1 << 9)   // PC9
#define RED_LED_PIN      (1 << 6)   // PC6
#define BLUE_LED_PIN     (1 << 7)   // PC7
#define DC_MOTOR_PIN     (1 << 8)   // PC8
#define WATER_PUMP_PIN   (1 << 2)   // PC2
#define FIRE_SWITCH_PIN  (1 << 7)   // Pb7
#define SMOKE_SWITCH_PIN (1 << 3)   // Pb3
#define GAS_POT_PIN      (1 << 4)   // PC1 (ADC input)

// UART3 pins (PC10 - TX, PC11 - RX)
#define UART3_TX_PIN     (1 << 10)  // PC10
#define UART3_RX_PIN     (1 << 11)  // PC11

// Button definitions from secq.c
#define UP_BUTTON_PIN     (1 << 7)   // PB7
#define DOWN_BUTTON_PIN   (1 << 3)   // PB3
#define LEFT_BUTTON_PIN   (1 << 4)   // PB4
#define ENTER_BUTTON_PIN  (1 << 15)  // PA15

// Test parameters
#define SWITCH_TEST_COUNT 5
#define SWITCH_TIMEOUT_MS 10000  // 10 seconds

// Test results
typedef enum {
    TEST_PASS = 0,
    TEST_FAIL = 1,
    TEST_TIMEOUT = 2
} test_result_t;

// Function prototypes
void test_gpio_init(void);
void adc_init(void);
void test_uart_init(void);
uint8_t read_button(GPIO_TypeDef *port, uint16_t pin_mask);
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

// Initialize GPIO pins for test mode
void test_gpio_init(void) {
    // Enable clocks for GPIOA, GPIOB, GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // Configure output pins - Buzzer (PC9), LEDs (PC6, PC7), Motor (PC8), Pump (PC2)
    GPIOC->MODER |= (1 << (9 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2)) |
                    (1 << (8 * 2)) | (1 << (2 * 2));  // Set as output

    // Configure input pins - Fire switch (PC3), Smoke switch (PC0) with pull-up
    GPIOC->MODER &= ~((3 << (3 * 2)) | (3 << (0 * 2)));  // Set as input
    GPIOC->PUPDR |= (1 << (3 * 2)) | (1 << (0 * 2));     // Pull-up

    // Configure PC1 for ADC (analog input) - Gas sensor
    GPIOC->MODER |= (3 << (1 * 2));  // Analog mode

    // Configure UART3 pins (PC10 - TX, PC11 - RX)
    // Configure PC10 (TX) and PC11 (RX) as alternate function
    	    GPIOC->MODER |= (1 << 21);
    	    GPIOC->MODER &= ~(1 << 20);
    	    GPIOC->MODER |= (1 << 23);
    	    GPIOC->MODER &= ~(1 << 22);
    	// Set alternate function AF7 (USART3)
    	    GPIOC->AFR[1] |= (1 << 8) | (1 << 9) | (1 << 10);
    	    GPIOC->AFR[1] &= ~(1 << 11);
    	    GPIOC->AFR[1] |= (1 << 12) | (1 << 13) | (1 << 14);
    	    GPIOC->AFR[1] &= ~(1 << 15);

    // Initialize all outputs to LOW
    GPIOC->ODR &= ~(BUZZER_PIN | RED_LED_PIN | BLUE_LED_PIN | DC_MOTOR_PIN | WATER_PUMP_PIN);
}

// Initialize ADC for gas sensor
void adc_init(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC1
    ADC1->CR2 |= ADC_CR2_ADON;      // Turn on ADC
    ADC1->SQR1 = 0;                 // 1 conversion
    ADC1->SQR3 = 11;                // Channel 11 (PC1)
    ADC1->SMPR1 |= (7 << (1 * 3));  // Sampling time for channel 11 (corrected shift)

    delay_timer_ms(10);  // Wait for ADC to stabilize
}

// Initialize UART3
void test_uart_init(void) {
    // Enable USART3 clock
	RCC->APB1ENR |= (1 << 18);  // Enable clock for USART3



    // Configure USART3
	USART3->BRR = 0X683;   // Baud rate setting (9600 @16MHz)

	    USART3->CR1 |= (1 << 3);  // Enable Transmitter
	    USART3->CR2 = 0;
	    USART3->CR3 = 0;
	    USART3->CR1 |= (1 << 13); // Enable USART3
}



// Test 1: Buzzer test for 5 seconds
void test_buzzer(void) {
    lcd(0x01, 0);  // Clear display
    lcd_string("Buzzer Test");
    lcd(0xC0, 0);  // Second line
    lcd_string("5 sec beep...");

    // Beep for 5 seconds
    GPIOC->ODR |= BUZZER_PIN;  // Turn on buzzer
    delay_timer_ms(100);      // Wait 1 seconds
    GPIOC->ODR &= ~BUZZER_PIN; // Turn off buzzer

    display_test_result("Buzzer", TEST_PASS);
    delay_timer_ms(1000);
}

// Test 2: LED blink test for 5 seconds
void test_leds(void) {
    lcd(0x01, 0);
    lcd_string("LED Blink Test");
    lcd(0xC0, 0);
    lcd_string("5 sec blink...");

    uint32_t start_time = ms_counter;
    while ((ms_counter - start_time) < 5000) {
        // Toggle LEDs every 500ms
        GPIOC->ODR ^= (RED_LED_PIN | BLUE_LED_PIN);
        delay_timer_ms(500);
    }

    // Turn off LEDs
    GPIOC->ODR &= ~(RED_LED_PIN | BLUE_LED_PIN);

    display_test_result("LED Blink", TEST_PASS);
    delay_timer_ms(1000);
}

// Test 3: DC Motor test for 10 seconds
void test_motor(void) {
    lcd(0x01, 0);
    lcd_string("DC Motor Test");
    lcd(0xC0, 0);
    lcd_string("10 sec run...");

    // Start motor
    GPIOC->ODR |= DC_MOTOR_PIN;
    delay_timer_ms(100);  // Run for 10 seconds
    GPIOC->ODR &= ~DC_MOTOR_PIN;  // Stop motor

    display_test_result("DC Motor", TEST_PASS);
    delay_timer_ms(1000);
}

// Test 4: Water pump test for 10 seconds
void test_pump(void) {
    lcd(0x01, 0);
    lcd_string("Water Pump Test");
    lcd(0xC0, 0);
    lcd_string("10 sec run...");

    // Start pump
    GPIOC->ODR |= WATER_PUMP_PIN;
    delay_timer_ms(100);  // Run for 10 seconds
    GPIOC->ODR &= ~WATER_PUMP_PIN;  // Stop pump

    display_test_result("Water Pump", TEST_PASS);
    delay_timer_ms(1000);
}

// Test 5: Fire sensor test (5 button presses within 10 seconds)
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
    delay_timer_ms(1000);
}

// Test 6: Smoke sensor test (5 button presses within 10 seconds)
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
    delay_timer_ms(1000);
}

// Test 7: Gas sensor test (read ADC and display voltage)
void test_gas_sensor(void) {
    lcd(0x01, 0);
    lcd_string("Gas Sensor Test");

    for (int i = 0; i < 10; i++) {  // Read 10 samples
        uint16_t adc_value = read_adc();
        float voltage = (adc_value * 3.3) / 4095.0;  // Convert to voltage

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
    delay_timer_ms(1000);
}

// Test 8: UART communication test
void test_uart_communication(void) {
    lcd(0x01, 0);
    lcd_string("UART Test");
    lcd(0xC0, 0);
    lcd_string("Sending...");

    // Send welcome message
    uart3_send_string("Welcome from Master STM32!\r\n");

    delay_timer_ms(1000);

    lcd(0xC0, 0);
    lcd_string("Receiving...");

    // Try to receive response (simplified - just wait for any character)
    uint32_t start_time = ms_counter;
    char received = 0;

    while ((ms_counter - start_time) < 5000) {  // 5 second timeout
        if (USART3->SR & USART_SR_RXNE) {
            received = uart3_receive_char();
            break;
        }
    }

    if (received) {
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
}

// Main test mode function
void test_mode_run(void) {
    // Initialize all peripherals
    test_gpio_init();
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
