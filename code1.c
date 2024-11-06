#include "stm32f4xx.h"

//threshold temperature in tenths of a degree (27.3 °C)
const int threshold = 273; 

void delay(int time) {
    for (int i = 0; i < time; i++) {
        __NOP(); 
    }
}

// DHT11 reading logic
int readDHT11(void)
{
    // Start signal
    GPIOA->MODER |= (1U << (2 * 2));   // Set PA2 as output
    GPIOA->ODR &= ~(1U << 2);          // Pull down the pin
    delay(20000);                      // 18ms delay for DHT11
    GPIOA->ODR |= (1U << 2);           // Release the pin
    delay(30);                         // Wait for 30us

    GPIOA->MODER &= ~(3U << (2 * 2));  // Set PA2 as input

    // Wait for DHT11 response
    while ((GPIOA->IDR & (1U << 2)) != 0);  // Wait for low
    while ((GPIOA->IDR & (1U << 2)) == 0);  // Wait for high
    while ((GPIOA->IDR & (1U << 2)) != 0);  // Wait for low

    // Now read the 5 bytes from DHT11
    uint8_t data[5] = {0};

    for (int i = 0; i < 5; i++)
		{
        for (int j = 0; j < 8; j++)
			  {
            while ((GPIOA->IDR & (1U << 2)) == 0); // Wait for high
            delay(40);  // Wait for 40us
            if (GPIOA->IDR & (1U << 2)) 
						{
                data[i] |= (1 << (7 - j)); // Set bit
            }
            while ((GPIOA->IDR & (1U << 2)) != 0); // Wait for low
        }
    }
    // Calculate temperature (data[0] contains the integer part of temperature)
    int temperature = data[0]; // in °C
    return temperature * 10; // Convert to tenths of degrees (e.g., 25 °C = 250)
}

int main(void) {
    // Enable GPIOA clock for PIR sensor and temperature sensor
    RCC->AHB1ENR |= (1U << 0);
    // Enable GPIOB clock for LED on PB9
    RCC->AHB1ENR |= (1U << 1);
    // Enable GPIOC clock for onboard LED
    RCC->AHB1ENR |= (1U << 2);

    // Set PA0 (where the PIR OUT is connected) to input mode
    GPIOA->MODER &= ~(3U << (0 * 2));
    // Set PA1 (where the light sensor is connected) to input mode
    GPIOA->MODER &= ~(3U << (1 * 2));
    // Set PA2 (where the DHT11 sensor is connected) to input mode
    GPIOA->MODER &= ~(3U << (2 * 2)); // Set PA2 as input

    // Set PC13 to output mode (for onboard LED)
    GPIOC->MODER |= (1U << 26);  // Set PC13 to output mode
    GPIOC->MODER &= ~(1U << 27);  // Clear the second bit for PC13

    // Set PC14 to output mode (for temperature indicator LED)
    GPIOC->MODER |= (1U << 28);   // Set PC14 to output mode
    GPIOC->MODER &= ~(1U << 29);   // Clear the second bit for PC14

    // Set PB9 to output mode (for external LED)
    GPIOB->MODER |= (1U << 18);   // Set PB9 to output mode
    GPIOB->MODER &= ~(1U << 19);   // Clear the second bit for PB9

    while (1) {
        // Read the PIR sensor output from PA0
        int motionDetected = (GPIOA->IDR & (1U << 0)) ? 1 : 0;

        // Read the light sensor output from PA1 (assuming active low)
        int lightDetected = (GPIOA->IDR & (1U << 1)) ? 1 : 0; // High if light is detected

        // Read the temperature from DHT11 connected to PA2
        int temperature = readDHT11(); // Use the read function to get temperature

        // If motion is detected (OUT is high), turn on PC13
        if (motionDetected) 
				{
            GPIOC->ODR &= ~(1U << 13);  // Turn on PC13 (active low LED)
            if (lightDetected) 
						{
                GPIOB->ODR = (1U << 9);  // Turn on PB9
                delay(100000);
                GPIOB->ODR &= ~(1U << 9);  // Turn off PB9
            }
        } 
				else 
				{
            GPIOC->ODR |= (1U << 13);   // Turn off PC13 (active low LED)
            GPIOB->ODR &= ~(1U << 9);   // Turn off PB9
        }

        // Control the LED at PC14 based on the temperature reading
        if (temperature > threshold) 
				{ 
            GPIOC->ODR ^= (1U << 14);  // Blink PC14 LED
            delay(500000);  // Adjust delay for blinking speed
        } 
				else
				{
            // Turn off PC14 LED
            GPIOC->ODR &= ~(1U << 14);  // Turn off PC14 LED
        }

        // Optional: Add a small delay to debounce (avoid flickering)
        delay(100000);  // Adjust this delay as necessary
    }
}
