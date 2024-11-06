#include "stm32f4xx.h"

void TIM2_PWM_Init(void);
void Servo_SetAngle(uint8_t angle);

int main(void) {
    TIM2_PWM_Init(); // Initialize Timer 2 for PWM

    // Set a fixed angle to test
    Servo_SetAngle(90); // Set servo to 90 degrees
    while (1); // Loop indefinitely
}

void TIM2_PWM_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    GPIOA->MODER &= ~(3UL << (2 * 5));
    GPIOA->MODER |= (2UL << (2 * 5));
    GPIOA->AFR[0] |= (1UL << (4 * 5));
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 1600 - 1;             // Prescaler to 1 MHz
    TIM2->ARR = 20000 - 1;            // 50 Hz
    TIM2->CCMR1 |= (6UL << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->CCR1 = 1500;                // Initial pulse width for 90 degrees
    TIM2->CR1 |= TIM_CR1_CEN;         // Enable TIM2
}

void Servo_SetAngle(uint8_t angle) {
    if (angle > 180) {
        angle = 180;  // Cap angle to 180 degrees
    }
    uint16_t pulse_width = 1000 + ((angle * 1000) / 180);
    TIM2->CCR1 = pulse_width; // Set the calculated pulse width
}
