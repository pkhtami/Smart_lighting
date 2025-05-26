#include "stm32f1xx.h"

volatile uint32_t millis = 0;

void delay_ms(uint32_t ms) {
    uint32_t start = millis;
    while ((millis - start) < ms);
}

void gpio_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;

    // PA0 = ورودی دیجیتال KY-037 (دست زدن)
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= (0b10 << GPIO_CRL_CNF0_Pos); // input pull-up/down
    GPIOA->ODR &= ~(1 << 0); // pull-down

    // PA1 = ورودی دیجیتال سنسور حرکت
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= (0b10 << GPIO_CRL_CNF1_Pos); // input pull-up/down
    GPIOA->ODR &= ~(1 << 1); // pull-down

    // PC13 = خروجی LED
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= (0b10 << GPIO_CRH_MODE13_Pos); // output 2 MHz
    GPIOC->CRH |= (0b00 << GPIO_CRH_CNF13_Pos);  // push-pull
    GPIOC->ODR |= (1 << 13); // LED خاموش

    // PA2 = خروجی رله اکتیو لاو (SSR)
    GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= (0b10 << GPIO_CRL_MODE2_Pos); // output 2 MHz
    GPIOA->CRL |= (0b00 << GPIO_CRL_CNF2_Pos);  // push-pull
    GPIOA->ODR |= (1 << 2); // رله خاموش (قطع)

    SysTick_Config(SystemCoreClock / 1000); // هر 1ms
}

void led_and_relay_on(void) {
    GPIOC->ODR &= ~(1 << 13); // LED روشن
    GPIOA->ODR &= ~(1 << 2);  // رله روشن (وصل)
}

void led_and_relay_off(void) {
    GPIOC->ODR |= (1 << 13); // LED خاموش
    GPIOA->ODR |= (1 << 2);  // رله خاموش (قطع)
}

int detect_clap(void) {
    static uint8_t last_state = 0;
    uint8_t current = (GPIOA->IDR & 1);
    int detected = 0;
    if (current == 1 && last_state == 0) {
        detected = 1;
    }
    last_state = current;
    return detected;
}

int motion_detected(void) {
    return (GPIOA->IDR & (1 << 1)) != 0;
}

int main(void) {
    gpio_init();

    uint8_t clap_count = 0;
    uint32_t clap_times[3] = {0};
    uint8_t led_state = 0;

    uint32_t last_motion_time = millis;
    uint8_t led_disabled_due_to_timeout = 0;

    while (1) {
        // بررسی حرکت
        if (motion_detected()) {
            last_motion_time = millis;
        }

        // اگر LED روشن است و ۱۰ دقیقه از آخرین حرکت گذشته است، چراغ خاموش شود
        if (led_state && ((millis - last_motion_time) > 600000)) { // 600000ms = 10 دقیقه
            led_and_relay_off();
            led_state = 0;
            led_disabled_due_to_timeout = 1;
        }

        // تشخیص دست زدن
        if (detect_clap()) {
            uint32_t now = millis;

            if (clap_count == 0 || (now - clap_times[clap_count - 1] < 1500)) {
                clap_times[clap_count++] = now;
            } else {
                clap_count = 1;
                clap_times[0] = now;
            }

            delay_ms(100); // حذف نویز
        }

        if (clap_count == 2) {
            if ((millis - clap_times[1]) > 800) {
                if (!led_state && led_disabled_due_to_timeout) {
                    led_and_relay_on();
                    led_state = 1;
                    led_disabled_due_to_timeout = 0;
                } else {
                    led_state = !led_state;
                    if (led_state)
                        led_and_relay_on();
                    else
                        led_and_relay_off();
                }
                clap_count = 0;
            }
        }

        if (clap_count >= 3) {
            clap_count = 0;
        }
    }
}
