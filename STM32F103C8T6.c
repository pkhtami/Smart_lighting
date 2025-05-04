#include "stm32f1xx_hal.h"

#define RELAY_PIN        GPIO_PIN_0
#define RELAY_PORT       GPIOA
#define SOUND_PIN        GPIO_PIN_1
#define SOUND_PORT       GPIOA
#define MOTION_PIN       GPIO_PIN_3
#define MOTION_PORT      GPIOA
#define BH1750_ADDR      (0x23 << 1)
#define LIGHT_THRESHOLD  500.0
#define INACTIVITY_TIME  600000

I2C_HandleTypeDef hi2c1;
uint8_t relayState = 0;
uint32_t lastMotionTime = 0;
uint32_t lastClapTime = 0;
uint8_t clapCount = 0;

float BH1750_ReadLight() {
    uint8_t data[2] = {0};
    uint8_t cmd = 0x10;
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(180);
    HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDR, data, 2, HAL_MAX_DELAY);
    return ((data[0] << 8) | data[1]) / 1.2;
}

void Relay_Control(uint8_t state) {
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    relayState = state;
}

void CheckDoubleClap() {
    if (HAL_GPIO_ReadPin(SOUND_PORT, SOUND_PIN) == GPIO_PIN_SET) {
        uint32_t currentTime = HAL_GetTick();
        if ((currentTime - lastClapTime) < 500) {
            if (++clapCount >= 2) {
                Relay_Control(!relayState);
                clapCount = 0;
            }
        } else {
            clapCount = 1;
        }
        lastClapTime = currentTime;
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    while (1) {
        float lux = BH1750_ReadLight();
        
        if (HAL_GPIO_ReadPin(MOTION_PORT, MOTION_PIN) == GPIO_PIN_SET) {
            lastMotionTime = HAL_GetTick();
        }

        CheckDoubleClap();

        if (lux > LIGHT_THRESHOLD || (HAL_GetTick() - lastMotionTime) > INACTIVITY_TIME) {
            Relay_Control(0);
        }

        HAL_Delay(100);
    }
}

void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c1);
}

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = RELAY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(RELAY_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SOUND_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SOUND_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTION_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(MOTION_PORT, &GPIO_InitStruct);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}