#include "I2C.h"

#define I2C_TIMEOUT 10000  // таймаут цикла

void I2C_Config(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB8, PB9 в AF
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER |=  (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);

    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9;

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
    GPIOB->PUPDR |=  GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;

    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1] |=  (4 << 0) | (4 << 4);

    // Reset I2C
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // APB1 = 48 MHz
    I2C1->CR2 &= ~I2C_CR2_FREQ;
    I2C1->CR2 |= 48;

    // 100 kHz
    I2C1->CCR = 240;
    I2C1->TRISE = 49;

    I2C1->CR1 |= I2C_CR1_PE;
}

// --- Вспомогательная функция с таймаутом ---
static int I2C_WaitFlag(volatile uint32_t *reg, uint32_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (!(*reg & flag) && timeout--) { __NOP(); }
    return (timeout == 0) ? -1 : 0;
}

// --- I2C операции ---
int I2C_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;
    return I2C_WaitFlag(&I2C1->SR1, I2C_SR1_SB);
}

int I2C_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

int I2C_Address(uint8_t addr)
{
    I2C1->DR = addr;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_ADDR) != 0) return -1;
    volatile uint32_t tmp = I2C1->SR1 | I2C1->SR2;
    (void)tmp;
    return 0;
}

int I2C_Write(uint8_t data)
{
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_TXE) != 0) return -1;
    I2C1->DR = data;
    if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_BTF) != 0) return -1;
    return 0;
}

// --- MPU операции с проверкой ошибок ---
int MPU_Write(uint8_t dev, uint8_t reg, uint8_t data)
{
    if (I2C_Start() != 0) return -1;
    if (I2C_Address(dev << 1) != 0) { I2C_Stop(); return -1; }
    if (I2C_Write(reg) != 0) { I2C_Stop(); return -1; }
    if (I2C_Write(data) != 0) { I2C_Stop(); return -1; }
    I2C_Stop();
    return 0;
}

int MPU_Read(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (I2C_Start() != 0) return -1;
    if (I2C_Address(dev << 1) != 0) { I2C_Stop(); return -1; }
    if (I2C_Write(reg) != 0) { I2C_Stop(); return -1; }

    // Повторный запуск для чтения
    if (I2C_Start() != 0) return -1;

    if (len == 1)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;      
        if (I2C_Address((dev << 1) | 1) != 0) { I2C_Stop(); return -1; }

        if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_RXNE) != 0) { I2C_Stop(); return -1; }
        buf[0] = I2C1->DR;
        I2C_Stop();
    }
    else
    {
        I2C1->CR1 |= I2C_CR1_ACK;
        if (I2C_Address((dev << 1) | 1) != 0) { I2C_Stop(); return -1; }

        for (uint8_t i = 0; i < len; i++)
        {
            if (i == len - 1)
            {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                I2C_Stop();
            }
            if (I2C_WaitFlag(&I2C1->SR1, I2C_SR1_RXNE) != 0) return -1;
            buf[i] = I2C1->DR;
        }
    }
    return 0;
}