#include "I2C_soft.h" 

// 简单的微秒延时，用于模拟I2C时序

static void delay_us(uint32_t us)
{
    uint32_t i = us * 10; // 这里的系数可以根据实际示波器波形调整，10大概适配72Mhz
    while(i--);
}

void MyI2C_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (GPIO_PinState)BitValue);
    delay_us(2); // 稍微延时即可，不需要10ms
}

void MyI2C_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (GPIO_PinState)BitValue);
    delay_us(2);
}

uint8_t MyI2C_R_SDA()
{
    uint8_t BitValue;
    BitValue = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    delay_us(2);
    return BitValue;
}

void MyI2C_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*  开启GPIOB时钟  */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 开启AFIO时钟并禁用JTAG (重要! PB3/PB4默认是JTAG口，不关掉无法做GPIO) */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); 

    /*  配置GPIO */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_3, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;         // 建议开启内部上拉，除非外部有强上拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MyI2C_Start()
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

void MyI2C_Stop()
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        MyI2C_W_SDA(Byte & (0x80 >> i));
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
}

uint8_t MyI2C_ReceiveByte()
{
    uint8_t i;
    uint8_t temp = 0;
    MyI2C_W_SDA(1); // 释放SDA总线
    for(i=0; i<8; i++)
    {
        MyI2C_W_SCL(1);
        if(MyI2C_R_SDA() == 1)
        {
            temp |= (0x80 >> i);
        }
        MyI2C_W_SCL(0);
    }
    return temp;
}

void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck()
{
    uint8_t AckBit = 0;
    MyI2C_W_SDA(1); // 释放SDA
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);
    return AckBit;
}

