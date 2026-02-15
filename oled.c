//******************************************************************************
//  * @file           : oled.c
//  ******************************************************************************

#include "oled.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define OLED_SCL_W(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (GPIO_PinState)(x))
#define OLED_SDA_W(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (GPIO_PinState)(x))

static void OLED_I2C_Delay(void) {
    uint32_t i = 10; 
    while(i--);
}

void OLED_I2C_Start(void) {
    OLED_SDA_W(1); OLED_SCL_W(1); OLED_I2C_Delay();
    OLED_SDA_W(0); OLED_I2C_Delay();
    OLED_SCL_W(0); OLED_I2C_Delay();
}

void OLED_I2C_Stop(void) {
    OLED_SDA_W(0); OLED_SCL_W(1); OLED_I2C_Delay();
    OLED_SDA_W(1); OLED_I2C_Delay();
}

void OLED_I2C_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        OLED_SDA_W(byte & (0x80 >> i));
        OLED_I2C_Delay();
        OLED_SCL_W(1); OLED_I2C_Delay();
        OLED_SCL_W(0); OLED_I2C_Delay();
    }
    OLED_SDA_W(1); 
    OLED_SCL_W(1); OLED_I2C_Delay();
    OLED_SCL_W(0); OLED_I2C_Delay();
}

void OLED_Send_Soft(uint8_t *data, uint8_t len) {
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78); 
    for (uint8_t i = 0; i < len; i++) {
        OLED_I2C_SendByte(data[i]);
    }
    OLED_I2C_Stop();
}

void OLED_SendCmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    OLED_Send_Soft(buf, 2);
}

void OLED_SendData(uint8_t *data, uint16_t len) {
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x40);
    for (uint16_t i = 0; i < len; i++) {
        OLED_I2C_SendByte(data[i]);
    }
    OLED_I2C_Stop();
}



#define OLED_PAGE 8
#define OLED_COLUMN 128
uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN];

void OLED_Init(void) {
    OLED_SCL_W(1); OLED_SDA_W(1);
    HAL_Delay(50); 
    

    OLED_SendCmd(0xAE); OLED_SendCmd(0x20); OLED_SendCmd(0x10);
    OLED_SendCmd(0xB0); OLED_SendCmd(0xC8); OLED_SendCmd(0x00);
    OLED_SendCmd(0x10); OLED_SendCmd(0x40); OLED_SendCmd(0x81);
    OLED_SendCmd(0xDF); OLED_SendCmd(0xA1); OLED_SendCmd(0xA6);
    OLED_SendCmd(0xA8); OLED_SendCmd(0x3F); OLED_SendCmd(0xA4);
    OLED_SendCmd(0xD3); OLED_SendCmd(0x00); OLED_SendCmd(0xD5);
    OLED_SendCmd(0xF0); OLED_SendCmd(0xD9); OLED_SendCmd(0x22);
    OLED_SendCmd(0xDA); OLED_SendCmd(0x12); OLED_SendCmd(0xDB);
    OLED_SendCmd(0x20); OLED_SendCmd(0x8D); OLED_SendCmd(0x14);

    OLED_NewFrame();
    OLED_ShowFrame();
    OLED_SendCmd(0xAF);
}

void OLED_NewFrame(void) {
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

void OLED_ShowFrame(void) {
    for (uint8_t i = 0; i < OLED_PAGE; i++) {
        OLED_SendCmd(0xB0 + i); 
        OLED_SendCmd(0x00);     
        OLED_SendCmd(0x10);     
        OLED_SendData(OLED_GRAM[i], OLED_COLUMN);
    }
}


void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color) {
    if (x >= OLED_COLUMN || y >= 64) return;
    if (!color) OLED_GRAM[y / 8][x] |= 1 << (y % 8);
    else OLED_GRAM[y / 8][x] &= ~(1 << (y % 8));
}


void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color) {
    uint8_t oneLen = ((font->h + 7) / 8) * font->w;
    const uint8_t *charData = font->chars + (ch - ' ') * oneLen;
    
    for (uint8_t i = 0; i < font->w; i++) {
        for (uint8_t j = 0; j < font->h; j++) {
            uint8_t bit = (charData[i + (j / 8) * font->w] >> (j % 8)) & 0x01;
            if (bit) OLED_SetPixel(x + i, y + j, color);
        }
    }
}

void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color) {
    while (*str) {
        OLED_PrintASCIIChar(x, y, *str, font, color);
        x += font->w;
        str++;
    }
}
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color)
{
  static uint8_t temp = 0;
  if (x1 == x2)
  {
    if (y1 > y2)
    {
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (uint8_t y = y1; y <= y2; y++)
    {
      OLED_SetPixel(x1, y, color);
    }
  }
  else if (y1 == y2)
  {
    if (x1 > x2)
    {
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (uint8_t x = x1; x <= x2; x++)
    {
      OLED_SetPixel(x, y1, color);
    }
  }
  else
  {

    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    int16_t ux = ((dx > 0) << 1) - 1;
    int16_t uy = ((dy > 0) << 1) - 1;
    int16_t x = x1, y = y1, eps = 0;
    dx = abs(dx);
    dy = abs(dy);
    if (dx > dy)
    {
      for (x = x1; x != x2; x += ux)
      {
        OLED_SetPixel(x, y, color);
        eps += dy;
        if ((eps << 1) >= dx)
        {
          y += uy;
          eps -= dx;
        }
      }
    }
    else
    {
      for (y = y1; y != y2; y += uy)
      {
        OLED_SetPixel(x, y, color);
        eps += dx;
        if ((eps << 1) >= dy)
        {
          x += ux;
          eps -= dy;
        }
      }
    }
  }
}

void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color)
{
  OLED_DrawLine(x, y, x + w, y, color);
  OLED_DrawLine(x, y + h, x + w, y + h, color);
  OLED_DrawLine(x, y, x, y + h, color);
  OLED_DrawLine(x + w, y, x + w, y + h, color);
}

void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color)
{
  for (uint8_t i = 0; i < h; i++)
  {
    OLED_DrawLine(x, y + i, x + w, y + i, color);
  }
}

